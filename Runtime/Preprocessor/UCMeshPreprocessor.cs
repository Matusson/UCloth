using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Creates simulation data from a mesh.
    /// </summary>
    public class UCMeshPreprocessor : IUCPreprocessor
    {
        private List<UCEdge> _edges;
        private Dictionary<UCEdge, int> _edgeRefCount;

        private Dictionary<float3, ushort> _hashedVertices;
        private NativeParallelMultiHashMap<ushort, ushort> _neighbours;


        /// <param name="input"> Mesh to convert to sim data. </param>
        public bool ConvertMeshData(object input, out UCMeshData data)
        {
            Mesh mesh = input as Mesh;

            if (!mesh.isReadable)
            {
                Debug.LogError("Mesh is not readable. Turn on \"Read/Write\" in model import settings.");
                data = new();
                return false;
            }

            if (mesh.vertexCount >= ushort.MaxValue)
            {
                Debug.LogWarning($"Mesh has more than {ushort.MaxValue} vertices. You might experience issues simulating it. " +
                    $"Consider decreasing vertex count.");

            }

            //--- COMBINE VERTICES

            // Positions can be fetched directly from the mesh data
            // However, some vertices will be duplicated. This is due to uv seams.
            // To solve this, we check if any position before was the same, and if yes, we replace it
            _hashedVertices = new(mesh.vertexCount);

            // We still need a way to know which vertices have been merged in this process,
            // Otherwise we cannot assemble the mesh again. 
            NativeParallelHashMap<int, int> swaps = new(0, Allocator.Persistent);
            List<float3> positions = new(mesh.vertexCount);

            // As we only keep the simulation data in those arrays, accessing them for rendering is problematic (as duplicated indexes are skipped)
            // So we store a lookup for which rendering vertex is which sim index.
            NativeArray<int> renderToSimLookup = new(mesh.vertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            ushort uniqueVertId = 0;

            for (int i = 0; i < mesh.vertexCount; i++)
            {
                var position = mesh.vertices[i];

                if (_hashedVertices.ContainsKey(position))
                {
                    int trueVertex = _hashedVertices[position];
                    swaps[i] = trueVertex;
                    renderToSimLookup[i] = trueVertex;
                }
                else
                {
                    _hashedVertices[position] = uniqueVertId;
                    renderToSimLookup[i] = uniqueVertId;
                    uniqueVertId++;

                    positions.Add(position);
                }

            }


            //--- CONNECTED EDGES

            // Connected nodes are a bit more tricky, but can be extracted from triangle indices,
            // as each triangle has always 3 edges.
            _edges = new(positions.Count * 3);  // estimation

            // However, some care needs to be taken to ensure that we don't add the same edge twice, as triangles do share vertices.
            // For this reason, we keep a dictionary of created edges from each node
            // This is also used to pick bending points later on
            _neighbours = new(positions.Count, Allocator.Persistent);

            // Every triangle has 3 indices - and 3 edges
            // Three edges can be formed between these indices:
            // 1 - 2, 2 - 3, 1 - 3

            // Additionally, during this process we count the number of references per edge, to then find bounding edges
            // Bounding edges are only referenced by 1 triangle.
            _edgeRefCount = new();

            Dictionary<UCTriangle, int> trianglesCache = new();

            int trisCount = mesh.triangles.Length / 3;
            for (int i = 0; i < trisCount; i++)
            {
                int triangleIndex = i * 3;

                int ind1 = mesh.triangles[triangleIndex];
                int ind2 = mesh.triangles[triangleIndex + 1];
                int ind3 = mesh.triangles[triangleIndex + 2];

                // Some vertices might no longer be valid, because some were discarded in position stage
                // So we look at the hashed values instead
                UCEdge edge1 = GetHashedEdge(mesh, ind1, ind2);
                UCEdge edge2 = GetHashedEdge(mesh, ind2, ind3);
                UCEdge edge3 = GetHashedEdge(mesh, ind1, ind3);

                trianglesCache.Add(new UCTriangle(edge1.nodeIndex1, edge1.nodeIndex2, edge2.nodeIndex2), i);

                // And then create edges from the specified indices
                CreateEdge(edge1);
                CreateEdge(edge2);
                CreateEdge(edge3);
            }

            // Then identify bounding edges by reference count
            List<UCEdge> boundingEdges = new();
            for (int i = 0; i < trisCount; i++)
            {
                int triangleIndex = i * 3;

                int ind1 = mesh.triangles[triangleIndex];
                int ind2 = mesh.triangles[triangleIndex + 1];
                int ind3 = mesh.triangles[triangleIndex + 2];

                // Same idea as above
                UCEdge edge1 = GetHashedEdge(mesh, ind1, ind2);
                UCEdge edge2 = GetHashedEdge(mesh, ind2, ind3);
                UCEdge edge3 = GetHashedEdge(mesh, ind1, ind3);

                // Referenced once are bounding edges
                if (_edgeRefCount[edge1] == 1)
                    boundingEdges.Add(new UCEdge((ushort)ind1, (ushort)ind2));

                if (_edgeRefCount[edge2] == 1)
                    boundingEdges.Add(new UCEdge((ushort)ind2, (ushort)ind3));

                if (_edgeRefCount[edge3] == 1)
                    boundingEdges.Add(new UCEdge((ushort)ind3, (ushort)ind1));
            }


            //--- BENDING ELEMENTS

            // Then we pick bending elements
            // I don't think it's possible when creating edges initially, since the neighbours dictionary needs to be constructed?
            List<UCBendingEdge> bendingEdges = new();
            for (int i = 0; i < _edges.Count; i++)
            {
                UCEdge edge = _edges[i];

                // The bending elements are picked by finding the intersection of neighbours of both edge ends
                // Two should be found - one on the "left" of the edge, and one on the "right"
                // If there's one or none, then no bending element is created (as force would be unbalanced)
                ushort index1 = edge.nodeIndex1;
                ushort index2 = edge.nodeIndex2;
                var enu1 = _neighbours.GetValuesForKey(index1);
                var enu2 = _neighbours.GetValuesForKey(index2);

                List<int> commonNeighbours = new();
                enu1.Reset();
                enu2.Reset();

                // Finds common neighbours using the enumerators
                while (enu1.MoveNext())
                {
                    while (enu2.MoveNext())
                    {
                        if (enu1.Current == enu2.Current)
                        {
                            if (!commonNeighbours.Contains(enu2.Current))
                                commonNeighbours.Add(enu2.Current);
                        }
                    }
                    enu2.Reset();
                }

                if (commonNeighbours.Count == 2)
                {
                    int tri1 = trianglesCache[new UCTriangle(index1, index2, (ushort)commonNeighbours[0])];
                    int tri2 = trianglesCache[new UCTriangle(index2, index1, (ushort)commonNeighbours[1])];

                    // Order shouldn't matter here
                    UCBendingEdge bendingEdge = new(commonNeighbours[0], commonNeighbours[1], tri1, tri2);
                    bendingEdges.Add(bendingEdge);
                }
            }

            // Sort the edges by Y value - so that they're later processed vertically
            _edges = _edges.OrderBy(e =>
            {
                return (positions[e.nodeIndex1] + positions[e.nodeIndex2]).y;
            }).ToList();

            // Check if there were any stray vertices, not connected by any edges
            HashSet<ushort> usedVertices = new(positions.Count);
            for(int i = 0; i < _edges.Count; i++)
            {
                var edge = _edges[i];
                usedVertices.Add(edge.nodeIndex1);
                usedVertices.Add(edge.nodeIndex2);
            }
            int missingVertices = positions.Count - usedVertices.Count;
            if (missingVertices > 0)
                Debug.LogWarning("Some vertices are not connected by any edges. They will not be properly simulated.");

            data = new()
            {
                positions = positions,
                edges = _edges,
                bendingEdges = bendingEdges,
                boundingEdges = boundingEdges,

                triangles = new NativeArray<int>(mesh.triangles, Allocator.Persistent),
                neighbours = _neighbours,

                vertexMerges = swaps,
                renderToSimLookup = renderToSimLookup
            };
            return true;
        }


        //--- HELPER

        /// <summary>
        /// Creates an edge from given indices, checks for doubling.
        /// </summary>
        /// <param name="hashedIndex1"></param>
        /// <param name="hashedIndex2"></param>
        private void CreateEdge(UCEdge edge)
        {
            if (!DoesEdgeAlreadyExist(edge))
            {
                _edges.Add(edge);

                _neighbours.Add(edge.nodeIndex1, edge.nodeIndex2);
                _neighbours.Add(edge.nodeIndex2, edge.nodeIndex1);
            }

            // Count edge reference
            bool alreadyContains = _edgeRefCount.ContainsKey(edge);
            int refCount = alreadyContains ? _edgeRefCount[edge] : 0;

            if (!alreadyContains)
                _edgeRefCount.Add(edge, refCount + 1);
            else
                _edgeRefCount[edge] = refCount + 1;
        }

        /// <summary>
        /// Creates an edge using hashed indices.
        /// </summary>
        /// <returns></returns>
        private UCEdge GetHashedEdge(Mesh mesh, int index1, int index2)
        {
            float3 pos1 = mesh.vertices[index1];
            float3 pos2 = mesh.vertices[index2];

            ushort ind1Hashed = FindHashedIndex(pos1);
            ushort ind2Hashed = FindHashedIndex(pos2);

            return new UCEdge(ind1Hashed, ind2Hashed);
        }

        /// <summary>
        /// Returns the index of the node at the specified position.
        /// </summary>
        /// <param name="pos"></param>
        /// <returns></returns>
        /// <exception cref="Exception"> Thrown when position wasn't hashed before. </exception>
        private ushort FindHashedIndex(float3 pos)
        {
            if (!_hashedVertices.ContainsKey(pos))
            {
                throw new Exception($"Could not find hashed vertex position {pos} in UC preprocessor!");
            }
            return _hashedVertices[pos];
        }

        /// <summary>
        /// Checks if a given edge was already created.
        /// </summary>
        /// <param name="edge"></param>
        /// <returns></returns>
        private bool DoesEdgeAlreadyExist(UCEdge edge)
        {
            // Ensure the keys exist
            if (!_neighbours.ContainsKey((ushort)edge.nodeIndex1))
                return false;

            if (!_neighbours.ContainsKey((ushort)edge.nodeIndex2))
                return false;


            // The edge exists if the node is already a neighbour
            if (_neighbours.KeyContainsValue((ushort)edge.nodeIndex1, (ushort)edge.nodeIndex2))
                return true;

            if (_neighbours.KeyContainsValue((ushort)edge.nodeIndex2, (ushort)edge.nodeIndex1))
                return true;


            return false;
        }
    }
}