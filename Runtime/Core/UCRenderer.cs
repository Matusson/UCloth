using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Handles rendering the cloth simulated in <see cref="UCCloth"/>
    /// </summary>
    internal class UCRenderer : IDisposable
    {
        private readonly UCCloth _scheduler;
        private readonly Transform _transform;
        private readonly MeshFilter _filter;
        private readonly MeshCollider _collider;

        // The preprocessors will merge vertices with the same positions, otherwise weird issues happen with UV seams
        // This dictionary stores those merges so the mesh can still be reconstructed correctly.
        private readonly Dictionary<int, int> _vertexSwaps;
        private readonly NativeArray<int> _renderToSimIndexLookup;

        private UCRenderingMeshData _data;
        private readonly int _rawVertexCount;

        internal UCRenderer(UCCloth scheduler, UCMeshData data, MeshFilter filter, MeshCollider collider)
        {
            _scheduler = scheduler;
            _transform = scheduler.transform;
            _filter = filter;
            _collider = collider;

            _rawVertexCount = _filter.mesh.vertexCount;
            _vertexSwaps = data.vertexMerges;
            _renderToSimIndexLookup = data.renderToSimLookup;

            _data = new()
            {
                vertices = new NativeArray<float3>(filter.mesh.vertices.Length, Allocator.Persistent),
                normals = new NativeArray<float3>(filter.mesh.normals.Length, Allocator.Persistent),
                triangles = filter.mesh.triangles,
                uvs = filter.mesh.uv
            };
        }

        public void Dispose()
        {
            _data.vertices.Dispose();
            _data.normals.Dispose();
        }


        /// <summary>
        /// Updates the rendered mesh.
        /// </summary>
        internal void UpdateRenderedMesh()
        {
            if (!_scheduler.simData.positionsReadOnly.IsCreated)
                return;

            // Update positions
            var worldToLocal = _transform.worldToLocalMatrix;
            for (int i = 0; i < _rawVertexCount; i++)
            {
                int targetVertex = i;

                if (_vertexSwaps.ContainsKey(i))
                    targetVertex = _vertexSwaps[i];

                targetVertex = _renderToSimIndexLookup[targetVertex];

                // Translate into local space
                _data.vertices[i] = worldToLocal.MultiplyPoint(_scheduler.simData.positionsReadOnly[targetVertex]);
            }

            bool updateTrisUvs = ApplyPostprocessors();
            var mesh = _filter.mesh;

            // If tris or UVs modified in postprocessors
            // This has to be split around SetVertices, otherwise Unity complains
            // (either out of bounds vertices, or triangles)
            if (updateTrisUvs)
            {
                if (_data.triangles.Length < mesh.triangles.Length)
                    mesh.triangles = _data.triangles;
            }

            // Vertices
            mesh.SetVertices(_data.vertices, 0, _data.vertices.Length, UnityEngine.Rendering.MeshUpdateFlags.DontValidateIndices);

            if (updateTrisUvs)
            {
                if (_data.triangles.Length > mesh.triangles.Length)
                    mesh.triangles = _data.triangles;

                mesh.uv = _data.uvs;
            }


            mesh.SetNormals(_data.normals);
            mesh.RecalculateBounds();

            // Update the collider if attached
            if (_collider != null)
                _collider.sharedMesh = mesh;
        }

        /// <summary>
        /// Updates normals used for rendering.
        /// </summary>
        /// <param name="newNormals"></param>
        internal void UpdateRenderingNormals(NativeArray<float3> newNormals)
        {
            // Normals need to be in local space
            for (int i = 0; i < _rawVertexCount; i++)
            {
                int targetIndex = _renderToSimIndexLookup[i];
                _data.normals[i] = _transform.InverseTransformDirection(newNormals[targetIndex]);
            }
        }

        /// <summary>
        /// Applies all postprocessors. Returns true if triangle or UV data was modified.
        /// </summary>
        /// <returns> Triangles or UVs were modified. </returns>
        private bool ApplyPostprocessors()
        {
            int initialTrisCount = _data.triangles.Length;
            int initialVertCount = _data.vertices.Length;

            List<IUCPostprocessor> combinedPostprocessors = new(_scheduler.internalPostprocessors);
            combinedPostprocessors.AddRange(_scheduler.postprocessors);

            foreach (var postprocessor in combinedPostprocessors)
            {
                if (postprocessor == null)
                    continue;

                // Cleanup should remove all data initially added by the postprocessor.
                if (postprocessor.ScheduledToCleanup)
                    _data = postprocessor.Cleanup(_data);
                else
                    _data = postprocessor.Process(_data);
            }

            // Disposes and removes disabled postprocessors
            for (int i = 0; i < combinedPostprocessors.Count; i++)
            {
                if (combinedPostprocessors[i] == null)
                    continue;

                if (combinedPostprocessors[i].Deactivated)
                {
                    combinedPostprocessors[i].Dispose();
                    continue;
                }
            }

            // Then remove from list/array
            for (int i = 0; i < _scheduler.internalPostprocessors.Length; i++)
            {
                if (_scheduler.internalPostprocessors[i] == null)
                    continue;

                if (_scheduler.internalPostprocessors[i].Deactivated)
                {
                    _scheduler.internalPostprocessors[i] = null;
                    continue;
                }
            }
            for (int i = 0; i < _scheduler.postprocessors.Count; i++)
            {
                if (_scheduler.postprocessors[i].Deactivated)
                {
                    _scheduler.postprocessors.RemoveAt(i);
                    continue;
                }
            }


            // Check if need to update triangles or UVs
            // (vertices and normals are updated always)
            if (initialTrisCount != _data.triangles.Length || initialVertCount != _data.vertices.Length)
                return true;

            return false;
        }
    }
}