using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Adds thickness to the simulated mesh.
    /// </summary>
    public class UCThicknessPostprocessor : IUCPostprocessor, IDisposable
    {
        private bool _firstRun = true;
        private int _originalTriCount;

        private readonly UCCloth _scheduler;

        public UCThicknessPostprocessor(UCCloth scheduler)
        {
            _scheduler = scheduler;
        }

        public bool ScheduledToCleanup { get; set; }

        public bool Deactivated { get => _deactInternal; }
        private bool _deactInternal = false;

        public UCRenderingMeshData Process(UCRenderingMeshData data)
        {
            // If first run, create new vertices
            if (_firstRun)
            {
                // Verts
                var verts = new NativeArray<float3>(data.vertices.Length * 2, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                for (int i = 0; i < data.vertices.Length; i++)       // Copy original data
                {
                    verts[i] = data.vertices[i];
                }
                ushort vertOffset = (ushort)data.vertices.Length;

                // Normals
                var normals = new NativeArray<float3>(data.normals.Length * 2, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                for (int i = 0; i < data.normals.Length; i++)       // Copy original data
                {
                    normals[i] = data.normals[i];
                }


                // Triangles
                int trisOffset = data.triangles.Length;
                _originalTriCount = trisOffset;
                List<int> tris = new(trisOffset * 2);
                for (int i = 0; i < trisOffset; i++)     // Copy original data
                {
                    tris.Add(data.triangles[i]);
                }

                // Make triangles for offset vertices
                for (int i = 0; i < trisOffset; i++)
                {
                    // Winding order has to be inverted here
                    if (i % 3 == 0)
                        tris.Add(data.triangles[i + 2] + vertOffset);
                    else if (i % 3 == 1)
                        tris.Add(data.triangles[i] + vertOffset);
                    else if(i % 3 == 2)
                        tris.Add(data.triangles[i - 2] + vertOffset);
                }

                // And side triangles
                for (int i = 0; i < _scheduler.initialMeshData.boundingEdges.Count; i++)
                {
                    var edge = _scheduler.initialMeshData.boundingEdges[i];
                    var solidifiedEdge = new UCEdge((ushort)(edge.nodeIndex1 + vertOffset), (ushort)(edge.nodeIndex2 + vertOffset));

                    tris.Add(edge.nodeIndex1);
                    tris.Add(solidifiedEdge.nodeIndex1);
                    tris.Add(solidifiedEdge.nodeIndex2);

                    tris.Add(edge.nodeIndex1);
                    tris.Add(solidifiedEdge.nodeIndex2);
                    tris.Add(edge.nodeIndex2);

                }

                // UVs (copied directly)
                var uvs = new Vector2[data.uvs.Length * 2];
                for (int i = 0; i < data.uvs.Length * 2; i++)
                {
                    uvs[i] = data.uvs[i % vertOffset];
                }


                _firstRun = false;
                data.vertices.Dispose();
                data.normals.Dispose();

                data = new UCRenderingMeshData()
                {
                    vertices = verts,
                    triangles = tris.ToArray(),
                    normals = normals,
                    uvs = uvs
                };
            }

            data = RunProcessorLogic(data);

            return data;
        }

        private UCRenderingMeshData RunProcessorLogic(UCRenderingMeshData data)
        {
            UCThicknessJob thicknessJob = new()
            {
                vertices = data.vertices,
                normals = data.normals,
                offsetFront = _scheduler.offsetFront,
                thickness = _scheduler.thickness / 1000f
            };

            thicknessJob.Run();

            return data;
        }

        public UCRenderingMeshData Cleanup(UCRenderingMeshData data)
        {
            // We can clean up by removing upper half of all previously modified arrays
            // Except for triangles, due to the rim
            var verts = new NativeArray<float3>(data.vertices.Length / 2, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < verts.Length; i++)       // Copy original data
            {
                verts[i] = data.vertices[i];
            }

            // Triangles
            var tris = new int[_originalTriCount];
            for (int i = 0; i < _originalTriCount; i++)     // Copy original data
            {
                tris[i] = data.triangles[i];
            }

            // Normals
            var normals = new NativeArray<float3>(data.normals.Length / 2, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < normals.Length; i++)       // Copy original data
            {
                normals[i] = data.normals[i];
            }

            // UVs
            var uvs = new Vector2[data.uvs.Length / 2];
            for (int i = 0; i < uvs.Length; i++)
            {
                uvs[i] = data.uvs[i];
            }

            _deactInternal = true;
            data.vertices.Dispose();
            data.normals.Dispose();

            return new UCRenderingMeshData()
            {
                vertices = verts,
                triangles = tris,
                normals = normals,
                uvs = uvs
            };
        }

        public void Dispose()
        {

        }
    }
}