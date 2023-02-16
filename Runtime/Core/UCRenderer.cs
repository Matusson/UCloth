using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

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
        // This lookup array stores those merges so the mesh can still be reconstructed correctly.
        private readonly NativeArray<int> _renderToSimIndexLookup;

        private NativeArray<float3> _latestWorldSpacePositions;
        private NativeArray<float3> _latestWorldSpaceNormals;

        private JobHandle _positionTransformJob;
        private JobHandle _normalTransformJob;
        private UCRenderingMeshData _data;
        private readonly int _rawVertexCount;
        private bool canReuseData;

        private const int BATCH_SIZE = 512;

        internal UCRenderer(UCCloth scheduler, UCMeshData data, MeshFilter filter, MeshCollider collider)
        {
            _scheduler = scheduler;
            _transform = scheduler.transform;
            _filter = filter;
            _collider = collider;

            _rawVertexCount = _filter.mesh.vertexCount;
            _renderToSimIndexLookup = data.renderToSimLookup;

            _latestWorldSpacePositions = new(data.positions.Count, Allocator.Persistent);
            _latestWorldSpaceNormals = new(data.positions.Count, Allocator.Persistent);


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
            _positionTransformJob.Complete();
            _normalTransformJob.Complete();

            _data.vertices.Dispose();
            _data.normals.Dispose();

            _latestWorldSpacePositions.Dispose();
            _latestWorldSpaceNormals.Dispose();
        }


        /// <summary>
        /// Updates the rendered mesh.
        /// </summary>
        internal void UpdateRenderedMesh()
        {
            if (!_scheduler.simData.positionsReadOnly.IsCreated)
                return;

            Profiler.BeginSample("UCRendererUpdate");

            // If transformations are still running, they need to complete
            _positionTransformJob.Complete();
            _normalTransformJob.Complete();

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
            Profiler.EndSample();
        }

        /// <summary>
        /// Updates mesh positions for rendering.
        /// </summary>
        /// <param name="newPositions"></param>
        internal void UpdateRenderingPositions(NativeArray<float3> newPositions)
        {
            _positionTransformJob.Complete();

            // Need to copy, otherwise simulation will override it
            _latestWorldSpacePositions.CopyFrom(newPositions);
        }

        /// <summary>
        /// Updates normals used for rendering.
        /// </summary>
        /// <param name="newNormals"></param>
        internal void UpdateRenderingNormals(NativeArray<float3> newNormals)
        {
            _normalTransformJob.Complete();

            // Need to copy, otherwise normal recomputation will overwrite it
            _latestWorldSpaceNormals.CopyFrom(newNormals);
        }


        /// <summary>
        /// Schedules transformation jobs.
        /// </summary>
        internal void ScheduleTransformations()
        {
            _positionTransformJob.Complete();
            _normalTransformJob.Complete();

            // Transform vertices into local space
            float4x4 worldToLocal = _transform.worldToLocalMatrix;
            TransformVerticesToLocalJob transformJob = new()
            {
                localSpaceVertices = _data.vertices,
                worldSpaceVertices = _scheduler.simData.positionsReadOnly,
                renderToSimIndexLookup = _renderToSimIndexLookup,
                worldToLocal = worldToLocal
            };

            // No need to block the main thread while these are computing
            _positionTransformJob = transformJob.Schedule(_rawVertexCount, BATCH_SIZE);

            // And normals as well
            TransformNormalsToLocalJob normalJob = new()
            {
                normalsLocalSpace = _data.normals,
                normalsWorldSpace = _latestWorldSpaceNormals,
                renderToSimIndexLookup = _renderToSimIndexLookup,
                worldToLocal = worldToLocal
            };

            _normalTransformJob = normalJob.Schedule(_rawVertexCount, BATCH_SIZE);
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