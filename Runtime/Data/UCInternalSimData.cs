using System;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

namespace UCloth
{
    /// <summary>
    /// Stores simulation data used by <see cref="UCCloth"/>.
    /// </summary>
    public class UCInternalSimData : IDisposable
    {
        // --- Integration Data

        /// <summary>
        /// Read-only access to node positions.
        /// </summary>
        public NativeArray<float3> positionsReadOnly;
        internal NativeArray<float3> cPositions;

        /// <summary>
        /// Read-only access to node velocities.
        /// </summary>
        public NativeArray<float3> velocityReadOnly;
        internal NativeArray<float3> cVelocity;

        internal NativeArray<float3> cAcceleration;
        internal NativeArray<float3> cTempAcceleration;


        // --- Mesh Data

        // For those fields, it's not common to write them from outside, so they can be kept as read-only
        // These map directly to memory used in the sim

        /// <summary>
        /// Read-only access to edges (node connections).
        /// </summary>
        public NativeArray<UCEdge> edgesReadOnly;

        /// <summary>
        /// Read-only access to node connections at which bending forces are applied.
        /// </summary>
        public NativeArray<UCBendingEdge> bendingEdgesReadOnly;

        /// <summary>
        /// Read-only access to resting edge lengths.
        /// </summary>
        public NativeArray<float> restDistancesReadOnly;

        /// <summary>
        /// Read-only access to node neighbours.
        /// </summary>
        public NativeParallelMultiHashMap<ushort, ushort> neighboursReadOnly;


        // --- Helper Data

        /// <summary>
        /// Read-only access to node normals.
        /// </summary>
        public NativeArray<float3> normalsReadOnly;

        /// <summary>
        /// Read-only access to triangle normals.
        /// </summary>
        public NativeArray<float3> triangleNormalsReadOnly;


        /// <summary>
        /// The reciprocal of the weight. 0 for pinned nodes. Writeable.
        /// </summary>
        public NativeArray<float> reciprocalWeight;
        internal NativeArray<float> cReciprocalWeight;

        // --- Pins
        /// <summary>
        /// World-space position specified only for pinned nodes. Writeable.
        /// </summary>
        public NativeParallelHashMap<ushort, float3> pinnedPositions;
        internal NativeParallelHashMap<ushort, float3> cPinnedPositions;

        // --- Spatial Partitioning
        internal Native3DHashmapArray<ushort> cSelfCollisionRegions;
        internal NativeParallelHashSet<int3> cUtilizedSelfColRegions;


        private bool _applyPending = true;

        internal void PrepareCopies()
        {
            if (!positionsReadOnly.IsCreated)
                positionsReadOnly = new NativeArray<float3>(cPositions, Allocator.Persistent);

            if (!velocityReadOnly.IsCreated)
                velocityReadOnly = new NativeArray<float3>(cVelocity, Allocator.Persistent);


            if (!cReciprocalWeight.IsCreated)
                cReciprocalWeight = new NativeArray<float>(reciprocalWeight, Allocator.Persistent);

            if (!cPinnedPositions.IsCreated)
                cPinnedPositions = new NativeParallelHashMap<ushort, float3>(pinnedPositions.Capacity, Allocator.Persistent);
        }

        internal void CopyWriteableData()
        {
            // Read-only
            positionsReadOnly.CopyFrom(cPositions);
            velocityReadOnly.CopyFrom(cVelocity);

            // No need to do this every frame, only do this if modified
            if (_applyPending)
            {
                // Pinned positions
                cPinnedPositions.Clear();
                NativeParallelHashMapCopyJob<ushort, float3> copyJob = new(pinnedPositions, cPinnedPositions);

                var copyHandle = copyJob.ScheduleBatch(pinnedPositions.Capacity, 256);
                copyHandle.Complete();

                // Weight
                cReciprocalWeight.CopyFrom(reciprocalWeight);
                _applyPending = false;
            }
        }


        /// <summary>
        /// Applies any external modifications to writeable data.
        /// </summary>
        public void ApplyModifiedData()
        {
            _applyPending = true;
        }

        public void Dispose()
        {
            positionsReadOnly.Dispose();
            cPositions.Dispose();

            velocityReadOnly.Dispose();
            cVelocity.Dispose();

            cAcceleration.Dispose();
            cTempAcceleration.Dispose();

            edgesReadOnly.Dispose();
            bendingEdgesReadOnly.Dispose();
            neighboursReadOnly.Dispose();
            normalsReadOnly.Dispose();
            triangleNormalsReadOnly.Dispose();
            restDistancesReadOnly.Dispose();
            reciprocalWeight.Dispose();
            cReciprocalWeight.Dispose();
            pinnedPositions.Dispose();
            cPinnedPositions.Dispose();

            if (cSelfCollisionRegions.IsCreated())
                cSelfCollisionRegions.Dispose();

            if (cUtilizedSelfColRegions.IsCreated)
                cUtilizedSelfColRegions.Dispose();
        }
    }
}