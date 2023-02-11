using System;
using Unity.Collections;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Stores simulation data used by <see cref="UCCloth"/>.
    /// </summary>
    public class UCInternalSimData : IDisposable
    {
        // Data the sim writes to needs to be copied
        internal NativeArray<float3> cPositions;
        public NativeArray<float3> positionsReadOnly;

        internal NativeArray<float3> cVelocity;
        public NativeArray<float3> velocityReadOnly;


        internal NativeArray<float3> cAcceleration;
        internal NativeArray<float3> cTempAcceleration;

        internal Native3DHashmapArray<ushort> cSelfCollisionRegions;
        internal NativeParallelHashSet<int3> cUtilizedSelfColRegions;

        // For those fields, it's not common to write them from outside, so they can be kept as read-only
        // These map directly to memory used in the sim
        public NativeArray<UCEdge> edgesReadOnly;
        public NativeArray<UCBendingEdge> bendingEdgesReadOnly;
        public NativeArray<float> restDistancesReadOnly;

        public NativeParallelMultiHashMap<ushort, ushort> neighboursReadOnly;

        public NativeArray<float3> normalsReadOnly;
        public NativeArray<float3> triangleNormalsReadOnly;

        /// <summary>
        /// The reciprocal of the weight. This is preferred to storing weight directly as it allows 0 to be used for pinned nodes,
        /// and replaces division with multiplication (which tends to be faster)
        /// </summary>
        public NativeArray<float> reciprocalWeight;
        internal NativeArray<float> cReciprocalWeight;

        public NativeParallelHashMap<ushort, float3> pinnedLocalPositions;
        internal NativeParallelHashMap<ushort, float3> cPinnedLocalPositions;



        internal void PrepareCopies()
        {
            if (!positionsReadOnly.IsCreated)
                positionsReadOnly = new NativeArray<float3>(cPositions, Allocator.Persistent);

            if (!velocityReadOnly.IsCreated)
                velocityReadOnly = new NativeArray<float3>(cVelocity, Allocator.Persistent);


            if (!cReciprocalWeight.IsCreated)
                cReciprocalWeight = new NativeArray<float>(reciprocalWeight, Allocator.Persistent);

            if (!cPinnedLocalPositions.IsCreated)
                cPinnedLocalPositions = new NativeParallelHashMap<ushort, float3>(pinnedLocalPositions.Capacity, Allocator.Persistent);
        }

        internal void CopyWriteableData()
        {
            // Read-only
            positionsReadOnly.CopyFrom(cPositions);
            velocityReadOnly.CopyFrom(cVelocity);

            cReciprocalWeight.CopyFrom(reciprocalWeight);

            // TODO: This gets triggered every time and is unnecessary, fix if possible
            if (cPinnedLocalPositions.GetHashCode() != pinnedLocalPositions.GetHashCode())
            {
                cPinnedLocalPositions.Clear();
                var keyValues = pinnedLocalPositions.GetKeyValueArrays(Allocator.Temp);

                int length = keyValues.Keys.Length;
                for (int i = 0; i < length; i++)
                {
                    ushort key = keyValues.Keys[i];
                    float3 value = keyValues.Values[i];

                    cPinnedLocalPositions.Add(key, value);
                }

                keyValues.Dispose();
            }
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
            pinnedLocalPositions.Dispose();
            cPinnedLocalPositions.Dispose();

            if (cSelfCollisionRegions.IsCreated())
                cSelfCollisionRegions.Dispose();

            if (cUtilizedSelfColRegions.IsCreated)
                cUtilizedSelfColRegions.Dispose();
        }
    }
}