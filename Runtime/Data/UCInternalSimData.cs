using System;
using Unity.Collections;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Stores simulation data used by <see cref="UCloth"/>.
    /// </summary>
    public class UCInternalSimData : IDisposable
    {
        public NativeArray<float3> cPositions;
        public NativeArray<float3> cVelocity;
        public NativeArray<float3> cAcceleration;
        internal NativeArray<float3> cTempAcceleration;

        internal Native3DHashmapArray<ushort> cSelfCollisionRegions;
        internal NativeParallelHashSet<int3> cUtilizedSelfColRegions;


        public NativeArray<UCEdge> cEdges;
        public NativeArray<UCBendingEdge> cBendingEdges;
        public NativeParallelMultiHashMap<ushort, ushort> cNeighbours;
        public NativeArray<float3> cNormals;
        public NativeArray<float3> cTriangleNormals;
        public NativeArray<float> cRestDistance;

        /// <summary>
        /// The reciprocal of the weight. This is preferred to storing weight directly as it allows 0 to be used for pinned nodes,
        /// and replaces division with multiplication (which tends to be faster)
        /// </summary>
        public NativeArray<float> cReciprocalWeight;
        public NativeParallelHashMap<ushort, float3> cPinnedLocalPos;

        public void Dispose()
        {
            cPositions.Dispose();
            cVelocity.Dispose();
            cAcceleration.Dispose();
            cTempAcceleration.Dispose();

            cEdges.Dispose();
            cBendingEdges.Dispose();
            cNeighbours.Dispose();
            cNormals.Dispose();
            cTriangleNormals.Dispose();
            cRestDistance.Dispose();
            cReciprocalWeight.Dispose();
            cPinnedLocalPos.Dispose();

            if (cSelfCollisionRegions.IsCreated())
                cSelfCollisionRegions.Dispose();

            if (cUtilizedSelfColRegions.IsCreated)
                cUtilizedSelfColRegions.Dispose();
        }
    }
}