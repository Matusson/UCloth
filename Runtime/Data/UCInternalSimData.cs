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
        public NativeArray<float> cRestDistance;

        // First value is the global ID of the node, second is "which of the pinned nodes is it"
        public NativeParallelHashMap<ushort, ushort> cPinned;
        public NativeList<float3> cPinnedLocalPos;

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
            cRestDistance.Dispose();
            cPinned.Dispose();
            cPinnedLocalPos.Dispose();

            if (cSelfCollisionRegions.IsCreated())
                cSelfCollisionRegions.Dispose();

            if (cUtilizedSelfColRegions.IsCreated)
                cUtilizedSelfColRegions.Dispose();
        }
    }
}