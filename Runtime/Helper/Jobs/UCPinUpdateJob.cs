using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace UCloth
{
    [BurstCompile]
    internal struct UCPinUpdateJob : IJob
    {
        [ReadOnly]
        internal NativeList<ushort> nodeIds;

        [ReadOnly]
        internal NativeList<float3> relativePositions;


        [WriteOnly]
        [NativeDisableContainerSafetyRestriction]
        // It's safe to disable safety restrictions here, as each node can only be pinned to one object.
        // Therefore, these jobs won't write to the same index at the same time.
        internal NativeParallelHashMap<ushort, float3> pinnedPositions;

        internal float4x4 localToWorld;

        public void Execute()
        {
            // Set the pinned positions of nodes
            int length = nodeIds.Length;
            for (int i = 0; i < length; i++)
            {
                ushort nodeId = nodeIds[i];
                float3 relative = relativePositions[i];

                // Target space is world space
                float3 worldSpace = math.transform(localToWorld, relative);
                pinnedPositions[nodeId] = worldSpace;
            }
        }
    }
}
