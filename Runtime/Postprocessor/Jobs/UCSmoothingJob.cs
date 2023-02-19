using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UCloth
{
    [BurstCompile]
    internal struct UCSmoothingJob : IJob
    {
        [WriteOnly]
        public NativeArray<float3> smoothedPositionsLocalSpace;

        [ReadOnly]
        public NativeArray<float3> currentPositions;
        public NativeArray<float3> historicalPositions;
        public float4x4 worldToLocal;
        public float4x4 localToWorld;
        public float smoothingAlpha;

        public void Execute()
        {
            // Smoothing happens in world space
            int vertCount = currentPositions.Length;
            for (int i = 0; i < vertCount; i++)
            {
                // Exponential moving average
                float3 newPosWs = math.transform(localToWorld, currentPositions[i]);
                float3 smoothed = smoothingAlpha * newPosWs + (1 - smoothingAlpha) * historicalPositions[i];

                historicalPositions[i] = smoothed;

                // Convert back to local space
                smoothedPositionsLocalSpace[i] = math.transform(worldToLocal, smoothed);
            }
        }
    }
}