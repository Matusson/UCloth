using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Temporally smoothes out node positions.
    /// </summary>
    internal class UCSmoothingPostprocessor : IUCPostprocessor
    {
        private NativeArray<float3> resultsBuffer;
        private NativeArray<float3> positionBuffer;
        private readonly UCCloth _scheduler;

        public UCSmoothingPostprocessor(UCCloth scheduler)
        {
            _scheduler = scheduler;
        }

        public bool ScheduledToCleanup { get; set; }
        public bool Deactivated { get => _deactInternal; }
        private bool _deactInternal = false;


        public UCRenderingMeshData Process(UCRenderingMeshData data)
        {
            // Create the buffer
            if (!positionBuffer.IsCreated || positionBuffer.Length != data.vertices.Length)
            {
                positionBuffer = new NativeArray<float3>(data.vertices, Allocator.Persistent);
                resultsBuffer = new NativeArray<float3>(data.vertices, Allocator.Persistent);

                // No smoothing on first iteration
                return data;
            }


            data = RunProcessorLogic(data);

            return data;
        }

        private UCRenderingMeshData RunProcessorLogic(UCRenderingMeshData data)
        {
            float4x4 worldToLocal = _scheduler.transform.worldToLocalMatrix;
            float alpha = 1 - _scheduler.smoothing;

            UCSmoothingJob smoothingJob = new()
            {
                // Using scheduler data instead of rendering data as we need it in world space
                currentPositions = _scheduler.simData.cPositions,
                historicalPositions = positionBuffer,
                smoothedPositionsLocalSpace = resultsBuffer,
                smoothingAlpha = alpha,
                worldToLocal = worldToLocal
            };
            smoothingJob.Run();

            data.vertices.CopyFrom(resultsBuffer);
            return data;
        }

        public UCRenderingMeshData Cleanup(UCRenderingMeshData data)
        {
            // This postprocessor doesn't create any extra data, nothing to clean up
            _deactInternal = true;
            return data;
        }

        public void Dispose()
        {
            positionBuffer.Dispose();
            resultsBuffer.Dispose();
        }
    }
}