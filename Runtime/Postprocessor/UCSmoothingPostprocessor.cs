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
        private bool _firstIter = true;

        public UCSmoothingPostprocessor(UCCloth scheduler)
        {
            _scheduler = scheduler;
        }

        public bool ScheduledToCleanup { get; set; }
        public bool Deactivated { get => _deactInternal; }
        private bool _deactInternal = false;


        public UCRenderingMeshData Process(UCRenderingMeshData data)
        {
            // Some data is still uninitialized on first frame, one way to get around it is to return early once
            if (_firstIter)
            {
                _firstIter = false;
                return data;
            }

            // Create the buffer
            if (!positionBuffer.IsCreated || positionBuffer.Length != data.vertices.Length)
            {
                positionBuffer = new NativeArray<float3>(data.vertices.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                resultsBuffer = new NativeArray<float3>(data.vertices.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

                // Vertices are in local space by default - convert into world space
                float4x4 localToWorld = _scheduler.transform.localToWorldMatrix;
                for(int i = 0; i < data.vertices.Length; i++)
                {
                    float3 worldSpaceVertex = math.transform(localToWorld, data.vertices[i]);

                    positionBuffer[i] = worldSpaceVertex;
                    resultsBuffer[i] = worldSpaceVertex;
                }

                // No smoothing on first iteration
                return data;
            }


            data = RunProcessorLogic(data);

            return data;
        }

        private UCRenderingMeshData RunProcessorLogic(UCRenderingMeshData data)
        {
            float4x4 worldToLocal = _scheduler.transform.worldToLocalMatrix;
            float4x4 localToWorld = _scheduler.transform.localToWorldMatrix;
            float alpha = 1 - _scheduler.smoothing;

            UCSmoothingJob smoothingJob = new()
            {
                // Using scheduler data instead of rendering data as we need it in world space
                currentPositions = data.vertices,
                historicalPositions = positionBuffer,
                smoothedPositionsLocalSpace = resultsBuffer,
                smoothingAlpha = alpha,
                worldToLocal = worldToLocal,
                localToWorld = localToWorld
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