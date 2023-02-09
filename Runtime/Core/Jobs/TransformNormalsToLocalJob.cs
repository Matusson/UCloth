using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Transforms normals from world-space simulation to local-space for rendering.
    /// </summary>
    internal struct TransformNormalsToLocalJob : IJobParallelFor
    {
        [WriteOnly]
        public NativeArray<float3> normalsLocalSpace;

        [ReadOnly]
        public NativeArray<float3> normalsWorldSpace;

        [ReadOnly]
        public NativeArray<int> renderToSimIndexLookup;

        public float4x4 worldToLocal;


        public void Execute(int index)
        {
            int targetIndex = renderToSimIndexLookup[index];
            normalsLocalSpace[index] = math.rotate(worldToLocal, normalsWorldSpace[targetIndex]);
        }
    }
}
