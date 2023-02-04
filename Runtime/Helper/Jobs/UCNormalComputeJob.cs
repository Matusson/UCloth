using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Computes vertex normals for given vertices.
    /// </summary>
    [BurstCompile]
    internal struct UCNormalComputeJob : IJob
    {
        [ReadOnly]
        internal NativeArray<int> triangles;
        [ReadOnly]
        internal NativeArray<float3> vertices;

        internal NativeArray<float3> normals;

        [WriteOnly]
        internal NativeArray<float3> triangleNormals;

        [ReadOnly]
        internal NativeArray<int> renderToSimLookup;


        public void Execute()
        {
            // Vertex normal calculation is done by computing a triangle normal, then adding it to every vertex that makes it up
            int trisCount = triangles.Length / 3;
            for (int i = 0; i < trisCount; i++)
            {
                int triangleIndex = i * 3;
                int vert1 = renderToSimLookup[triangles[triangleIndex]];
                int vert2 = renderToSimLookup[triangles[triangleIndex + 1]];
                int vert3 = renderToSimLookup[triangles[triangleIndex + 2]];

                // TODO: Since we're computing face normals anyway, see if using those for bending stiffness would create better results
                float3 triangleNormal = CalculateTriangleNormal(vert1, vert2, vert3);
                triangleNormals[i] = triangleNormal;

                normals[vert1] += triangleNormal;
                normals[vert2] += triangleNormal;
                normals[vert3] += triangleNormal;
            }

            // After that, normalize the vertex normals
            int vertexCount = vertices.Length;
            for (int i = 0; i < vertexCount; i++)
            {
                normals[i] = math.normalize(normals[i]);
            }
        }

        private float3 CalculateTriangleNormal(int vert1, int vert2, int vert3)
        {
            float3 pos1 = vertices[vert1];
            float3 pos2 = vertices[vert2];
            float3 pos3 = vertices[vert3];

            float3 ab = pos2 - pos1;
            float3 ac = pos3 - pos1;

            return math.normalize(math.cross(ab, ac));
        }
    }
}