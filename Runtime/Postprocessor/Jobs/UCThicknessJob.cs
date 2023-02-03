using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Offsets backside vertices and copies their normals.
    /// </summary>
    [BurstCompile]
    internal struct UCThicknessJob : IJob
    {
        public NativeArray<float3> vertices;
        public NativeArray<float3> normals;

        public bool offsetFront;
        public float thickness;

        public void Execute()
        {
            int length = vertices.Length;
            int startOffset = length / 2; // Should always be divisible because while preparing it was multiplied by 2
            float halfThickness = thickness / 2;

            // The front vertices can be offset as well, which will make the simulation run "inside" the mesh, rather than front
            if (offsetFront)
            {
                for (int i = 0; i < startOffset; i++)
                {
                    float3 pos = vertices[i];
                    float3 normal = normals[i];

                    float3 newPos = pos + normal * halfThickness;
                    vertices[i] = newPos;
                }
            }

            // Backside
            for (int i = startOffset; i < length; i++)
            {
                // Offsetting backside
                float3 pos = vertices[i - startOffset];
                float3 normal = normals[i - startOffset];

                float3 newPos = pos - normal * thickness;
                vertices[i] = newPos;

                // Copying normal
                normals[i] = -normal;
            }
        }
    }
}