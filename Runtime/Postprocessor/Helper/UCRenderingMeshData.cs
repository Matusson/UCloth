using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Stores data for rendering.
    /// </summary>
    public class UCRenderingMeshData
    {
        public NativeArray<float3> vertices;
        public NativeArray<float3> normals;
        public int[] triangles;
        public Vector2[] uvs;
    }
}