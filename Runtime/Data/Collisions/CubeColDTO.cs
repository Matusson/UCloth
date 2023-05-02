using Unity.Mathematics;

namespace UCloth
{

    /// <summary>
    /// Stores information about a cube collider.
    /// </summary>
    public struct CubeColDTO
    {
        public float3 position;
        public float3 offset;
        public float3 size;

        public float4x4 localMatrix;
        public float4x4 worldMatrix;   // Theoretically not necessary, since world is inverse of global, but matrix inversion is expensive

        public float3 velocity;
        public float friction;
    }
}