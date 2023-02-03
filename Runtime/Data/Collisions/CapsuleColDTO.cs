using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Stores information about a capsule collider.
    /// </summary>
    public struct CapsuleColDTO
    {
        public float3 a;
        public float3 ba;   // Storing b coordinates is the same as storing the `ba` vector, but b is only used to calculate `ba`, so this saves memory 

        public float radius;
        public float friction;
    }
}
