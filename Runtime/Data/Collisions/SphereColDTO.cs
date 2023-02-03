using Unity.Mathematics;

namespace UCloth
{

    /// <summary>
    /// Stores information about a sphere collider.
    /// </summary>
    public struct SphereColDTO
    {
        public float3 position;
        public float radius;

        public float friction;
    }
}