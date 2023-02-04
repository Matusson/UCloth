using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Represents a points query for <see cref="UCCloth"/>.
    /// </summary>
    public struct UCPointQueryData
    {
        /// <summary>
        /// World-space position to query.
        /// </summary>
        public float3 position;

        /// <summary>
        /// Radius for point search.
        /// </summary>
        public float radius;
    }
}
