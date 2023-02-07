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
        /// Radius for point search. Zero will pick one point.
        /// </summary>
        public float radius;


        public UCPointQueryData(float3 position, float radius = 0)
        {
            this.position = position;
            this.radius = radius;
        }
    }
}
