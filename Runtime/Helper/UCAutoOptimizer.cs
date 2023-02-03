
namespace UCloth
{
    /// <summary>
    /// Provides functionality to automatically optimize parameters for <see cref="UCloth"/>
    /// </summary>
    internal class UCAutoOptimizer
    {
        private readonly UCloth _scheduler;

        internal UCAutoOptimizer(UCloth scheduler)
        {
            _scheduler = scheduler;
        }

        public float OptimizeDensity(UCAutoOptimizeData data)
        {
            float current = _scheduler.collisionProperties.gridDensity;

            // For best performance, there should be a small amount of nodes per cell
            // However, going too low will cause missing collisions.
            // It should be safe going with about 1-2 nodes per cell, and 5-8 surrounding nodes.

            if (data.averageSurroundingNodes < 4)
            {
                return current * 0.5f;
            }

            if (data.averageSurroundingNodes < 5)
            {
                return current * 0.85f;
            }

            if (data.averageNodesPerActiveCell > 10)
            {
                // Huge correction, return early
                return current * 2f;
            }

            if (data.averageNodesPerActiveCell > 3)
            {
                return current * 1.25f;
            }

            // In some cases, the algorithm will approach zero, which will lead to a single region.
            if (current < 0.1f)
                current = 0.1f;

            // Too many regions can cause issues as well
            if (current > 100f)
                current = 100f;

            return current;
        }
    }
}