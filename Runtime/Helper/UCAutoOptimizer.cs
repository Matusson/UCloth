
using System;
using UnityEngine;

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
            var mode = _scheduler.collisionProperties.AutoAdjustGridDensity;

            // For best performance, there should be a small amount of nodes per cell
            // However, going too low will cause missing collisions.
            // It should be safe going with about 1-2 nodes per cell, and 5-8 surrounding nodes.

            var surroundRange = GetSurroundingRange(mode);
            if (data.averageSurroundingNodes < surroundRange.Item1)
            {
                current *= 0.5f;
            }

            if (data.averageSurroundingNodes < surroundRange.Item2)
            {
                current *= 0.85f;
            }

            var localRange = GetLocalRange(mode);
            if (data.averageNodesPerActiveCell > localRange.Item1)
            {
                current *= 2f;
            }

            if (data.averageNodesPerActiveCell > localRange.Item2)
            {
                current *= 1.25f;
            }

            // Going too low
            if (data.averageNodesPerActiveCell < 1.5)
            {
                current *= 0.5f;
            }

            // In some cases, the algorithm will approach zero, which will lead to a single region.
            if (current < 0.1f)
                current = 0.1f;

            // Too many regions can cause issues as well
            if (current > 100f)
                current = 100f;

            return current;
        }

        private static (float, float) GetSurroundingRange(UCAutoAdjustGridOption mode)
        {
            if (mode == UCAutoAdjustGridOption.Performance)
            {
                return (4,6);
            }

            if (mode == UCAutoAdjustGridOption.Accuracy)
            {
                return (5, 9);
            }

            return (0, 0);
        }

        private static (float, float) GetLocalRange(UCAutoAdjustGridOption mode)
        {
            if (mode == UCAutoAdjustGridOption.Performance)
            {
                return (10, 2.5f);
            }

            if (mode == UCAutoAdjustGridOption.Accuracy)
            {
                return (10, 5);
            }

            return (0, 0);
        }
    }
}