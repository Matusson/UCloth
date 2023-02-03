using System;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Stores collision settings.
    /// </summary>
    [Serializable]
    public struct UCCollisionSettings
    {
        [Header("Objects")]
        [Tooltip("Multiplier for friction for colliders.")]
        [Range(0f, 2f)]
        public float collisionFriction;

        [Tooltip("Extra buffer between cloth and colliders.")]
        public float collisionContactOffset;


        [Header("Self-Collision")]
        [Tooltip("Skips self-collision solve when disabled.")]
        public bool enableSelfCollision;


        [Tooltip("Distance at which self collision will start to be applied.")]
        public float selfCollisionDistance;

        [Tooltip("Strength at which colliding vertices are pushed apart.")]
        public float selfCollisionStiffness;

        [Range(0f, 1f)]
        [Tooltip("How much velocity is affected by collision.")]
        public float selfCollisionFriction;

        [Range(0f, 1f)]
        [Tooltip("Inject back some velocity to lighten velocity bleed. Can cause instability and clipping if high, or velocity bleed if low.")]
        public float selfCollisionVelocityConservation;


        [Header("Performance")]
        [Tooltip("0 - Skips checking surrounding cells. 1 - Only directly adjacent cells. 2 - Directly adjacent cells and corners.")]
        [Range(0, 2)]
        public int selfCollisionAccuracy;

        [Tooltip("Automatically adjusts grid density for optimal performance. Disable if you want full control.")]
        public bool AutoAdjustGridDensity;

        [Tooltip("Multiplier for grid density for spatial partitioning.")]
        public float gridDensity;
    }
}