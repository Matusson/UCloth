using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Tracks collider velocity.
    /// </summary>
    internal class UCColliderTracker
    {
        private readonly Dictionary<int, float3> _lastPositionsCache;  // Uses instance IDs as key


        public UCColliderTracker()
        {
            _lastPositionsCache = new();
        }

        /// <summary>
        /// Fetches latest velocity of the collider and updates the position for next call.
        /// </summary>
        /// <param name="collider"></param>
        /// <returns></returns>
        internal float3 GetAndUpdateVelocity(Collider collider)
        {
            int id = collider.GetInstanceID();
            float3 currentPosition = (float3)collider.transform.position;

            // First run, nothing to track
            if (!_lastPositionsCache.ContainsKey(id))
            {
                _lastPositionsCache.Add(id, currentPosition);
                return new float3();
            }

            // Get the velocity and update for next call
            float3 velocity = currentPosition - _lastPositionsCache[id];
            _lastPositionsCache[id] = currentPosition;

            return velocity;
        }
    }
}
