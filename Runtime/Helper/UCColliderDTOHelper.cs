using System.Collections.Generic;
using UnityEngine;

namespace UCloth
{
    internal static class UCColliderDTOHelper
    {
        /// <summary>
        /// Discards colliders that don't need to be computed
        /// </summary>
        /// <typeparam name="T"> Type of collider. </typeparam>
        /// <param name="potentialColliders"> Colliders to check for validity. </param>
        /// <param name="clothBounds"> Bounding box of the cloth simulation. </param>
        /// <param name="boundOffset"> Expansion to apply to bounds before checking intersection. </param>
        /// <returns> List of valid colliders which need to be computed. </returns>
        internal static List<T> FilterColliders<T>(T[] potentialColliders, Bounds clothBounds, Vector3 boundOffset) where T : Collider
        {
            List<T> validColliders = new(potentialColliders.Length);
            for (int i = 0; i < potentialColliders.Length; i++)
            {
                var sphere = potentialColliders[i];

                // If disabled
                if (!sphere.gameObject.activeInHierarchy)
                    continue;

                // If not within bounds
                // Some offset is added to account for motion that can happen during simulation
                var bounds = sphere.bounds;
                bounds.Expand(boundOffset);

                if (!clothBounds.Intersects(bounds))
                    continue;

                // If here, collider was valid
                validColliders.Add(sphere);
            }

            return validColliders;
        }

        /// <summary>
        /// Gets the friction of the assigned material, or the default friction.
        /// </summary>
        /// <param name="collider"></param>
        /// <returns></returns>
        internal static float GetFriction(Collider collider)
        {
            return collider.sharedMaterial != null ? collider.sharedMaterial.dynamicFriction : 0.1f;
        }
    }
}