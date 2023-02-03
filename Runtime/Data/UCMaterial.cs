using System;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Describes the properties of a material from which the cloth is made.
    /// </summary>
    // TODO: Turn this into a ScriptableObject when closer to finish
    [Serializable]
    public struct UCMaterial
    {
        [Header("Springs")]
        [Tooltip("How strong the springs holding the cloth together are. High values can resist stretching better, but can explode with too few iterations.")]
        public float stiffnessCoefficient;

        [Tooltip("How strongly the springs dampen velocity. High values lead to less oscillation, but can make the simulation seem \"sluggish\".")]
        public float dampingCoefficient;

        [Header("Edge length constraint")]
        [Tooltip("Maximum allowed stretch, keep around 1 to keep surface area similiar.")]
        public float maxStretch;

        [Tooltip("Multiplier to apply to velocity during constraining. 1 is the most physically accurate and results in most energetic simulation, but can be a bit unstable.")]
        public float energyConservation;

        [Header("Bending")]
        [Tooltip("Bending constant in newtons per angle (radians). High values are used for materials like leather, which resist bending.")]
        public float bendingCoefficient;

        [Space]
        public float vertexMass;
    }
}