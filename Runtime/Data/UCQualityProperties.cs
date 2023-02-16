using System;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Stores quality properties of a simulation.
    /// </summary>
    [Serializable]
    public struct UCQualityProperties
    {
        public int simFrequency;
        public float timeScaleMultiplier;
        public float maxTimestep;

        [Space]
        public int iterations;
        public int constraintIterations;

        [Space]
        [Tooltip("When enabled, the simulator will always use the most up-to-data data, at the cost of main thread performance.")]
        public bool minimizeLatency;
    }
}