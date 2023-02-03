using System;


namespace UCloth
{
    /// <summary>
    /// Stores general properties of a simulation.
    /// </summary>
    [Serializable]
    public struct UCSimulationProperties
    {
        public float gravityMultiplier;
        public float airResistanceMultiplier;
    }
}