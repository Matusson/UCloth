namespace UCloth
{
    /// <summary>
    /// Initializes default values for <see cref="UCloth"/>.
    /// </summary>
    internal static class UCInitializer
    {
        // This class only exists as in this version of C#, it's not possible to initialize structs with default values.

        internal static void Initialize(UCloth cloth)
        {
            cloth.materialProperties = new UCMaterial()
            {
                stiffnessCoefficient = 1500,
                dampingCoefficient = 10,
                maxStretch = 1.01f,
                energyConservation = 1f,
                bendingCoefficient = 1f,
                vertexMass = 1f
            };

            cloth.simulationProperties = new UCSimulationProperties()
            {
                gravityMultiplier = 1f,
                airResistanceMultiplier = 0.5f
            };

            cloth.qualityProperties = new UCQualityProperties()
            {
                simFrequency = 60,
                timeScaleMultiplier = 1f,
                maxTimestep = 0.03f,
                iterations = 2,
                constraintIterations = 5
            };

            cloth.collisionProperties = new UCCollisionSettings()
            {
                collisionFriction = 1f,
                collisionContactOffset = 0.001f,
                enableSelfCollision = false,
                selfCollisionDistance = 0.3f,
                selfCollisionStiffness = 1f,
                selfCollisionFriction = 0.9f,
                selfCollisionVelocityConservation = 0.75f,
                selfCollisionAccuracy = 1,
                AutoAdjustGridDensity = UCAutoAdjustGridOption.Performance,
                gridDensity = 1f
            };
        }
    }
}