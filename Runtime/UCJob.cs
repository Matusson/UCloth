using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    [BurstCompile(FloatPrecision = FloatPrecision.Low, OptimizeFor = OptimizeFor.Performance, FloatMode = FloatMode.Fast)]
    public struct UCJob : IJob
    {
        // NOTE: This Job uses ushort node indexes. This limits the node amounts to 65,535.

        public NativeArray<float3> positions;
        public NativeArray<float3> velocity;
        public NativeArray<float3> acceleration;
        public NativeArray<float3> tempAcceleration;

        internal Native3DHashmapArray<ushort> selfCollisionRegions;
        internal NativeParallelHashSet<int3> utilizedSelfColRegions;

        [ReadOnly]
        public NativeArray<UCEdge> edges;

        [ReadOnly]
        public NativeArray<UCBendingEdge> bendingEdges;

        [ReadOnly]
        public NativeParallelMultiHashMap<ushort, ushort> neighbours;

        [ReadOnly]
        public NativeArray<float3> normals;

        [ReadOnly]
        public NativeArray<float> restDistances;

        [ReadOnly]
        public NativeParallelHashMap<ushort, ushort> pinned;

        // Local-space positions of pinned nodes for correction
        [ReadOnly]
        public NativeList<float3> pinnedLocalPos;


        [ReadOnly]
        public NativeArray<SphereColDTO> sphereColliders;

        [ReadOnly]
        public NativeArray<CapsuleColDTO> capsuleColliders;

        [ReadOnly]
        public NativeArray<CubeColDTO> cubeColliders;

        public Bounds bounds;
        public float4x4 localToWorldMatrix;

        public UCMaterial material;
        public UCCollisionSettings collisionSettings;

        internal NativeReference<UCAutoOptimizeData> optimizationData;

        public UCSimulationProperties simProperties;
        public UCQualityProperties qualityProperties;
        public float baseTimestep;
        public float extraThickness;



        private ushort _nodeCount;
        private int _edgeCount;
        private int _bendingEdgeCount;

        private static readonly float3[] _axes = new float3[6]
        {
        new float3(1, 0, 0),
        new float3(0, 1, 0),
        new float3(0, 0, 1),
        new float3(-1, 0, 0),
        new float3(0, -1, 0),
        new float3(0, 0, -1),
        };

        /// <summary>
        /// Executes the Job.
        /// </summary>
        public void Execute()
        {
            _nodeCount = (ushort)positions.Length;
            _edgeCount = edges.Length;
            _bendingEdgeCount = bendingEdges.Length;

            float timestep = baseTimestep / qualityProperties.iterations;

            for (int i = 0; i < qualityProperties.iterations; i++)
            {
                Integrate(timestep);
            }

            for (int i = 0; i < qualityProperties.constraintIterations; i++)
            {
                ConstrainEdges();
            }

            ApplyCollisions();
            ApplySelfCollisions();
        }

        /// <summary>
        /// Performs Verlet integration.
        /// </summary>
        /// <param name="dt"></param>
        public void Integrate(float dt)
        {
            // Adapted from https://en.wikipedia.org/wiki/Verlet_integration#Algorithmic_representation
            for (int i = 0; i < _nodeCount; i++)
            {
                float3 acc = acceleration[i] / material.vertexMass;
                float3 newPos = positions[i] + (velocity[i] * dt) + (acc * (dt * dt * 0.5f));
                positions[i] = newPos;


                // Save so it persists between forces update
                tempAcceleration[i] = acc;

            }
            UpdateForces();

            for (int i = 0; i < _nodeCount; i++)
            {
                float3 acc = acceleration[i] / material.vertexMass;
                float3 newVel = velocity[i] + (tempAcceleration[i] + acc) * (dt * 0.5f);


                // And update the values
                velocity[i] = newVel - (newVel * simProperties.airResistanceMultiplier * dt);
            }
        }

        /// <summary>
        /// Calculates forces for every node. 
        /// </summary>
        public void UpdateForces()
        {
            ApplyGravity();
            ApplySprings();
            ApplyBending();

            ResetPinned();
        }


        //--- Components

        /// <summary>
        /// Runs edge length constraints.
        /// </summary>
        private void ConstrainEdges()
        {
            for (int i = 0; i < _edgeCount; i++)
            {
                var edge = edges[i];
                float3 pos1 = positions[edge.nodeIndex1];
                float3 pos2 = positions[edge.nodeIndex2];
                float3 vec1To2 = pos2 - pos1;

                // Based on Hooke's spring
                float distance = math.length(vec1To2);
                float restDistance = restDistances[i];
                float stretchAmount = distance / restDistance;

                // Only apply the correction if above the threshold
                if (stretchAmount <= material.maxStretch)
                    continue;

                float correction = (distance - restDistance * material.maxStretch) / distance;

                bool pin1 = pinned.ContainsKey(edge.nodeIndex1);
                bool pin2 = pinned.ContainsKey(edge.nodeIndex2);

                if (pin1 || pin2)
                    correction *= 2f;

                if (!pin1)
                {
                    positions[edge.nodeIndex1] = pos1 + vec1To2 * correction;
                    velocity[edge.nodeIndex1] = velocity[edge.nodeIndex1] + correction * material.energyConservation * vec1To2;
                }

                if (!pin2)
                {
                    positions[edge.nodeIndex2] = pos2 - vec1To2 * correction;
                    velocity[edge.nodeIndex2] = velocity[edge.nodeIndex2] - correction * material.energyConservation * vec1To2;
                }

            }
        }

        private void ApplyCollisions()
        {
            float correctedContactOffset = collisionSettings.collisionContactOffset + extraThickness;
            float3 contactOffset3d = new(correctedContactOffset, correctedContactOffset, correctedContactOffset);

            for (int i = 0; i < _edgeCount; i++)
            {
                UCEdge edge = edges[i];

                // SPHERE COLLIDERS
                for (int j = 0; j < sphereColliders.Length; j++)
                {
                    float3 position = (positions[edge.nodeIndex1] + positions[edge.nodeIndex2]) / 2;
                    SphereColDTO collider = sphereColliders[j];

                    float dist = math.distance(position, collider.position);
                    float distDelta = (collider.radius + correctedContactOffset) - dist;

                    if (distDelta > 0f)
                    {
                        float3 awayFromCentre = math.normalize(position - collider.position);
                        float3 force = awayFromCentre * distDelta;
                        positions[edge.nodeIndex1] = positions[edge.nodeIndex1] + force;
                        positions[edge.nodeIndex2] = positions[edge.nodeIndex2] + force;

                        // Inverting velocity helps with nodes passing through with low density, but adds a lot of jitter
                        float frictionMultiplier = 1f - (collider.friction * collisionSettings.collisionFriction);
                        frictionMultiplier = math.clamp(frictionMultiplier, 0, 1);
                        velocity[edge.nodeIndex1] = velocity[edge.nodeIndex1] * frictionMultiplier;
                        velocity[edge.nodeIndex2] = velocity[edge.nodeIndex2] * frictionMultiplier;
                    }
                }

                // CAPSULE COLLIDERS
                for (int j = 0; j < capsuleColliders.Length; j++)
                {
                    float3 position = (positions[edge.nodeIndex1] + positions[edge.nodeIndex2]) / 2;
                    CapsuleColDTO collider = capsuleColliders[j];

                    // Calculating the SDF of the capsule
                    float3 pa = position - collider.a;
                    float projection = math.clamp(math.dot(pa, collider.ba) / math.dot(collider.ba, collider.ba), 0f, 1f);

                    float sdf = math.length(pa - collider.ba * projection) - (collider.radius + correctedContactOffset);

                    if (sdf < 0f)
                    {
                        // Calculating the normal of closest point
                        float3 closest_point = collider.a + projection * collider.ba;
                        float3 normal = math.normalize(position - closest_point);

                        float3 force = -normal * sdf;
                        positions[edge.nodeIndex1] = positions[edge.nodeIndex1] + force;
                        positions[edge.nodeIndex2] = positions[edge.nodeIndex2] + force;

                        // Inverting velocity helps with nodes passing through with low density, but adds a lot of jitter
                        float frictionMultiplier = 1f - (collider.friction * collisionSettings.collisionFriction);
                        frictionMultiplier = math.clamp(frictionMultiplier, 0, 1);
                        velocity[edge.nodeIndex1] = velocity[edge.nodeIndex1] * frictionMultiplier;
                        velocity[edge.nodeIndex2] = velocity[edge.nodeIndex2] * frictionMultiplier;
                    }
                }

                // CUBE COLLIDERS
                for (int j = 0; j < cubeColliders.Length; j++)
                {
                    float3 position = (positions[edge.nodeIndex1] + positions[edge.nodeIndex2]) / 2;
                    CubeColDTO collider = cubeColliders[j];

                    float3 localPos = math.mul(collider.localMatrix, new float4(position, 1f)).xyz - collider.offset;

                    // NOTE: Box colliders can not be scaled as the object! Only scaling using the "size" vector of the collider component is allowed.
                    // Scaling the gameobject itself causes under/overestimation of the distance function 

                    // Box SDF calculation
                    // Adapted from https://www.alanzucconi.com/2016/07/01/signed-distance-functions/

                    float3 distTemp = math.abs(localPos) - ((collider.size + contactOffset3d) / 2);
                    float sdf = math.max(math.max(distTemp.x, distTemp.y), distTemp.z);

                    if (sdf < 0f)
                    {
                        // Compute the direction in which the force should be applied
                        // (the normal of the closest face)
                        int minAxisId = 0;
                        float minDot = 999;

                        // Axis with lowest dot product is the closest face normal
                        // TODO: This might be possible to vectorize nicely?
                        for (int k = 0; k < 6; k++)
                        {
                            float dot = math.dot(localPos / collider.size, _axes[k]);

                            if (dot < minDot)
                            {
                                minDot = dot;
                                minAxisId = k;
                            }
                        }
                        // Rotate the normal into world space
                        float3 normalDir = -math.rotate(collider.worldMatrix, _axes[minAxisId]);

                        float3 force = -normalDir * sdf;
                        positions[edge.nodeIndex1] = positions[edge.nodeIndex1] + force;
                        positions[edge.nodeIndex2] = positions[edge.nodeIndex2] + force;

                        // Inverting velocity helps with nodes passing through with low density, but adds a lot of jitter
                        float frictionMultiplier = 1f - (collider.friction * collisionSettings.collisionFriction);
                        frictionMultiplier = math.clamp(frictionMultiplier, 0, 1);
                        velocity[edge.nodeIndex1] = velocity[edge.nodeIndex1] * frictionMultiplier;
                        velocity[edge.nodeIndex2] = velocity[edge.nodeIndex2] * frictionMultiplier;
                    }
                }
            }
        }

        private void ApplySelfCollisions()
        {
            if (!collisionSettings.enableSelfCollision)
                return;

            utilizedSelfColRegions.Clear();
            NativeList<int3> regionsIndexes = new(64, Allocator.Temp);

            // Spatial partitioning into the 3D "array"
            // - not actually array as node count can vary
            for (ushort i = 0; i < _nodeCount; i++)
            {
                float3 pos = positions[i];
                float3 relativePos = (pos - (float3)bounds.min) / bounds.size;
                int3 index = (int3)math.trunc(relativePos * selfCollisionRegions.size);

                // Ensure in bounds
                index = math.clamp(index, new int3(0, 0, 0), selfCollisionRegions.size - new int3(1, 1, 1));

                selfCollisionRegions.Add(index, i);

                if (!utilizedSelfColRegions.Contains(index))
                {
                    utilizedSelfColRegions.Add(index);

                    // NOTE: Potential optimization: if there is a used index in the direct neighbourhood, no need to process the cell, explained below
                    regionsIndexes.Add(index);
                }
            }

            // Gather data for automatic optimization
            float averageNodesPerActiveCell = (float)_nodeCount / regionsIndexes.Length;
            float averageSurroundingNodes = 0f;

            NativeList<ushort> nodes = new(32, Allocator.Temp);
            NativeList<ushort> surrounding = new(64, Allocator.Temp);

            for (int region = 0; region < regionsIndexes.Length; region++)
            {
                nodes.Clear();
                surrounding.Clear();
                int3 index = regionsIndexes[region];

                // If no nodes located in this region
                if (!selfCollisionRegions.TryGetItemsNoAlloc(index, ref nodes))
                {
                    continue;
                }

                // Gather surrounding cells
                UCJobHelper.GetSurroundingNodesNoAlloc(ref selfCollisionRegions, index, surrounding, collisionSettings.selfCollisionAccuracy);
                averageSurroundingNodes += surrounding.Length;

                // Between local and surrounding cells
                int sameCellNodeCount = nodes.Length;
                int surroundingCellCount = surrounding.Length;
                for (int i = 0; i < sameCellNodeCount; i++)
                {
                    for (int j = 0; j < surroundingCellCount; j++)
                    {
                        int index1 = nodes[i];
                        int index2 = surrounding[j];

                        CalculateSelfCollision(index1, index2);
                    }
                }

                // Between local cells
                // This way index collisions are avoided
                for (int i = 1; i < sameCellNodeCount; i++)
                {
                    for (int j = 0; j < i; j++)
                    {
                        int index1 = nodes[i];
                        int index2 = nodes[j];

                        CalculateSelfCollision(index1, index2);
                    }
                }

            }
            averageSurroundingNodes /= regionsIndexes.Length;

            optimizationData.Value = new UCAutoOptimizeData()
            {
                averageNodesPerActiveCell = averageNodesPerActiveCell,
                averageSurroundingNodes = averageSurroundingNodes
            };
        }

        private void CalculateSelfCollision(int index1, int index2)
        {
            // Do not process neighbouring vertices, otherwise self collision distance is pretty limited
            if (neighbours.KeyContainsValue((ushort)index1, (ushort)index2) || neighbours.KeyContainsValue((ushort)index2, (ushort)index1))
                return;

            float3 posDiff = positions[index1] - positions[index2];
            float dist = math.length(posDiff);

            // Early discard if over the distance
            if (dist < collisionSettings.selfCollisionDistance)
            {
                float3 ratio = collisionSettings.selfCollisionDistance / dist;
                float forceScale = collisionSettings.selfCollisionStiffness / (2 * material.vertexMass);
                float3 posDelta = forceScale * material.vertexMass * (posDiff - (posDiff * ratio));

                positions[index1] = positions[index1] - posDelta;
                positions[index2] = positions[index2] + posDelta;

                // Remove velocity along the normal of the node
                // This is not a perfect solution as it can cause jittering and some clipping
                // However doing this along the collision vector is worse (more clipping), and not doing this at all causes velocity bleed
                // To maintain stability, this is blended with posDelta. 

                float3 normal1 = normals[index1];
                float velocityAlongNormal1 = math.dot(velocity[index1], normal1);
                float3 newVel1 = velocity[index1] - normal1 * velocityAlongNormal1;
                float3 blendedVel1 = (1f - collisionSettings.selfCollisionVelocityConservation) * posDelta + (collisionSettings.selfCollisionVelocityConservation * newVel1);

                float3 normal2 = normals[index2];
                float velocityAlongNormal2 = math.dot(velocity[index2], normal2);
                float3 newVel2 = velocity[index2] - normal2 * velocityAlongNormal2;
                float3 blendedVel2 = (1f - collisionSettings.selfCollisionVelocityConservation) * -posDelta + (collisionSettings.selfCollisionVelocityConservation * newVel2);

                // Friction - how much velocity is applied back
                velocity[index1] = collisionSettings.selfCollisionFriction * blendedVel1 + (1f - collisionSettings.selfCollisionFriction) * velocity[index1];
                velocity[index2] = collisionSettings.selfCollisionFriction * blendedVel2 + (1f - collisionSettings.selfCollisionFriction) * velocity[index2];
            }
        }


        //--- Forces
        private void ApplySprings()
        {
            for (int i = 0; i < _edgeCount; i++)
            {
                var edge = edges[i];
                float3 pos1 = positions[edge.nodeIndex1];
                float3 pos2 = positions[edge.nodeIndex2];
                float3 dir1To2 = math.normalize(pos2 - pos1);

                // Based on Hooke's spring
                float distance = math.distance(pos1, pos2);
                float restDistance = restDistances[i];

                float delta = distance - restDistance;

                // Calculate damping to avoid spring oscillation
                float3 velocityDelta = velocity[edge.nodeIndex2] - velocity[edge.nodeIndex1];
                float velocityAlongDistance = math.dot(velocityDelta, dir1To2);


                float3 force = (material.stiffnessCoefficient * delta + material.dampingCoefficient * velocityAlongDistance) * dir1To2;

                acceleration[edge.nodeIndex1] = acceleration[edge.nodeIndex1] + force * 0.5f;
                acceleration[edge.nodeIndex2] = acceleration[edge.nodeIndex2] - force * 0.5f;
            }
        }

        private void ApplyBending()
        {
            // No need to perform expensive calculations if the force is disabled
            if (material.bendingCoefficient < 0.01f)
                return;

            for (int i = 0; i < _bendingEdgeCount; i++)
            {
                UCBendingEdge edge = bendingEdges[i];

                float3 normal1 = normals[edge.bendingNode1];
                float3 normal2 = normals[edge.bendingNode2];

                float dot = math.dot(normal1, normal2);
                dot = math.clamp(dot, -1f, 1f); // Due to matrix multiplication, normal might not be fully normalized

                float strength = material.bendingCoefficient * math.acos(dot) / math.PI;

                // The force needs to be inverted if the edge if folded the other way
                float3 vec2To1 = positions[edge.bendingNode2] - positions[edge.bendingNode1];
                bool inverted = math.dot(normal2, vec2To1) > 0;

                if (inverted)
                    strength = -strength;

                // And then apply the forces to nodes
                acceleration[edge.bendingNode1] = acceleration[edge.bendingNode1] - normal1 * strength;
                acceleration[edge.bendingNode2] = acceleration[edge.bendingNode2] - normal2 * strength;
            }
        }

        private void ApplyGravity()
        {
            float3 gravity = simProperties.gravityMultiplier * material.vertexMass * Physics.clothGravity;

            for (int i = 0; i < _nodeCount; i++)
            {
                acceleration[i] = gravity;
            }
        }


        /// <summary>
        /// Resets pinned nodes so they don't move.
        /// </summary>
        private void ResetPinned()
        {
            for (ushort i = 0; i < _nodeCount; i++)
            {
                if (pinned.ContainsKey(i))
                {
                    acceleration[i] = new();
                    velocity[i] = new();
                    positions[i] = math.transform(localToWorldMatrix, pinnedLocalPos[pinned[i]]);
                }
            }
        }
    }
}