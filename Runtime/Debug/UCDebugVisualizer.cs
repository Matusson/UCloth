#if UNITY_EDITOR    // Can't place into another assembly as we refer to some internal components
using System;
using System.Collections;
using Unity.Collections;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Allows viewing debug information on <see cref="UCCloth"/> objects.
    /// </summary>
    public class UCDebugVisualizer : MonoBehaviour
    {
        [Header("Views")]
        public bool visualizePositions;
        public bool visualizeNormals;
        public bool visualizeEdges;
        public bool visualizeVelocity;
        public bool visualizePinned;
        public bool visualizeSelfCollisionRegions;

        public float visualizationScale = 1f;

        [Space]
        [Header("Edges")]
        public Gradient stretchingGradient;

        [Space]
        [Header("Velocity")]
        public Gradient velocityGradient;
        public float maxDebugVelocity = 5;

        [Space]
        [Header("Self-Collision Regions")]
        public Gradient regionDensityGradient;
        public int maxDebugElements = 5;
        public bool skipEmpty = true;

        private UCCloth _scheduler;
        private Renderer _renderer;

        private Native3DHashmapArray<ushort> _selfColRegions;


        private void Start()
        {
            _scheduler = GetComponent<UCCloth>();
            _renderer = GetComponent<Renderer>();

            _scheduler.OnSimulationFinished += CopySelfCol;
        }

        private void OnDestroy()
        {
            _scheduler.OnSimulationFinished -= CopySelfCol;
            _selfColRegions.Dispose();
        }

        public void Reset()
        {
            // Set initial gradients
            var keyset1 = new GradientColorKey[5]
            {
                new GradientColorKey(new Color(0f, 0.175f, 1f), 0f),
                new GradientColorKey(new Color(0.283f, 0.645f, 0.95f), 0.4f),
                new GradientColorKey(new Color(1f, 1f, 1f), 0.5f),
                new GradientColorKey(new Color(1f, 0.72f, 0f), 0.6f),
                new GradientColorKey(new Color(1f, 0f, 0f), 1f)
            };
            stretchingGradient = new();
            stretchingGradient.colorKeys = keyset1;

            velocityGradient = new();
            velocityGradient.colorKeys = keyset1;

            var keyset2 = new GradientColorKey[3]
            {
                new GradientColorKey(new Color(1f, 0, 0f), 0f),
                new GradientColorKey(new Color(1f, 1f, 1f), 0.5f),
                new GradientColorKey(new Color(0f, 0f, 1f), 1f)
            };
            regionDensityGradient = new();
            regionDensityGradient.colorKeys = keyset2;
        }

        private void OnDrawGizmos()
        {
            if (_scheduler == null)
                return;

            var positions = _scheduler.simData.positionsReadOnly;

            // POSITIONS
            if (visualizePositions)
            {
                Gizmos.color = Color.white;
                Vector3 size = new(visualizationScale * 0.01f, visualizationScale * 0.01f, visualizationScale * 0.01f);
                foreach (var position in positions)
                {
                    Gizmos.DrawCube(position, size);
                }
            }

            // NORMALS
            if (visualizeNormals)
            {
                var normals = _scheduler.simData.normalsReadOnly;

                for (int i = 0; i < normals.Length; i++)
                {
                    var normal = normals[i];
                    var position = positions[i];

                    Color rayColor = Color.green;
                    Debug.DrawRay(position, normal * visualizationScale * 0.01f, rayColor);
                }
            }

            // EDGES
            if (visualizeEdges)
            {
                Gizmos.color = Color.red;
                var edges = _scheduler.simData.edgesReadOnly;

                int edgeIndex = 0;
                foreach (var edge in edges)
                {
                    Vector3 ogPosition = positions[edge.nodeIndex1];
                    Vector3 tgtPosition = positions[edge.nodeIndex2];

                    //Color info
                    float expectedDistance = _scheduler.simData.restDistancesReadOnly[edgeIndex];
                    float realDistance = math.distance(ogPosition, tgtPosition);
                    float error = (realDistance - expectedDistance) / expectedDistance;

                    error += 0.5f;    //To handle negative values
                    error = math.clamp(error, -1f, 1f);

                    Color guicolor = stretchingGradient.Evaluate(error);
                    Gizmos.color = guicolor;

                    Handles.color = guicolor;
                    Handles.DrawLine(ogPosition, tgtPosition, 2f);

                    edgeIndex++;
                }
            }

            // VELOCITY
            if (visualizeVelocity)
            {
                Gizmos.color = Color.white;
                var velocities = _scheduler.simData.velocityReadOnly;

                for (int i = 0; i < positions.Length; i++)
                {
                    float velocity = math.length(velocities[i]);
                    Gizmos.color = velocityGradient.Evaluate(velocity / maxDebugVelocity);

                    Gizmos.DrawRay(positions[i], velocities[i] * visualizationScale * 0.01f);
                }
            }

            // PINNED
            if (visualizePinned)
            {
                Gizmos.color = Color.blue;

                var weight = _scheduler.simData.reciprocalWeight;
                Vector3 size = new(visualizationScale * 0.013f, visualizationScale * 0.013f, visualizationScale * 0.013f);

                for (ushort i = 0; i < positions.Length; i++)
                {
                    float3 position = positions[i];
                    if (weight[i] < 0.001f)
                        Gizmos.DrawCube(position, size);
                }
            }


            // SELF COLLISION REGIONS
            if (visualizeSelfCollisionRegions)
            {
                var bounds = _renderer.bounds;
                var boundSize = (float3)bounds.size;

                var regionSize = _selfColRegions.size;

                if (!_selfColRegions.IsCreated())
                    return;

                // Draw Bounding box
                Gizmos.color = Color.black;
                Gizmos.DrawWireCube(bounds.center, boundSize);

                // Calculating offsets for aligning to BB correctly
                float3 offset = boundSize / _selfColRegions.size / 2;
                float3 extents = offset * 2f;
                extents = math.max(extents, new float3(0.0001f, 0.0001f, 0.0001f));

                for (int x = 0; x < regionSize.x; x++)
                {
                    for (int y = 0; y < regionSize.y; y++)
                    {
                        for (int z = 0; z < regionSize.z; z++)
                        {
                            float3 posIndex = new(x, y, z);
                            float3 pos = (float3)bounds.min + (posIndex * 2 + 1) * offset;

                            // Count how many nodes in the region, if any
                            int3 index = new(x, y, z);
                            bool any = _selfColRegions.TryGetItems(index, out NativeList<ushort> items);

                            int count = any ? items.Length : 0;

                            if (skipEmpty && count == 0)            //Should empty ones be skipped?
                            {
                                items.Dispose();
                                continue;
                            }

                            var color = regionDensityGradient.Evaluate(count / (float)maxDebugElements);

                            Gizmos.color = color;
                            Gizmos.DrawWireCube(pos, extents);
                            items.Dispose();
                        }
                    }
                }
            }
        }



        // Data safety helpers
        private void CopySelfCol(object sender, EventArgs _)
        {
            var schedRegions = _scheduler.simData.cSelfCollisionRegions;

            // If populated, clear and reallocate
            if (_selfColRegions.IsCreated())
                _selfColRegions.Dispose();

            _selfColRegions = new(schedRegions.size, Allocator.Persistent);

            // Copy data to the new array
            var keyValues = schedRegions._internalHm.GetKeyValueArrays(Allocator.Temp);

            for (int i = 0; i < keyValues.Length; i++)
            {
                var key = keyValues.Keys[i];
                var val = keyValues.Values[i];

                _selfColRegions._internalHm.Add(key, val);
            }
            keyValues.Dispose();
        }
    }
}
#endif