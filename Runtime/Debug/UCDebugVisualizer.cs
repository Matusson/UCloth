using Unity.Collections;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace UCloth
{
    public class UCDebugVisualizer : MonoBehaviour
    {
        [Header("Views")]
        public bool visualizePositions;
        public bool visualizeNormals;
        public bool visualizeRelations;
        public bool visualizeVelocity;
        public bool visualizeForces;
        public bool visualizePinned;
        public bool visualizeSelfCollisionRegions;

        [Space]
        [Header("Normals")]
        public float normalLength;
        [Tooltip("Due to precision errors in matrix multiplication, some normals can be not perfectly normalized.")]
        public bool markNonNormalized;

        [Space]
        [Header("Relations")]
        public Gradient distanceGradient;
        public int textSize;

        [Space]
        [Header("Velocity")]
        public Gradient velocityGradient;
        public float velocityLength;
        public float maxDebugVelocity;

        [Space]
        [Header("Force (Acceleration)")]
        public Gradient forceGradient;
        public float forceLength;
        public float maxDebugForce;

        [Space]
        [Header("Self-Collision Regions")]
        public Gradient regionsGradient;
        public int tempMaxElements = 1;
        public bool skipEmpty;

        private UCCloth _scheduler;
        private Renderer _renderer;

        private void Start()
        {
            _scheduler = GetComponent<UCCloth>();
            _renderer = GetComponent<Renderer>();
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
                foreach (var position in positions)
                {
                    Gizmos.DrawCube(position, new(0.1f, 0.1f, 0.1f));
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

                    // Change colors if needed 
                    Color rayColor = Color.green;
                    if (markNonNormalized)
                    {
                        float length = math.length(normal);

                        if (length > 0.9999f && length < 1.0001f)
                            rayColor = Color.red;

                    }
                    Debug.DrawRay(position, normal * normalLength, rayColor);
                }
            }

            // RELATIONS
            if (visualizeRelations)
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

                    Color guicolor = distanceGradient.Evaluate(error);
                    Gizmos.color = guicolor;

                    //Text
                    if (textSize > 0)
                    {
                        Handles.color = guicolor;

                        Vector3 edgeCenter = (ogPosition + tgtPosition) / 2;
                        string text = ((error - 0.5f) * 100f).ToString("F0") + "%";

                        GUIStyle style = EditorStyles.boldLabel;
                        style.fontSize = textSize;
                        style.normal.textColor = guicolor;
                        Handles.Label(edgeCenter, text, style);
                    }

                    Handles.color = guicolor;
                    Handles.DrawLine(ogPosition, tgtPosition, 2f);
                    //Gizmos.DrawLine(ogPosition, tgtPosition);
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

                    Gizmos.DrawRay(positions[i], velocities[i] * velocityLength);
                }
            }

            // FORCES
            if (visualizeForces)
            {
                Gizmos.color = Color.white;
                var forces = _scheduler.simData.cAcceleration;

                for (int i = 0; i < positions.Length; i++)
                {
                    float force = math.length(forces[i]);
                    Gizmos.color = forceGradient.Evaluate(force / maxDebugForce);

                    Gizmos.DrawRay(positions[i], forces[i] * forceLength);
                }
            }

            // PINNED
            if (visualizePinned)
            {
                Gizmos.color = Color.blue;

                var weight = _scheduler.simData.reciprocalWeight;
                for (ushort i = 0; i < positions.Length; i++)
                {
                    float3 position = positions[i];
                    if (weight[i] < 0.001f)
                        Gizmos.DrawCube(position, new(0.13f, 0.13f, 0.13f));
                }
            }


            // SELF COLLISION REGIONS
            if (visualizeSelfCollisionRegions)
            {
                var bounds = _renderer.bounds;
                var boundSize = (float3)bounds.size;

                var regions = _scheduler.simData.cSelfCollisionRegions;
                var regionSize = regions.size;

                if (!regions.IsCreated())
                    return;

                // Draw Bounding box
                Gizmos.color = Color.black;
                Gizmos.DrawWireCube(bounds.center, boundSize);

                // Calculating offsets for aligning to BB correctly
                float3 offset = boundSize / regions.size / 2;
                float3 extents = offset * 2f;
                extents = math.max(extents, new float3(0.1f, 0.1f, 0.1f));

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
                            bool any = regions.TryGetItems(index, out NativeList<ushort> items);

                            int count = any ? items.Length : 0;

                            if (skipEmpty && count == 0)            //Should empty ones be skipped?
                            {
                                items.Dispose();
                                continue;
                            }

                            var color = regionsGradient.Evaluate(count / (float)tempMaxElements);

                            Gizmos.color = color;
                            Gizmos.DrawWireCube(pos, extents);
                            items.Dispose();
                        }
                    }
                }
            }
        }
    }
}