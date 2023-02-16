using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

namespace UCloth
{
    /// <summary>
    /// Schedules cloth simulation for a GameObject.
    /// </summary>
    // NOTE: This class is also referred to as 'UCScheduler' or 'scheduler'.
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public class UCCloth : MonoBehaviour
    {
        public UCPreprocessorType preprocessorType;

        [Tooltip("These colliders will be used at startup to determine if a vertex is pinned or not.")]
        public List<Collider> pinColliders;

        [Space]
        [Header("Settings")]
        public UCMaterial materialProperties;
        public UCSimulationProperties simulationProperties;
        public UCQualityProperties qualityProperties;
        public UCCollisionSettings collisionProperties;

        [Space]
        public SphereCollider[] sphereColliders;
        public CapsuleCollider[] capsuleColliders;
        public BoxCollider[] cubeColliders;

        [Header("Post processing")]
        public float thickness;
        public bool offsetFront;
        [Range(0f, 0.95f)]
        public float smoothing;

        public UCInternalSimData simData;

        private NativeArray<SphereColDTO> sphereColDTOs;
        private NativeArray<CapsuleColDTO> capsuleColDTOs;
        private NativeArray<CubeColDTO> cubeColDTOs;

        // Public ones can be managed by users, internal are included by default and managed by this class
        public List<IUCPostprocessor> postprocessors;
        internal IUCPostprocessor[] internalPostprocessors;

        private UCRenderer _ucRenderer;
        private UCAutoOptimizer _ucOptimizer;
        private UCPinner _pinner;
        private MeshRenderer _meshRenderer;
        internal UCMeshData initialMeshData;
        internal NativeReference<UCAutoOptimizeData> optimizationData;

        // For point queries
        private NativeList<UCPointQueryData> pointQueries;
        private NativeList<UCPointQueryData> queryCopy;
        private NativeList<ushort> pointQueryResults;
        private NativeList<ushort> pointQueryIndexCounts;

        private JobHandle? _job;
        private TaskCompletionSource<bool> onBeforeStart;
        private TaskCompletionSource<bool> onFinished;

        private int _lastSimFrequency;
        private float _timestep;


        private void Start()
        {
            _pinner = new(this);

            bool success = SetUpData(out var meshData);

            if (!success)
                return;

            postprocessors = new List<IUCPostprocessor>();
            internalPostprocessors = new IUCPostprocessor[2];

            _ucRenderer = new(this, meshData,
                GetComponent<MeshFilter>(), GetComponent<MeshCollider>());

            _ucOptimizer = new(this);

            _meshRenderer = GetComponent<MeshRenderer>();
            InitializeTimer();
        }

        private void Reset()
        {
            UCInitializer.Initialize(this);
        }

        private void OnDestroy()
        {
            _job?.Complete();

            simData?.Dispose();

            if (sphereColDTOs.IsCreated)
                sphereColDTOs.Dispose();

            if (capsuleColDTOs.IsCreated)
                capsuleColDTOs.Dispose();

            if (cubeColDTOs.IsCreated)
                cubeColDTOs.Dispose();

            StopTimer();
            _ucRenderer?.Dispose();

            if (initialMeshData.triangles.IsCreated)
                initialMeshData.triangles.Dispose();

            if (initialMeshData.renderToSimLookup.IsCreated)
                initialMeshData.renderToSimLookup.Dispose();

            if (optimizationData.IsCreated)
                optimizationData.Dispose();

            pointQueries.Dispose();
            pointQueryResults.Dispose();
            pointQueryIndexCounts.Dispose();

            if (queryCopy.IsCreated)
                queryCopy.Dispose();

            for (int i = 0; i < postprocessors?.Count; i++)
            {
                postprocessors[i]?.Dispose();
            }

            for (int i = 0; i < internalPostprocessors?.Length; i++)
            {
                internalPostprocessors[i]?.Dispose();
            }
        }

        private void Update()
        {
            // If sim data not created successfully
            if (simData == null)
                return;

            // If possible, we start this work early, however this can increase latency
            if (!qualityProperties.minimizeLatency)
                _ucRenderer.ScheduleTransformations();

            _pinner.UpdateMoved();
            UpdateInternalPostprocessor();

            UpdateTimer();
        }

        private void LateUpdate()
        {
            if (qualityProperties.minimizeLatency)
                _ucRenderer.ScheduleTransformations();

            _ucRenderer.UpdateRenderedMesh();
        }

        // ----- PUBLIC APIs

        /// <summary>
        /// Fetches list of at least 1 closest point to the specified position. Call asynchronously!
        /// </summary>
        /// <param name="query"></param>
        /// <returns> List of point IDs, used as indices for simData. </returns>
        public async Task<List<ushort>> QueryClosestPoints(UCPointQueryData query)
        {
            // We can add the query straight away
            int queryIndex = pointQueries.Length;
            pointQueries.Add(query);

            // Then we wait until a new task is scheduled, so that queries are copied into it
            onBeforeStart ??= new();
            await onBeforeStart.Task;

            // And then wait for the job to finish
            onFinished ??= new();
            await onFinished.Task;

            // Results should be ready now
            // Get start index for this query
            int startIndex = queryIndex == 0 ? 0 : pointQueryIndexCounts[queryIndex - 1];

            int endIndex = pointQueryIndexCounts[queryIndex];

            // Read the results
            List<ushort> results = new();
            for (int i = startIndex; i < endIndex; i++)
            {
                results.Add(pointQueryResults[i]);
            }

            pointQueries.RemoveAt(0);
            return results;
        }



        // ----- SCHEDULING CODE

        /// <summary>
        /// Fetches the results of last simulation, and starts a new one.
        /// </summary>
        internal void FetchAndSimulate()
        {
            ScheduleFinish();
            ScheduleStart();
        }

        /// <summary>
        /// Schedules a simulation to take place in the background.
        /// </summary>
        private void ScheduleStart()
        {
            // Update normals
            UCNormalComputeJob normalJob = new()
            {
                vertices = simData.positionsReadOnly,
                normals = simData.normalsReadOnly,
                triangleNormals = simData.triangleNormalsReadOnly,
                triangles = initialMeshData.triangles,
                renderToSimLookup = initialMeshData.renderToSimLookup
            };
            var normalRecompute = normalJob.Schedule();

            UpdateColliderDTOs();
            VerifyDataValidity();

            _timestep = Time.timeScale * qualityProperties.timeScaleMultiplier / qualityProperties.simFrequency;
            _timestep = math.clamp(_timestep, 0f, qualityProperties.maxTimestep);

            // Finish up completion before start
            // Technically possible to make normal recomputation a dependency, but updating normals in UCRenderer is tricky
            normalRecompute.Complete();
            _ucRenderer.UpdateRenderingNormals(simData.normalsReadOnly);

            onBeforeStart?.SetResult(true);
            onBeforeStart = null;

            // Copy data that might be written to by the Job
            simData.CopyWriteableData();

            queryCopy = new(pointQueries.Capacity, Allocator.TempJob);
            queryCopy.CopyFrom(pointQueries);

            UCJob job = new()
            {
                positions = simData.cPositions,
                velocity = simData.cVelocity,
                acceleration = simData.cAcceleration,
                tempAcceleration = simData.cTempAcceleration,

                selfCollisionRegions = simData.cSelfCollisionRegions,
                utilizedRegionSet = simData.cUtilizedSelfColRegions,

                edges = simData.edgesReadOnly,
                bendingEdges = simData.bendingEdgesReadOnly,
                neighbours = simData.neighboursReadOnly,
                normals = simData.normalsReadOnly,
                normalsTriangles = simData.triangleNormalsReadOnly,
                restDistances = simData.restDistancesReadOnly,
                reciprocalWeight = simData.cReciprocalWeight,
                pinnedPos = simData.cPinnedPositions,

                material = materialProperties,
                collisionSettings = collisionProperties,

                sphereColliders = sphereColDTOs,
                capsuleColliders = capsuleColDTOs,
                cubeColliders = cubeColDTOs,

                simProperties = simulationProperties,
                baseTimestep = _timestep,
                extraThickness = thickness / 1000f,
                qualityProperties = qualityProperties,

                pointQueries = queryCopy,
                pointQueryResults = pointQueryResults,
                pointQueryIndexCounts = pointQueryIndexCounts,

                bounds = _meshRenderer.bounds,
                optimizationData = optimizationData
            };

            _job = job.Schedule();
        }

        /// <summary>
        /// Fetches the result of the simulation.
        /// </summary>
        private void ScheduleFinish()
        {
            if (_job == null)
                return;

            _job.Value.Complete();

            onFinished?.SetResult(true);
            onFinished = null;

            _ucRenderer.UpdateRenderingPositions(simData.cPositions);
            UpdateAutooptimisation();

            // Dispose of colliders since they're updated every frame
            sphereColDTOs.Dispose();
            capsuleColDTOs.Dispose();
            cubeColDTOs.Dispose();

            // Clear query arrays
            queryCopy.Dispose();

            _job = null;
        }


        // ----- TIMER CODE

        private void InitializeTimer()
        {
            InvokeRepeating(nameof(FetchAndSimulate), 0f, 1f / qualityProperties.simFrequency);
        }

        private void UpdateTimer()
        {
            // Frequency was changed
            if (_lastSimFrequency != qualityProperties.simFrequency)
            {
                StopTimer();
                InitializeTimer();
            }

            _lastSimFrequency = qualityProperties.simFrequency;
        }

        private void StopTimer()
        {
            CancelInvoke(nameof(FetchAndSimulate));
        }




        /// <summary>
        /// Updates the collider DTOs used for simulation.
        /// </summary>
        private void UpdateColliderDTOs()
        {
            // Expansion offset is set as 1% of the cloth bounds
            var clothBounds = _meshRenderer.bounds;
            Vector3 expansionOffset = (clothBounds.size * 0.01f)
                + new Vector3(collisionProperties.collisionContactOffset, collisionProperties.collisionContactOffset, collisionProperties.collisionContactOffset);

            // Spheres
            // Before creating the DTOs, we go through potential colliders and discard those that don't need computation
            var validSpheres = UCColliderDTOHelper.FilterColliders(sphereColliders, clothBounds, expansionOffset);

            sphereColDTOs = new NativeArray<SphereColDTO>(validSpheres.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < validSpheres.Count; i++)
            {
                var sphere = validSpheres[i];
                Vector3 objScale = sphere.transform.localScale;

                float radiusScale = math.max(math.max(objScale.x, objScale.y), objScale.z);
                sphereColDTOs[i] = new SphereColDTO()
                {
                    position = sphere.transform.TransformPoint(sphere.center),
                    radius = sphere.radius * radiusScale,
                    friction = UCColliderDTOHelper.GetFriction(sphere)
                };
            }

            // Capsules
            // NOTE: Currently the "Direction" property is not supported and defaults to Y axis
            var validCapsules = UCColliderDTOHelper.FilterColliders(capsuleColliders, clothBounds, expansionOffset);

            capsuleColDTOs = new NativeArray<CapsuleColDTO>(validCapsules.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < validCapsules.Count; i++)
            {
                var capsule = validCapsules[i];
                Vector3 objScale = capsule.transform.localScale;
                Vector3 quarterHeight = capsule.height * objScale.y * 0.25f * capsule.transform.up;

                Vector3 offsetPosition = capsule.transform.TransformPoint(capsule.center);
                float radiusScale = math.max(objScale.x, objScale.z);

                capsuleColDTOs[i] = new CapsuleColDTO()
                {
                    a = offsetPosition + quarterHeight,
                    ba = -2 * quarterHeight,
                    radius = capsule.radius * radiusScale,
                    friction = UCColliderDTOHelper.GetFriction(capsule)
                };
            }

            // Cubes
            var validCubes = UCColliderDTOHelper.FilterColliders(cubeColliders, clothBounds, expansionOffset);

            cubeColDTOs = new NativeArray<CubeColDTO>(validCubes.Count, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < validCubes.Count; i++)
            {
                var cube = validCubes[i];

                cubeColDTOs[i] = new CubeColDTO()
                {
                    position = cube.transform.position,
                    offset = cube.center,
                    size = cube.size,
                    localMatrix = cube.transform.worldToLocalMatrix,
                    worldMatrix = cube.transform.localToWorldMatrix,
                    friction = UCColliderDTOHelper.GetFriction(cube)
                };
            }

            if (simData.cSelfCollisionRegions.IsCreated())
                simData.cSelfCollisionRegions.Dispose();

            // Self-collision preparation
            int3 regions = (int3)math.ceil(_meshRenderer.bounds.size * collisionProperties.gridDensity);
            simData.cSelfCollisionRegions = new Native3DHashmapArray<ushort>(regions, Allocator.TempJob);
        }

        /// <summary>
        /// Automatically adjusts some simulation performance parameters.
        /// </summary>
        private void UpdateAutooptimisation()
        {
            if (collisionProperties.AutoAdjustGridDensity != UCAutoAdjustGridOption.None)
                collisionProperties.gridDensity = _ucOptimizer.OptimizeDensity(optimizationData.Value);
        }

        /// <summary>
        /// Checks if postprocessors need to be added or scheduled for removal.
        /// </summary>
        private void UpdateInternalPostprocessor()
        {
            // 0 - smoothing
            // 1 - thickness

            bool usingSmoothing = internalPostprocessors[0] != null;
            bool usingThickness = internalPostprocessors[1] != null;

            // Smoothing
            if (usingSmoothing && smoothing < 0.0000001)
            {
                internalPostprocessors[0].ScheduledToCleanup = true;
            }
            else if (!usingSmoothing && smoothing >= 0.0000001)
            {
                internalPostprocessors[0] = new UCSmoothingPostprocessor(this);
            }

            // Thickness
            if (usingThickness && math.abs(thickness) < 0.0000001)
            {
                internalPostprocessors[1].ScheduledToCleanup = true;
            }
            else if (!usingThickness && math.abs(thickness) >= 0.0000001)
            {
                internalPostprocessors[1] = new UCThicknessPostprocessor(this);
            }
        }

        /// <summary>
        /// Validates that data is in valid range.
        /// </summary>
        private void VerifyDataValidity()
        {
            // TODO: Add more parameters as required
            materialProperties.vertexMass = math.max(0.01f, materialProperties.vertexMass);
            qualityProperties.iterations = math.max(1, qualityProperties.iterations);
            qualityProperties.constraintIterations = math.max(0, qualityProperties.constraintIterations);

            collisionProperties.collisionFriction = math.max(0, collisionProperties.collisionFriction);
            collisionProperties.selfCollisionDistance = math.max(0, collisionProperties.selfCollisionDistance);
            collisionProperties.selfCollisionStiffness = math.max(0, collisionProperties.selfCollisionStiffness);
            collisionProperties.selfCollisionFriction = math.max(0, collisionProperties.selfCollisionFriction);
            collisionProperties.gridDensity = math.max(0, collisionProperties.gridDensity);
        }


        //--- SETUP

        /// <summary>
        /// Sets up simulation data from selected source.
        /// </summary>
        private bool SetUpData(out UCMeshData data)
        {
            IUCPreprocessor preprocessor;
            object input;

            // Different preprocessor types can use different inputs
            switch (preprocessorType)
            {
                case UCPreprocessorType.Mesh:
                    preprocessor = new UCMeshPreprocessor();
                    input = GetComponent<MeshFilter>().mesh;
                    break;

                default:
                    Debug.LogError("Invalid preprocessor type in UCloth scheduler!", this);
                    throw new UnityException();
            }

            bool success = preprocessor.ConvertMeshData(input, out data);
            initialMeshData = data;

            if (!success)
                return false;

            // Translate positions into world space for later calculations
            float3[] positions = new float3[data.positions.Count];
            var localToWorldMatrix = transform.localToWorldMatrix;
            for (int i = 0; i < data.positions.Count; i++)
            {
                positions[i] = localToWorldMatrix.MultiplyPoint(data.positions[i]);
            }

            simData = new()
            {
                // Transform data to NativeArrays
                cPositions = new NativeArray<float3>(positions, Allocator.Persistent),
                edgesReadOnly = new NativeArray<UCEdge>(data.edges.ToArray(), Allocator.Persistent),
                bendingEdgesReadOnly = new NativeArray<UCBendingEdge>(data.bendingEdges.ToArray(), Allocator.Persistent),
                neighboursReadOnly = data.neighbours,
                normalsReadOnly = new NativeArray<float3>(positions.Length, Allocator.Persistent),
                triangleNormalsReadOnly = new NativeArray<float3>(initialMeshData.triangles.Length, Allocator.Persistent)
            };

            // The rest distance can be computed for every edge easily
            // This cannot be computed in the preprocessor! This is because if the mesh is scaled, it won't be reflected
            simData.restDistancesReadOnly = new NativeArray<float>(simData.edgesReadOnly.Length, Allocator.Persistent);
            for (int i = 0; i < data.edges.Count; i++)
            {
                UCEdge edge = data.edges[i];

                float distance = math.distance(positions[edge.nodeIndex1], positions[edge.nodeIndex2]);
                simData.restDistancesReadOnly[i] = distance;
            }

            // Velocity is initialized to 0
            simData.cVelocity = new NativeArray<float3>(simData.cPositions.Length, Allocator.Persistent);
            simData.cAcceleration = new NativeArray<float3>(simData.cPositions.Length, Allocator.Persistent);
            simData.cTempAcceleration = new NativeArray<float3>(simData.cPositions.Length, Allocator.Persistent);

            simData.cUtilizedSelfColRegions = new NativeParallelHashSet<int3>(100, Allocator.Persistent);   //TODO: Arbitrary magic number, fix

            optimizationData = new NativeReference<UCAutoOptimizeData>(Allocator.Persistent);

            // Query data
            pointQueries = new(Allocator.Persistent);
            pointQueryResults = new(Allocator.Persistent);
            pointQueryIndexCounts = new(Allocator.Persistent);

            _pinner.SetUpDataPinned();
            simData.PrepareCopies();
            simData.CopyWriteableData();
            return true;
        }
    }
}