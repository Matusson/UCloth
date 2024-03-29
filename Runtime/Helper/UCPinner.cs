﻿using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Handles moving pinned nodes.
    /// </summary>
    internal class UCPinner : IDisposable
    {
        private readonly UCCloth _scheduler;
        private UCInternalSimData _simData;

        private List<Collider> _colliders;
        private readonly Dictionary<Collider, UCPinData> _pinData;

        internal UCPinner(UCCloth scheduler)
        {
            _scheduler = scheduler;
            _pinData = new();
        }

        internal void UpdateMoved()
        {
            List<JobHandle> scheduledUpdates = new();

            foreach (var collider in _colliders)
            {
                var pinData = _pinData[collider];

                // If not modified transform, no need to update
                float4x4 currentMatrix = pinData.cachedTransformRef.localToWorldMatrix;
                if (currentMatrix.Equals(pinData.lastTransform))
                    continue;

                pinData.lastTransform = currentMatrix;

                // Schedule the update, don't block the main thread
                UCPinUpdateJob updateJob = new()
                {
                    nodeIds = pinData.pinnedNodeIds,
                    relativePositions = pinData.relativePositions,
                    pinnedPositions = _simData.pinnedPositions,
                    localToWorld = currentMatrix
                };
                var handle = updateJob.Schedule();
                scheduledUpdates.Add(handle);
            }

            // Complete the jobs before applying data
            foreach (var handle in scheduledUpdates)
            {
                handle.Complete();
            }

            // No need to apply if no jobs were scheduled
            if (scheduledUpdates.Count != 0)
                _simData.ApplyModifiedData();
        }

        /// <summary>
        /// Checks if points are within the pin colliders, and marks them accordingly.
        /// </summary>
        internal void SetUpDataPinned()
        {
            _simData = _scheduler.simData;

            _simData.reciprocalWeight = new NativeArray<float>(_simData.cPositions.Length, Allocator.Persistent);
            //TODO: We need to set the capacity here to fit all nodes either way - maybe keep it an array?
            _simData.pinnedPositions = new NativeParallelHashMap<ushort, float3>(_simData.cPositions.Length, Allocator.Persistent);

            // Sort the pin colliders from smallest to biggest, so small colliders override big ones
            _colliders = _scheduler.pinColliders.Where(x => x.enabled && x.gameObject.activeInHierarchy).OrderBy(x => x.bounds.size.magnitude).ToList();

            // Create pin data for every collider
            for (int i = 0; i < _colliders.Count; i++)
            {
                var collider = _colliders[i];
                _pinData.Add(collider, new(collider.transform));
            }

            // Then assign nodes to colliders
            for (ushort i = 0; i < _simData.reciprocalWeight.Length; i++)
            {
                // Set initial weight
                _simData.reciprocalWeight[i] = 1f;

                float3 position = _simData.cPositions[i];
                foreach (var collider in _colliders)
                {
                    // Check if within collider
                    float3 closestOnCollider = collider.ClosestPoint(position);
                    if (math.all(math.abs(position - closestOnCollider) < 0.00001f))
                    {
                        // Simulation data
                        _simData.reciprocalWeight[i] = 0f;
                        _simData.pinnedPositions.Add(i, position);

                        // Pin data
                        _pinData[collider].pinnedNodeIds.Add(i);
                        _pinData[collider].relativePositions.Add(collider.transform.InverseTransformPoint(position));
                        break;
                    }
                }
            }

            // There might be edges which are fully pinned. There's no point computing those, so we discard them
            RemoveUnusedEdges();
        }

        /// <summary>
        /// Removes edges which are fully pinned.
        /// </summary>
        private void RemoveUnusedEdges()
        {
            List<UCEdge> tempEdges = new(_simData.edgesReadOnly.Length);
            List<float> tempEdgeLengths = new(_simData.edgesReadOnly.Length);

            for (int i = 0; i < _simData.edgesReadOnly.Length; i++)
            {
                var edge = _simData.edgesReadOnly[i];

                if (_simData.reciprocalWeight[edge.nodeIndex1] > 0.0001f || _simData.reciprocalWeight[edge.nodeIndex2] > 0.0001f)
                {
                    tempEdges.Add(edge);
                    tempEdgeLengths.Add(_simData.restDistancesReadOnly[i]);
                }
            }

            // Clear old data and replace with cleaned up data
            _simData.edgesReadOnly.Dispose();
            _simData.edgesReadOnly = new(tempEdges.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            _simData.restDistancesReadOnly.Dispose();
            _simData.restDistancesReadOnly = new(tempEdges.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < _simData.edgesReadOnly.Length; i++)
            {
                _simData.edgesReadOnly[i] = tempEdges[i];
                _simData.restDistancesReadOnly[i] = tempEdgeLengths[i];
            }

            // And then bending edges
            List<UCBendingEdge> tempBending = new(_simData.bendingEdgesReadOnly);
            for (int i = 0; i < _simData.bendingEdgesReadOnly.Length; i++)
            {
                var edge = _simData.bendingEdgesReadOnly[i];

                if (_simData.reciprocalWeight[edge.bendingNode1] > 0.0001f || _simData.reciprocalWeight[edge.bendingNode2] > 0.0001f)
                {
                    tempBending.Add(edge);
                }
            }
            _simData.bendingEdgesReadOnly.Dispose();
            _simData.bendingEdgesReadOnly = new(tempBending.Count, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < _simData.bendingEdgesReadOnly.Length; i++)
            {
                _simData.bendingEdgesReadOnly[i] = tempBending[i];
            }
        }


        public void Dispose()
        {
            var pinDatas = _pinData.Values;

            foreach (var data in pinDatas)
            {
                data.Dispose();
            }
        }
    }
}
