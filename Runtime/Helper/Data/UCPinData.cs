using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Stores pin information for a single collider.
    /// </summary>
    internal class UCPinData : IDisposable
    {
        internal readonly NativeList<ushort> pinnedNodeIds;
        internal readonly NativeList<float3> relativePositions;

        internal Transform cachedTransformRef;
        internal float4x4 lastTransform;

        internal UCPinData(Transform colliderTransform)
        {
            pinnedNodeIds = new(Allocator.Persistent);
            relativePositions = new(Allocator.Persistent);

            cachedTransformRef = colliderTransform.transform;
            lastTransform = cachedTransformRef.localToWorldMatrix;
        }

        public void Dispose()
        {
            pinnedNodeIds.Dispose();
            relativePositions.Dispose();
        }
    }
}
