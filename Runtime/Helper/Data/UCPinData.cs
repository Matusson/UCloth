using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace UCloth
{
    /// <summary>
    /// Stores pin information for a single collider.
    /// </summary>
    internal class UCPinData
    {
        internal readonly List<ushort> pinnedNodeIds;
        internal readonly List<float3> relativePositions;

        internal Transform cachedTransformRef;
        internal float4x4 lastTransform;

        internal UCPinData(Transform colliderTransform)
        {
            pinnedNodeIds = new();
            relativePositions = new();

            cachedTransformRef = colliderTransform.transform;
            lastTransform = cachedTransformRef.localToWorldMatrix;
        }
    }
}
