﻿using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Represents mesh data after preprocessing.
    /// </summary>
    public struct UCMeshData
    {
        public List<float3> positions;
        public List<UCEdge> edges;
        public List<UCBendingEdge> bendingEdges;
        public List<int> boundingEdges;

        public NativeArray<int> triangles;
        public NativeParallelMultiHashMap<ushort, ushort> neighbours;

        public Dictionary<int, int> vertexMerges;
        public NativeArray<int> renderToSimLookup;
    }
}