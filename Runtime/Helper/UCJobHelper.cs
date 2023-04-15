using System;
using Unity.Burst;
using Unity.Burst.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace UCloth
{
    [BurstCompatible]
    internal static class UCJobHelper
    {
        internal static NativeList<ushort> GetSurroundingNodesNoAlloc(ref Native3DHashmapArray<ushort> regions, int3 index, NativeList<ushort> surrounding, int accuracy)
        {
            if (accuracy == 0)
                return surrounding;

            // Using hints to minimize branch mispredictions (most of the time, the condition will be true)
            // Directly adjacent
            if (Hint.Likely(index.x > 0))
                regions.TryGetItemsNoAlloc(index - new int3(1, 0, 0), ref surrounding);

            if (Hint.Likely(index.y > 0))
                regions.TryGetItemsNoAlloc(index - new int3(0, 1, 0), ref surrounding);

            if (Hint.Likely(index.z > 0))
                regions.TryGetItemsNoAlloc(index - new int3(0, 0, 1), ref surrounding);


            if (Hint.Likely(index.x < regions.size.x - 1))
                regions.TryGetItemsNoAlloc(index + new int3(1, 0, 0), ref surrounding);

            if (Hint.Likely(index.y < regions.size.y - 1))
                regions.TryGetItemsNoAlloc(index + new int3(0, 1, 0), ref surrounding);

            if (Hint.Likely(index.z < regions.size.z - 1))
                regions.TryGetItemsNoAlloc(index + new int3(0, 0, 1), ref surrounding);

            // No corners on 1st accuracy level
            if (accuracy == 1)
                return surrounding;


            // Corners
            if (Hint.Likely(index.x > 0 && index.y > 0 && index.z > 0))
                regions.TryGetItemsNoAlloc(index + new int3(-1, -1, -1), ref surrounding);

            if (Hint.Likely(index.x < regions.size.x - 1 && index.y > 0 && index.z > 0))
                regions.TryGetItemsNoAlloc(index + new int3(1, -1, -1), ref surrounding);

            if (Hint.Likely(index.x > 0 && index.y < regions.size.y - 1 && index.z > 0))
                regions.TryGetItemsNoAlloc(index + new int3(-1, 1, -1), ref surrounding);

            if (Hint.Likely(index.x < regions.size.x - 1 && index.y < regions.size.y - 1 && index.z > 0))
                regions.TryGetItemsNoAlloc(index + new int3(1, 1, -1), ref surrounding);

            if (Hint.Likely(index.x > 0 && index.y > 0 && index.z < regions.size.z - 1))
                regions.TryGetItemsNoAlloc(index + new int3(-1, -1, 1), ref surrounding);

            if (Hint.Likely(index.x < regions.size.x - 1 && index.y > 0 && index.z < regions.size.z - 1))
                regions.TryGetItemsNoAlloc(index + new int3(1, -1, 1), ref surrounding);

            if (Hint.Likely(index.x > 0 && index.y < regions.size.y - 1 && index.z < regions.size.z - 1))
                regions.TryGetItemsNoAlloc(index + new int3(-1, 1, 1), ref surrounding);

            if (Hint.Likely(index.x < regions.size.x - 1 && index.y < regions.size.y - 1 && index.z < regions.size.z - 1))
                regions.TryGetItemsNoAlloc(index + new int3(1, 1, 1), ref surrounding);

            return surrounding;
        }
    }
}