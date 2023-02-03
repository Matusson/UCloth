using System;
using Unity.Collections;

namespace UCloth
{
    /// <summary>
    /// Helper class containing various utilities for NativeParallelMultiHashMap.
    /// </summary>
    internal static class NativeParallelMultiHashMapExtensions
    {
        /// <summary>
        /// Returns true if given key contains the specified value.
        /// </summary>
        /// <returns> True if given <paramref name="key"/> contains <paramref name="value"/>. </returns>
        [BurstCompatible]
        internal static bool KeyContainsValue<TKey, TValue>(this NativeParallelMultiHashMap<TKey, TValue> hashmap, TKey key, TValue value)
            where TKey : struct, IEquatable<TKey> where TValue : struct, IEquatable<TValue>
        {
            // Checks if contains any value
            if (!hashmap.TryGetFirstValue(key, out TValue outVal, out var iterator))
                return false;

            if (outVal.Equals(value))
                return true;

            // Continues iterating until either value is found, or reached the end
            while (hashmap.TryGetNextValue(out outVal, ref iterator))
            {
                if (outVal.Equals(value))
                    return true;
            }

            // And then check if ran out of elements to check, or because it found the value
            return false;
        }
    }
}