using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Mathematics;

namespace UCloth
{
    /// <summary>
    /// Multi-hashmap accessible using a int3 index. For internal UC use only.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    [BurstCompatible]
    internal struct Native3DHashmapArray<T> : IDisposable where T : unmanaged
    {
        internal NativeParallelMultiHashMap<int, T> _internalHm;
        internal int3 size;

        private const ushort MAX_SIZE = 64;

        internal Native3DHashmapArray(int3 size, Allocator allocator)
        {
            // Make sure the index is within max size
            int3 maxSize = new(MAX_SIZE, MAX_SIZE, MAX_SIZE);
            size = math.clamp(size, new int3(0, 0, 0), maxSize);

            this.size = size;

            // Instead of keeping int3s in the hash map, which are 12 bytes each, we flatten the index (Mathematics does not provide ushort3 or byte3 types)
            // 64 x 64 x 64 = 262 144, so we need at least 3 bytes - we use a 4-byte int
            _internalHm = new NativeParallelMultiHashMap<int, T>(this.size.x * this.size.y * this.size.z, allocator);
        }

        [BurstCompatible]
        internal bool IsCreated()
        {
            return _internalHm.IsCreated;
        }

        [BurstCompatible]
        internal bool ContainsKey(int3 index)
        {

            return _internalHm.TryGetFirstValue(GetFlatIndex(index), out T _, out var _);
        }

        [BurstCompatible]
        internal bool TryGetItems(int3 index, out NativeList<T> items)
        {
            items = new NativeList<T>(4, Allocator.Temp);
            if (!_internalHm.TryGetFirstValue(GetFlatIndex(index), out T item, out var iterator))
                return false;

            items.Add(item);

            while (_internalHm.TryGetNextValue(out item, ref iterator))
            {
                items.Add(item);
            }

            return true;
        }

        /// <summary>
        /// This variant of TryGetItems does not create a new allocation, instead adds to the existing list
        /// </summary>
        /// <returns></returns>
        [BurstCompatible]
        internal bool TryGetItemsNoAlloc(int3 index, ref NativeList<T> items)
        {
            if (!_internalHm.TryGetFirstValue(GetFlatIndex(index), out T item, out var iterator))
                return false;

            items.Add(item);

            while (_internalHm.TryGetNextValue(out item, ref iterator))
            {
                items.Add(item);
            }

            return true;
        }

        [BurstCompatible]
        internal void Add(int3 index, T value)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            if (math.any(index > size) || math.any(index < new int3(0, 0, 0)))
                throw new ArgumentException("Index out of bounds.");
#endif

            _internalHm.Add(GetFlatIndex(index), value);
        }


        [BurstCompatible]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int GetFlatIndex(int3 index)
        {
            return index.x + size.x * (index.y + size.y * index.z);
        }

        [BurstCompatible]
        public void Dispose()
        {
            _internalHm.Dispose();
        }
    }
}