using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace UCloth
{
    /// <summary>
    /// Copies <see cref="NativeParallelHashMap{TKey, TValue}"/> from source to dest.
    /// </summary>
    [BurstCompile]
    internal struct NativeParallelHashMapCopyJob<TKey, TValue> : IJobParallelFor
        where TKey : struct, IEquatable<TKey> where TValue : struct
    {
        [ReadOnly]
        [DeallocateOnJobCompletion]
        private NativeKeyValueArrays<TKey, TValue> KeyValues;

        [WriteOnly]
        private NativeParallelHashMap<TKey, TValue>.ParallelWriter OutputWriter;

        public NativeParallelHashMapCopyJob(NativeParallelHashMap<TKey, TValue> source, NativeParallelHashMap<TKey, TValue> dest)
        {
            KeyValues = source.GetKeyValueArrays(Allocator.TempJob);
            OutputWriter = dest.AsParallelWriter();
        }

        public void Execute(int index)
        {
            OutputWriter.TryAdd(KeyValues.Keys[index], KeyValues.Values[index]);
        }
    }
}