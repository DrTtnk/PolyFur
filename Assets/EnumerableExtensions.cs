using System;
using System.Collections.Generic;
using System.Linq;

namespace DefaultNamespace
{
    public static class EnumerableExtensions
    {
        public static IEnumerable<T[]> Split<T>(this IEnumerable<T> source, int size) {
            var result = new List<T>(size);
            foreach (var x in source) {
                result.Add(x);
                if (result.Count != size) 
                    continue;
                yield return result.ToArray();
                result = new List<T>(size);
            }
        }

        public static IEnumerable<(T first, T second)> PairUp<T>(this IReadOnlyList<T> source) 
            => source.Zip(source.Skip(1), (first, second) => (first, second));

        public static void ForEach<T>(this IEnumerable<T> source, Action<T> fn) {
            foreach (var element in source) fn(element);
        }
    }
}