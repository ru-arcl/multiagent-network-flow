"""HashSet<Integer> iteration order changed in JDK 8, and the original's results depend on it.

splitPaths takes the first free vertex of a HashSet of candidates (PathPlanner.java:615-628),
and the models' column order follows a HashSet frontier. mppflow emulates JDK 8+ (the JVM the
golden fixtures were recorded with). JDK 6/7 spread the hash differently
(h ^= (h>>>20) ^ (h>>>12); h ^ (h>>>7) ^ (h>>>4)) and inserted new entries at the head of a
bucket. Example: robot 0 -> 63 on an 8x8 grid; its candidates are the anti-diagonal.
"""
import _common
from _common import grid

from mppflow._java import MathRandom
from mppflow.pathfinder import find_goals_in_distance_range
from mppflow.planner import split_paths


def jdk7_order(keys, capacity=16):
    """HashSet<Integer> iteration order in JDK 6/7 (no resize: len(keys) <= 12)."""
    buckets = {}
    for k in keys:
        h = k ^ (k >> 20) ^ (k >> 12)
        h = h ^ (h >> 7) ^ (h >> 4)
        buckets.setdefault(h & (capacity - 1), []).insert(0, k)  # new entry at the head
    return [k for b in sorted(buckets) for k in buckets[b]]


g = grid(8, 8)
cand = find_goals_in_distance_range(g, 0, 7, 7)  # distance 7 from 0 (= from 63 on the diagonal)
print("candidates, JDK 8+ order:", cand)
print("candidates, JDK 6/7 order:", jdk7_order(sorted(cand)))
print("splitPaths intermediate goal (mppflow, JDK 8+ order):", split_paths(g, [0], [63], rng=MathRandom(1)))
