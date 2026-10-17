"""PathPlanner.planPathsAdvancedSplitNoCycle (PathPlanner.java:716-774).

1. Top-level retries are unbounded (`tries-- > 0 || top`, lines 730/737/740) while the result
   has rotations, and the time is never charged: on an instance whose only plans are
   rotations it never returns. 6-cycle, robot i -> i+2.
2. Retried pieces are not null-checked (only the first pair is, line 728); a null makes the
   next hasCycles(null) throw NullPointerException (line 1152). splits = 2 on the same ring:
   a quarter piece is infeasible.
3. With MultiagentGraphSolverGurobiTime.bPrintPaths = true, the flag is cleared at entry
   (lines 718-721) but not restored on the splits == 0 return (line 767) or the early null
   return (line 728).
"""
import _common
from _common import edges_graph, grid, show

from mppflow._java import MathRandom
from mppflow.planner import PathPlanner, SolverFlags, has_cycles

ring = edges_graph(6, [(i, (i + 1) % 6) for i in range(6)])
s, t = list(range(6)), [(i + 2) % 6 for i in range(6)]


def summary(p):
    return "null" if p is None else f"makespan {len(p[0]) - 1}, rotations (hasCycles) {has_cycles(p)}"


for c in ("original", "fixed"):
    pl = PathPlanner(compat=c, rng=MathRandom(1))
    show(f"{c}: ring, splits 1, top", lambda: summary(pl.plan_paths_advanced_split_no_cycle(ring, s, t, True, 10.0, 1, True)))
    print(f"   leaf solves: {len(pl.solves)}; {pl.failures[:1]}")
    pl = PathPlanner(compat=c, rng=MathRandom(1))
    show(f"{c}: ring, splits 2, top", lambda: summary(pl.plan_paths_advanced_split_no_cycle(ring, s, t, True, 100.0, 2, True)))
for c in ("original", "fixed"):
    pl = PathPlanner(compat=c, rng=MathRandom(1), flags=SolverFlags.java_defaults())
    pl.plan_paths_advanced_split_no_cycle(grid(3, 3), [0, 2], [8, 6], True, -1.0, 0, True)
    print(f"{c}: splits 0 -> bPrintPaths afterwards {pl.flags.time_print_paths}, paths printed: {'Agent 0' in pl.log}")
