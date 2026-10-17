"""Time-limit accounting of MultiagentGraphSolverGurobiTime.solve (same code in TimeR).

The loop does `timeLimit = timeLimit - (now - startTime)` at every iteration with a fixed
startTime, so earlier elapsed time is subtracted again and again; the time-out test
subtracts the elapsed time once more; prepareModel maps a limit <= 0 to "no limit" (1e10);
and with hasTimeLimit = true a limit <= 0 returns null at the first failed horizon. A fake
clock that advances 0.5 s per reading makes the runs deterministic (debug=True shows the
limit the original prints). HiGHS backend.
"""
import itertools

import _common
from _common import edges_graph, grid

from mppflow._java import MathRandom
from mppflow.planner import PathPlanner
from mppflow.solvers import solve_time


def clock(step=0.5):
    c = itertools.count()
    return lambda: next(c) * step


star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])  # Main case 3; swap 0 <-> 3: T=2,3 infeasible, T=4 optimal
g3 = grid(3, 3)
s3, t3 = [0, 2, 6], [8, 6, 1]  # feasible at its lower bound T = 4

print("(a) limits printed per horizon, time limit 10 s:")
for c in ("original", "fixed"):
    r = solve_time(star, [0, 3], [3, 0], False, True, 0, True, 10.0, compat=c, debug=True, clock=clock())
    lims = [ln.split("max time limit ")[1].split("...")[0] for ln in r.log.splitlines() if "max time limit" in ln]
    print(f"  {c}: {lims} -> makespan {r.makespan}")
print("(b) time limit 3.2 s:")
for c in ("original", "fixed"):
    r = solve_time(star, [0, 3], [3, 0], False, True, 0, True, 3.2, compat=c, clock=clock())
    print(f"  {c}: status {r.status}, makespan {r.makespan}")
print("(c) time limit 0.2 s, shorter than the setup (first remaining limit -0.3 s):")
for c in ("original", "fixed"):
    r = solve_time(g3, s3, t3, False, True, 0, True, 0.2, compat=c, clock=clock())
    print(f"  {c}: status {r.status}, makespan {r.makespan}, models solved {len(r.attempts)}")
print("(d) hasTimeLimit = true with timeLimit = -1 (the code base's 'no limit'):")
for c in ("original", "fixed"):
    r = solve_time(star, [0, 3], [3, 0], False, True, 0, True, -1.0, compat=c, clock=clock(0.0))
    print(f"  {c}: status {r.status}, makespan {r.makespan}")
for c in ("original", "fixed"):
    r = solve_time(g3, s3, t3, True, True, 0, True, -1.0, compat=c, clock=clock(0.0))
    print(f"  {c}, LP relaxation first: status {r.status}, makespan {r.makespan}, "
          f"log: {[ln for ln in r.log.splitlines() if 'relaxed' in ln]}")
print("(e) multiThreadedSplitPlanning(timeLimit = -1, splits = 1) always passes hasTimeLimit = true:")
for c in ("original", "fixed"):
    p = PathPlanner(compat=c, rng=MathRandom(1)).multi_threaded_split_planning(g3, s3, t3, -1.0, 1)
    print(f"  {c}: {'null' if p is None else 'makespan %d' % (len(p[0]) - 1)}")
