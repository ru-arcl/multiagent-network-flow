"""The local-repair heuristics (PathPlanner / HeuristicPathPlanner).

1. planHeuristicPath called directly: distanceToGoals is only set by planHeuristicPaths
   (HeuristicPathPlanner.java:51), so createSubProblem (line 396; PathPlanner.java:386) throws
   NullPointerException at the first conflict. Instance: getLongStraightWithOneGarageProblem.
2. Zero robots: pathArray[0] (HeuristicPathPlanner.java:102) throws.
3. Edge ids max*10000 + min in int (HeuristicPathPlanner.java:130, 267; PathPlanner.java:157,
   452) collide once an id is >= 10000: {10001, 20001} and {1, 20002} both give 200020001,
   a false head-on conflict and a non-optimal plan. Chain 0..20000 with leaves 20001 (on
   10001) and 20002 (on 1); the isomorphic graph with ids < 10000 is the control.
"""
import _common
from _common import edges_graph, show

from mppflow._java import MathRandom
from mppflow.heuristic import HeuristicPathPlanner, LocalRepairPlanner, exists_edge_collision
from mppflow.problem import get_long_straight_with_one_garage_problem

p = get_long_straight_with_one_garage_problem()
big = edges_graph(20003, [(i, i + 1) for i in range(20000)] + [(10001, 20001), (1, 20002)])
small = edges_graph(9003, [(i, i + 1) for i in range(9000)] + [(5001, 9001), (1, 9002)])
for c in ("original", "fixed"):
    for cls in (LocalRepairPlanner, HeuristicPathPlanner):
        pl = cls(compat=c, rng=MathRandom(1))
        show(f"{c}: {cls.__name__}.plan_heuristic_path (direct)",
             lambda: pl.plan_heuristic_path(p.graph, list(p.starts), p.goals, 5.0).status)
    show(f"{c}: zero robots", lambda: HeuristicPathPlanner(compat=c).plan_heuristic_paths(p.graph, [], []).paths)
    show(f"{c}: existsEdgeCollision([[10001, 20001], [1, 20002]], 0)", exists_edge_collision,
         [[10001, 20001], [1, 20002]], 0, compat=c)
    for g, s, t in ((big, [10001, 1], [20001, 20002]), (small, [5001, 1], [9001, 9002])):
        r = HeuristicPathPlanner(compat=c, rng=MathRandom(1)).plan_heuristic_paths(g, s, t)
        print(f"{c}: {s} -> {t}: makespan {r.objective}, {r.paths}")
