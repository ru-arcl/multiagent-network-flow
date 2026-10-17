"""PathPlanner.isPathSetValid / hasCycles / printPaths on empty, ragged or null input
(PathPlanner.java:40-41, 1152, 1186): paths[0] of an empty array throws, empty rows and a
longer second row pass the check, null throws NullPointerException.
"""
import _common
from _common import edges_graph, show

from mppflow.planner import has_cycles, is_path_set_valid, print_paths

star = edges_graph(4, [(0, 1), (1, 2), (1, 3)])
for c in ("original", "fixed"):
    show(f"{c}: isPathSetValid(int[0][0])", is_path_set_valid, [], star, [], [], compat=c)
    show(f"{c}: hasCycles(int[0][0])", has_cycles, [], compat=c)
    show(f"{c}: isPathSetValid(int[2][0], starts {{0,3}}, goals {{3,0}})", is_path_set_valid,
         [[], []], star, [0, 3], [3, 0], compat=c)
    show(f"{c}: isPathSetValid({{{{0,1}},{{3,3,1,0}}}}, goals {{1,0}})", is_path_set_valid,
         [[0, 1], [3, 3, 1, 0]], star, [0, 3], [1, 0], compat=c)
    show(f"{c}: printPaths(null)", print_paths, star, None, compat=c)
