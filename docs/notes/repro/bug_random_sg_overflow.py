"""More agents (or holes) than vertices: getRandomStartGoal and createGeneric2DGridGraphWithHoles
loop forever, getRandomStartGoalMany throws IndexOutOfBoundsException.
"""
import _common
from _common import show

from mppflow._java import MathRandom
from mppflow.graph import (create_2d_grid_graph, create_generic_2d_grid_graph_with_holes,
                           get_random_start_goal, get_random_start_goal_many)

raw = create_2d_grid_graph(2, 2, False)  # 4 vertices, ids 1..4
conv = create_2d_grid_graph(2, 2, True)
for c in ("original", "fixed"):
    show(f"{c} getRandomStartGoal(2x2, 5)", get_random_start_goal, raw, 5, rng=MathRandom(1), compat=c)
    show(f"{c} getRandomStartGoalMany(2x2, 5)", get_random_start_goal_many, conv, 5, rng=MathRandom(1), compat=c)
    show(f"{c} holes(3x3, 150%)", create_generic_2d_grid_graph_with_holes, 3, 3, 1.5, rng=MathRandom(1), compat=c)
