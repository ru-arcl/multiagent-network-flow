"""mppflow: optimal multi-robot path planning on graphs via network flow and ILP.

A Python port of our original Java code for the paper J. Yu and S. M. LaValle, *Optimal
Multirobot Path Planning on Graphs: Complete Algorithms and Effective Heuristics*, IEEE T-RO
32(5), 2016.

The names below are the public API (see ``docs/API.md``); everything else lives in the
submodules (``mppflow.graph``, ``mppflow.problem``, ``mppflow.solvers``,
``mppflow.planner``, ``mppflow.heuristic``, ``mppflow.puzzle``, ``mppflow.lp``,
``mppflow.experiments``, ...). Importing ``mppflow`` is cheap: it loads neither numpy,
scipy nor gurobipy; a solver backend is imported on first use.

Quick start::

    import mppflow
    p = mppflow.create_grid_problem(8, 8, 0.1, 10, rng=mppflow.MathRandom(1))
    r = mppflow.plan(p, "makespan")            # HiGHS by default; backend="gurobi"
    assert mppflow.validate(p, r.paths) is None
    print(r.status, r.makespan, r.total_distance)
"""

from __future__ import annotations

__version__ = "0.1.0"

from . import errors
from ._java import MathRandom
from .errors import (
    InexactSolutionWarning,
    Infeasible,
    JavaException,
    MPPError,
    NotInOriginal,
    OriginalWouldHang,
    TimeLimitReached,
)
from .graph import (
    Graph,
    convert_graph,
    create_2d_grid_graph,
    create_2d_grid_graph_8connected,
    create_generic_2d_grid_graph_with_holes,
    create_generic_2d_grid_graph_with_holes_8connected,
    get_random_start_goal,
)
from .lp import available_backends
from .pathfinder import get_makespan_lower_bound, get_total_time_lower_bound
from .planner import (
    PathPlanner,
    PlanResult,
    has_cycles,
    is_path_set_valid,
    path_set_error,
    plan,
    plan_metrics,
    validate,
)
from .problem import (
    Problem,
    create_grid_problem,
    create_grid_problem_8connected,
    create_n2_puzzle,
    get_a9_puzzle,
    problem_to_txt,
    read_dat,
    read_txt,
    write_dat,
    write_txt,
)
from .solvers import (
    SolveResult,
    min_makespan,
    min_max_distance,
    min_total_distance,
    min_total_time,
)

from_adjacency = Graph.from_adjacency
"""Build a :class:`Graph` from neighbour lists (alias of :meth:`Graph.from_adjacency`)."""

__all__ = [
    "__version__",
    # data model
    "Graph",
    "Problem",
    "MathRandom",
    "from_adjacency",
    # generators
    "create_2d_grid_graph",
    "create_2d_grid_graph_8connected",
    "create_generic_2d_grid_graph_with_holes",
    "create_generic_2d_grid_graph_with_holes_8connected",
    "convert_graph",
    "get_random_start_goal",
    "create_grid_problem",
    "create_grid_problem_8connected",
    "create_n2_puzzle",
    "get_a9_puzzle",
    # file I/O
    "read_dat",
    "write_dat",
    "read_txt",
    "write_txt",
    "problem_to_txt",
    # optimal solvers
    "min_makespan",
    "min_total_time",
    "min_total_distance",
    "min_max_distance",
    "SolveResult",
    # high-level planning
    "plan",
    "PlanResult",
    "PathPlanner",
    "plan_metrics",
    "available_backends",
    # validation and bounds
    "validate",
    "is_path_set_valid",
    "path_set_error",
    "has_cycles",
    "get_makespan_lower_bound",
    "get_total_time_lower_bound",
    # errors
    "errors",
    "MPPError",
    "Infeasible",
    "TimeLimitReached",
    "NotInOriginal",
    "OriginalWouldHang",
    "JavaException",
    "InexactSolutionWarning",
]
