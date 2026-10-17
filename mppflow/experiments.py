"""The experiment drivers of the original's ``ILP/Main.java``, its statistics writers, the
paper's figure/table mapping and seeded data regeneration.

Ports
-----

==========================================================  ====================================
mppflow (:class:`Experiments` methods)                      original (``Main``)
==========================================================  ====================================
``solve_problem_suboptimal``                                ``solveProblemSuboptimal`` (k-way split)
``solve_problem``                                           ``solveProblem`` (exact MINMAKESPAN)
``solve_problem_pmg``                                       ``solveProblemPMG`` (*dead*)
``solve_problem_total_time``                                ``solveProblemTotalTime``
``solve_problem_distance``                                  ``solveProblemDistance``
``solve_problem_distance_split``                            ``solveProblemDistanceSplit``
``solve_puzzle_problem``                                    ``solvePuzzleProblem``
``solve_problem_with_heuristic``                            ``solveProblemWithHeuristic``
``reset_stats``, ``reset_heuristic_stats``                  ``resetStats``, ``resetHeuristicStats``
``process_stats``, ``process_heuristic_stats``              ``processStats``, ``processHeuristicStats``
``write_stats``, ``write_heuristic_stats``                  ``writeStats``, ``writeHeuristicStats``
``write_puzzle_stats``, ``write_string``                    ``writePuzzleStats``, ``writeString``
``write_puzzle_result``                                     ``writePuzzleResult``
``run_*`` (18 drivers, see :data:`DRIVERS`)                 ``run*``
``main(argv)``                                              ``main(String[])`` (cases 0..6)
==========================================================  ====================================

Module functions: :func:`main` (a fresh :class:`Experiments` running ``Main.main``),
:func:`run` (one driver by name), :func:`generate` (``Problem.main``, seeded),
:func:`load_problem` (``.dat`` / ``.txt`` / ``.json``), the ``solve_*`` helpers as
one-shot functions, and :data:`FIGURES` (paper figure/table -> driver and parameters).

State, output and time
----------------------
One :class:`Experiments` instance stands for one run of the original JVM: it holds
``Main``'s static statistics fields (``current_test``, ``sub_total``, ``path_lengths``,
``total_time``, ``success_count``, ``goals_reached``, ``path_length``, ``exp_path_length``,
``finishing_time_set``, ``make_span_total``), the solvers' static flags (``flags``, a
:class:`~mppflow.planner.SolverFlags`; by default as ``Main.main`` leaves them: the Time
solver quiet, TimeR/TotalTime/Distance verbose, as in the original), the global
``PathFinder.bHeuristic`` (``b_heuristic``) and the ``Math.random()`` stream (``rng``).
Each ``solve_*`` creates a fresh :class:`~mppflow.planner.PathPlanner` on that stream, as
``Main`` creates a fresh ``PathPlanner``. Everything ``Main`` (and the planners it calls)
prints to ``System.out`` is collected in :attr:`Experiments.log` (``echo=True`` also prints
it); stack traces go to :attr:`Experiments.stderr` (first line only, plus the frame for
the two ``writeStats`` traces). Text written to the result files is also kept in
:attr:`Experiments.results` / :attr:`Experiments.puzzle_results`.

Times are wall clock (``clock``, seconds; ``System.currentTimeMillis()`` is
``floor(clock() * 1000)``), so the time columns of the statistics are machine dependent,
as in the original. Pass a fake ``clock`` for reproducible statistics (the planners, the
puzzle solver and the local-repair heuristic get the same clock, so with a clock that does
not advance no time limit binds). Every number that
does not depend on the clock (makespans, lower bounds, distances, success counts) is
reproducible from the seed.

Data files: drivers read ``<base_path>/<family>/<name>.dat``, the names of
:func:`mppflow.problem.generate_benchmark_files` (``24x18-grid/{n}-agt-{obs}-pct-obs-{id}``,
``8x8-grid/{a}-agts-{id}``, ``32x32-grid/20-pct-obs-{n}-agts-{id}`` ...; ``id`` = 1001...).
Results are appended to ``<results_dir>/n2puzzle-result.txt`` (every driver; the name is
the original's) and ``<results_dir>/25puzzle.txt`` (``write_puzzle_result``).

Reduced runs: every driver takes ``scale`` (:class:`Scale`: instances per row, a cap on
the robot count, a subset of obstacle percentages, of sections, and a time limit
override). Without ``scale`` the loops are exactly the original's.

compat modes
------------
``compat="original"`` reproduces ``Main`` exactly, crashes included:

* paths are built as ``basePath + "\\\\<dir>\\\\<file>"`` (on POSIX the backslashes are part of
  the file name, as for the original on Linux; :func:`generate` in ``original`` mode writes
  such names), the default ``base_path`` is ``"D:/temp/data/dmrpp/"``;
* a missing or corrupt data file prints its exception to :attr:`stderr` and yields a
  ``Problem`` without ``sg``; the next ``p.sg[0]`` raises
  :class:`~mppflow.errors.JavaNullPointerException` (original-bugs B24);
* ``solveProblemSuboptimal`` prints the paths before checking for ``null``: a failed
  plan raises ``JavaNullPointerException`` (original-bugs B21), so the
  drivers' ``stat == null`` branches never run after a split failure;
* ``solveProblemTotalTime`` / ``solveProblemDistance`` dereference a failed horizon split
  (``JavaNullPointerException``, original-bugs B22) and return ``null``
  for a non-positive time limit;
* ``writeStats`` takes the median/min/max over a ``TreeSet<Double>`` (equal times
  collapse; original-bugs B23); with no success it throws
  ``ArrayIndexOutOfBoundsException`` (caught: no line is written);
* ``solveProblemWithHeuristic`` reports ``paths[0].length`` (vertices) and both path
  totals one too small, and leaves ``PathFinder.bHeuristic`` set for the rest of the run
  (original-bugs B19; emulated on the
  instance's ``b_heuristic``, the process-wide flag is restored after each call);
* ``bReturnAnySolution`` stays set when a fixed-time driver throws;
* ``main`` case 5 falls through into case 6 (original-bugs B20); a non-integer
  case raises :class:`JavaNumberFormatException`; case 4 without a file name raises
  ``JavaArrayIndexOutOfBoundsException``;
* the legacy heuristic driver reads names without ``-agts-`` and labels its rows
  "100 runs"; ``run32x32PerformanceMetricSplitting8Connected`` reads text files as
  serialized data (original-bugs B24);
* the counters are Java ``int`` (32-bit wrap).

Differences of ``compat="fixed"`` (it equals the original wherever the original is
correct: same instances, same draws, same models, same stat vectors and lines):

* Files: ``os.path.join(base_path, dir, file)``; a missing or corrupt file raises
  (``FileNotFoundError`` / ``ValueError``); the result files go to ``results_dir``
  (default ``base_path``; with neither, they are only kept in memory) and I/O errors raise.
* ``solve_problem_suboptimal`` prints the paths only when there are some and returns
  ``None`` on failure, so the drivers take their intended failure branches.
* ``solve_problem_total_time`` / ``solve_problem_distance``: a failed horizon split
  returns ``None``; a non-positive time limit means no limit. Called directly (and through
  the one-shot module functions), they follow Algorithm 1 (``horizon="search"``, DESIGN
  section 4; ``docs/notes/paper-vs-code.md``, "TT and DIST are solved at one horizon"): the
  4-way-split makespan is only the first ``T`` of the search over ``[T, n * T_opt]``, so the
  result is optimal (within ``gap``) where the original's single horizon is not (e.g. TT 41
  / TD 33 instead of 45 / 37 on the corridor of ``tests/test_experiments.py``). Pass ``horizon="single"`` for the original's one horizon.
* Every ``solve_*`` returns ``None`` (and records why in :attr:`failures`) when the
  instance is infeasible (``fixed`` solvers raise :class:`~mppflow.errors.Infeasible`;
  the original loops forever).
* ``write_stats`` / ``write_heuristic_stats`` take the median/min/max over all times
  (a multiset; the median is the upper median ``sorted[n // 2]``, as the original's
  index); with no success the row is written with ``Total success: 0`` and ``NaN``
  fields instead of being dropped.
* ``solve_problem_with_heuristic`` returns ``[makespan, ms, goals, moves, lb]`` (the
  original: ``[makespan + 1, ms, goals, moves - 1, lb - 1]``) and does not leak
  ``PathFinder.bHeuristic`` (the flag affects only the heuristic's own searches).
* The fixed-time drivers restore ``bReturnAnySolution`` even when they throw.
* ``main``: case 5 no longer falls through into case 6; a bad case number (not an integer
  in 0..6) raises ``ValueError``; case 4 loads ``.dat``, ``.txt`` or ``.json`` files
  (:func:`load_problem`) and raises when the file is missing.
* Legacy drivers: ``run_32x32_performance_metric_heuristic`` reads the generator's
  ``-agts-`` names and labels its rows with the actual number of runs;
  ``run_32x32_performance_metric_splitting_8connected`` reads ``.dat`` files. Agent
  counts that the generator never produces (multiples of 25) still raise
  ``FileNotFoundError``.
* Counters do not wrap at 32 bits.
* All differences of the planners and solvers called (:mod:`mppflow.planner`,
  :mod:`mppflow.solvers`, :mod:`mppflow.puzzle`, :mod:`mppflow.heuristic`) apply.

Kept in both modes (they are the experiments' design, not errors): the robot counts,
instance ranges, time limits, gap formulas and section headers; the horizon of the total
time / total distance *drivers* (T-RO Figs. 18 and 20 and the unplotted TD sweep) is the
makespan of a 4-way split, as in the paper's experiments (T-RO Sec. V-B remark), unless
the constructor is given ``horizon="search"`` explicitly; the per-driver failure handling
(break before or after writing the partial row); the console strings (including the ones
that drop ``-agts``); the verbose TimeR/TotalTime/Distance solver messages; one results
file name for all experiments.

Not reproduced: wall-clock numbers (``multiThreadedSplitPlanning``'s 100 ms polling floor
is not emulated, the pieces run sequentially), Gurobi's own console log and the Java
stack-trace frames (only exception lines are kept).
"""
from __future__ import annotations

import contextlib
import io
import json
import math
import os
import re
import sys
import time
import warnings
from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

from ._java import MathRandom, i32, java_double_str, resolve_rng
from .errors import (
    Infeasible,
    JavaArrayIndexOutOfBoundsException,
    JavaException,
    JavaNullPointerException,
    JavaNumberFormatException,
)
from .graph import (
    Graph,
    check_compat,
    create_main_case3_graph,
    create_main_case6_tree,
    get_random_start_goal_many,
)
from .pathfinder import (
    B_HEURISTIC,
    find_shortest_paths,
    get_makespan_lower_bound,
    get_total_distance,
    get_total_time_lower_bound,
)
from .planner import (
    PathPlanner,
    SolverFlags,
    has_cycles,
    number_of_goals_reached,
    path_set_error,
    print_path_to_string,
    print_paths,
    print_start_and_goals,
)
from .problem import (
    BENCHMARK_FAMILIES,
    Problem,
    create_grid_problem,
    create_n2_puzzle,
    generate_benchmark_files,
    read_dat,
    read_txt,
)

__all__ = [
    "JavaNumberFormatException",
    "RESULT_FILE",
    "PUZZLE_RESULT_FILE",
    "DEFAULT_BASE_PATH",
    "Scale",
    "Experiments",
    "DRIVERS",
    "FIGURES",
    "Figure",
    "figures_text",
    "main",
    "run",
    "generate",
    "load_problem",
    "problem_from_json",
    "problem_to_json",
    "main_solver_flags",
    "solve_problem_suboptimal",
    "solve_problem",
    "solve_problem_pmg",
    "solve_problem_total_time",
    "solve_problem_distance",
    "solve_problem_distance_split",
    "solve_puzzle_problem",
    "solve_problem_with_heuristic",
]

RESULT_FILE = "n2puzzle-result.txt"
"""The file every driver appends its rows to (the original's name, for all experiments)."""

PUZZLE_RESULT_FILE = "25puzzle.txt"
"""The file :meth:`Experiments.write_puzzle_result` appends to (for every puzzle size)."""

DEFAULT_BASE_PATH = "D:/temp/data/dmrpp/"
"""The original's ``Main.basePath``."""

_NPE_SG = 'Cannot load from object array because "<parameter1>.sg" is null'
_NPE_SG_MAIN = 'Cannot load from object array because "<local2>.sg" is null'
_NPE_GRAPH = 'Cannot read field "adjacencySetMap" because "g" is null'
_NPE_PATHS0 = 'Cannot load from object array because "<local6>" is null'
_WRITE_STATS_FRAME = "\tat projects.multipath.ILP.Main.writeStats(Main.java:430)\n"
_WRITE_HSTATS_FRAME = "\tat projects.multipath.ILP.Main.writeHeuristicStats(Main.java:443)\n"


def _check_horizon(horizon: str | None, orig: bool) -> None:
    if horizon not in (None, "single", "search"):
        raise ValueError("horizon must be None, 'single' or 'search'")
    if horizon == "search" and orig:
        raise ValueError("horizon='search' is not in the original")


def main_solver_flags() -> SolverFlags:
    """The solver flags as ``Main.main`` leaves them: the declared defaults with
    ``MultiagentGraphSolverGurobiTime.bDebugInfo`` and ``bPrintPaths`` off."""
    f = SolverFlags.java_defaults()
    f.time_debug = False
    f.time_print_paths = False
    return f


def _jdiv(a: float, b: float) -> float:
    """Java ``double / double`` (no exception for a zero divisor)."""
    try:
        return a / b
    except ZeroDivisionError:
        if a == 0 or math.isnan(a):
            return math.nan
        return math.copysign(math.inf, a) * math.copysign(1.0, b)


def _jd(x: float) -> str:
    return java_double_str(float(x))


def _parse_int_java(s: str) -> int:
    """``Integer.parseInt(s)``."""
    if s is None or not re.fullmatch(r"[+-]?[0-9]+", s):
        raise JavaNumberFormatException(f'For input string: "{s}"' if s is not None else "Cannot parse null string: null")
    v = int(s)
    if v < -2 ** 31 or v > 2 ** 31 - 1:
        raise JavaNumberFormatException(f'For input string: "{s}"')
    return v


# --------------------------------------------------------------------------- scale
@dataclass(frozen=True)
class Scale:
    """A reduced version of a driver's sweep.

    Attributes:
        instances: instances per row (the original: 10, or 100 for the N^2 puzzles).
        max_agents: the largest robot count of every sweep (rows above it are skipped; for
            the N^2-puzzles, the sizes with more than ``max_agents`` tiles).
        obstacles: the obstacle percentages to run (24x18 drivers; others ignore it).
        sections: the sections to run (names in :data:`DRIVERS`, e.g. ``"exact"``,
            ``"4-way"``, ``"8x8/2-way-nc"``); ``None`` = all.
        time_limit: replaces every time limit of the driver (seconds).

    Rows are labelled with the actual number of instances when ``instances`` is set.
    """

    instances: int | None = None
    max_agents: int | None = None
    obstacles: tuple[int, ...] | None = None
    sections: tuple[str, ...] | None = None
    time_limit: float | None = None

    @classmethod
    def smoke(cls, **kw) -> "Scale":
        """One instance per row, at most 20 robots, a 60 s limit (overridable)."""
        d = {"instances": 1, "max_agents": 20, "time_limit": 60.0}
        d.update(kw)
        return cls(**d)


# ---------------------------------------------------------------------- problem I/O
def problem_to_json(p: Problem) -> dict:
    """``{"graph": Graph.to_dict(), "starts": [...], "goals": [...]}``."""
    return {"graph": p.graph.to_dict() if p.graph is not None else None,
            "starts": None if p.starts is None else list(p.starts),
            "goals": None if p.goals is None else list(p.goals)}


def _graph_from_dict(d) -> Graph:
    if isinstance(d, list):  # plain neighbour lists, ids 0..n-1
        return Graph.from_adjacency(d)
    verts = d.get("vertices")
    adj = d.get("adj")
    if adj is None:
        raise ValueError('JSON graph needs "adj"')
    g = Graph()
    if isinstance(adj, dict):  # Graph.to_dict()
        for k, p in (d.get("xy") or {}).items():
            g.xy[int(k)] = None if p is None else tuple(p)
        for k, nb in adj.items():
            g.adj[int(k)] = sorted(set(nb))
            g.xy.setdefault(int(k), None)
        g.vertices = [int(v) for v in verts] if verts is not None else sorted(g.xy)
    elif adj and isinstance(adj[0], list) and len(adj[0]) == 2 and isinstance(adj[0][1], list):
        # golden-record encoding: "adj": [[v, [nbrs]], ...], "vertices": [[id, x, y] | [id], ...]
        if verts is not None:
            for e in verts:
                e = e if isinstance(e, list) else [e]
                g.xy[e[0]] = (e[1], e[2]) if len(e) == 3 else None
            g.vertices = [(e[0] if isinstance(e, list) else e) for e in verts]
        for v, nb in adj:
            g.adj[v] = sorted(set(nb))
            g.xy.setdefault(v, None)
        if verts is None:
            g.vertices = sorted(g.xy)
    else:  # neighbour lists for ids 0..n-1
        xy = d.get("xy")
        if isinstance(xy, dict):
            xy = [xy.get(str(v)) for v in range(len(adj))]
        g = Graph.from_adjacency(adj, xy)
    if g.adj and not any(v in nb for v, nb in g.adj.items()):
        # the verbatim encodings: a hand-written file that lists only the real edges
        warnings.warn("no vertex of the JSON graph has a wait self-loop, so no robot can wait: list v in "
                      "its own adjacency (the to_dict and golden-record encodings are used as written) or "
                      "use plain neighbour lists, which add the self-loops", UserWarning, stacklevel=3)
    return g


def problem_from_json(d: dict) -> Problem:
    """A :class:`~mppflow.problem.Problem` from JSON: ``{"graph": G, "starts": [...],
    "goals": [...]}`` (or ``"sg": [starts, goals]``), for example
    ``{"graph": {"adj": [[1], [0, 2], [1]]}, "starts": [0], "goals": [2]}``. The output of
    ``mppflow solve --json --include-problem`` is accepted too (the problem is its
    ``"problem"`` member). ``G`` is one of:

    * plain neighbour lists for ids ``0..n-1`` (``{"adj": [[1], [0, 2], [1]], "xy": ...}``,
      or the bare list): list every edge in both directions; the wait self-loops are
      added, as :meth:`~mppflow.graph.Graph.from_adjacency` does. Use this form for
      hand-written files.
    * :meth:`~mppflow.graph.Graph.to_dict` output, or the golden-record encoding
      (``{"vertices": [[id, x, y], ...], "adj": [[v, [nbrs]], ...]}``): the adjacency is
      used **as written**, so a robot can wait at ``v`` only if ``v`` is listed among its
      own neighbours (``Graph.adj`` includes the vertex itself). These encodings can thus
      describe graphs without wait loops; a ``UserWarning`` is issued when no vertex has
      one (typically a hand-written file that lists only the real edges)."""
    if not isinstance(d, dict):
        raise ValueError(f"JSON problem must be an object, not {type(d).__name__}")
    if "graph" not in d and isinstance(d.get("problem"), dict) and "graph" in d["problem"]:
        d = d["problem"]  # solve --json --include-problem
    if "graph" not in d:
        raise ValueError('JSON problem needs a "graph", e.g. {"graph": {"adj": [[1], [0, 2], [1]]}, '
                         '"starts": [0], "goals": [2]}')
    g = _graph_from_dict(d["graph"])
    if "sg" in d:
        starts, goals = d["sg"]
    else:
        starts, goals = d.get("starts"), d.get("goals")
    if starts is None or goals is None:
        raise ValueError('JSON problem needs "starts" and "goals" (or "sg")')
    return Problem(g, starts, goals)


def load_problem(path: str, compat: str = "fixed") -> Problem:
    """Read a problem: ``.txt`` (:func:`~mppflow.problem.read_txt`), ``.json``
    (:func:`problem_from_json`), anything else as the original's serialized ``.dat``
    (:func:`~mppflow.problem.read_dat`, with ``compat``)."""
    ext = os.path.splitext(path)[1].lower()
    if ext == ".txt":
        return read_txt(path)
    if ext == ".json":
        with open(path, encoding="utf-8") as f:
            return problem_from_json(json.load(f))
    return read_dat(path, compat)


# ---------------------------------------------------------------------- experiments
class Experiments:
    """``Main``'s static state and methods (one instance = one run of the original JVM).

    Args:
        base_path: the data folder (``Main.basePath``). ``original`` default
            ``"D:/temp/data/dmrpp/"`` (paths are concatenated with backslashes, so end it
            with ``/``); ``fixed``: required by the drivers, joined with ``os.path.join``.
        compat: ``"fixed"`` or ``"original"`` (see the module docstring).
        backend, params: LP backend of every solve (``"gurobi"`` with ``{"Threads": 1}``
            reproduces the original's plans).
        rng: the ``Math.random()`` stream: a :class:`~mppflow._java.MathRandom`, a seed, or
            ``None`` (the module stream).
        flags: the solvers' static flags (default :func:`main_solver_flags`); shared by
            every planner the instance creates, as statics are.
        clock: wall clock in seconds (default ``time.time``); also the clock of every
            planner, puzzle solver and heuristic run the instance starts.
        echo: also print the log as it is produced.
        results_dir: ``fixed``: the folder of the result files (default ``base_path``;
            with neither, results are only kept in :attr:`results`).
        results_file, puzzle_results_file: the result file names.
        scale: default :class:`Scale` of the drivers (each driver also takes one).
        horizon: horizon rule of the total time / total distance solves. ``"single"``:
            one solve at the 4-way-split makespan (the original); ``"search"``: Algorithm 1
            from that makespan up to ``n * T_opt`` (``fixed`` only); ``None`` (default):
            ``original`` always ``"single"``; ``fixed`` ``"search"`` for direct
            :meth:`solve_problem_total_time` / :meth:`solve_problem_distance` calls and
            ``"single"`` inside the drivers (the paper's experimental setup). An explicit
            value applies to both.
        planner_options: extra :class:`~mppflow.planner.PathPlanner` arguments.
        heuristic_options: extra :func:`~mppflow.heuristic.plan_heuristic_paths` arguments
            (e.g. ``max_subproblems``; ``clock`` defaults to the instance's ``clock``).
        puzzle_options: extra :class:`~mppflow.puzzle.PuzzleSolver` arguments.
    """

    def __init__(self, base_path: str | None = None, *, compat: str = "fixed", backend: str = "highs",
                 params: dict | None = None, rng: MathRandom | int | None = None, flags: SolverFlags | None = None,
                 clock: Callable[[], float] | None = None, echo: bool = False, results_dir: str | None = None,
                 results_file: str = RESULT_FILE, puzzle_results_file: str = PUZZLE_RESULT_FILE,
                 scale: Scale | None = None, horizon: str | None = None, planner_options: dict | None = None,
                 heuristic_options: dict | None = None, puzzle_options: dict | None = None):
        self.orig = check_compat(compat)
        self.compat = compat
        if base_path is None and self.orig:
            base_path = DEFAULT_BASE_PATH
        self.base_path = base_path
        self.results_dir = results_dir
        self.results_file = results_file
        self.puzzle_results_file = puzzle_results_file
        self.backend = backend
        self.params = params
        self.rng = MathRandom(rng) if isinstance(rng, int) else resolve_rng(rng)
        self.flags = flags if flags is not None else main_solver_flags()
        self.clock = clock or time.time
        self.echo = echo
        self.scale = scale
        _check_horizon(horizon, self.orig)
        self.horizon = horizon
        self._driver_hz: str | None = None  # set while a TT/TD driver runs
        self.planner_options = dict(planner_options or {})
        self.heuristic_options = dict(heuristic_options or {})
        self.puzzle_options = dict(puzzle_options or {})
        # Main's statics
        self.current_test: str | None = None
        self.sub_total = 0.0
        self.path_lengths = 0
        self.solved_instances = 0
        self.total_time = 0.0
        self.success_count = 0
        self.goals_reached = 0
        self.heuristic_runs = 0
        """``fixed``: the runs counted by :meth:`process_heuristic_stats` (not in the original)."""
        self.path_length = 0
        self.exp_path_length = 0
        self.finishing_time_set: list[float] = []
        """``finishingTimeSet``: sorted; ``original``: distinct values (a ``TreeSet``)."""
        self.make_span_total = 0
        self.b_heuristic = False
        """``PathFinder.bHeuristic`` as seen by this run (``original`` only)."""
        self.failures: list[str] = []
        """``fixed``: why ``solve_*`` calls returned ``None``."""
        self.planners: list[PathPlanner] = []
        """The planners created (the last few; for inspection)."""
        self.last_paths: list[list[int]] | None = None
        """The plan of the last ``solve_*`` call (``None`` if it failed)."""
        self._log: list[str] = []
        self._err: list[str] = []
        self._files: dict[str, list[str]] = {}
        self._depth = 0

    # ------------------------------------------------------------------ output
    @property
    def log(self) -> str:
        """Everything the original prints to ``System.out``."""
        return "".join(self._log)

    @property
    def stderr(self) -> str:
        """The exception lines the original prints to ``System.err``."""
        return "".join(self._err)

    @property
    def results(self) -> str:
        """The text appended to the results file (``n2puzzle-result.txt``) by this instance."""
        return "".join(self._files.get(self.results_file, []))

    @property
    def puzzle_results(self) -> str:
        """The text appended to ``25puzzle.txt`` by this instance."""
        return "".join(self._files.get(self.puzzle_results_file, []))

    def clear_log(self) -> None:
        self._log.clear()
        self._err.clear()

    def _print(self, s: str, echoed: bool = False) -> None:
        if not s:
            return
        self._log.append(s)
        if self.echo and not echoed:
            sys.stdout.write(s)

    def _println(self, s: str = "") -> None:
        self._print(s + "\n")

    def _eprint(self, s: str, echoed: bool = False) -> None:
        if not s:
            return
        self._err.append(s)
        if self.echo and not echoed:
            sys.stderr.write(s)

    def _trace(self, e: BaseException) -> None:
        cls = getattr(e, "java_class", type(e).__name__)
        self._eprint(f"{cls}: {e}\n")

    def _millis(self) -> int:
        return int(math.floor(self.clock() * 1000))

    # ------------------------------------------------------------------ statics
    @contextlib.contextmanager
    def _jvm(self):
        """``original``: the run's ``PathFinder.bHeuristic`` is the process-wide flag
        during the call (restored afterwards)."""
        if not self.orig:
            yield
            return
        outer = self._depth == 0
        saved = B_HEURISTIC.value
        if outer:
            B_HEURISTIC.set(self.b_heuristic)
        self._depth += 1
        try:
            yield
        finally:
            self._depth -= 1
            if outer:
                self.b_heuristic = B_HEURISTIC.value
                B_HEURISTIC.set(saved)

    def _planner(self) -> PathPlanner:
        pl = PathPlanner(backend=self.backend, params=self.params, rng=self.rng, compat=self.compat,
                         flags=self.flags, echo=self.echo, clock=self.clock, **self.planner_options)
        self.planners.append(pl)
        del self.planners[:-4]
        return pl

    @contextlib.contextmanager
    def _absorb(self, pl: PathPlanner):
        n_log, n_err = len(pl._log), len(pl._err)
        try:
            yield pl
        finally:
            self._print("".join(pl._log[n_log:]), echoed=True)
            self._eprint("".join(pl._err[n_err:]), echoed=True)

    def _check_problem(self, p: Problem | None, msg: str = _NPE_SG) -> None:
        if p is None:
            if self.orig:
                raise JavaNullPointerException('Cannot read field "sg" because "<parameter1>" is null')
            raise ValueError("problem is None")
        if p.starts is None or p.goals is None:
            if self.orig:
                raise JavaNullPointerException(msg)
            raise ValueError("the problem has no starts/goals")
        if p.graph is None:
            if self.orig:
                raise JavaNullPointerException(_NPE_GRAPH)
            raise ValueError("the problem has no graph")

    def _fail(self, what: str, e: BaseException) -> None:
        self.failures.append(f"{what}: {e}")

    def _cycles(self, paths) -> None:
        c = has_cycles(paths, self.compat)
        if c > 0:
            self._println(f"\nThere are {c} cycles in the result paths")

    def _valid(self, paths, p: Problem) -> bool:
        err = path_set_error(paths, p.graph, p.starts, p.goals, self.compat)
        if err is not None:
            self._println(err)
        return err is None

    # ------------------------------------------------------------------ solve*
    def solve_problem_suboptimal(self, p: Problem, solve_lp: bool = False, set_optimal: bool = True,
                                 extra_steps: int = 0, time_limit: float = -1.0, split_level: int = 0,
                                 no_cycle: bool = False) -> list[int] | None:
        """``solveProblemSuboptimal``: the k-way split (``2^split_level`` pieces).

        Order: the makespan lower bound (draws), then
        :meth:`~mppflow.planner.PathPlanner.plan_paths_advanced_split_no_cycle` (``no_cycle``,
        with ``set_optimal``, ``top = true``) or
        :meth:`~mppflow.planner.PathPlanner.multi_threaded_split_planning` (``set_optimal``
        is not passed: hard-coded ``true``), then :func:`~mppflow.planner.print_paths`, then
        the validity check. ``solve_lp`` and ``extra_steps`` are unused, as in the original.
        Returns ``[makespan, ms, 0, makespan_lb]`` (``ms`` includes the printing, not the
        lower bound), or ``None``.

        ``original``: a failed plan raises :class:`JavaNullPointerException` (paths printed
        before the ``null`` check). Differs from the original: ``fixed`` returns ``None``.
        """
        with self._jvm():
            self._check_problem(p)
            g, s, goal = p.graph, list(p.starts), list(p.goals)
            pl = self._planner()
            try:
                lb = get_makespan_lower_bound(g, s, goal, self.rng, compat=self.compat)
                t0 = self._millis()
                with self._absorb(pl):
                    if no_cycle:
                        paths = pl.plan_paths_advanced_split_no_cycle(g, s, goal, set_optimal, time_limit,
                                                                      split_level, True)
                    else:
                        paths = pl.multi_threaded_split_planning(g, s, goal, time_limit, split_level)
            except Infeasible as e:
                if self.orig:  # pragma: no cover - original solvers never raise Infeasible
                    raise
                self._fail("solveProblemSuboptimal", e)
                return None
            if self.orig:
                try:
                    self._print(print_paths(g, paths, self.compat))
                except JavaException as e:
                    self._print(getattr(e, "output", "") or "")
                    raise
            elif paths is not None:
                self._print(print_paths(g, paths, self.compat))
            elapsed = self._millis() - t0
            self.last_paths = paths
            if paths is not None and self._valid(paths, p):
                self._cycles(paths)
                return [len(paths[0]) - 1, elapsed, 0, lb - 1]
            if not self.orig:
                why = "no plan" if paths is None else "invalid plan"
                self._fail("solveProblemSuboptimal", why + (f" ({'; '.join(pl.failures)})" if pl.failures else ""))
            return None

    def _exact(self, method: str, p: Problem, solve_lp: bool, time_limit: float, print_plan: bool):
        with self._jvm():
            self._check_problem(p)
            g, s, goal = p.graph, list(p.starts), list(p.goals)
            pl = self._planner()
            t0 = self._millis()
            try:
                with self._absorb(pl):
                    paths = getattr(pl, method)(g, s, goal, solve_lp, 0, True, time_limit)
                elapsed = self._millis() - t0
                lb = get_makespan_lower_bound(g, s, goal, self.rng, compat=self.compat)
            except Infeasible as e:
                if self.orig:  # pragma: no cover
                    raise
                self._fail(method, e)
                return None
            self.last_paths = paths
            if paths is None:
                if not self.orig:
                    self._fail(method, "no plan")
                return None
            self._cycles(paths)
            if print_plan:
                self._print(print_paths(g, paths, self.compat))
            return [len(paths[0]) - 1, elapsed, get_total_distance(paths), lb - 1]

    def solve_problem(self, p: Problem, solve_lp: bool = True, time_limit: float = -1.0) -> list[int] | None:
        """``solveProblem``: exact MINMAKESPAN
        (:meth:`~mppflow.planner.PathPlanner.plan_paths_advanced` with ``solve_lp``,
        ``set_optimal = true``; ``time_limit <= 0``: no limit). The lower bound is computed
        **after** the solve (draws). Prints the cycle count and the paths. Returns
        ``[makespan, ms, total_distance, makespan_lb]`` or ``None``."""
        return self._exact("plan_paths_advanced", p, solve_lp, time_limit, True)

    def solve_problem_pmg(self, p: Problem, solve_lp: bool = True, time_limit: float = -1.0) -> list[int] | None:
        """``solveProblemPMG`` (*dead*): as :meth:`solve_problem` with the one-move-per-step
        model (:meth:`~mppflow.planner.PathPlanner.plan_paths_advanced_pmg`), without
        printing the paths."""
        return self._exact("plan_paths_advanced_pmg", p, solve_lp, time_limit, False)

    def _resolve_horizon(self, horizon: str | None) -> str:
        """The TT/TD horizon rule: an explicit ``horizon`` argument, else the running
        driver's (:meth:`_driver_horizon`), else the constructor's, else ``"search"``
        (``fixed``); always ``"single"`` in ``original``."""
        _check_horizon(horizon, self.orig)
        if self.orig:
            return "single"
        return horizon or self._driver_hz or self.horizon or "search"

    @contextlib.contextmanager
    def _driver_horizon(self):
        """While a total time / total distance driver runs, its solves use the paper's
        4-way-split horizon unless the constructor chose a horizon explicitly."""
        prev = self._driver_hz
        self._driver_hz = self.horizon or "single"
        try:
            yield
        finally:
            self._driver_hz = prev

    def _horizon_then(self, kind: str, p: Problem, gap: float, time_limit: float, horizon: str | None = None):
        hz = self._resolve_horizon(horizon)
        with self._jvm():
            self._check_problem(p)
            g, s, goal = p.graph, list(p.starts), list(p.goals)
            pl = self._planner()
            t0 = self._millis()
            opt = [0]
            try:
                with self._absorb(pl):
                    paths = pl.plan_paths_advanced_split(g, s, goal, False, -1.0, 2)
                    tl = time_limit - (self._millis() - t0) / 1000.0
                    if self.orig:
                        if tl <= 0:
                            return None
                        if paths is None:
                            raise JavaNullPointerException(_NPE_PATHS0)
                    else:
                        if time_limit > 0 and tl <= 0:
                            self._fail(kind, "time limit reached by the horizon split")
                            return None
                        if time_limit <= 0:
                            tl = -1.0
                        if paths is None:
                            self._fail(kind, "the 4-way split (horizon) failed"
                                       + (f" ({'; '.join(pl.failures)})" if pl.failures else ""))
                            return None
                    T = len(_row0(paths, self.orig)) - 1
                    kw = {} if self.orig else {"horizon": hz}
                    if kind == "tt":
                        paths = pl.plan_paths_advanced_tt(g, s, goal, opt, gap, T, tl, **kw)
                    else:
                        paths = pl.plan_paths_advanced_td(g, s, goal, gap, T, tl, **kw)
                elapsed = self._millis() - t0
                lb = get_total_time_lower_bound(g, s, goal, self.rng, compat=self.compat)
            except Infeasible as e:
                if self.orig:  # pragma: no cover
                    raise
                self._fail(kind, e)
                return None
            self.last_paths = paths
            if paths is None:
                if not self.orig:
                    self._fail(kind, "no plan")
                return None
            self._cycles(paths)
            d = get_total_distance(paths)
            return [opt[0] if kind == "tt" else d, elapsed, d, lb]

    def solve_problem_total_time(self, p: Problem, gap: float = 0.0, time_limit: float = -1.0, *,
                                 horizon: str | None = None) -> list[int] | None:
        """``solveProblemTotalTime``: horizon ``T`` = makespan of the sequential 4-way split
        :meth:`~mppflow.planner.PathPlanner.plan_paths_advanced_split` ``(false, -1, 2)``,
        the elapsed time is subtracted from ``time_limit``, then MINTOTALTIME at ``T``
        (:meth:`~mppflow.planner.PathPlanner.plan_paths_advanced_tt`, MIP gap ``gap``).
        The total-time lower bound is computed after the solves. Returns
        ``[total_time, ms (incl. the split), total_distance, tt_lb]`` or ``None``.

        ``horizon`` (``fixed`` only; default: see the constructor's ``horizon``):
        ``"single"`` solves at ``T`` only, ``"search"`` runs Algorithm 1 from ``T`` up to
        ``n * T_opt`` (:func:`~mppflow.solvers.solve_total_time`; ``time_limit`` is the
        budget of the whole search).

        ``original``: ``None`` when the remaining limit is ``<= 0`` (also for a
        non-positive ``time_limit``); a failed split raises :class:`JavaNullPointerException`.
        Differs from the original: ``fixed`` treats ``time_limit <= 0`` as no limit and
        returns ``None`` for a failed split; called outside a driver it searches the horizon
        (Algorithm 1, DESIGN section 4) instead of solving only at ``T``, so it returns the
        optimum where the single horizon is too short (``docs/notes/paper-vs-code.md``, "TT and
        DIST are solved at one horizon").
        """
        return self._horizon_then("tt", p, gap, time_limit, horizon)

    def solve_problem_distance(self, p: Problem, gap: float = 0.0, time_limit: float = -1.0, *,
                               horizon: str | None = None) -> list[int] | None:
        """``solveProblemDistance``: as :meth:`solve_problem_total_time` with MINTOTALDIST
        (:meth:`~mppflow.planner.PathPlanner.plan_paths_advanced_td`). Returns
        ``[total_distance, ms, total_distance, tt_lb]`` or ``None`` (same ``horizon`` rule
        and the same differences from the original, including the horizon search)."""
        return self._horizon_then("td", p, gap, time_limit, horizon)

    def solve_problem_distance_split(self, p: Problem, gap: float = 0.0, time_limit: float = -1.0,
                                     splits: int = 1) -> list[int] | None:
        """``solveProblemDistanceSplit``: the split MINTOTALDIST
        (:meth:`~mppflow.planner.PathPlanner.plan_paths_advanced_split_dist`), then the
        lower bound (draws), the cycle count and the paths. Returns
        ``[total_distance, ms, total_distance, tt_lb]`` or ``None``."""
        with self._jvm():
            self._check_problem(p)
            g, s, goal = p.graph, list(p.starts), list(p.goals)
            pl = self._planner()
            t0 = self._millis()
            try:
                with self._absorb(pl):
                    paths = pl.plan_paths_advanced_split_dist(g, s, goal, gap, time_limit, splits)
                elapsed = self._millis() - t0
                lb = get_total_time_lower_bound(g, s, goal, self.rng, compat=self.compat)
            except Infeasible as e:
                if self.orig:  # pragma: no cover
                    raise
                self._fail("solveProblemDistanceSplit", e)
                return None
            self.last_paths = paths
            if paths is None:
                if not self.orig:
                    self._fail("solveProblemDistanceSplit", "no plan"
                               + (f" ({'; '.join(pl.failures)})" if pl.failures else ""))
                return None
            self._cycles(paths)
            self._print(print_paths(g, paths, self.compat))
            d = get_total_distance(paths)
            return [d, elapsed, d, lb]

    def solve_puzzle_problem(self, instance: int, p: Problem, time_limit: float = -1.0) -> list[int] | None:
        """``solvePuzzleProblem``: ``PuzzleSolver.solve(p, timeLimit)``
        (:func:`mppflow.puzzle.solve_problem`: the two-model race for more than 16 robots,
        else MINMAKESPAN without the LP stage), the lower bound after it, then on success
        :meth:`write_puzzle_result` and the paths. Returns
        ``[makespan, ms, total_distance, makespan_lb]`` or ``None``."""
        from .puzzle import solve_problem as _puzzle_solve

        with self._jvm():
            t0 = self._millis()
            try:
                res = _puzzle_solve(p, time_limit, compat=self.compat, backend=self.backend, params=self.params,
                                    rng=self.rng, clock=self.clock, debug=self.flags.time_debug,
                                    print_paths=self.flags.time_print_paths, echo=self.echo, **self.puzzle_options)
            except Infeasible as e:
                if self.orig:  # pragma: no cover
                    raise
                self._fail("solvePuzzleProblem", e)
                return None
            except Exception as e:
                self._print(getattr(e, "output", "") or "", echoed=True)
                raise
            self._print(res.log, echoed=True)
            paths = res.paths
            self.last_paths = paths
            elapsed = self._millis() - t0
            self._check_problem(p)
            lb = get_makespan_lower_bound(p.graph, p.starts, p.goals, self.rng, compat=self.compat)
            if paths is None:
                if not self.orig:
                    self._fail("solvePuzzleProblem", f"no plan ({res.status})")
                return None
            self.write_puzzle_result(instance, float(elapsed), p.graph, paths)
            self._print(print_paths(p.graph, paths, self.compat))
            return [len(paths[0]) - 1, elapsed, get_total_distance(paths), lb - 1]

    def solve_problem_with_heuristic(self, p: Problem) -> list[int] | None:
        """``solveProblemWithHeuristic``: the local-repair heuristic
        (:func:`mppflow.heuristic.plan_heuristic_paths`) with ``PathFinder.bHeuristic``
        set; prints the starts and goals first and "Path set is INVALID." for an invalid
        plan. Returns ``[makespan, ms, goals_reached, moves, moves_lb]`` (the last two 0
        unless every robot reached its goal) or ``None``.

        ``original``: ``[makespan + 1, ms, goals, moves - 1, moves_lb - 1]`` and
        ``bHeuristic`` stays set for the rest of the run (``self.b_heuristic``). Differs from
        the original: ``fixed`` reports the true values and leaks nothing.
        """
        from .heuristic import plan_heuristic_paths

        with self._jvm():
            if self.orig:
                B_HEURISTIC.set(True)
            self._check_problem(p)
            g, s, goal = p.graph, list(p.starts), list(p.goals)
            t0 = self._millis()
            self._print(print_start_and_goals(g, s, goal, self.compat))
            kw = dict(self.heuristic_options)
            # The heuristic runs on the instance's clock, as the planners and the puzzle
            # solver do (its 3 s attempt budgets and the sub-solvers' remaining time are
            # wall clock in the original); heuristic_options["clock"] overrides it.
            kw.setdefault("clock", self.clock)
            try:
                res = plan_heuristic_paths(g, s, goal, compat=self.compat, backend=self.backend, params=self.params,
                                           rng=self.rng, heuristic=None if self.orig else True, echo=self.echo, **kw)
            except Infeasible as e:
                if self.orig:  # pragma: no cover
                    raise
                self._fail("solveProblemWithHeuristic", e)
                return None
            except Exception as e:
                self._print(getattr(e, "output", "") or "", echoed=True)
                raise
            self._print(res.log, echoed=True)
            if self.orig and res.extra.get("statics_after"):
                self.flags.apply_java(res.extra["statics_after"])
            paths = res.paths
            self.last_paths = paths
            elapsed = self._millis() - t0
            if paths is None:
                if not self.orig:
                    self._fail("solveProblemWithHeuristic", f"no plan ({res.status})")
                return None
            reached = number_of_goals_reached(paths, goal, self.compat)
            err = path_set_error(paths, g, s, goal, self.compat)
            if err is not None:
                self._println(err)
                self._println("Path set is INVALID.")
            if self.orig:
                B_HEURISTIC.set(False)
                hp = find_shortest_paths(g, s, goal, self.rng, compat=self.compat)
                B_HEURISTIC.set(True)
            else:
                hp = find_shortest_paths(g, s, goal, self.rng, heuristic=False, compat=self.compat)
            least = 0
            for a in range(len(s)):
                if hp[a] is None:
                    if self.orig:
                        raise JavaNullPointerException('Cannot read field "vertexList" because "hp[a]" is null')
                    raise Infeasible(f"goal of robot {a} is unreachable")  # pragma: no cover
                least += len(hp[a]) - 1
            length = get_total_distance(paths)
            done = reached == len(s)
            if self.orig:
                return [len(paths[0]), elapsed, reached, (length - 1) if done else 0, (least - 1) if done else 0]
            return [len(paths[0]) - 1 if paths else 0, elapsed, reached, length if done else 0, least if done else 0]

    # ------------------------------------------------------------------ stats
    def reset_stats(self, test: str) -> None:
        """``resetStats(test)``: the row label and the row counters (not ``total_time``)."""
        self.current_test = test
        self.sub_total = 0.0
        self.path_lengths = 0
        self.solved_instances = 0
        self.success_count = 0
        self.make_span_total = 0
        self.finishing_time_set = []

    def reset_heuristic_stats(self, test: str) -> None:
        """``resetHeuristicStats(test)``: as :meth:`reset_stats` plus the heuristic counters,
        but not ``make_span_total``."""
        self.current_test = test
        self.goals_reached = 0
        self.sub_total = 0.0
        self.path_lengths = 0
        self.solved_instances = 0
        self.success_count = 0
        self.path_length = 0
        self.exp_path_length = 0
        self.finishing_time_set = []
        self.heuristic_runs = 0

    def _add_time(self, t: float) -> None:
        ft = self.finishing_time_set
        if self.orig and t in ft:  # TreeSet<Double>: duplicates collapse
            return
        lo, hi = 0, len(ft)
        while lo < hi:
            mid = (lo + hi) // 2
            if ft[mid] <= t:
                lo = mid + 1
            else:
                hi = mid
        ft.insert(lo, t)

    def _int(self, x: int) -> int:
        return i32(x) if self.orig else x

    def process_stats(self, stat: Sequence[int] | None) -> None:
        """``processStats(stat)``: count a success (``None`` is ignored), print
        "Time spent = X seconds." and "Total time spent: Y seconds."."""
        if stat is None:
            return
        length = i32(int(stat[0])) if self.orig else int(stat[0])
        t = int(stat[1])
        sec = t / 1000.0
        self.success_count += 1
        self.path_lengths = self._int(self.path_lengths + length)
        self._add_time(sec)
        self._println(f"Time spent = {_jd(sec)} seconds.")
        self.sub_total += sec
        self.total_time += sec
        self.make_span_total = self._int(self.make_span_total + int(stat[3]))
        self._println(f"Total time spent: {_jd(self.total_time)} seconds.\n")

    def process_heuristic_stats(self, stat: Sequence[int] | None, n: int) -> None:
        """``processHeuristicStats(stat, n)``: a success is ``stat[2] == n`` (every goal
        reached); prints "Goals reached = G" and the times."""
        if stat is None:
            return
        length = i32(int(stat[0])) if self.orig else int(stat[0])
        t = int(stat[1])
        sec = t / 1000.0
        self.path_lengths = self._int(self.path_lengths + length)
        self._add_time(sec)
        self.goals_reached += int(stat[2])
        self.heuristic_runs += 1
        self._println(f"Goals reached = {self.goals_reached}")
        if int(stat[2]) == n:
            self.success_count += 1
        self._println(f"Time spent = {_jd(sec)} seconds.")
        self.sub_total += sec
        self.total_time += sec
        self._println(f"Total time spent: {_jd(self.total_time)} seconds.\n")
        self.path_length += int(stat[3])
        self.exp_path_length += int(stat[4])

    def _ft(self, frame: str):
        ft = self.finishing_time_set
        if not ft:
            if self.orig:
                self._eprint("java.lang.ArrayIndexOutOfBoundsException: Index 0 out of bounds for length 0\n" + frame)
                return None
            return ("NaN", "NaN", "NaN")
        return (_jd(ft[len(ft) // 2]), _jd(ft[0]), _jd(ft[-1]))

    def _label(self) -> str:
        return "null" if self.current_test is None else self.current_test

    def write_stats(self) -> str | None:
        """``writeStats()``: append ``"<label>Total success: S, ave time: A, median:M, min:
        m, max: X, ave solution: L, min possible: B\\n"`` to the results file (Java
        ``Double.toString`` numbers; averages over the successes). Returns the line (``None``
        when nothing is written).

        ``original``: median/min/max over the distinct times; no success -> the
        ``ArrayIndexOutOfBoundsException`` is printed and nothing written. Differs from the
        original: ``fixed`` uses all times and writes a ``NaN`` row for no success.
        """
        ave = _jdiv(self.sub_total, self.success_count)
        r = self._ft(_WRITE_STATS_FRAME)
        if r is None:
            return None
        med, mn, mx = r
        line = (f"{self._label()}Total success: {self.success_count}, ave time: {_jd(ave)}, median:{med}, "
                f"min: {mn}, max: {mx}, ave solution: {_jd(_jdiv(self.path_lengths * 1.0, self.success_count))}, "
                f"min possible: {_jd(_jdiv(self.make_span_total * 1.0, self.success_count))}\n")
        self.write_string(line)
        return line

    def write_heuristic_stats(self) -> str | None:
        """``writeHeuristicStats()``: ``"<label>Total success: S, goals reached: G, ave time:
        A, median:M, min: m, max: X, steps: P, exp: E, ratio: P/E\\n"`` (same differences
        as :meth:`write_stats`).

        ``original``: ``ave time`` is the time of **all** runs (partial ones included) divided
        by the number of successes, so a row with partial runs overstates it (original-bugs
        B23). ``fixed``: the mean over all runs, the same runs as the median, min and max."""
        ave = _jdiv(self.sub_total, self.success_count if self.orig else self.heuristic_runs)
        r = self._ft(_WRITE_HSTATS_FRAME)
        if r is None:
            return None
        med, mn, mx = r
        line = (f"{self._label()}Total success: {self.success_count}, goals reached: {self.goals_reached}, "
                f"ave time: {_jd(ave)}, median:{med}, min: {mn}, max: {mx}, steps: {self.path_length}, "
                f"exp: {self.exp_path_length}, ratio: {_jd(_jdiv(self.path_length * 1.0, self.exp_path_length))}\n")
        self._append(self.results_file, line)
        return line

    def write_puzzle_stats(self, instance: int, time_s: float) -> None:
        """``writePuzzleStats(instance, time)``: ``"Instance: i, time: t\\n"``."""
        self._append(self.results_file, f"Instance: {instance}, time: {_jd(time_s)}\n")

    def write_string(self, s: str) -> None:
        """``writeString(str)``: append ``s`` to the results file."""
        self._append(self.results_file, s)

    def write_puzzle_result(self, instance: int, time_ms: float, g: Graph, paths) -> None:
        """``writePuzzleResult(instance, time, g, paths)``: ``"Instance: <instance+1>,
        time: <time/1000>\\n"``, :func:`~mppflow.planner.print_path_to_string` and a newline,
        appended to ``25puzzle.txt``."""
        text = (f"Instance: {instance + 1}, time: {_jd(float(time_ms) / 1000)}\n"
                + print_path_to_string(g, paths, self.compat) + "\n")
        self._append(self.puzzle_results_file, text)

    def _result_path(self, name: str) -> str | None:
        if self.orig:
            return self.base_path + "\\" + name
        folder = self.results_dir if self.results_dir is not None else self.base_path
        return None if folder is None else os.path.join(folder, name)

    def _append(self, name: str, text: str) -> None:
        path = self._result_path(name)
        if self.orig:
            try:
                with open(path, "a", encoding="utf-8", newline="") as f:
                    f.write(text)
            except OSError as e:
                self._eprint(f"java.io.FileNotFoundException: {path} ({e.strerror})\n")
                return
        elif path is not None:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "a", encoding="utf-8", newline="") as f:
                f.write(text)
        self._files.setdefault(name, []).append(text)

    # ------------------------------------------------------------------ data
    def _data_path(self, sub: str, name: str) -> str:
        if self.orig:
            return self.base_path + "\\" + sub + "\\" + name
        if self.base_path is None:
            raise ValueError("base_path (the data folder) is required")
        return os.path.join(self.base_path, sub, name)

    def read_problem(self, sub: str, name: str) -> Problem:
        """``Problem.readFromFile(basePath + "\\\\" + sub + "\\\\" + name)``.

        ``original``: a failure is printed to :attr:`stderr` and the returned ``Problem``
        has no ``sg``. Differs from the original: ``fixed`` raises (``FileNotFoundError``,
        with a hint at :func:`generate`, or ``ValueError``)."""
        fn = self._data_path(sub, name)
        if self.orig:
            buf = io.StringIO()
            with contextlib.redirect_stderr(buf):
                p = read_dat(fn, "original")
            self._eprint(buf.getvalue())
            return p
        if not os.path.exists(fn):
            raise FileNotFoundError(f"{fn}: no such data file (generate the data set with "
                                    f"'python -m mppflow generate' / mppflow.experiments.generate)")
        return read_dat(fn, "fixed")

    # ------------------------------------------------------------------ sweeps
    def _scale(self, scale: Scale | None) -> Scale:
        if scale is not None:
            return scale
        return self.scale if self.scale is not None else Scale()

    @staticmethod
    def _agents(sc: Scale, lo: int, hi: int, step: int) -> range:
        if sc.max_agents is not None:
            hi = min(hi, sc.max_agents)
        return range(lo, hi + 1, step)

    @staticmethod
    def _n(sc: Scale, n: int) -> int:
        return n if sc.instances is None else sc.instances

    @staticmethod
    def _runs(sc: Scale, n: int) -> int:
        return n if sc.instances is None else sc.instances

    @staticmethod
    def _tl(sc: Scale, tl: float) -> float:
        return tl if sc.time_limit is None else float(sc.time_limit)

    @staticmethod
    def _obs(sc: Scale, lo: int, hi: int, step: int) -> list[int]:
        return [o for o in range(lo, hi + 1, step) if sc.obstacles is None or o in sc.obstacles]

    @staticmethod
    def _section(sc: Scale, name: str) -> bool:
        return sc.sections is None or name in sc.sections

    # ------------------------------------------------------------------ drivers
    def run_n2_puzzle_performance_metric(self, *, scale: Scale | None = None) -> None:
        """``runN2PuzzlePerformanceMetric``: 9-, 16- and 25-puzzles
        (``n2puzzle/{n*n}-puzzle-{1001..1100}.dat``), :meth:`solve_puzzle_problem` without a
        time limit, a ``write_puzzle_stats`` line per solved instance and a
        ``write_stats`` row per size (T-RO Sec. VI-B, Fig. 14/15).

        ``original``: the whole sweep is inside ``try {...} catch (Exception e)``: the first
        Java exception (e.g. a missing file) is printed and ends the method normally.
        Differs from the original: ``fixed`` lets it propagate.
        """
        sc = self._scale(scale)
        with self._jvm():
            try:
                self.total_time = 0.0
                for n in range(3, 6):
                    if sc.max_agents is not None and n * n > sc.max_agents:
                        continue
                    self.reset_stats(f"{n * n}-puzzle, {self._runs(sc, 100)} runs. ")
                    for i in range(self._n(sc, 100)):
                        p = self.read_problem("n2puzzle", f"{n * n}-puzzle-{1001 + i}.dat")
                        self._println(f"Working on {n * n}-puzzle-{1001 + i}.dat")
                        stat = self.solve_puzzle_problem(i, p, self._tl(sc, -1.0))
                        if stat is not None:
                            self.process_stats(stat)
                            self.write_puzzle_stats(i + 1, stat[1] / 1000.0)
                    self.write_stats()
            except JavaException as e:
                if not self.orig:  # pragma: no cover - fixed raises no Java exceptions
                    raise
                self._trace(e)

    def _crowded(self, size: int, frm: int, to: int, time_limit: float, split_levels: int, no_cycle: bool,
                 sc: Scale) -> None:
        nc = " no cycles, " if no_cycle else ""
        self.total_time = 0.0
        for a in self._agents(sc, frm, to, 10):
            success = True
            self.reset_stats(f"{size}x{size}, {a} agents,{nc} {self._runs(sc, 10)} runs. ")
            for i in range(self._n(sc, 10)):
                p = self.read_problem(f"{size}x{size}-grid", f"{a}-agts-{1001 + i}.dat")
                if size == 8:
                    self._println(f"Working on 8x8 grid performance testing,{nc} problem: {a}-agts-{1001 + i}.dat")
                else:
                    self._println(f"Working on 16x16 grid performance testing, {nc}problem: {a}-agts-{1001 + i}.dat")
                stat = self.solve_problem_suboptimal(p, False, True, 0, self._tl(sc, time_limit), split_levels,
                                                     no_cycle)
                self.process_stats(stat)
                if stat is None:
                    success = False
                    break
            if not success:
                break
            self.write_stats()
        self.write_string("\n")

    def run_crowded_performance_metric_8x8(self, frm: int = 10, to: int = 60, time_limit: float = 1800.0,
                                           split_levels: int = 0, no_cycle: bool = False, *,
                                           scale: Scale | None = None) -> None:
        """``runCrowdedPerformanceMetric8x8(from, to, timeLimit, splitLevels, noCycle)``: 8x8
        grid without obstacles, ``a = from, from+10, ..., to`` robots, 10 instances each
        (``8x8-grid/{a}-agts-{id}.dat``), :meth:`solve_problem_suboptimal`. The first failure
        ends the sweep (no row for it); a final ``"\\n"`` is written."""
        sc = self._scale(scale)
        with self._jvm():
            self._crowded(8, frm, to, time_limit, split_levels, no_cycle, sc)

    def run_crowded_performance_metric_16x16(self, frm: int = 10, to: int = 250, time_limit: float = 1800.0,
                                             split_levels: int = 0, no_cycle: bool = False, *,
                                             scale: Scale | None = None) -> None:
        """``runCrowdedPerformanceMetric16x16``: as :meth:`run_crowded_performance_metric_8x8`
        on the 16x16 grid (``16x16-grid/...``)."""
        sc = self._scale(scale)
        with self._jvm():
            self._crowded(16, frm, to, time_limit, split_levels, no_cycle, sc)

    def run_crowded_performance_metric_crowded(self, *, scale: Scale | None = None) -> None:
        """``runCrowdedPerformanceMetricCrowded`` (T-RO Fig. 16): 8x8 exact (10..50 robots),
        2/4/8-way split (10..60) and the no-cycle splits (10..50); 16x16 exact (10..100),
        splits (10..250), no-cycle splits (10..250); 1800 s (the paper says 600 s).
        Sections: ``"8x8/exact"``, ``"8x8/2-way"``, ``"8x8/4-way"``, ``"8x8/8-way"``,
        ``"8x8/2-way-nc"`` ... and the same for ``"16x16/..."``."""
        sc = self._scale(scale)
        with self._jvm():
            for size, ex_to, sp_to, nc_to in ((8, 50, 60, 50), (16, 100, 250, 250)):
                g = f"{size}x{size}"
                hdr = f"{size} x {size} grid"
                if self._section(sc, f"{g}/exact"):
                    self.write_string(f"{hdr}, exact\n")
                    self._crowded(size, 10, ex_to, 1800.0, 0, False, sc)
                for k in range(1, 4):
                    if self._section(sc, f"{g}/{1 << k}-way"):
                        self.write_string(f"{hdr}, {1 << k}-way split\n")
                        self._crowded(size, 10, sp_to, 1800.0, k, False, sc)
                for k in range(1, 4):
                    if self._section(sc, f"{g}/{1 << k}-way-nc"):
                        self.write_string(f"{hdr}, {1 << k}-way split, no cycle\n")
                        self._crowded(size, 10, nc_to, 1800.0, k, True, sc)

    def _sweep_24x18(self, sc: Scale, obs_list, n_lo: int, n_hi: int, n_step: int, solve,
                     write_partial: bool, newline_after_obs: bool) -> None:
        for obs in obs_list:
            stop = False
            for n in self._agents(sc, n_lo, n_hi, n_step):
                self.reset_stats(f"24x18, {obs}% obstacles, {n} agents, {self._runs(sc, 10)} runs. ")
                for i in range(self._n(sc, 10)):
                    name = f"{n}-agt-{obs}-pct-obs-{1001 + i}.dat"
                    self._println(f"Working on 24x18 grid performance testing, problem: {name}")
                    p = self.read_problem("24x18-grid", name)
                    stat = solve(p, n, obs)
                    self.process_stats(stat)
                    if stat is None:
                        stop = True
                        break
                if write_partial:
                    self.write_stats()
                    if stop:
                        break
                else:
                    if stop:
                        break
                    self.write_stats()
            if newline_after_obs:
                self.write_string("\n")

    def run_24x18_performance_metric(self, *, scale: Scale | None = None) -> None:
        """``run24x18PerformanceMetric`` (T-RO Figs. 10-12): 24x18 grids with 0..30 %
        obstacles. Section ``"exact"``: 10..120 robots, the k-way split with one piece
        (``solveProblemSuboptimal(p, false, false, 0, 600, 0, false)``). Sections
        ``"2-way"``, ``"4-way"``, ``"8-way"``, ``"16-way"``: 20..300 robots, step 20, split
        levels 1..4, 600 s. A failure writes the partial row and moves on to the next
        obstacle percentage."""
        sc = self._scale(scale)
        with self._jvm():
            if self._section(sc, "exact"):
                self.write_string("24 x 18 grid, exact\n")
                self.total_time = 0.0
                self._sweep_24x18(sc, self._obs(sc, 0, 30, 5), 10, 120, 10,
                                  lambda p, n, obs: self.solve_problem_suboptimal(p, False, False, 0,
                                                                                  self._tl(sc, 600.0), 0, False),
                                  True, True)
            for k in range(0, 4):
                if not self._section(sc, f"{1 << (k + 1)}-way"):
                    continue
                self.write_string(f"24 x 18 grid, {1 << (k + 1)}-way split\n")
                self.total_time = 0.0
                self._sweep_24x18(sc, self._obs(sc, 0, 30, 5), 20, 300, 20,
                                  lambda p, n, obs, k=k: self.solve_problem_suboptimal(p, False, False, 0,
                                                                                       self._tl(sc, 600.0), k + 1,
                                                                                       False),
                                  True, False)

    def _sweep_32(self, sc: Scale, sub: str, lo: int, hi: int, step: int, label, solve, *, working: str,
                  heuristic_label=None) -> None:
        self.total_time = 0.0
        for n in self._agents(sc, lo, hi, step):
            self.reset_stats(label(n))
            if heuristic_label is not None:
                self.reset_heuristic_stats(heuristic_label(n))
            for i in range(self._n(sc, 10)):
                p = self.read_problem(sub, f"20-pct-obs-{n}-agts-{1001 + i}.dat")
                self._println(working.format(n=n, id=1001 + i))
                stat = solve(p)
                self.process_stats(stat)
            self.write_stats()

    def run_32x32_performance_metric(self, *, scale: Scale | None = None) -> None:
        """``run32x32PerformanceMetric`` (T-RO Fig. 17, ICRA'13 Table II): 32x32 grids with
        20 % obstacles. Section ``"exact"``: 10..90 robots, :meth:`solve_problem` with 1800 s.
        Sections ``"2-way"`` (20..120), ``"4-way"`` (20..160), ``"8-way"`` and ``"16-way"``
        (20..200), step 20: the **no-cycle** split (``noCycle = true``), 1800 s. Failures
        are only skipped."""
        sc = self._scale(scale)
        w = "Working on 32x32 grid performance testing, problem: 20-pct-obs-{n}-agts-{id}.dat"
        r = self._runs(sc, 10)
        with self._jvm():
            if self._section(sc, "exact"):
                self.write_string("32 x 32 grid, 20% obstacles, exact\n")
                self._sweep_32(sc, "32x32-grid", 10, 90, 10, lambda a: f"32x32, 20% obstacles, {a} agents, {r} runs. ",
                               lambda p: self.solve_problem(p, False, self._tl(sc, 1800.0)), working=w)
            for k, hi in ((1, 120), (2, 160), (3, 200), (4, 200)):
                if not self._section(sc, f"{1 << k}-way"):
                    continue
                self.write_string(f"32 x 32 grid, 20% obstacles, {1 << k}-way split\n")
                self._sweep_32(sc, "32x32-grid", 20, hi, 20,
                               lambda n: f"32x32, 20% obstacles, {n} agents, {r} runs. ",
                               lambda p, k=k: self.solve_problem_suboptimal(p, False, True, 0, self._tl(sc, 1800.0),
                                                                            k, True),
                               working=w,
                               heuristic_label=lambda n: f"32x32, 20% obstacles, {n} agents, no cycles,  {r} runs. ")

    def run_32x32_performance_metric_8connected_all(self, *, scale: Scale | None = None) -> None:
        """``run32x32PerformanceMetric8ConnectedAll`` (T-RO footnote 10): 8-connected 32x32
        grids with 20 % obstacles (``32x32-grid-8c/...``), the k-way split with 1800 s:
        sections ``"exact"`` (one piece, 20..100 robots), ``"2-way"`` (20..200), ``"4-way"``
        (20..300), ``"8-way"`` (20..400), step 20."""
        sc = self._scale(scale)
        w = "Working on 32x32 grid performance testing, problem: 20-pct-obs-{n}-{id}.dat"
        r = self._runs(sc, 10)
        with self._jvm():
            for k, name, hi in ((0, "exact", 100), (1, "2-way split", 200), (2, "4-way split", 300),
                                (3, "8-way split", 400)):
                if not self._section(sc, "exact" if k == 0 else f"{1 << k}-way"):
                    continue
                self.write_string(f"32 x 32 grid, 8 connected, 20% obstacles, {name}\n")
                self._sweep_32(sc, "32x32-grid-8c", 20, hi, 20, lambda a: f"32x32, 20% obstacles, {a} agents, {r} runs. ",
                               lambda p, k=k: self.solve_problem_suboptimal(p, False, False, 0, self._tl(sc, 1800.0),
                                                                            k, False),
                               working=w)

    def run_24x18_performance_metric_total_time(self, *, scale: Scale | None = None) -> None:
        """``run24x18PerformanceMetricTotalTime`` (T-RO Fig. 18): MINTOTALTIME on 24x18,
        0..30 % obstacles, 10..160 robots, 600 s, gap
        ``n*0.002 + obs*0.002 + (n > 50 ? n*0.001 : 0) + (n > 100 ? n*0.0005 : 0)``. A failure
        writes the partial row and moves on to the next obstacle percentage."""
        sc = self._scale(scale)

        def solve(p, n, obs):
            gap = n * 0.002 + obs * 0.002 + (n * 0.001 if n > 50 else 0) + (n * 0.0005 if n > 100 else 0)
            return self.solve_problem_total_time(p, gap, self._tl(sc, 600.0))

        with self._jvm(), self._driver_horizon():
            self.write_string("24 x 18 grid, total time solution\n")
            self.total_time = 0.0
            self._sweep_24x18(sc, self._obs(sc, 0, 30, 5), 10, 160, 10, solve, True, True)

    def run_24x18_performance_metric_total_distance(self, *, scale: Scale | None = None) -> None:
        """``run24x18PerformanceMetricTotalDistance`` (not plotted): MINTOTALDIST at the
        4-way-split horizon, 0..30 %, 10..160 robots, gap ``n*0.002 + obs*0.002``, 600 s. A
        failure skips the row and moves on to the next obstacle percentage."""
        sc = self._scale(scale)
        with self._jvm(), self._driver_horizon():
            self.write_string("24 x 18 grid, total distance solution\n")
            self.total_time = 0.0
            self._sweep_24x18(sc, self._obs(sc, 0, 30, 5), 10, 160, 10,
                              lambda p, n, obs: self.solve_problem_distance(p, n * 0.002 + obs * 0.002,
                                                                            self._tl(sc, 600.0)),
                              False, True)

    def run_24x18_performance_metric_total_distance_split(self, *, scale: Scale | None = None) -> None:
        """``run24x18PerformanceMetricTotalDistanceSplit`` (T-RO Fig. 19): the split
        MINTOTALDIST, 4-way for 0..20 % obstacles and **8-way for 25 and 30 %** (the caption
        says 4-way), gap ``n*0.001 + obs*0.0025 + (n > 70 ? n*0.001 : 0)``, 600 s."""
        sc = self._scale(scale)

        def solve(p, n, obs):
            split = 3 if obs > 20 else 2
            gap = n * 0.001 + obs * 0.0025 + (n * 0.001 if n > 70 else 0)
            return self.solve_problem_distance_split(p, gap, self._tl(sc, 600.0), split)

        with self._jvm():
            self.write_string("24 x 18 grid, total distance solution, split computation\n")
            self.total_time = 0.0
            self._sweep_24x18(sc, self._obs(sc, 0, 30, 5), 10, 160, 10, solve, False, True)

    @contextlib.contextmanager
    def _any_solution(self, attr: str):
        setattr(self.flags, attr, True)
        if self.orig:
            yield
            setattr(self.flags, attr, False)
            return
        try:
            yield
        finally:
            setattr(self.flags, attr, False)

    def run_24x18_performance_metric_total_time_fixed_time(self, *, scale: Scale | None = None) -> None:
        """``run24x18PerformanceMetricTotalTimeFixedTime`` (``main`` case 0; T-RO Fig. 20
        top): ``TotalTime.bReturnAnySolution = true``, 0..20 % obstacles, 70..100 robots,
        gap 0.001, **100 s** (the paper: 600 s), then the flag is reset.

        ``original``: an exception leaves the flag set. Differs from the original: ``fixed``
        always resets it."""
        sc = self._scale(scale)
        with self._jvm(), self._any_solution("tt_return_any_solution"), self._driver_horizon():
            self.write_string("24 x 18 grid, total time solution, fixed time (100s)\n")
            self.total_time = 0.0
            self._sweep_24x18(sc, self._obs(sc, 0, 20, 5), 70, 100, 10,
                              lambda p, n, obs: self.solve_problem_total_time(p, 0.001, self._tl(sc, 100.0)),
                              True, True)

    def run_24x18_performance_metric_total_distance_split_fixed_time(self, *, scale: Scale | None = None) -> None:
        """``run24x18PerformanceMetricTotalDistanceSplitFixedTime`` (T-RO Fig. 20 bottom):
        ``Distance.bReturnAnySolution = true``, 0..20 %, 10..150 robots, the 4-way split
        MINTOTALDIST with gap 0.001 and 100 s (same flag handling as the TT variant)."""
        sc = self._scale(scale)
        with self._jvm(), self._any_solution("dist_return_any_solution"):
            self.write_string("24 x 18 grid, total distance solution, split computation, fixed time (100s)\n")
            self.total_time = 0.0
            self._sweep_24x18(sc, self._obs(sc, 0, 20, 5), 10, 150, 10,
                              lambda p, n, obs: self.solve_problem_distance_split(p, 0.001, self._tl(sc, 100.0), 2),
                              False, True)

    def run_32x32_performance_metric_splitting(self, frm: int = 25, to: int = 250, spacing: int = 25,
                                               time_limit: float = 3600.0, split_levels: int = 1,
                                               no_cycle: bool = False, *, scale: Scale | None = None) -> None:
        """``run32x32PerformanceMetricSplitting(from, to, spacing, timeLimit, splitLevels,
        noCycle)`` (legacy): 32x32, 20 % obstacles, :meth:`solve_problem_suboptimal`."""
        sc = self._scale(scale)
        r = self._runs(sc, 10)
        nc = " no cycles, " if no_cycle else ""
        with self._jvm():
            self._sweep_32(sc, "32x32-grid", frm, to, spacing, lambda n: f"32x32, 20% obstacles, {n} agents, {r} runs. ",
                           lambda p: self.solve_problem_suboptimal(p, False, True, 0, self._tl(sc, time_limit),
                                                                   split_levels, no_cycle),
                           working="Working on 32x32 grid performance testing, problem: 20-pct-obs-{n}-agts-{id}.dat",
                           heuristic_label=lambda n: f"32x32, 20% obstacles, {n} agents,{nc} {r} runs. ")

    def run_32x32_performance_metric_8connected(self, *, scale: Scale | None = None) -> None:
        """``run32x32PerformanceMetric8Connected`` (legacy): 8-connected 32x32, 10..60
        robots, 2-way split, 3600 s."""
        sc = self._scale(scale)
        r = self._runs(sc, 10)
        with self._jvm():
            self._sweep_32(sc, "32x32-grid-8c", 10, 60, 10, lambda a: f"32x32, 20% obstacles, {a} agents, {r} runs. ",
                           lambda p: self.solve_problem_suboptimal(p, False, False, 0, self._tl(sc, 3600.0), 1, False),
                           working="Working on 32x32 grid performance testing, problem: 20-pct-obs-{n}-{id}.dat")

    def run_32x32_performance_metric_heuristic(self, *, scale: Scale | None = None) -> None:
        """``run32x32PerformanceMetricHeuristic`` (legacy, "do not use"; ICRA'13 Table III,
        approximately): the local-repair heuristic on 32x32, 20..200 robots, 10 instances,
        :meth:`write_heuristic_stats`.

        ``original``: reads ``20-pct-obs-{n}-{id}.dat`` (no ``-agts-``: the generator never
        writes these) and labels the rows "100 runs". Differs from the original: ``fixed``
        reads ``20-pct-obs-{n}-agts-{id}.dat`` and labels the rows with the number of runs.
        """
        sc = self._scale(scale)
        runs = (100 if self.orig else 10) if sc.instances is None else sc.instances
        with self._jvm():
            self.total_time = 0.0
            for n in self._agents(sc, 20, 200, 20):
                self.reset_heuristic_stats(f"32x32, 20% obstacles, {n} agents, {runs} runs. ")
                for i in range(self._n(sc, 10)):
                    self._println(f"Working on 32x32 grid performance testing, problem: 20-pct-obs-{n}-{1001 + i}.dat")
                    name = f"20-pct-obs-{n}-{1001 + i}.dat" if self.orig else f"20-pct-obs-{n}-agts-{1001 + i}.dat"
                    p = self.read_problem("32x32-grid", name)
                    stat = self.solve_problem_with_heuristic(p)
                    self.process_heuristic_stats(stat, n)
                self.write_heuristic_stats()

    def run_32x32_performance_metric_splitting_8connected(self, frm: int = 25, to: int = 250,
                                                          time_limit: float = 3600.0, split_levels: int = 1,
                                                          no_cycle: bool = False, *,
                                                          scale: Scale | None = None) -> None:
        """``run32x32PerformanceMetricSplitting8Connected(from, to, timeLimit, splitLevels,
        noCycle)`` (legacy): 8-connected 32x32, robots ``from, from+25, ..., to``.

        ``original``: reads the plain-text ``.txt`` files as serialized data (always a
        ``StreamCorruptedException``, then ``JavaNullPointerException``). Differs from the
        original: ``fixed`` reads the ``.dat`` files."""
        sc = self._scale(scale)
        r = self._runs(sc, 10)
        nc = " no cycles, " if no_cycle else ""
        ext = ".txt" if self.orig else ".dat"
        with self._jvm():
            self.total_time = 0.0
            for n in self._agents(sc, frm, to, 25):
                self.reset_stats(f"32x32, 20% obstacles, {n} agents, {r} runs. ")
                self.reset_heuristic_stats(f"32x32, 20% obstacles, {n} agents,{nc} {r} runs. ")
                for i in range(self._n(sc, 10)):
                    self._println(f"Working on 32x32 grid performance testing,{nc} problem: "
                                  f"20-pct-obs-{n}-agts-{1001 + i}.txt")
                    p = self.read_problem("32x32-grid-8c", f"20-pct-obs-{n}-agts-{1001 + i}{ext}")
                    stat = self.solve_problem_suboptimal(p, False, True, 0, self._tl(sc, time_limit), split_levels,
                                                         no_cycle)
                    self.process_stats(stat)
                self.write_stats()

    def run_full_performance_metric(self, *, scale: Scale | None = None) -> None:
        """``runFullPerformanceMetric`` (legacy, 3600 s): the 8x8 / 16x16 crowded sweeps and
        :meth:`run_32x32_performance_metric` without splitting, then 2/4/8/16-way with and
        without cycles, including ``run32x32PerformanceMetricSplitting`` with 25, 50, ...
        robots, which the generator never produces (``FileNotFoundError`` in ``fixed``;
        ``JavaNullPointerException`` in ``original``)."""
        sc = self._scale(scale)
        tl = 3600.0
        with self._jvm():
            self.write_string("No splitting\n")
            self._crowded(8, 10, 50, tl, 0, False, sc)
            self._crowded(16, 10, 70, tl, 0, False, sc)
            self.run_32x32_performance_metric(scale=sc)
            plan = [
                ("2 - splitting\n", [(8, 10, 60, 1, False), (16, 10, 120, 1, False)], (25, 125, 1)),
                ("2 - splitting, no cycle\n", [(8, 10, 40, 1, True), (16, 10, 80, 1, True)], None),
                ("4 - splitting\n", [(8, 10, 60, 2, False), (16, 10, 160, 2, False)], (25, 100, 2)),
                ("4 - splitting, no cycle\n", [(8, 10, 40, 2, True), (16, 10, 100, 2, True)], None),
                ("8 - splitting\n", [(8, 10, 60, 3, False), (16, 10, 210, 3, False)], (25, 150, 3)),
                ("8 - splitting, no cycle\n", [(8, 10, 40, 3, True), (16, 10, 120, 3, True)], None),
                ("16 - splitting\n", [(8, 20, 60, 4, False), (16, 20, 220, 4, False)], (25, 250, 4)),
                ("16 - splitting, no cycle\n", [(8, 20, 40, 4, True), (16, 20, 130, 4, True)], None),
            ]
            for header, crowded, split32 in plan:
                self.write_string(header)
                for size, frm, to, k, nc in crowded:
                    self._crowded(size, frm, to, tl, k, nc, sc)
                if split32 is not None:
                    frm, to, k = split32
                    self.run_32x32_performance_metric_splitting(frm, to, 25, tl, k, False, scale=sc)

    def run_full_performance_metric_extras(self, *, scale: Scale | None = None) -> None:
        """``runFullPerformanceMetricExtras`` (legacy): one row, 16x16 with 90 robots, exact
        (one-piece split), 3600 s."""
        sc = self._scale(scale)
        with self._jvm():
            self.write_string("No splitting\n")
            self._crowded(16, 90, 90, 3600.0, 0, False, sc)

    def run_driver(self, name: str, *args, scale: Scale | None = None):
        """Run a driver by its :data:`DRIVERS` key, Java name or method name."""
        key = _driver_key(name)
        return getattr(self, DRIVERS[key]["method"])(*args, scale=scale)

    # ------------------------------------------------------------------ main
    def main(self, argv: Sequence[str] = ()) -> str:
        """``Main.main(argv)``: ``argv[0]`` selects the case (default 1); returns the text
        printed by this call.

        * 0: :meth:`run_24x18_performance_metric_total_time_fixed_time`;
        * 1: a random 16-puzzle (:func:`~mppflow.problem.create_n2_puzzle`), exact
          MINMAKESPAN with the LP relaxation first, no time limit;
        * 2: a 15x15 grid with 20 % obstacles and 15 robots, 60 s;
        * 3: the star graph 0-1, 1-2, 1-3 with robots ``0 -> 3``, ``3 -> 0``;
        * 4: the problem in file ``argv[1]``, 1000 s;
        * 5: a 16x16 grid with 60 robots, 7200 s (``original``: then case 6);
        * 6: the 17-vertex spider tree with 8 random goals;
        * anything else: ``original``: nothing (Java's switch has no default); ``fixed``:
          :class:`ValueError`.

        The Time solver's flags are turned off first. ``original``: a non-integer
        ``argv[0]`` raises :class:`JavaNumberFormatException`, case 4 without ``argv[1]``
        ``JavaArrayIndexOutOfBoundsException``, a missing file ``JavaNullPointerException``
        (after the stack trace). Differs from the original: see the module docstring.
        """
        start = len(self._log)
        argv = list(argv)
        if not argv:
            option = 1
        elif self.orig:
            option = _parse_int_java(argv[0])
        else:
            try:
                option = int(argv[0])
            except (TypeError, ValueError):
                raise ValueError(f"case must be an integer, not {argv[0]!r}") from None
            if not 0 <= option <= 6:
                raise ValueError(f"case must be 0..6, not {option}")
        self.flags.time_debug = False
        self.flags.time_print_paths = False
        with self._jvm():
            if option == 0:
                self.run_24x18_performance_metric_total_time_fixed_time()
            elif option == 1:
                self._println("Solving a randomly generated 16-puzzle, generally solvable in 60 seconds on an intel "
                              "dual core x64 machine.\n")
                p = create_n2_puzzle(4, self.rng, self.compat)
                self._print(print_start_and_goals(p.graph, p.starts, p.goals, self.compat))
                self._println("\nSolution path set:\n")
                self.solve_problem(p, True, -1)
            elif option == 2:
                self._println("Solving a randomly generated multi-agent path planning problem on a 15x15 grid with "
                              "20% obstacles and 15 agents, with 60 second time limit.\n")
                p = create_grid_problem(15, 15, 0.2, 15, self.rng, self.compat)
                self._println("Printing adjacency list of the graph:")
                self._print(p.graph.print_graph_as_adjacency_list(self.compat))
                self._println("\nSolution path set:\n")
                self.solve_problem(p, True, 60)
            elif option == 3:
                self._println("Create a simple graph manually and then solve a problem on this graph.\n")
                g = create_main_case3_graph()
                self._println("Adjacency list:")
                self._print(g.print_graph_as_adjacency_list(self.compat))
                self._println("\nSolving the problem, solution path set: ")
                self.solve_problem(Problem(g, [0, 3], [3, 0]), True, -1)
            elif option == 4:
                if len(argv) < 2:
                    if self.orig:
                        raise JavaArrayIndexOutOfBoundsException(f"Index 1 out of bounds for length {len(argv)}")
                    raise ValueError("case 4 needs a problem file")
                self._println(f"Solving problem from file: {argv[1]} with 1000 seconds limit.\n")
                if self.orig:
                    buf = io.StringIO()
                    with contextlib.redirect_stderr(buf):
                        p = read_dat(argv[1], "original")
                    self._eprint(buf.getvalue())
                    if p.starts is None or p.goals is None:
                        raise JavaNullPointerException(_NPE_SG_MAIN)
                else:
                    p = load_problem(argv[1], self.compat)
                self._print(print_start_and_goals(p.graph, p.starts, p.goals, self.compat))
                self._println("\n")
                self.solve_problem(p, True, 1000)
            elif option in (5, 6):
                if option == 5:
                    p = create_grid_problem(16, 16, 0, 60, self.rng, self.compat)
                    self._print(print_start_and_goals(p.graph, p.starts, p.goals, self.compat))
                    self.solve_problem(p, True, 7200)
                if option == 6 or self.orig:  # original: case 5 has no break
                    t = create_main_case6_tree(4, 4)
                    starts, goals = get_random_start_goal_many(t, 8, self.rng, self.compat)
                    self._println("Adjacency list:")
                    self._print(t.print_graph_as_adjacency_list(self.compat))
                    p = Problem(t, starts, goals)
                    self._print(print_start_and_goals(p.graph, p.starts, p.goals, self.compat))
                    self._println("\nSolving the problem, solution path set: ")
                    self.solve_problem(p, True, -1)
        return "".join(self._log[start:])


def _row0(paths, orig: bool):
    if not paths:
        if orig:
            raise JavaArrayIndexOutOfBoundsException("Index 0 out of bounds for length 0")
        return [0]
    return paths[0]


# ----------------------------------------------------------------------- registry
DRIVERS: dict[str, dict] = {
    "n2puzzle": {"method": "run_n2_puzzle_performance_metric", "java": "runN2PuzzlePerformanceMetric",
                 "params": (), "sections": ()},
    "crowded-8x8": {"method": "run_crowded_performance_metric_8x8", "java": "runCrowdedPerformanceMetric8x8",
                    "params": ("from", "to", "time_limit", "split_levels", "no_cycle"), "sections": ()},
    "crowded-16x16": {"method": "run_crowded_performance_metric_16x16", "java": "runCrowdedPerformanceMetric16x16",
                      "params": ("from", "to", "time_limit", "split_levels", "no_cycle"), "sections": ()},
    "crowded": {"method": "run_crowded_performance_metric_crowded", "java": "runCrowdedPerformanceMetricCrowded",
                "params": (), "sections": tuple(f"{g}/{s}" for g in ("8x8", "16x16")
                                                for s in ("exact", "2-way", "4-way", "8-way", "2-way-nc", "4-way-nc",
                                                          "8-way-nc"))},
    "24x18": {"method": "run_24x18_performance_metric", "java": "run24x18PerformanceMetric", "params": (),
              "sections": ("exact", "2-way", "4-way", "8-way", "16-way")},
    "32x32": {"method": "run_32x32_performance_metric", "java": "run32x32PerformanceMetric", "params": (),
              "sections": ("exact", "2-way", "4-way", "8-way", "16-way")},
    "32x32-8c-all": {"method": "run_32x32_performance_metric_8connected_all",
                     "java": "run32x32PerformanceMetric8ConnectedAll", "params": (),
                     "sections": ("exact", "2-way", "4-way", "8-way")},
    "24x18-tt": {"method": "run_24x18_performance_metric_total_time", "java": "run24x18PerformanceMetricTotalTime",
                 "params": (), "sections": ()},
    "24x18-td": {"method": "run_24x18_performance_metric_total_distance",
                 "java": "run24x18PerformanceMetricTotalDistance", "params": (), "sections": ()},
    "24x18-td-split": {"method": "run_24x18_performance_metric_total_distance_split",
                       "java": "run24x18PerformanceMetricTotalDistanceSplit", "params": (), "sections": ()},
    "24x18-tt-fixed-time": {"method": "run_24x18_performance_metric_total_time_fixed_time",
                            "java": "run24x18PerformanceMetricTotalTimeFixedTime", "params": (), "sections": ()},
    "24x18-td-split-fixed-time": {"method": "run_24x18_performance_metric_total_distance_split_fixed_time",
                                  "java": "run24x18PerformanceMetricTotalDistanceSplitFixedTime", "params": (),
                                  "sections": ()},
    "32x32-splitting": {"method": "run_32x32_performance_metric_splitting",
                        "java": "run32x32PerformanceMetricSplitting",
                        "params": ("from", "to", "spacing", "time_limit", "split_levels", "no_cycle"), "sections": ()},
    "32x32-8c": {"method": "run_32x32_performance_metric_8connected", "java": "run32x32PerformanceMetric8Connected",
                 "params": (), "sections": ()},
    "32x32-heuristic": {"method": "run_32x32_performance_metric_heuristic",
                        "java": "run32x32PerformanceMetricHeuristic", "params": (), "sections": ()},
    "32x32-splitting-8c": {"method": "run_32x32_performance_metric_splitting_8connected",
                           "java": "run32x32PerformanceMetricSplitting8Connected",
                           "params": ("from", "to", "time_limit", "split_levels", "no_cycle"), "sections": ()},
    "full": {"method": "run_full_performance_metric", "java": "runFullPerformanceMetric", "params": (),
             "sections": ()},
    "full-extras": {"method": "run_full_performance_metric_extras", "java": "runFullPerformanceMetricExtras",
                    "params": (), "sections": ()},
}
"""Every ``Main.run*`` driver: key -> ``method`` (of :class:`Experiments`), ``java`` (the
original's name), positional ``params`` and ``sections`` (for :attr:`Scale.sections`)."""


def _driver_key(name: str) -> str:
    if name in DRIVERS:
        return name
    for k, d in DRIVERS.items():
        if name in (d["method"], d["java"]):
            return k
    raise KeyError(f"unknown driver {name!r}; choose from {sorted(DRIVERS)}")


# ------------------------------------------------------------------------ figures
@dataclass(frozen=True)
class Figure:
    """A paper figure or table and the driver that produced its data."""

    key: str
    paper: str
    item: str
    what: str
    driver: str | None
    sections: tuple[str, ...] | None = None
    data: str = ""
    params: str = ""
    notes: str = ""


FIGURES: dict[str, Figure] = {f.key: f for f in [
    Figure("tor-fig9", "T-RO 2016", "Fig. 9", "a 24x18 instance, 25% obstacles, 50 robots", None,
           data="24x18-grid/50-agt-25-pct-obs-{1001..1010}.dat",
           params="create_grid_problem(24, 18, 0.25, 50)",
           notes="instance family of Figs. 10-12 and 18-20; 24 rows x 18 columns (the figure is drawn rotated)"),
    Figure("tor-fig10", "T-RO 2016", "Fig. 10", "exact MINMAKESPAN on 24x18: time and average optimal makespan",
           "24x18", ("exact",), data="24x18-grid/{n}-agt-{obs}-pct-obs-{1001..1010}.dat",
           params="obs 0..30 step 5, n 10..120 step 10, 600 s, k-way split with one piece",
           notes="the paper plots 0-25% and n <= 100; 'optimal makespan' = ave solution"),
    Figure("tor-fig11", "T-RO 2016", "Fig. 11", "4-way split on 24x18", "24x18", ("4-way",),
           data="24x18-grid/...", params="n 20..300 step 20, 600 s, split level 2",
           notes="optimality ratio = ave solution / min possible (a ratio of averages)"),
    Figure("tor-fig12", "T-RO 2016", "Fig. 12", "2-, 8- and 16-way split on 24x18", "24x18",
           ("2-way", "8-way", "16-way"), data="24x18-grid/...", params="n 20..300 step 20, 600 s, split levels 1, 3, 4"),
    Figure("tor-fig13", "T-RO 2016", "Fig. 13", "independence detection (ID) comparison", None,
           notes="ID is Standley's C code, not part of the Java code"),
    Figure("tor-fig14-15", "T-RO 2016", "Figs. 14-15, Sec. VI-B",
           "9/16/25-puzzles (0.46 s / 4.2 s / 391.6 s average over 100)", "n2puzzle",
           data="n2puzzle/{9,16,25}-puzzle-{1001..1100}.dat", params="PuzzleSolver, no time limit",
           notes="a solved 25-puzzle is written to 25puzzle.txt; main case 1 is a single-instance demo"),
    Figure("tor-fig16-top", "T-RO 2016", "Fig. 16 (top)", "8x8 crowded: makespan vs lower bound", "crowded",
           ("8x8/exact", "8x8/2-way", "8x8/4-way", "8x8/8-way"), data="8x8-grid/{a}-agts-{1001..1010}.dat",
           params="exact 10..50, splits 10..60, 1800 s", notes="code time limit 1800 s, the paper says 600 s"),
    Figure("tor-fig16-bottom", "T-RO 2016", "Fig. 16 (bottom)", "16x16 crowded: makespan vs lower bound", "crowded",
           ("16x16/exact", "16x16/2-way", "16x16/4-way", "16x16/8-way"), data="16x16-grid/{a}-agts-{1001..1010}.dat",
           params="exact 10..100, splits 10..250, 1800 s", notes="code time limit 1800 s, the paper says 600 s"),
    Figure("tor-fig17", "T-RO 2016", "Fig. 17", "32x32 with 20% obstacles", "32x32", None,
           data="32x32-grid/20-pct-obs-{n}-agts-{1001..1010}.dat",
           params="exact 10..90 (1800 s); 2/4/8/16-way no-cycle splits, 1800 s",
           notes="the split runs use the cycle-free split planner (noCycle = true), not mentioned in the paper; "
                 "the paper shows no 16-way"),
    Figure("tor-fn10", "T-RO 2016", "footnote 10", "8-connected 32x32, up to 400 robots", "32x32-8c-all", None,
           data="32x32-grid-8c/20-pct-obs-{a}-agts-{1001..1010}.dat", params="up to 8-way, n <= 400, 1800 s"),
    Figure("tor-fig18", "T-RO 2016", "Fig. 18", "MINTOTALTIME, 600 s, adjusted gap", "24x18-tt", None,
           data="24x18-grid/...",
           params="obs 0..30, n 10..160, gap n*0.002 + obs*0.002 (+ n*0.001 if n > 50, + n*0.0005 if n > 100)",
           notes="horizon = makespan of a 4-way split; the gap applies to the complemented objective n*T - TT"),
    Figure("tor-fig19", "T-RO 2016", "Fig. 19", "MINTOTALDIST with the split", "24x18-td-split", None,
           data="24x18-grid/...", params="gap n*0.001 + obs*0.0025 (+ n*0.001 if n > 70), 600 s",
           notes="25% and 30% obstacles use the 8-way split (split = 3); the caption says 4-way"),
    Figure("tor-fig20-top", "T-RO 2016", "Fig. 20 (top)", "MINTOTALTIME vs ID, fixed time",
           "24x18-tt-fixed-time", None, data="24x18-grid/...",
           params="bReturnAnySolution, gap 0.001, 100 s, obs 0..20, n 70..100",
           notes="most likely driver; the paper says 600 s and no suboptimality threshold (the code: "
                 "gap 0.001), n 20..120, obs 0/10/20 (main case 0)"),
    Figure("tor-fig20-bottom", "T-RO 2016", "Fig. 20 (bottom)", "MINTOTALDIST vs ID, fixed time",
           "24x18-td-split-fixed-time", None, data="24x18-grid/...",
           params="bReturnAnySolution, gap 0.001, 100 s, 4-way split, obs 0..20, n 10..150",
           notes="most likely driver; the paper says 600 s and no suboptimality threshold (the code: "
                 "gap 0.001), n 20..140, obs 0/10/20"),
    Figure("icra-table1", "ICRA 2013", "Table I", "20x15 grids, 1000 s", None,
           notes="no generator or driver in the code (main case 4 runs one file with a 1000 s limit)"),
    Figure("icra-table2", "ICRA 2013", "Table II", "32x32 with 20% obstacles, 10..50 robots, 30 min", "32x32",
           ("exact",), data="32x32-grid/...", params="1800 s"),
    Figure("icra-table3", "ICRA 2013", "Table III", "the local-repair heuristic, 25..150 robots, 100 runs, 30 s",
           "32x32-heuristic", None, data="32x32-grid/20-pct-obs-{n}-{id}.dat (no -agts-, never generated)",
           params="n 20..200 step 20, 10 runs (labelled 100), no time limit",
           notes="legacy driver ('do not use'); only n = 100 hits a special attempt schedule"),
    Figure("tor-fig16-nc", "T-RO 2016", "(not plotted)", "no-cycle split variants on 8x8/16x16", "crowded",
           ("8x8/2-way-nc", "8x8/4-way-nc", "8x8/8-way-nc", "16x16/2-way-nc", "16x16/4-way-nc", "16x16/8-way-nc")),
    Figure("unplotted-td", "T-RO 2016", "(not plotted)", "MINTOTALDIST at the 4-way-split horizon, unsplit",
           "24x18-td", None, data="24x18-grid/...", params="gap n*0.002 + obs*0.002, 600 s"),
]}
"""Paper figure/table -> driver key (:data:`DRIVERS`), sections and parameters
(``docs/notes/paper-vs-code.md``, "Figure -> driver mapping")."""


def figures_text() -> str:
    """:data:`FIGURES` as a plain-text table."""
    lines = []
    for f in FIGURES.values():
        drv = f.driver or "-"
        if f.driver and f.sections:
            drv += " [" + ", ".join(f.sections) + "]"
        lines.append(f"{f.key:18s} {f.paper} {f.item}: {f.what}")
        lines.append(f"{'':18s}   driver: {drv}" + (f" ({DRIVERS[f.driver]['java']})" if f.driver else ""))
        if f.data:
            lines.append(f"{'':18s}   data: {f.data}")
        if f.params:
            lines.append(f"{'':18s}   parameters: {f.params}")
        if f.notes:
            lines.append(f"{'':18s}   notes: {f.notes}")
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------------- module API
def main(argv: Sequence[str] | int | None = None, file: str | None = None, **kwargs) -> Experiments:
    """``Main.main(argv)`` on a fresh :class:`Experiments` (``kwargs`` go to its
    constructor). ``argv`` may be a case number; ``file`` is case 4's ``argv[1]``.
    Returns the instance (``.log``, ``.stderr``, ``.rng``)."""
    if argv is None:
        args: list[str] = []
    elif isinstance(argv, (int, str)):
        args = [str(argv)]
    else:
        args = [str(a) for a in argv]
    if file is not None:
        args.append(file)
    ex = Experiments(**kwargs)
    ex.main(args)
    return ex


def run(driver: str, *args, base_path: str | None = None, scale: Scale | None = None, **kwargs) -> Experiments:
    """Run one driver (a :data:`DRIVERS` key, Java or method name) on a fresh
    :class:`Experiments`; returns it (``.results`` holds the rows written)."""
    ex = Experiments(base_path, scale=scale, **kwargs)
    ex.run_driver(driver, *args)
    return ex


def generate(seed: int | MathRandom | None, out_dir: str | None, families: Iterable[str] = BENCHMARK_FAMILIES,
             compat: str = "fixed", *, write_text: bool = True, write_binary: bool = True,
             failed: list[str] | None = None) -> list[str]:
    """Regenerate the benchmark data set (``Problem.main``) with ``Math.random`` seeded
    ``seed``: :func:`mppflow.problem.generate_benchmark_files` with the original's family
    order and names. ``original`` writes the backslash names the ``original``-mode drivers
    read (``out_dir`` should then end with ``/``); ``fixed`` writes ``out_dir/<family>/...``.
    Returns the paths written (``failed`` collects those ``original`` mode could not open).
    The families share one random stream in the order given, so a family's instances depend
    on the families listed before it: only the full default list reproduces the files of a
    full run with the same seed. The published instances came from an unseeded stream and
    cannot be recovered; any seed gives the same distribution."""
    rng = MathRandom(seed) if isinstance(seed, int) else seed
    return generate_benchmark_files(out_dir, rng, families, compat, write_text=write_text, write_binary=write_binary,
                                    failed=failed)


def _one_shot(method: str):
    def f(*args, compat: str = "fixed", backend: str = "highs", params: dict | None = None,
          rng: MathRandom | int | None = None, echo: bool = False, **kwargs):
        ex = Experiments(compat=compat, backend=backend, params=params, rng=rng, echo=echo, **kwargs)
        return getattr(ex, method)(*args)

    target = getattr(Experiments, method)
    f.__name__ = method
    f.__qualname__ = method
    f.__doc__ = (f"One-shot :meth:`Experiments.{method}` on a fresh :class:`Experiments` "
                 f"(keyword arguments as its constructor); returns the stat vector.\n\n"
                 + (target.__doc__ or ""))
    return f


solve_problem_suboptimal = _one_shot("solve_problem_suboptimal")
solve_problem = _one_shot("solve_problem")
solve_problem_pmg = _one_shot("solve_problem_pmg")
solve_problem_total_time = _one_shot("solve_problem_total_time")
solve_problem_distance = _one_shot("solve_problem_distance")
solve_problem_distance_split = _one_shot("solve_problem_distance_split")
solve_puzzle_problem = _one_shot("solve_puzzle_problem")
solve_problem_with_heuristic = _one_shot("solve_problem_with_heuristic")
