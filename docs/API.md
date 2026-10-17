# `mppflow` public API

Everything below can be imported from the package itself (`import mppflow`). The names are
listed in `mppflow.__all__`, together with `__version__`. Importing `mppflow` is cheap (about 20 ms): it does not load
numpy, scipy or gurobipy. A solver backend is imported the first time a model is solved.

Conventions (details in [DESIGN.md](DESIGN.md)):

* Vertices are `0..n-1` (the solvers raise `ValueError` otherwise; the grid generators with
  holes return 1-based ids with gaps, so call `convert_graph` first, passing it the start and
  goal lists, which it renumbers in place). `graph.adj[v]` lists the
  neighbours of `v` in ascending order and includes `v` itself (waiting is a move). A vertex
  without its self-loop is a vertex where robots cannot wait; the solvers support such graphs.
* Vertex ids are integers: numpy integers (e.g. `np.array(starts)`) are converted to `int`, and
  `bool`, floats and other types raise `TypeError`.
* Robot `i` goes from `starts[i]` to `goals[i]`.
* A plan is `paths[i][t]`, with all rows of length `T + 1`.
* Every random function takes `rng`, a `MathRandom(seed)`. With the same seed it draws the
  same stream as the original Java code.
* Most functions take `compat="fixed"` (the default, corrected) or `compat="original"`
  (reproduces the Java code exactly, bugs included).
* `backend` is `"highs"` (the default, scipy), `"gurobi"` (gurobipy) or `"builtin"` (pure
  Python, small models only).

## Usage

```python
import mppflow

rng = mppflow.MathRandom(7)                          # the original's seeded Math.random()
p = mppflow.create_grid_problem(6, 6, 0.1, 8, rng)   # 6x6 grid, 10% holes, 8 robots
r = mppflow.plan(p, "makespan")                      # optimal makespan (HiGHS by default)
print(r.status, r.makespan, r.lower_bounds["makespan"], mppflow.validate(p, r.paths))
t = mppflow.min_total_time(p.graph, p.starts, p.goals)   # searches the horizons T
print("total time", t.objective, "at T =", t.T)
g = mppflow.from_adjacency([[1], [0, 2], [1]])       # path 0-1-2 (self-loops added)
try:
    mppflow.min_makespan(g, [0, 2], [2, 0])          # two robots cannot swap on a path
except mppflow.Infeasible as e:
    print("infeasible:", e)
mppflow.write_dat(p, "problem.dat")                  # the original's serialized format
print(mppflow.read_dat("problem.dat").goals == p.goals)
```

Output (scipy 1.17.1, in about 0.5 s):

```text
optimal 5 5 None
total time 29 at T = 5
infeasible: the instance has no solution for any horizon (joint-state search)
True
```

## Data model

| name | description |
|---|---|
| `Graph` | A graph (undirected unless built one-way with `from_adjacency`): `vertices`, `adj` (ascending neighbour lists that include the vertex itself), optional coordinates `xy`. |
| `from_adjacency(adj, xy=None)` | Builds a `Graph` on `0..n-1` from neighbour lists and adds the wait self-loops (alias of `Graph.from_adjacency`). |
| `Problem(graph, starts, goals)` | An instance: a graph plus the start and goal vertex of each robot. |
| `MathRandom(seed)` | Java `Random` that stands in for the original's global `Math.random()`. With the same seed it produces the same instances. |

## Generators

| name | description |
|---|---|
| `create_2d_grid_graph(rows, cols)` | 4-connected `rows x cols` grid. |
| `create_2d_grid_graph_8connected(rows, cols)` | 8-connected grid. |
| `create_generic_2d_grid_graph_with_holes(rows, cols, percent_obstacle, rng)` | 4-connected grid with randomly removed vertices; the grid stays connected. Its ids stay 1-based with gaps (as in the original): pass it through `convert_graph` before solving. |
| `create_generic_2d_grid_graph_with_holes_8connected(...)` | The same on an 8-connected grid. |
| `convert_graph(graph, start=None, goal=None)` | Renumbers the vertices to `0..n-1` by ascending old id (the `i`-th smallest id becomes `i`), which every solver needs: after the generators with holes, or after removing vertices by hand. Returns the new graph and renumbers the start list `start` and the goal list `goal` **in place**, as the original does: afterwards the caller's lists hold the new ids, ready for the solvers, so do not map them again. They must be lists of equal length (pass both or neither): a tuple raises `TypeError`, unequal lengths or an id that is not a vertex raise `ValueError`, and an error leaves the lists unchanged. |
| `get_random_start_goal(g, number, rng)` | `[starts, goals]`: `number` pairwise distinct random starts and `number` pairwise distinct random goals (a start may equal a goal). |
| `create_grid_problem(col, row, frac_obs, n, rng)` | A grid problem with obstacles and `n` robots, as in the paper's experiments (converted graph). As in the original, the grid has `col` rows and `row` columns; `as_named=True` swaps them. |
| `create_grid_problem_8connected(...)` | The same on an 8-connected grid. |
| `create_n2_puzzle(n, rng)` | The fully occupied `n x n` puzzle (`n^2` robots, random goal permutation). |
| `get_a9_puzzle()` | The original's fixed 9-puzzle instance. |

## File I/O

| name | description |
|---|---|
| `read_dat(file_name)` / `write_dat(p, file_name)` | The original's Java-serialized `.dat` problem files, byte for byte. |
| `write_txt(p, file_name)` / `read_txt(file_name)` | The original's text problem format. `read_txt` is new: the original has no text reader. The adjacency is kept as written (a vertex without `v:v` has no wait loop). Coordinates are integers; `compat="fixed"` raises `ValueError` for a non-integral one (the original truncates), as `write_dat` does. |
| `problem_to_txt(p)` | The text `write_txt` writes, as a string. |

## Optimal solvers

Each solver takes `(graph, starts, goals, *, backend=..., time_limit=..., ...)` and
returns a `SolveResult`. If no plan exists, it raises `Infeasible`; with `t_max`, also when no
plan has makespan <= `t_max`, and with `horizon="single"`, when none fits in the given `T`.
When a goal has no wait self-loop, plans do not extend to longer horizons, so under
`horizon="single"` the `Infeasible` means only that no plan has makespan exactly `T` (or
`min(T, t_max)`), and its message says so; `horizon="search"` tries the shorter horizons.

`gap` must be a finite number >= 0 and `feasibility_budget` a number >= 1 (`ValueError`
otherwise; a budget of 0 or less would leave every infeasibility proof inconclusive).
`status` is `"optimal"` only with a proof for the objective: with `gap > 0` the solver stops
within the gap, so the status is `"feasible"` unless the bound closes (or the lower bound or the
joint-state search proves the value). For the total time, `gap` is the relative gap on the
total time itself: the model maximizes `n*T - TT` (the goal-waiting steps), so the backend gets
the gap `gap * sum(d_i) / (n*T - sum(d_i))` on that objective, which keeps the gap on the total
time within `gap` (the original passes `gap` unchanged, which allows a far larger gap on the
total time on long horizons). The
explicit `gap` (and `time_limit`) override the same settings in `params` on every backend;
a tolerance given only in `params` (HiGHS `mip_abs_gap`, Gurobi `MIPGapAbs`) is honoured, but
then a backend's "optimal" counts as a proof only when the bound closes. `t_max` bounds the
makespan of the returned plan; for the last three solvers the value is then the optimum among
the plans with makespan <= `t_max`, and `horizon="single"` solves at `min(T, t_max)`. `T` and
`t_max` must be integers >= 0 (`TypeError` for other types, `ValueError` for a negative value;
`numpy` integers are accepted).

`time_limit` (seconds) must be `None` (no limit, the default) or a finite number > 0, in
these functions and in `plan` (`ValueError` otherwise: a limit of 0, a negative one or NaN is
an error, not "no limit"). The lower-level solvers that keep the original's signature
(`solve_time`, `solve_total_time`, `solve_distance`, `solve_max_distance`, default `-1.0`) keep
its rule that a limit <= 0 means no limit, and reject NaN in `fixed` mode.
It is the total budget of the call, model building included: the builders
stop at the limit, and the backend gets what is left after the build and its own model
conversion. A backend can still overrun its share (HiGHS presolve does not check the limit), so
the wall time can exceed the limit somewhat. A node, solution or work limit passed in `params`
(e.g. `{"node_limit": 100}`) stops a solve without a decision: the horizon counts neither as
feasible nor as infeasible, the search stops with status `"time_limit"` (or `"feasible"` when a
plan was already found but not proved optimal), and `extra["solver_status"]` or the attempt
records show the backend's status. With `backend="gurobi"`, Ctrl-C during a solve raises
`KeyboardInterrupt`, and a Gurobi stop without a solution other than infeasibility or a limit
(e.g. `NUMERIC`, `MEM_LIMIT`) raises `RuntimeError`, as a HiGHS solver error does.

| name | description |
|---|---|
| `min_makespan` | Minimum makespan: tries `T = T_min, T_min+1, ...` (T-RO Sec. IV-A, Alg. 1); optional `t_max`. An infeasible instance is detected by a joint-state search whose work budget (`feasibility_budget`) grows with the number of failed horizons and with the time spent in the models, after polynomial certificates (robots that must pass each other on a path or change their cyclic order on a cycle, a robot that must cross a bridge of a fully occupied component, one connected component without a plan). The search is practical only for small joint state spaces: an infeasible instance that neither settles runs until `t_max` or `time_limit` (or for a very long time without either). |
| `min_total_time` | Minimum sum of arrival times. It searches the horizon `T` (`horizon="search"`, Alg. 1); `horizon="single"` solves at the given horizon `T` only (`plan(..., horizon="single")` uses the optimal makespan); a longer horizon can give a lower total time, so the status is `"feasible"` (with `extra["optimal_at_T"]`) unless the plan meets the lower bound. The original also solves at one horizon, but at the makespan of a 4-way split, which can exceed the optimal makespan; `compat="original"` reproduces that. |
| `min_total_distance` | Minimum total number of moves (waits are free), with the same horizon search. |
| `min_max_distance` | Minimum over plans of the largest number of moves any robot makes (T-RO eqs. 12-13; not in the original code). |
| `SolveResult` | `paths`, `status`, `objective`, `T`, `lower_bound`, `log`, `attempts` (one entry per model solved, plus one with phase `"joint"` when the builtin makespan loop returns the plan of its joint-state search, `extra["proof"]`), `extra`. |

The horizon search of the last three first solves at the optimal makespan `T0`. When that
value is above the trivial lower bound, optimality over every `T` still has to be proved. On
small instances (at most 10^6 injective configurations) an exact joint-state search
(`mppflow.solvers.joint_optimum`) usually does this in well under a second. Otherwise a model
at the largest horizon that can matter is solved, and that can take minutes. If the time limit
stops that proof, the plan at `T0` is returned with status `"feasible"`. `horizon="single"`
skips the proof, at a cost: the plan is optimal at `T0` only, so its status is `"feasible"`
unless it meets the lower bound. On `create_grid_problem(3, 4, 0.0, 6, MathRandom(35))` it
returns total time 14 at makespan 3, while the search finds 13 at makespan 4.

When a goal has no wait self-loop, a plan for `T` does not extend to `T + 1` and the optimum is
not monotone in `T`. The search then starts at the optimal makespan whatever `T` is given, and
when the joint-state search does not settle the value, it solves every horizon up to the
largest one that can matter (`extra["horizon_search"]["every_horizon"]`; a time limit during
that scan gives status `"feasible"` with `extra["time_limit_in_scan"]`).

## High-level planning

| name | description |
|---|---|
| `plan(problem, objective="makespan", split=1, ...)` | One entry point for all four objectives (`"makespan"`, `"total_time"`, `"total_distance"`, `"max_distance"`). `split=k` (a power of two, at most `planner.MAX_SPLIT` = 1024; `ValueError` otherwise) runs the k-way split heuristic (makespan, total distance). `time_limit` is `None` (no limit) or seconds > 0 (`ValueError` otherwise, see above; `compat="original"` keeps the original's rule that a limit <= 0 means no limit). With `split=k`, it applies to each piece (for total distance, a piece's makespan and distance solves share it; a piece whose limit runs out between them gives status `"time_limit"`), and the pieces run one after another, so the wall time can reach k times the limit (the original runs the pieces in parallel threads). `horizon` (`"search"` or `"single"`, for total time, total distance and max distance) picks the horizon search or one solve at the optimal makespan; it is ignored for the makespan and for `split > 1`. The k-way split is a heuristic and can fail on a solvable instance (status `"failed"`, the reasons in `extra["failures"]`), mostly on sparse graphs such as cycles, where an intermediate configuration taken from the shortest paths may be unreachable (see [paper-vs-code.md](notes/paper-vs-code.md)). Before splitting, the unsplit solve's instance checks run, so an instance proved infeasible (duplicate goals, an unreachable goal, or the joint-state search within `feasibility_budget`) raises `Infeasible` with any `split`. With `compat="original"`, `horizon="search"` raises `NotInOriginal`. It returns `PlanResult`. |
| `PlanResult` | `paths`, `status` (`optimal`: proved optimal for the objective; `feasible`: a plan without such a proof, e.g. a split, a gap, or a single horizon above the lower bound; `time_limit`, `failed`), `value`, the four metrics, `lower_bounds`, `intermediate_goals`, `stats` (`solves`, `lp_solves`, `ilp_solves`, `solver_time`, and per horizon tried, in order, `horizons` and `phases`: `"lp"`, `"ilp"`, or `"joint"` for a makespan plan or split piece that the makespan loop took from its joint-state search with `backend="builtin"`, at a horizon where no model was solved and not counted in `solves`), `log`. For total time and total distance (`split=1`), trailing steps in which every robot waits are removed, so `makespan` is the latest arrival. When one horizon is solved (total time, total distance and max distance with `horizon="single"`: the optimal makespan; total time and total distance under `compat="original"`: the 4-way split's makespan), it is in `extra["horizon"]`; the default horizon search solves several horizons (`stats["horizons"]`) and may take its plan from the joint-state search, so it sets no `extra["horizon"]`. A plan that meets the lower bound is `optimal` in both modes. |
| `PathPlanner` | The original's `PathPlanner`: `plan_paths_advanced`, the split variants and the heuristics, with all their options. |
| `plan_metrics(paths, goals)` | `makespan`, `total_time`, `total_distance` and `max_distance` of a plan. Raises `ValueError` for `None`, a row count other than `len(goals)`, or empty or ragged rows. |
| `available_backends()` | The backends usable here, in preference order (`gurobi` is listed when gurobipy imports; its license is not checked). |

## Validation and bounds

| name | description |
|---|---|
| `validate(problem, paths)` | `None` if the plan is valid, else the reason, e.g. `"Path 0 and 1 meet on vertex 4 at time step 2"`. |
| `is_path_set_valid(paths, graph, starts, goals)` | `True` or `False`, as the original's `isPathSetValid`. |
| `path_set_error(paths, graph, starts, goals)` | The original's message for an invalid plan, or `None`. |
| `has_cycles(paths)` | The number of rotations in a plan (robots moving around a cycle together), summed over time steps. |
| `get_makespan_lower_bound(g, starts, goals)` | The original's `getMakespanLowerBound`: the largest **number of vertices** on any robot's shortest path, which is the makespan lower bound + 1. It consumes random draws. `PlanResult.lower_bounds` gives the bound itself. |
| `get_total_time_lower_bound(g, starts, goals)` | The sum of the shortest-path lengths, a lower bound on both total time and total distance. It consumes random draws. |

## Errors

`mppflow.errors` holds every exception. The most common ones are also exported from the
package:

| name | description |
|---|---|
| `MPPError` | The base class of all mppflow errors. |
| `Infeasible` | The instance has no solution, or none within the given horizon bound. |
| `TimeLimitReached` | Reserved; mppflow does not raise it. A time limit shows up in the result instead: status `"time_limit"` with `paths = None`, or `"feasible"` when a plan was found but not proved optimal. Callers who prefer an exception can raise it on `res.status == "time_limit"`. |
| `NotInOriginal` | A feature that does not exist in the original code was asked for under `compat="original"`. |
| `OriginalWouldHang` | `compat="original"`: the Java code would loop forever on this input. |
| `JavaException` | `compat="original"`: the Java code throws here. The subclasses in `mppflow.errors` (`JavaNullPointerException`, `JavaArrayIndexOutOfBoundsException`, `JavaStackOverflowError`, `JavaGRBException`, ...) carry the Java class name in `java_class`. |
| `InexactSolutionWarning` | `compat="original"` with a non-Gurobi backend: an inexact binary value broke the original's `X == 1.0` path extraction. |

## Further modules

These are not re-exported:

* `mppflow.lp`: `LinearModel`, `solve(model, backend)`.
* `mppflow.model`: the ILP builders.
* `mppflow.network`: the time-expanded networks (`FullNetwork`, `CompactNetwork`).
* `mppflow.heuristic` and `mppflow.puzzle`: the ICRA'13 heuristic and the puzzle solver.
* `mppflow.experiments`: the paper's experiment drivers.
* `mppflow.parity`: writes the fixtures that tie the browser engine to the package
  (`python -m mppflow.parity`, see `docs/js/README.md`). It is a development tool: run it from a
  checkout (it writes `tests/parity/`); an installed package needs `--out DIR`.
* `mppflow.viewer`: the matplotlib viewer (below).
* `mppflow.cli`: `python -m mppflow` (below).

`LinearModel.to_canonical_text(names)` writes the model dump format of the golden records
(`tools/golden/README.md`). `mppflow.model.canonical_text` is an alias of it.

## Viewer (`mppflow.viewer`)

Needs matplotlib (`pip install ".[viewer]"` from a checkout, Pillow for GIFs); importing the module does
not load it. Each function takes the instance as its parameter `problem_or_graph`: a `Problem`, a
`(graph, starts, goals)` tuple, or a `Graph` followed by `starts` and `goals`. The second and third parameters are always
`starts` and `goals`, so pass `paths=` and `plans=` by keyword: `show(p, paths=r.paths)`, not
`show(p, r.paths)`. A plan is `paths`, a `PlanResult`, or a mapping with a `"paths"` entry;
`plans` is a mapping `label -> plan` or a list of plans.

| name | description |
|---|---|
| `show(problem_or_graph, starts=None, goals=None, paths=None, plans=None, title=None, ...)` | Opens an interactive window: robots as discs on the graph (shaded starts, unshaded goals, as in T-RO Fig. 1) and a timeline. Moving the mouse over the timeline scrubs time; hovering a robot shows its path. Keys: `space` play / pause, `left` / `right` step, `k` cycles the plans, `g` toggles the start / goal markers, `q` closes. |
| `save_gif(path, problem_or_graph, starts=None, goals=None, paths=None, plans=None, ...)` | Writes an animated GIF of a plan (or, with `all_plans=True`, of every plan in turn). |
| `render_frames(problem_or_graph, starts=None, goals=None, paths=None, ...)` | The frames as `H x W x 4` `uint8` arrays (off screen). |
| `make_viewer(problem_or_graph, starts=None, goals=None, paths=None, ...)` / `Viewer` | An off-screen viewer with its events connected (for tests and scripts) / the class behind it. |
| `layout(graph)`, `spring_layout(n, edges)` | Vertex positions: grid coordinates when every vertex has them, else a deterministic spring layout. |
| `plan_from(obj, label=None)` | `(label, paths, info)` for any accepted plan form. |

```python
import mppflow, mppflow.viewer
p = mppflow.create_n2_puzzle(3, mppflow.MathRandom(1))
plans = {f"k={k}": mppflow.plan(p, "makespan", k) for k in (1, 2)}
mppflow.viewer.show(p, plans=plans, title="9-puzzle")
```

## Command line (`python -m mppflow`, also the `mppflow` script)

| command | what it does |
|---|---|
| `solve` | Plans one instance: a file (`.dat` of the original, the text format, or JSON of the shape `{"graph": {"adj": [[1], [0, 2], [1]]}, "starts": [0], "goals": [2]}`: plain neighbour lists such as this `"graph"` get the wait self-loops, while the `Graph.to_dict` and golden-record encodings are used as written, see `mppflow.experiments.problem_from_json`; the output of `solve --json --include-problem` can be solved again) or a generated one (`--grid 8x8 --obstacles 0.2 --robots 20`, `--puzzle 3`), with `--objective`, `--split k`, `--horizon` (`search` or `single`), `--backend`, `--time-limit` (with `--split k`, per piece), `--compat`, `--seed`, `--json`, `--no-paths` (no paths in the text output, no `"paths"` key in the JSON), `--save-problem`. Exit status 0 when a plan was found. |
| `view` | Plans as `solve` does (same instance options) and animates the plan with `mppflow.viewer`: `view --puzzle 3`, `view --grid 8x8 --robots 20 --split 1,2,4` (`k` in the window cycles the plans). `--gif FILE` writes a GIF instead of opening a window. |
| `generate` | Regenerates the benchmark data set of the original's `Problem.main` from a seed (`--out DIR --seed S`). `--families` selects and orders families; they share one random stream, so only the default list reproduces the files of a full run with the same seed. Exit status 1 if a file could not be written. |
| `experiment` | Runs one of the original's experiment drivers (`Main`) on a data folder, optionally reduced. |
| `case` | The original's `Main.main` demo cases 0..6. |
| `figures` | Which driver produced which paper figure or table. |

`python -m mppflow <command> --help` lists every option, and `python -m mppflow --version` prints the version. A missing
optional dependency (gurobipy for `--backend gurobi`, matplotlib for `view`) prints one `error:` line, exit status 1,
and so does a Gurobi license failure (`error: GurobiError: ...`, for example a model too large for
pip's size-limited license).

## Running the tests

```sh
python3 -m pytest -q tests                 # everything (about 10 to 20 minutes, depending on the machine and its load)
python3 -m pytest -q -m "not slow" tests   # skips the long-running tests (about 4 minutes)
```

The markers (`slow`, `gurobi`, `java`) are registered in `pyproject.toml`, and
`--strict-markers` is on. Tests that need gurobipy skip themselves when it is not
installed or has only pip's size-limited license. The license is probed once, after
collection, and only when a selected test needs it (a token-server license whose server
cannot be reached makes the probe wait about a minute); `MPPFLOW_TEST_GUROBI=0` skips those
tests without probing, and the summary line `gurobi license probe: ...` reports the result. The `java` tests also need `MPPFLOW_JAVA_HARNESS` (see
`tools/golden/README.md`).
