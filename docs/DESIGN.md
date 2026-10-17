# Design: `mppflow`

`mppflow` is a Python (and browser JavaScript) implementation of our network-flow / ILP
algorithms for optimal multi-robot path planning on graphs (MPP):

> J. Yu and S. M. LaValle. *Optimal Multirobot Path Planning on Graphs: Complete Algorithms
> and Effective Heuristics.* IEEE T-RO 32(5):1163–1177, 2016.

It ports **every non-GUI function** of the original Java code
(<https://github.com/arc-l/optimal-mrppg-journal>) with two behaviours:

* `compat="fixed"` (default): corrected behaviour. It equals the original wherever the
  original is correct (same instances from the same seed, same models in the same column and
  row order, and therefore the same Gurobi solutions).
* `compat="original"`: reproduces the original exactly, bugs and crashes included. A Java
  crash raises the matching `mppflow.errors.Java*` exception. A Java infinite loop raises
  `OriginalWouldHang` as soon as the loop is detected, instead of hanging.

This document records the decisions the code is built on. The public API is in
[API.md](API.md); confirmed deviations of the papers and the original code are in
[`notes/`](notes/).

## 1. Package layout

```
mppflow/
  __init__.py      public API (mppflow.__all__): Graph, Problem, plan(), min_*(), validate(), ...
  __main__.py      `python -m mppflow` (calls cli.main)
  errors.py        Infeasible, TimeLimitReached, OriginalWouldHang, Java* exceptions
  _java.py         JavaRandom, MathRandom (global-stream emulation), JavaIntHashSet,
                   i32/i64 helpers, java_double_str (Double.toString), JavaTreeSet helpers
  graph.py         Graph + generators (grid 4/8-connected, holes, convert_graph, manual)
  problem.py       Problem + generators (grid problems, N^2 puzzles), .dat/.txt I/O,
                   batch generation equivalent to Problem.main
  pathfinder.py    BFS distances, randomized shortest paths (Math.random consumption),
                   distance-range goal search, lower bounds, path metrics
  network.py       time-expanded network: full (paper Fig. 5) and compact (Fig. 6/7),
                   reachability pruning; used by the model builders and the demo
  model.py         ILP model builders, producing a neutral LinearModel in the original's
                   exact column/row order:
                     makespan (Time), one-move-per-step (TimeR / "PMG"), puzzle (TimePuzzle),
                     total arrival time (TT), total distance (DIST), max distance (new)
  lp/
    linear.py      LinearModel (columns, sparse rows, objective, sense)
    builtin.py     dense bounded-variable simplex + depth-first branch and bound, pure Python;
                   the same algorithm, line by line, as docs/js/lp.js (parity-tested)
    highs.py       scipy.optimize.milp (HiGHS) backend: the open-source default
    gurobi.py      gurobipy backend (optional): exact reproduction of the original's solutions
  solvers.py       solve loops mirroring the Java solve() methods: MinMakespan T search with
                   LP-relaxation-first, MinTotalTime, MinTotalDist, MinMaxDist; time limits
  planner.py       PathPlanner: plan (planPathsAdvanced), k-way split variants, PMG/TT/TD
                   entry points, local-repair heuristic, path utilities (validate, cycles,
                   merge, print)
  heuristic.py     HeuristicPathPlanner (our ICRA'13 Sec. VI-C conflict-local ILP heuristic)
  puzzle.py        PuzzleSolver (two-model race, run deterministically)
  experiments.py   Main's solve*/run* drivers and stats writers; seeded data regeneration
  cli.py           `python -m mppflow solve|view|generate|experiment|case|figures`
  viewer.py        matplotlib viewer (optional dependency, loaded lazily): interactive window
                   with time scrubbing and plan cycling, off-screen frames, animated GIFs;
                   `python -m mppflow view`
  parity.py        `python -m mppflow.parity`: writes tests/parity/*.json, the inputs and
                   outputs that the browser engine must reproduce exactly (§7)
```

Optional extras: `gurobi` (gurobipy), `viewer` (matplotlib), `test` (pytest). Importing
`mppflow` loads neither numpy, scipy, gurobipy nor matplotlib.

## 2. Core data model

* **Graph**: vertices `0..n-1` (after conversion), `adj[v]` = ascending list **including `v`**
  (the wait self-loop), optional coordinates `xy[v] = (x, y)` (x = column, y = row), and
  `rows`, `cols` as the original stores them (0 after `convert_graph`). Adjacency iteration
  order is the TreeSet order (ascending). Sub-graphs built by the planner heuristics keep
  the original's `HashSet<Integer>` order; this is modelled by `Graph.adj_order` overrides.
* **Problem**: `graph`, `starts`, `goals` (lists of vertex ids, robot `i` goes
  `starts[i] -> goals[i]`).
* **Paths**: `list[list[int]]`, `paths[i][t]` is the vertex of robot `i` at time `t`; all
  rows have the same length `T+1`. This matches the original's `int[][]`.
* **Randomness**: every random function takes `rng`, a `MathRandom` (a `JavaRandom` that
  stands in for `java.lang.Math.random()`). `MathRandom(seed)` produces the same stream as the
  original JVM after `Math`'s generator is seeded with `setSeed(seed)`. Functions consume
  draws in exactly the order the original does (documented per function), including the
  draws that only break ties. With `rng=None`, a module-level default stream is used,
  mirroring the original's global `Math.random()`.

## 3. Model building (exact order)

The column and row order is the original's (`ILP/MultiagentGraphSolverGurobi*.java`). It is
pinned by the model dumps of the golden records (`tests/golden/models.jsonl`, canonical text
plus SHA-256; format in `tools/golden/README.md`) and summarized here.

1. Reachability: `reach[a][v][t] = dist(s_a, v) <= t and dist(v, g_a) <= T - t`
   (BFS distances; unreachable vertices get distance 0, as in the original).
2. Columns, agent-major. For `t = 1..T`, snapshot the frontier set `rVs` (a Java
   `HashSet<Integer>`, emulated exactly, including the bin-overflow resize). For each `curV`
   in snapshot order and each `nextV` in `adj[curV]` (ascending): if `reach[a][nextV][t]`,
   add the column `x[a, curV -> nextV, t-1]` and add `nextV` to `rVs`. After the inner loop,
   remove `curV` from `rVs` if `not reach[a][curV][t]`.
3. Loopback columns `x[a, goal -> start, T]`, one per agent, with lower bound 1 when
   `set_optimal`. The objective maximizes their sum (makespan) or is replaced (TT/DIST).
4. Rows, in this order:
   (a) vertex capacity: for `t = 0..T-1` and `v = 0..n-1`, the sum of all columns entering
   `(v, t+1)` is `<= 1`, emitted only when it has >= 2 terms (TOR eq. 18);
   (b) flow conservation: for each agent `a`, `t = 0..T`, `v = 0..n-1`:
   `out - in = 0` (TOR eq. 9);
   (c) head-on: for each unordered edge `{u, v}` (`u != v`) and time `t`, the sum of both
   directions over all agents is `<= 1`, emitted only with >= 2 terms (TOR eq. 17), in the
   order of the original's TreeMap over **int32-wrapped** keys
   `i32(min*10^9 + max*10^4 + t)`.
   TT and DIST add their own rows (see §4).
5. The int32 key arithmetic is reproduced in both modes. Ordering is not a bug. In
   `fixed` mode, a key collision (possible only on large graphs with long horizons) is
   detected, and the colliding families are kept separate instead of merged.

## 4. Objectives

| objective | original | paper | notes |
|---|---|---|---|
| makespan | Time.solve: T = T_min, LP relaxation first, then ILP for T, T+1, ... | Sec. IV-A, Alg. 1 | `fixed`: optional `t_max`, raises `Infeasible` when exceeded, when trivially infeasible (a goal not reachable), or when the joint-state search or one of its polynomial certificates (path order, cyclic order, bridges of a fully occupied component, a component without a plan) proves it |
| total arrival time | TT.solve at one horizon T | Sec. IV-C, eqs. (14)–(15) | code's y-chain is exact; the paper's eq. (15) is off by one (see notes) |
| total distance | DIST.solve at one horizon T | Sec. IV-D, eq. (16) | code uses zero-cost waits (correct); the paper's stated unit-cost wait arcs make (16) constant |
| max distance | **missing** | Sec. IV-B, eqs. (12)–(13) | new in `mppflow`; `compat="original"` raises `NotInOriginal` |

In `fixed` mode, total time, total distance and max distance follow Algorithm 1: search
`T in [T_opt, n * T_opt]`, where `T_opt` is the optimal makespan, and keep the best objective
(`horizon="search"`). `horizon="single"` solves at one horizon, as the original does; the
original's horizon is the makespan of a 4-way split, which `compat="original"` uses, while
`plan(..., horizon="single")` uses the optimal makespan. The result is then optimal at that
horizon only, so its status is `"feasible"` unless it meets the lower bound (see
[API](API.md#optimal-solvers)). A `t_max` caps every horizon (a single horizon is solved at
`min(T, t_max)`): the result is the optimum among
the plans with makespan <= `t_max`, and `Infeasible` is raised when the optimal makespan
exceeds it. For max distance the joint-state search keeps only the Pareto-minimal move
vectors of each configuration.
The search solves at `T0 = T_opt`, then proves optimality over all horizons. On small
instances (at most 10^6 injective configurations) it uses an exact joint-state search
(`solvers.joint_optimum`). Otherwise it solves one more model at a horizon `T_hi` that every
optimal plan fits in (see `solvers._search`). That shortcut assumes that a plan for `T` extends
to `T + 1` by waiting at the goals. When a goal has no wait self-loop it does not, the optimum
need not be monotone in `T`, and the search solves every horizon from `T_opt` to `T_hi`
(`solvers._search_every`; the joint-state search still applies). Earlier versions solved
only `T0` and `T_hi` there and could report a worse plan as optimal.

## 5. Backends

`backend="highs"` (default; scipy >= 1.15.3, the oldest release the suite passes on: older
HiGHS releases ignore `gap=` or return wrong results, scipy 1.15.0-1.15.2 take quadratic time
to return the solution of a large model, and a solve warns), `"gurobi"`
(requires gurobipy), and `"builtin"` (pure Python; small models only).

* The model is built column-for-column and row-for-row as in Java, so with `backend="gurobi"`,
  the same Gurobi version (13.0.2 for the golden fixtures) and `Threads=1`
  (`params={"Threads": 1}` / `--threads 1`; the backend does not set it), solutions are identical.
  Golden tests compare plans exactly only under this backend. Other backends are checked on
  objective values and plan validity.
* The original sets only `OutputFlag`, `TimeLimit` (1e10 when there is no limit) and, for
  TT/DIST, `MIPGap`. Its puzzle model (`TimePuzzle`) sets only logging parameters
  (`OutputFlag` 1, `LogToConsole` 0 and a `LogFile` under `C:\Temp`) and no `TimeLimit`. We set
  the same solver parameters, except that with no limit we set no `TimeLimit` at all (Gurobi's
  default is 1e100), and we write no log file (`OutputFlag` 0, see
  [B17](notes/original-bugs.md#b-puzzle)).

## 6. Time limits and determinism

Wall-clock limits make the original machine-dependent. Both modes accept `time_limit`
(seconds, `None` = unlimited). In `fixed` mode, `plan` and the `min_*` functions accept only
`None` or a finite limit > 0 (`ValueError` otherwise), so a computed budget that reaches 0 is
an error rather than an unbounded solve; the solvers with the original's signature keep its
sentinel (`-1.0`, any limit <= 0: no limit) and reject NaN. `compat="original"` reproduces the
original's accounting, including its repeated subtraction of elapsed time and the "limit <= 0
means 1e10" lift.
Multi-threaded split planning runs its pieces sequentially. Its results are identical,
because thread interleaving changes only RNG tie-breaks, which do not change path lengths.
The RNG is consumed in thread-index order.

## 7. Browser demo (`docs/`)

* Static page (`docs/index.html`, `docs/css/demo.css`, `docs/js/demo.js`): vanilla JS, classic
  `<script>` tags, no build step, no network access; it works from `file://` and is served by
  GitHub Pages from `main` `/docs` (`docs/.nojekyll` turns off Jekyll, so files such as
  `notes/repro/_common.py` are served and `{{...}}` in the notes is not read as Liquid).
* `docs/js/engine.js` (`window.MPP`) ports, line by line and in `fixed` mode: JavaRandom /
  MathRandom, the JavaIntHashSet emulation, graph and problem generators, BFS and lower
  bounds, reachability, the full and compact time-expanded networks, the model builders
  (makespan, total time, total distance, max distance), the four solvers with the horizon
  search, k-way split goal selection, path validation, metrics and cycle counting.
  `docs/js/lp.js` ports `mppflow.lp.builtin` (`docs/js/README.md`).
* Parity: `tests/parity/*.json` is generated by `python -m mppflow.parity` (inputs plus
  expected outputs, in a fast and a slow tier). `npm test` runs `docs/js/test/parity.test.js`
  under Node (no dependencies) and requires exact equality, bit for bit on floats.
  `docs/js/test/demo-data.test.js` checks the precomputed plans of `docs/data/demo-data.js`
  with the engine (validity, stored metrics and values, statuses, lower bounds).
  `tests/test_parity.py` checks that the fixtures are current.
* Maps and plans: `docs/data/demo-data.js` (`window.MPP_DEMO`), generated by
  `docs/data/build_demo_data.py` with the public API (committed build: Gurobi 13.0.2). Each map
  is a paper figure (Figs. 1/3, 4(a), 8, 14/15), the star graph of the original code's `Main`
  case 3, or an instance regenerated from a seed (16-puzzle, crowded 8x8, 24x18 with 50 robots); `docs/data/README.md` gives the provenance and format.
  Every plan is validated before it is written.
* Live solving: plans marked `live` (small models whose builtin solve stays within a
  deterministic simplex-iteration budget), the k-way split for the makespan, and instances
  edited in the page (grids up to 8x8: drag starts and goals, randomize, obstacles, add robots) are solved in
  a Web Worker created from a Blob, which works from `file://` (without Workers, on the main thread, a
  k-way split piece by piece, with the same options and memory guard). The page's time limit covers the whole
  solve: a k-way split gives each piece the time left and stops at the first failed piece (engine option
  `timeLimitPerPiece: false`), and every LP gets the same wall-clock deadline. `plan(split=k)` instead
  gives each piece the full limit.
* Views: the graph with robots and goals (play, step, scrub, speed, zoom, path highlight),
  the goal configuration, a metrics table against the lower bounds, and the time-expanded
  network (full or compact drawing, the plan's flow, a horizon slider) with the merge-split gadget.
  Light and dark themes; the URL hash selects the map and plan and stores edited instances
  (`#edit=`).
* `tools/check_demo.mjs` (Node >= 22, headless Chrome, no dependencies) plays every
  precomputed plan, runs a live solve per live map, exercises the controls and edit mode,
  checks phone widths, contrast, live regions and keyboard moves, repeats the split and the memory guard
  with `window.Worker` hidden, and fails on any console error or network request.
* `docs/media/`: the README figures (`make_concepts.py`, drawn from `mppflow` output) and
  `demo.gif` (screenshots of the page, `make_demo_gif.mjs` + `make_demo_gif.py`).

## 8. Testing

`python -m pytest` from the repository root (`pyproject.toml` puts `.` and `tests/` on the path;
markers `slow`, `gurobi`, `java`, with `--strict-markers`). `python -m pytest -m "not slow"`
skips the long-running tests. Tests that need gurobipy, Node or the Java harness skip
themselves when it is missing.

* `tests/golden/*.jsonl`: records of the live original (JDK 17, Gurobi 13.0.2), written by
  `tools/golden/` (a Java harness compiled against the unmodified original sources;
  Math.random seeded by reflection). They cover generators, BFS draws, lower bounds, split
  goals, model dumps (canonical text plus SHA-256), solver plans, planner variants,
  utilities, stats lines, crashes (exception class) and hangs (detected with a watchdog).
  The `test_*.py` module of each area compares against them (`test_golden_core.py` for the
  `graph`, `problem` and `pathfinder` groups); plans are compared exactly only under the
  Gurobi backend (§5).
* `tests/test_paper_examples.py` with `tests/data/paper/*.json`: every worked example in the
  papers (9-puzzle, 25-puzzle, Figs. 4-8 networks and the split example, the eq. (15)
  counterexample, ...), each with an independent verification.
* `tests/data/core/*.json.gz`: generator, pathfinder and problem I/O cases (graphs, `.dat`
  streams and rows, writers, benchmark families, HashSet tree bins), recorded from the
  original Java code (`tools/golden/core/`, see `tools/golden/README.md`).
* `tests/oracles.py`: independent oracles written from the problem definition only
  (joint-state BFS for exact makespans on tiny instances, brute-force enumeration for total
  time and distance), used on randomized inputs together with HiGHS / builtin / Gurobi
  agreement.
* `tests/parity/*.json` and `tests/test_parity.py`: the browser engine's fixtures (§7).
* Differential: `tools/golden/differential.py` (and `tests/test_differential.py`, marker
  `java`) runs the live original on fresh random inputs and compares; it needs Java, Gurobi
  and `MPPFLOW_JAVA_HARNESS`.
* `npm test`: the JavaScript engine against the parity fixtures, and the demo's precomputed
  plans against the engine.

## 9. Paper and code errata

`docs/notes/` documents every confirmed deviation, with reproduction scripts in
`docs/notes/repro/`: paper errors E1-E16 (`paper-errata.md`), original-code bugs B1-B27
(`original-bugs.md`), paper-versus-code differences (`paper-vs-code.md`), and solver
behaviour worth knowing (HiGHS presolve defects, Gurobi determinism, builtin limits;
`solver-notes.md`).
