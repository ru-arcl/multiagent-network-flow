# The browser engine (`docs/js/`)

The browser demo runs our algorithms in plain JavaScript. Two classic scripts hold the
engine. They have no dependencies and no build step, and they work from `file://`:

| file | what it is |
|---|---|
| `engine.js` | defines `MPP` (`window.MPP` in a browser, `self.MPP` in a worker, `module.exports` in Node): a line-by-line port of the Python package `mppflow` in its default `compat="fixed"` mode |
| `lp.js` | `MPP.lp.solve`: a line-by-line port of `mppflow/lp/builtin.py`, the dense bounded-variable simplex with depth-first branch and bound |
| `test/parity.test.js` | `npm test`: checks both files against the Python package, exactly |
| `bench.js` | `node docs/js/bench.js`: the solver timings below |

Load both scripts, in either order:

```html
<script src="js/lp.js"></script>
<script src="js/engine.js"></script>
```

## What `MPP` contains

Each function has the semantics of the `mppflow` function named next to it, in `fixed`
mode, with `backend="builtin"`. It returns the same values and consumes the same random
draws.

| JavaScript | mppflow |
|---|---|
| `new MathRandom(seed).random()`, `JavaRandom`, `JavaIntHashSet` | `_java` (`Math.random()`, `java.util.Random`, the iteration order of `HashSet<Integer>`, treeified bins included) |
| `grid(rows, cols)`, `gridWithHoles(rows, cols, frac, rng)`, `createGridProblem(col, row, frac, n, rng)`, `createN2Puzzle(n, rng)`, `fromAdjacency(adj, xy)` | `create_2d_grid_graph`, `convert_graph(create_generic_2d_grid_graph_with_holes(...))`, `create_grid_problem` (as in the original, `col` is the number of **rows**: the paper's 24x18 grids have 24 rows and 18 columns), `create_n2_puzzle`, `Graph.from_adjacency` |
| `bfsDistances`, `findGoalsInDistanceRange`, `findShortestPaths`, `getMakespanLowerBound`, `getTotalTimeLowerBound`, `lowerBounds` | `pathfinder` (and `PlanResult.lower_bounds`) |
| `reachability(g, s, t, T)`, `compactArcs`, `network(g, s, t, T, {variant: 'full' \| 'compact', prune})` | `update_reachable_sets`, `compact_arcs`, `FullNetwork` / `CompactNetwork` `.to_json()` |
| `buildModel(kind, g, s, t, T, {integer, setOptimal})`, `model.canonicalText()` | `build_time_model` (`'makespan'`), `build_time_r_model`, `build_total_time_model`, `build_distance_model` (`'total_distance'`), `build_max_distance_model`; `canonical_text` |
| `lp.solve(model, {mipGap, nodeLimit, workLimit, timeLimit, deadlineMs})` | `lp.builtin.solve` |
| `minMakespan`, `minTotalTime`, `minTotalDistance`, `minMaxDistance` | `solvers.min_*`, including the horizon search of Algorithm 1 and the joint-state searches |
| `splitGoals(g, s, t, {random, rng})`, `splitConfigurations(g, s, t, splits, {rng})` | `PathPlanner.split_paths`, the split loop of `multi_threaded_split_planning` |
| `planSplit(g, s, t, k, {rng, timeLimit, timeLimitPerPiece})` | `plan(problem, "makespan", split=k)`: `k = 1` is the optimal solve (LP relaxation first), and `k = 2, 4, 8, 16` is the k-way split |
| `validate`, `pathSetError`, `hasCycles`, `metrics`, `mergePaths` | `is_path_set_valid`, `path_set_error`, `has_cycles`, `plan_metrics` (camelCase keys), `merge_paths` |

A graph is a plain object `{n, adj, xy, rows, cols}`. Its vertices are `0..n-1`. `adj[v]`
is ascending and includes `v`, because waiting is a move. `xy[v]` is `[x, y]` or `null`.
For grids, `rows` and `cols` are the grid's size, and no engine function reads them.
(`mppflow` sets them to 0 after `convert_graph`.) A plan is `paths[robot][t]`. When no plan
exists, a solver throws `MPP.Infeasible`. `timeLimit` (seconds) is `null` or left out for no
limit, or a finite number > 0; anything else (0, a negative limit, `NaN`) throws, as in
`mppflow.plan`.

In the drawn networks, an arc's `kind` is `green` (a wait), `blue`, `gadget` (with `part`
`in`, `mid` or `out` in the full network, and the two move arcs of an edge in the compact
network) or `loopback`. `pyKind` holds the Python kind.

`rng` arguments default to a module-level `MathRandom(0)`, as `rng=None` does in Python.
Pass `new MPP.MathRandom(seed)` to reproduce a Python run.

## How parity is enforced

1. `python -m mppflow.parity` writes `tests/parity/*.json`. Each file holds the inputs and
   the outputs of the Python package for every function above. They include the
   SHA-256 of the canonical model text for every objective (and the full text of the
   small models), and the builtin solver's solutions with their simplex iteration and
   branch-and-bound node counts. They also include split goals, the full 16-way split of
   a 24x18 instance, and the plans of every planner.
2. `npm test` (Node 18 or later, no dependencies) runs `test/parity.test.js`. The test
   recomputes every case in JavaScript and requires **exact** equality. Floats are
   compared with `Object.is`, so they must be bit-identical, and `-0` differs from `0`.
   The test also loads both files as classic scripts in a sandbox (`window.MPP`), and it
   runs `MPP.workerSource()` as a worker.
3. `tests/test_parity.py` regenerates the fixtures in memory and compares them with the
   committed files. A change to the package that changes any of these outputs fails
   there until the fixtures are regenerated. If Node is installed, the test also runs
   `npm test`.

`lp.js` can match exactly because `builtin.py` was written to be ported. It uses plain lists
of doubles, loops in a fixed order, only `+ - * /`, comparisons and `abs`, and a floor
computed by truncation. It never sums in an order that depends on hashing. JavaScript
numbers are IEEE doubles, so the same operations in the same order give the same bits.
Python's `round` (half to even) is `MPP.pyRound`.

The fixtures come in two tiers. The **fast** tier (`java`, `graphs`, `pathfinder`,
`network`, `models`, `lp`, `planners`) is regenerated in about 20 s of Python. The **slow**
tier (`lp_slow`, `planners_slow`: the 9-puzzle ILP, a work-limited 16-puzzle LP, and the
9-puzzle planned with k = 1, 2, 4) takes minutes in pure Python. Its `pytest` check carries
the `slow` marker. `npm test` runs both tiers in about 20 s. To regenerate:

```sh
python -m mppflow.parity            # all fixtures
python -m mppflow.parity --fast     # the fast tier only
python -m mppflow.parity lp models  # selected files
npm test
```

## Live solving in the browser

Run the solvers in a Web Worker so that the page stays responsive. From `file://`, a worker
cannot load a script file, but a worker built from a Blob works. Chrome is checked.
`MPP.workerSource()` returns the complete worker program: `lp.js`, `engine.js` and a small
message handler. `MPP.createWorker()` builds the worker from it:

```js
const w = MPP.createWorker();
w.onmessage = (e) => { /* e.data = {id, ok: true, result} or {id, ok: false, error: {name, message}} */ };
w.postMessage({ id: 1, fn: 'planSplit', args: [graph, starts, goals, 2, { timeLimit: 20 }], seed: 1 });
```

`seed` adds `rng: new MathRandom(seed)` to the options object, which is the last argument.
To stop a solve, call `w.terminate()`.

### Timings and the size cap

The table shows Node 22 on one core of a desktop machine (`node docs/js/bench.js`; its
last section prints the model sizes of the 40-robot rows without solving them; the
9-puzzle rows of both tables come from one run of `node docs/js/bench.js --quick`, which
leaves out the 16-puzzle ILP, the 16-puzzle split and most 24x18 pieces but still runs the
three models that stop at the 60 s limit, about 4 minutes in all). Each
row is a single `lp.solve` of a makespan model at the given horizon, unless stated
otherwise. "LP" is the relaxation, which `planSplit(k=1)` solves first.

| model | T | columns x rows | time |
|---|---|---|---|
| Fig. 4(a) tree, Fig. 8 4x4, star swap (all four objectives) | 3-5 | 16-70 x 18-59 | < 0.01 s each |
| 9-puzzle (Fig. 1), LP | 4 | 264 x 210 | 0.07 s (1531 iterations) |
| 9-puzzle (Fig. 1), ILP | 4 | 264 x 210 | 1.2 s (55025 iterations, 32 nodes) |
| 9-puzzle total-time ILP (the second model of `minTotalTime`) | 4 | 309 x 257 | 3.2 s (112951 iterations, 45 nodes) |
| 5x5 grid, 20% obstacles, 4 robots | 5 | 121 x 119 | < 0.01 s |
| 6x6 grid, 10% obstacles, 8 robots | 5 | 269 x 253 | 0.08 s |
| 8x8 grid, 20% obstacles, 10 robots | 6 | 515 x 442 | 0.33 s |
| 24x18 grid, 25% obstacles, 50 robots: pieces of the 16-way split, LP | 2-3 | 228-819 x 257-777 | 0.002-0.17 s |
| the same, pieces of the 8-way split, LP | 3-4 | 511-1206 x 503-1129 | 0.03-0.8 s |
| the same, pieces of the 4-way split, LP | 7-8 | 3898-6767 x 3184-5136 | 23 s to over 60 s |
| 16-puzzle (seed 2016), LP | 6 | 1503 x 792 | over 60 s |
| 8x8 grid, no obstacles, 40 robots (the demo's crowded map): pieces of the 4-way split (model size only) | 3; 4 | 495-556 x 514-570 (infeasible at T = 3); 1413-1495 x 1150-1197 | (T = 3 under the cap, T = 4 over it) |
| the same, pieces of the 2-way split; the unsplit model | 6; 10 | 3804-3975 x 2285-2351; 14346 x 6008 | (far over the cap) |
| 8x8 grid, no obstacles, 20 robots, LP | 12 | 14061 x 5658 | far over 60 s |

The next table shows whole planner runs.

| run | time |
|---|---|
| Fig. 8: `planSplit` k = 1 and k = 2 (makespan 4 and 5); `minTotalTime`; star `minTotalDistance` | < 0.02 s each |
| 9-puzzle `planSplit` k = 1 | 1.9 s |
| 9-puzzle `planSplit` k = 2 | 4.7 s |
| 9-puzzle `planSplit` k = 4 | 29 s (fully occupied pieces fail several horizons each) |
| 9-puzzle `minTotalTime` (two ILPs at T = 4: the makespan, then the total time) | 4.3 s |
| 16-puzzle `planSplit` k = 4 | over 240 s (time limit) |
| 24x18, 50 robots, `planSplit` k = 16 (60 s limit per piece) | 504 s, and one piece hit the limit |
| 24x18, 50 robots, `planSplit` k = 8 (60 s limit per piece) | 311 s, and pieces hit the limit |

A dense pivot costs `O(rows x columns)`. The iteration count grows with the model size and
with its degeneracy: fully occupied puzzles and models with free waits are the worst
cases. These numbers give the **recommended live-solve cap**:

* **Solve live only if the makespan model at the lower-bound horizon has at most 800
  columns.** `MPP.liveSolveCheck(g, starts, goals)` builds that model and returns
  `{T, columns, rows, live}`. The cap is `MPP.LIVE_MAX_COLUMNS`. It is a size heuristic, not
  a promise that the solve finishes. Under the cap, one model usually takes well under a
  second, but not always. A fully occupied puzzle can need many branch-and-bound nodes (the
  9-puzzle ILP takes 1.2 s). The total-time models are larger and slower: on the 9-puzzle,
  `minTotalTime` takes 4.3 s, 3.2 s of it in the total-time ILP. And the cap looks only at the
  lower-bound horizon, while the optimal makespan can sit well above it: on a 3x4 grid with 2
  obstacles and 6 robots (lower bound 3, 65 columns), `planSplit(k = 1)` proves T = 4, 5 and 6
  infeasible, and its ILP at T = 7 (467 x 318, still under the cap) does not finish within
  120 s, the demo's largest time limit, in Node or with the Python builtin solver, while
  HiGHS solves the instance (makespan 7) in under half a second. Depth-first branch and bound
  can stall on such a model, so a longer time limit does not always help; the 2-way split
  finds a makespan-10 plan in about 1 s. The cap does not bound the models of an explicit `T`,
  `horizon: 'single'` or the total-time and max-distance searches either, and one LP of such a
  model can stall too: the max-distance model at T = 8 of an 11-vertex graph with 5 robots
  (638 columns) has a node LP on which Bland's rule needs 268876 almost all degenerate pivots
  (no cycle), and the whole solve takes about 9 minutes in Node; HiGHS solves that model at
  once. Each simplex phase stops after `max(1000000, 1000 * (rows + columns))` iterations, and
  the solve then ends with status `time_limit` (the attempt record has
  `raw_status: 'iteration_cap'`) instead of an exception.
* Live: Fig. 4(a), Fig. 8 (k = 1 and k = 2), the star, the 9-puzzle for makespan with k = 1
  and 2, and small grids (up to about 8x8 with 10 robots, usually; see above).
* Play precomputed plans: the 16-puzzle and larger puzzles, the 9-puzzle's k = 4 split and
  its total-time and total-distance plans, the 24x18 grid with 50 robots (every k), and the
  crowded 8x8 instance (every k). Its 4-way pieces are under the cap only at T = 3, where
  they are infeasible; at T = 4, where the split solves them, they have 1413-1495 columns.
* Always pass a `timeLimit` (for example 20 s) and solve in the worker. A solve that runs out
  of time returns `status: 'time_limit'`, and the page can then fall back to the
  precomputed plan.
* **Memory.** `lp.js` keeps the constraint matrix and, per LP, the tableau and a copy of it
  as dense arrays: at most `MPP.lp.denseEntries(columns, rows)` = `rows x columns +
  2 rows x (columns + 2 rows)` numbers. A tab that runs out of memory crashes instead of
  reaching its time limit: in Chrome, the 14346 x 6008 model of the crowded 8x8 instance
  crashed the tab after about 7 s, while 10811 x 4798 did not. Pass `lp: {maxEntries}` to
  refuse larger models: `lp.solve` then throws an error named `ModelTooLarge` before it
  allocates anything, for every model of the solve (each horizon, each piece). The demo
  passes 2e8 (about 1.6 GB of doubles) and does not offer k = 1 when the lower-bound model
  is already over it. This option exists only in JavaScript.
* A terminated worker can stay alive for a few seconds while its heap is freed (about 2 s
  for a 7419 x 3602 model in Chrome). Starting the next solve at once builds a second large
  model next to it, so the demo waits for a while (scaled with the model) before it allows
  the next solve.

## Limits and differences

* Only `compat="fixed"` is ported. `original` mode, the Gurobi and HiGHS backends, the
  local-repair heuristic and the puzzle solver stay in Python.
* `timeLimit` is wall-clock time. With a limit that binds, results depend on the machine,
  as in Python. The parity fixtures use no time limits, only the deterministic
  `nodeLimit` and `workLimit`. A model whose LP stops at a limit returns `x: null`, as in
  Python.
* The k-way split solves its pieces one after another, as `mppflow` does. Each piece gets
  the full `timeLimit` by default, so the wall time can reach k times the limit. The demo
  passes `timeLimitPerPiece: false` instead (an option `mppflow` does not have): the limit
  then covers the whole split, each piece gets the time left, and the split stops at the
  first piece without a plan (`stats.skipped_pieces` counts the pieces it did not attempt).
  Results that finish within the limit are the same either way.
* After a failed makespan horizon, the loop runs the joint-state search (within
  `feasibilityBudget`) as its infeasibility check, as `mppflow` does; when that search reaches
  the goal configuration, the engine returns its plan at the smallest feasible horizon, as
  `mppflow` does with `backend="builtin"`. That plan is recorded as an attempt with phase
  `'joint'` at its horizon (no model was solved there; `stats.solves` does not count it, and
  `stats.horizons` / `stats.phases` list it in order), so the demo can say where a plan, or a
  split's piece, comes from. Depth-first branch and bound needs work exponential
  in the horizon there: a 2-way split piece of an 8-vertex tree with 4 robots took 25-50 s
  before, and now takes milliseconds. A fully occupied puzzle has too many configurations for
  the budget, so the 9-puzzle's k = 4 split still solves every horizon.
* A node limit (`lp: {nodeLimit}`) decides nothing: a horizon stopped by it ends the search
  with status `'time_limit'` (`extra.solver_status: 'node_limit'` for the makespan), never with
  `Infeasible` or a move to the next horizon. `T` and `tMax` must be integers (`TypeError`), and
  so must the start and goal ids, the neighbours given to `fromAdjacency` and `planSplit`'s `k`,
  as in `mppflow`: a string, boolean or fractional id raises `TypeError` instead of being solved
  or reported as `Infeasible`.
* `planSplit(k = 1)` solves the LP relaxation first, as `plan()` does, and `minMakespan`
  does not (as `min_makespan`). Pass `solveRelaxedFirst` to change either.
* Floats are bit-identical between Python and JavaScript. Run time is not: Node is roughly
  10-20 times faster than CPython on the same model. On one machine, the 9-puzzle ILP at
  T = 4 took 24 s in Python and 1.2 s in Node, and its k = 2 split 54 s and 4.7 s
  (`node docs/js/bench.js --quick`, the run of the tables above; the Python times match the
  `python_seconds` recorded in `docs/data/demo-data.js`).
