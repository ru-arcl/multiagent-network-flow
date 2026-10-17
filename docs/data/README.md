# Demo data

`demo-data.js` holds the maps and precomputed plans that the browser demo plays.
`build_demo_data.py` generates it with the public `mppflow` API. Do not edit the generated
file by hand.

## Regenerating

From the repository root:

```sh
python3 docs/data/build_demo_data.py                          # every map
python3 docs/data/build_demo_data.py --only tor16-fig8-split  # one map; keeps the others
python3 docs/data/build_demo_data.py --list                   # the map ids
python3 docs/data/build_demo_data.py --refresh-text           # texts only (captions, sources, plan notes); keeps every plan
```

The ILP backend is Gurobi when `gurobipy` is installed, and HiGHS (`scipy`) otherwise.
`--backend` picks one. The file records the backend and its version, and so does every
plan. The committed file was built with Gurobi 13.0.2.

A full build takes roughly an hour, mostly on two jobs:

* The total-time plan of the 24x18 map: one total-time ILP at T = the optimal makespan,
  which took about 34 minutes with Gurobi (its limit is 2400 s).
* The live checks (see below), which run the pure-Python solver on the 9-puzzle models.

`--time-limit` bounds each solve (default 1800 s per plan; the 24x18 total-time plan has
its own limit of 2400 s). `--seed` sets the `MathRandom` seed passed to every
`mppflow.plan` call (default 1). The split heuristics draw from this stream, and every
plan records it as `rng_seed`.

A rebuild with the same backend version gives the same plans, except in two places:

* The `seconds` fields are wall-clock times, so they always change.
* The 24x18 `total_time/k1` plan is kept only if the solver proves it optimal at its one
  horizon within the 2400 s limit, so whether it is built depends on the machine. If it is
  not, the map lists it under `skipped`.

Another backend can return a different plan with the same optimal value, or a different
split plan.

## Where the maps come from

The original benchmark files of the T-RO 2016 experiments are lost. Each map is either
transcribed from a figure of the paper or regenerated from a seed with `mppflow`'s ports of
the original generators. A `MathRandom(seed)` stream reproduces the original
`Math.random()` after `setSeed(seed)`.

| id | provenance | plans |
|---|---|---|
| `tor16-fig1-9puzzle` | T-RO 2016 Fig. 1 (instance) and Fig. 3 (plan), p. 1165, from `tests/data/paper/tor16_fig1_fig3_9puzzle.json`. The `paper` plan is Fig. 3, read arrow by arrow. Our `makespan/k1` plan is the plan the original Java code returns with Gurobi 13.0.2 (`matches_original_java: true`), and that plan is Fig. 3. | all four objectives with k = 1; makespan with k = 2, 4; paper |
| `tor16-fig4a-tree` | T-RO 2016 Fig. 4(a), the two-robot tree of Figs. 5-7, from `tests/data/paper/tor16_fig4_fig7_network.json` (`vertex_names` gives the paper's labels). The coordinates follow the drawing: a vertical trunk with two branches on the left. | all four with k = 1; makespan with k = 2 |
| `tor16-fig8-split` | T-RO 2016 Fig. 8 (4x4 vertices, 3x3 cells), from `tests/data/paper/tor16_fig8_split.json`. `makespan/k1` has makespan 4. `makespan/k2` has makespan 5 (3 + 2, intermediate goals 5 and 10), where the paper says 2 + 2 (see `docs/notes/paper-errata.md`). | all four with k = 1; makespan with k = 2 |
| `main-case3-star` | The star graph of the original code's `Main` case 3 (edges 0-1, 1-2, 1-3; robots 0 -> 3 and 3 -> 0), from `mppflow.graph.create_main_case3_graph`. The coordinates were added for drawing. | all four with k = 1; makespan with k = 2 |
| `random-16puzzle-seed1` | Regenerated, seed 1: `create_n2_puzzle(4, MathRandom(1))`, the port of `Problem.createN2Puzzle`. | makespan with k = 1, 2, 4; max distance; total time and total distance at one horizon |
| `tor16-fig14-25puzzle` | T-RO 2016 Fig. 14 (instance) and Fig. 15 (7-step plan), p. 1173, from `tests/data/paper/tor16_fig14_fig15_25puzzle.json`. The `paper` plan is Fig. 15, transcribed from its 134 arrows (7 panels). Our `makespan/k1` plan equals the original Java code's plan, which is a different optimal plan than Fig. 15 (total distance 144 against 134). | makespan with k = 1, 2, 4; paper |
| `crowded-8x8-40-seed1` | Regenerated, seed 1: `create_grid_problem(8, 8, 0.0, 40, MathRandom(1))`. This is not a paper figure: it is a dense instance (40 robots on 64 vertices) for comparing k-way splits. | makespan with k = 1, 2, 4; max distance |
| `tor16-fig9-24x18-seed1` | Regenerated, seed 1: `create_grid_problem(24, 18, 0.25, 50, MathRandom(1))`, the port of `Problem.createGridProblem`. 25% of the vertices are removed (connectivity is kept) and there are 50 robots, as in T-RO Fig. 9 and Sec. VI-A. The build checks that this is the instance the original Java generator produces with `Math.random` seeded 1 (`tests/data/paper/tor16_java_original_outputs.json`). As in the original generator, the grid has 24 rows and 18 columns; Fig. 9 draws it transposed. The instance drawn in Fig. 9 is lost, so this map is a different instance. | makespan with k = 1, 2, 4, 8, 16; total distance with k = 4; total time at one horizon (optimal at that horizon, solved within 2400 s) |

Some objectives are missing on the larger puzzles and on the crowded grid. In our tests,
an exact total-time or total-distance solve there (the horizon search of Alg. 1) did not
finish within 5 minutes on a loaded machine. On the 16-puzzle, the solve at the optimal
makespan alone finishes in about 2 minutes (Gurobi), so those two plans use one horizon.

`source` in each map says the same in one line. A regenerated map also has `generator`,
the engine call that rebuilds the instance in the browser: `{fn, args, seed}` means
`MPP[fn](...args, new MPP.MathRandom(seed))`.

## Format

`demo-data.js` is a classic script. It sets `window.MPP_DEMO` and works from `file://`.
Under Node, `require` returns the same object.

```text
MPP_DEMO = {
  version: 1, generator: "docs/data/build_demo_data.py", mppflow_version, backend, backend_version,
  maps: [{
    id, title, caption, source,             // source: paper figure, or "regenerated, seed S: <call>"
    figure?,                                // the paper figure of the map, shown in the map menu
    generator?: {fn, args, seed},           // regenerated maps only
    vertex_names?: [...],                   // Fig. 4(a) labels
    graph: {n, adj, xy, rows, cols},        // adj[v] ascending and includes v (the wait move);
                                            // xy[v] = [x = column, y = row], row 0 at the top;
                                            // rows/cols are null for a non-grid graph
    starts, goals,                          // robot i: starts[i] -> goals[i]
    lower_bounds: {makespan, total_time, total_distance, max_distance},   // from BFS distances
    live,                                   // makespan/k1 can be re-solved in the browser
    live_plans: ["<objective>/k<k>", ...],  // every plan that can be re-solved in the browser
    skipped?: [{key, reason}],              // optional plans that found nothing in their time limit
    plans: {
      "<objective>/k<k>": {                 // objective: makespan | total_time | total_distance | max_distance
        objective, k,
        status,                             // "optimal" (proved), or "feasible" (split, single horizon, time limit)
        value,                              // the objective's value for this plan
        paths,                              // paths[i][t]; every row has length makespan + 1
        metrics: {makespan, totalTime, totalDistance, maxDistance},   // mppflow.plan_metrics
        rotations,                          // mppflow.has_cycles
        backend, seconds,                   // seconds: wall clock of the mppflow.plan call
        solves, horizons,                   // number of models solved and the horizon T of each
        phases,                             // "lp" (LP relaxation) or "ilp", one per model
        rng_seed,                           // MathRandom seed of the call
        horizon?, note?,                    // "single": solved at T = optimal makespan only
        model: {columns, rows, T} | null,   // the ILP at the plan's horizon; for k > 1, the largest
                                            // piece (null for a split without intermediate_goals)
        intermediate_goals?,                // splits: the k - 1 configurations between pieces
        pieces?: [{columns, rows, T, t0}],  // splits: each piece's model (of the plan's objective),
                                            // horizon and start time
        matches_original_java?,             // makespan/k1 of the two puzzles
        live,
        live_check?: {backend: "builtin", work_limit, status, value, python_seconds}
      },
      "paper": {paths, note, metrics, rotations, live: false}     // the transcribed paper plan
    }
  }]
}
```

Before a plan is written, the build checks it with `mppflow.validate` and
`mppflow.is_path_set_valid`. This includes the paper plans and the plans of the live
checks. A plan that fails stops the build.

## Live plans

The demo can re-solve some plans in the browser with `docs/js/lp.js`, a pivot-for-pivot
port of `mppflow.lp.builtin`. A plan is marked `live` only if three things hold:

* The builder ran the same `mppflow.plan` call (same seed) with `backend="builtin"`.
* That call reached the same objective value.
* Each model it solved took at most `--live-work` simplex iterations (default 100000).

The iteration budget is deterministic, so the flag does not depend on the build machine.
The builder only tries models with at most `--live-columns` columns (default 600).

`live_check` records what the builtin solver returned. Its plan can differ from the stored
plan when several plans are optimal. The live plans are those of the tree, the Fig. 8
grid and the star, plus the 9-puzzle's `makespan/k1`, `max_distance/k1` and
`makespan/k2`. Each 9-puzzle check takes about 25 to 100 s in CPython (25 to 60 s for the
three live plans). The 9-puzzle's total time, total distance and `makespan/k4` checks stop
at the iteration budget without a plan (`status: "time_limit"`, `value: null`); the page then
says that the build's Python run did not finish, rather than quoting its time as a solve time.
