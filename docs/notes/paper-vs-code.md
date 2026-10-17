# Our papers versus the original code

Where the experiments in our papers (J. Yu and S. M. LaValle, T-RO 32(5) 2016 **[TRO16]**;
ICRA 2013 **[ICRA13]**) and the released Java code
(<https://github.com/arc-l/optimal-mrppg-journal>, `source/projects/multipath/`) differ, and how
`mppflow` handles each difference. Errors in the papers are in [paper-errata.md](paper-errata.md),
bugs in the code in [original-bugs.md](original-bugs.md).

Unless stated otherwise, `mppflow` keeps the code's experiment design in **both** compat modes,
since that is what produced (or most likely produced) the published data: the drivers in
`mppflow.experiments` loop over the same robot counts, obstacle levels, time limits and gap
formulas as `ILP/Main.java`. The reproductions are in [`repro/`](repro/); the output shown is
what they print.

## 1. Experiment parameters

| item | paper | code | mppflow |
|---|---|---|---|
| "24 × 18" grids (Figs. 9–12, 18–20) | "24 × 18 grid"; Fig. 9 is drawn rotated | 24 rows × 18 columns: `Problem.createGridProblem(col, row, ...)` passes `col` as the row count (`advanced/Problem.java:126-140`) | the same (`create_grid_problem(24, 18, ...)`); `as_named=True` swaps the roles |
| 24x18 obstacle levels | 0–25% | 0–30% in steps of 5, generated and run (`Problem.java:240-250`; `Main.java:549, 574, 762, 791, 821`) | the same; `Scale(obstacles=...)` selects levels |
| 8x8 and 16x16 grids (Fig. 16) | "8 × 8 grids, 16 × 16 grids, and 32 × 32 grids with 20% obstacles" (Sec. VI-C) | no obstacles (`Problem.java:205, 212`); 20% only for 32x32. The paper's ratios (60 robots = 94%, 190 = 74%) agree | the same |
| Fig. 16 time limit | 600 s per instance | 1800 s (`runCrowdedPerformanceMetricCrowded`, `Main.java:729-752`); the legacy `runFullPerformanceMetric` uses 3600 s | the same; `Scale(time_limit=600)` gives the paper's value |
| Fig. 17 (32x32, 20% obstacles) | exact, 2-, 4-, 8-way k-way split, up to 180 robots | split sections use the **cycle-free** split planner (`noCycle = true`, `Main.java:611`), 2/4/8/16-way, up to 200 robots, 1800 s | the same; the cycle-free planner is `PathPlanner.plan_paths_advanced_split_no_cycle` |
| Fig. 19 (MINTOTALDIST with split) | four-way split | `split = 2` (4-way), switched permanently to 3 (**8-way**) for 25% and 30% obstacles (`Main.java:820-822`) | the same |
| Fig. 20 (fixed time, vs. ID) | 600 s; TT n = 20..120, TD n = 20..140; 0/10/20% obstacles; "we do not set a suboptimal threshold here" (Sec. VI-D) | the most likely drivers use **100 s** and a **MIP gap of 0.001** (`solveProblemTotalTime(p, 0.001, 100)`, `solveProblemDistanceSplit(p, 0.001, 100, 2)`, `Main.java:859, 890`); TT n = 70..100, TD n = 10..150; 0–20% in steps of 5 (`Main.java:848-905`; TT is `main` case 0) | the same; `Scale` overrides |
| Fig. 18 gap | "automatically adjusted" | `n*0.002 + obs*0.002 (+ n*0.001 if n > 50) (+ n*0.0005 if n > 100)` (`Main.java:769`), applied by Gurobi to the solver's objective `nT − TT` (the goal-waiting steps), so the gap on the total time itself is up to `gap · (nT − TT) / TT`, larger than the nominal gap whenever the robots wait at their goals for more steps than they travel (the usual case) | `original`: the same; `fixed`: the gap applies to the total time (the backend gets `gap · Σd / (nT − Σd)` on `nT − TT`) |
| Fig. 10 "exact" runs | MINMAKESPAN | `solveProblemSuboptimal(..., 600, 0, false)`: the k-way split with one piece (no LP relaxation first; measured times have the 100 ms polling floor of the threads) | the same (no polling floor) |
| [ICRA13] Table III (heuristic) | 25–150 robots, 100 runs, 30 s | `run32x32PerformanceMetricHeuristic` ("do not use!"): n = 20..200, 10 runs labelled "100 runs", file names that the generator never writes, attempt budgets tabulated only for n = 25, 50, ..., 150 | `original`: the same; `fixed`: the generator's names and the real run count |
| [ICRA13] Table I (20x15 grid) | 1000 s | no generator, no driver | – |

The 8x8/16x16 and 24x18 values are checked below; the others are constants in `Main.java`
(`python -m mppflow figures` prints the mapping of section 4).

`python3 docs/notes/repro/pvc_grids.py`

```text
create_grid_problem(24, 18, 0, 10): 24 rows x 18 columns
24x18 vertices left per obstacle level: {0: 432, 5: 411, 10: 389, 15: 368, 20: 346, 25: 324, 30: 303}
8x8 with 0% obstacles: 64 vertices, 60 robots = 93.8%
8x8 with 20% obstacles: 52 vertices, 60 robots = 115.4%
16x16 with 0% obstacles: 256 vertices, 190 robots = 74.2%
16x16 with 20% obstacles: 205 vertices, 190 robots = 92.7%
```


## 2. Algorithms

### TT and DIST are solved at one horizon (no search up to n·T_opt)

* **Paper.** Algorithm 1 and Sec. IV-E ([TRO16] p. 1168) search `T` up to `T_max = nT_opt` for
  MINTOTALTIME and MINTOTALDIST; this is what makes them complete and optimal (Prop. 2). The
  remark of Sec. V-B says the k-way split can "quickly compute an initial T".
* **Code.** `Main.solveProblemTotalTime` / `solveProblemDistance` (`Main.java:221-262`) take
  `T` = the makespan of a 4-way split (`planPathsAdvancedSplit(..., -1, 2)`) and solve once at that
  `T`; `planPathsAdvancedSplitDist` (`PathPlanner.java:708-709`) solves each piece at its optimal
  makespan. Nothing searches larger `T`. Usually the split's horizon has enough slack, but not
  always: below, the optimal total time needs robot 0 to wait (makespan 32 > 29).
* **mppflow.** `compat="fixed"`: direct calls (`solve_total_time`, `solve_distance`,
  `Experiments.solve_problem_total_time`, ...) use `horizon="search"` (Algorithm 1);
  `horizon="single"` gives the code's behaviour. The experiment drivers keep the single horizon
  in both modes (the paper's experimental setup) unless `Experiments(horizon="search")`.

`python3 docs/notes/repro/pvc_tt_td_horizon.py`

```text
T_opt = 24 , n * T_opt = 48
original: total time [45, 33], total distance [37, 33]
fixed: total time [41, 33], total distance [33, 33]
fixed, horizon='single': 45 37
```


### MINMAXDIST is not in the code

[TRO16] Sec. VI says "We implemented all algorithms (MINMAKESPAN, MINMAXDIST, MINTOTALTIME, and
MINTOTALDIST)", but the released code has no min-max distance model (no solver bounds every
robot's distance by one variable); Sec. VI-D used MINMAKESPAN in its place. `mppflow` adds it
(`solve_max_distance`, eqs. (11)–(13) on the compact network); `compat="original"` raises
`NotInOriginal`.

`python3 docs/notes/repro/pvc_maxdist.py`

```text
fixed: max distance: 3
original: NotInOriginal: MINMAXDIST is not implemented in the original code
makespan (the paper's stand-in): 4
```


### LP relaxation first

`Main.solveProblem` (and every `planPathsAdvanced(..., solveRelaxed = true, ...)`) first raises
`T` until the LP relaxation reaches value `n` (`MultiagentGraphSolverGurobiTime.java:58-85`), then
solves the ILP from there. None of the papers describes this step. It is sound (an LP max-flow
below `n` proves the ILP infeasible) and changes only the work done; `mppflow` keeps it
(`solve_relaxed_first`).

`python3 docs/notes/repro/pvc_lp_first.py`

```text
solveRelaxedFirst=False: makespan 4, models [('ilp', 2, None), ('ilp', 3, None), ('ilp', 4, 2.0)]
solveRelaxedFirst=True: makespan 4, models [('lp', 2, 1.0), ('lp', 3, 2.0), ('ilp', 3, None), ('ilp', 4, 2.0)]
```


### The local-repair heuristic is from ICRA 2013 only

The code's `PathPlanner.planHeuristicPaths` and `HeuristicPathPlanner` implement the heuristic of
[ICRA13] Sec. VI-C ("MinMakespan as a generic heuristic"); [TRO16] replaces it by the k-way split
and does not mention it. The code differs from the [ICRA13] description: the region is the
distance-2 neighbourhood plus a random part of the distance-3 ring (one `Math.random()` per
distance-2 visit, threshold 0.5 or 0.75), robots standing in the region join it, an attempt gives
up after 5 stalled rounds in total (the counter is never reset), and the attempt budgets are
wall-clock tables keyed by the robot count. `mppflow.heuristic` ports both variants exactly,
with a virtual clock by default (see [original-bugs.md](original-bugs.md#b-nondet)).

`python3 docs/notes/repro/pvc_heuristic_region.py`

```text
distance-2 ball: 13 vertices; distance-3 ball: 25
PathPlanner regions, seeds 1..10: [24, 22, 24, 24, 22, 24, 23, 22, 23, 23]
HeuristicPathPlanner regions, seeds 1..10: [19, 21, 22, 19, 17, 15, 19, 17, 19, 21]
```


<a id="split-incomplete"></a>
### The k-way split can fail on a solvable instance

[TRO16] Sec. V-B splits an instance at intermediate goals taken from the robots' shortest paths
and says that once each child instance is solved, "the solutions can be glued together by
concatenating the results". We call the method a heuristic there, which covers longer plans,
but we do not say that a piece can be infeasible. It can: an intermediate configuration may be unreachable from the start, or the
goals unreachable from it, although the whole instance is solvable. On a cycle robots cannot
change their cyclic order, so a configuration that swaps two of them kills the split. Both the
code and `mppflow` (both modes) then return no plan: `plan` reports status `"failed"`
(`compat="original"` reports `"time_limit"` without a time limit, see
[original-bugs.md, B13](original-bugs.md#b-splitdist) and B2). It needs a sparse graph, such
as a cycle or a corridor, where the robots' relative order is constrained. `mppflow` does not
retry with other intermediate goals: the split stays the paper's heuristic. Use `split=1` when
a split fails.

`python3 docs/notes/repro/pvc_split_incomplete.py`

```text
2x2 grid (a 4-cycle): k = 1 optimal, makespan 2
  makespan k = 2: failed, intermediate goals [[1, 0, 3]]
  total_distance k = 2: failed, intermediate goals [[1, 0, 3]]
  makespan k = 4: failed, intermediate goals [[0, 1, 3], [1, 0, 3], [1, 0, 3]]
6-cycle: k = 1 optimal, makespan 3
  makespan k = 2: failed, intermediate goals [[5, 0, 1]]
  total_distance k = 2: failed, intermediate goals [[5, 0, 1]]
  makespan k = 4: failed, intermediate goals [[0, 5, 1], [5, 0, 1], [4, 1, 0]]
original, 2x2 grid, makespan k = 2: time_limit, intermediate goals [[1, 0, 2]]
```

### Other differences

* **Distance costs.** The code charges 1 per move and 0 per wait (`MultiagentGraphSolverGurobiDistance`),
  as it must; the papers' stated costs make the objective constant
  ([paper-errata.md, E2](paper-errata.md#e2)).
* **Cycle-free split.** `planPathsAdvancedSplitNoCycle` (used for Fig. 17) is not in the papers:
  it re-plans pieces whose solutions contain rotations (with random re-splits). It can change
  timings a lot and, on instances whose solutions all rotate, never ends
  ([original-bugs.md, B12](original-bugs.md#b-nocycle)). Example below (3x3 grid, 6 robots, 2-way;
  the pure-Python `builtin` backend, so that the plans, and with them the rotation counts, are the
  same on every platform).
* **Models.** The code builds only the compact network of [TRO16] Sec. V-A (Figs. 6–7, eqs.
  (17)–(18)) with reachability pruning, never the full network of Sec. III; `mppflow` builds both
  (`mppflow.network`, `build_full_network_model`).
* **8-connected grids** (footnote 10) add diagonals as ordinary edges, so two robots may cross
  diagonals of one cell; [ICRA13]'s diagonal gadget (Fig. 6) is not used.

`python3 docs/notes/repro/pvc_nocycle.py`

```text
seed 5: k-way split makespan 7, rotations 1, solves 2 | no-cycle split makespan 5, rotations 0, solves 4
seed 6: k-way split makespan 6, rotations 1, solves 2 | no-cycle split makespan 3, rotations 0, solves 6
seed 7: k-way split makespan 4, rotations 0, solves 2 | no-cycle split makespan 4, rotations 0, solves 2
```


<a id="reproducibility"></a>
## 3. Reproducibility of the published numbers

* **The data files are lost.** The experiments read `D:/temp/data/dmrpp/...` files written by
  `Problem.main` from an unseeded `Math.random()` stream; they are not in the repository, and the
  instances cannot be recovered. `mppflow.experiments.generate(seed, folder)` (CLI:
  `python -m mppflow generate --out DIR --seed S`) regenerates the same families, file names and
  file formats (`.dat` byte-compatible with the original's Java serialization) from a seeded
  stream: the same distribution, not the same instances. All families draw from this one
  stream in the order they run, so a seed fixes the files of the full default run only:
  `--families` with a subset, or with another order, gives other instances for every family but
  the first (e.g. the 24x18 files of `--families 24x18 --seed 1` differ from those of the full
  run with `--seed 1`, which generates the four other families first).
* **JDK version.** Intermediate goals of the split heuristics and the column order of the models
  depend on the iteration order of `HashSet<Integer>`, which changed in JDK 8 (different hash
  spreading, and new entries at the tail instead of the head of a bucket). The code was written
  for JDK 6/7, and the [ICRA13] runs (2012) necessarily used one of them; we did not record the
  JDK of the [TRO16] runs. `mppflow` and our golden fixtures follow JDK 8+ (recorded with JDK 17),
  so split results computed from the same instance can differ from a JDK 6/7 run (example
  below). Optimal makespans do not change, but which optimal plan is returned can.
* **Gurobi.** The plans are reproducible only with the same Gurobi version and `Threads = 1`
  (see [solver-notes.md](solver-notes.md)); the papers do not record either.
* **Time.** All limits are wall clock, so which instances time out (and therefore where each
  curve stops) depends on the machine; the reported times are hardware-specific ([TRO16]: Intel
  Core i7-4850HQ; [ICRA13]: Intel Q6600).

`python3 docs/notes/repro/pvc_hashset_order.py`

```text
candidates, JDK 8+ order: [49, 35, 21, 7, 56, 42, 28, 14]
candidates, JDK 6/7 order: [35, 49, 21, 7, 42, 56, 28, 14]
splitPaths intermediate goal (mppflow, JDK 8+ order): [49]
```

`python3 docs/notes/repro/pvc_regenerate.py`

```text
seed 42 -> (620, ['16x16-grid/10-agts-1001.dat', '16x16-grid/10-agts-1001.txt'], 'c6c413f4f9e7a1b2')
seed 42 -> (620, ['16x16-grid/10-agts-1001.dat', '16x16-grid/10-agts-1001.txt'], 'c6c413f4f9e7a1b2')
seed 7 -> (620, ['16x16-grid/10-agts-1001.dat', '16x16-grid/10-agts-1001.txt'], '624ca70723c99ea1')
seed 1, families n2puzzle -> n2puzzle .dat files: 300 15ae3773d093e50d
seed 1, families n2puzzle,crowded -> n2puzzle .dat files: 300 15ae3773d093e50d
seed 1, families crowded,n2puzzle -> n2puzzle .dat files: 300 0af38161570e7ed6
```


## 4. Figure → driver mapping

`python -m mppflow figures` prints this table from `mppflow.experiments.FIGURES`;
`python -m mppflow experiment <driver> --data DIR` runs a driver (on data from
`python -m mppflow generate`; `--instances`, `--max-agents`, `--obstacles`, `--sections` and
`--time-limit` give reduced runs).

| figure / table | content | `mppflow` driver [sections] | Java method (`ILP/Main.java`) | parameters in the code | deviations |
|---|---|---|---|---|---|
| [TRO16] Fig. 9 | a 24x18 instance, 25% obstacles, 50 robots | – (`create_grid_problem(24, 18, 0.25, 50)`) | – | 24 rows × 18 columns | drawn rotated |
| [TRO16] Fig. 10 | exact MINMAKESPAN on 24x18 | `24x18` [exact] | `run24x18PerformanceMetric` | obs 0..30 step 5, n 10..120 step 10, 600 s, k-way split with one piece | paper plots 0–25%, n ≤ 100 |
| [TRO16] Fig. 11 | 4-way split on 24x18 | `24x18` [4-way] | same | n 20..300 step 20, 600 s, split level 2 | ratio = ave solution / min possible (a ratio of averages) |
| [TRO16] Fig. 12 | 2-, 8-, 16-way split on 24x18 | `24x18` [2-way, 8-way, 16-way] | same | split levels 1, 3, 4 | – |
| [TRO16] Fig. 13 | comparison with ID | – | – | – | ID is Standley's code, not part of ours |
| [TRO16] Figs. 14–15, Sec. VI-B | 9/16/25-puzzles | `n2puzzle` | `runN2PuzzlePerformanceMetric` | 100 instances each, PuzzleSolver, no limit | `main` case 1 is a single 16-puzzle demo |
| [TRO16] Fig. 16 (top) | 8x8, makespan vs. lower bound | `crowded` [8x8/exact, 8x8/2-way, 8x8/4-way, 8x8/8-way] | `runCrowdedPerformanceMetricCrowded` | exact 10..50, splits 10..60, 1800 s | paper: 600 s |
| [TRO16] Fig. 16 (bottom) | 16x16, makespan vs. lower bound | `crowded` [16x16/exact, 16x16/2-way, 16x16/4-way, 16x16/8-way] | same | exact 10..100, splits 10..250, 1800 s | paper: 600 s |
| [TRO16] Fig. 17 | 32x32, 20% obstacles | `32x32` | `run32x32PerformanceMetric` | exact 10..90 (1800 s); 2/4/8/16-way, 1800 s | splits are cycle-free (`noCycle`); paper shows no 16-way |
| [TRO16] footnote 10 | 8-connected 32x32, up to 400 robots | `32x32-8c-all` | `run32x32PerformanceMetric8ConnectedAll` | up to 8-way, n ≤ 400, 1800 s | – |
| [TRO16] Fig. 18 | MINTOTALTIME, 600 s | `24x18-tt` | `run24x18PerformanceMetricTotalTime` | obs 0..30, n 10..160, gap formula, horizon of a 4-way split | gap on `nT − TT`; single horizon |
| [TRO16] Fig. 19 | MINTOTALDIST with the split | `24x18-td-split` | `run24x18PerformanceMetricTotalDistanceSplit` | gap `n*0.001 + obs*0.0025 (+ n*0.001 if n > 70)`, 600 s | 8-way at 25–30% obstacles |
| [TRO16] Fig. 20 (top) | MINTOTALTIME vs. ID, fixed time | `24x18-tt-fixed-time` | `run24x18PerformanceMetricTotalTimeFixedTime` (`main` case 0) | `bReturnAnySolution`, gap 0.001, 100 s, obs 0..20, n 70..100 | paper: 600 s, no suboptimality threshold (code: gap 0.001), n 20..120, obs 0/10/20 |
| [TRO16] Fig. 20 (bottom) | MINTOTALDIST vs. ID, fixed time | `24x18-td-split-fixed-time` | `run24x18PerformanceMetricTotalDistanceSplitFixedTime` | `bReturnAnySolution`, gap 0.001, 100 s, 4-way, obs 0..20, n 10..150 | paper: 600 s, no suboptimality threshold (code: gap 0.001), n 20..140, obs 0/10/20 |
| (not plotted) | no-cycle splits on 8x8/16x16 | `crowded` [*-nc sections] | `runCrowdedPerformanceMetricCrowded` | as Fig. 16 | – |
| (not plotted) | MINTOTALDIST at the split horizon, unsplit | `24x18-td` | `run24x18PerformanceMetricTotalDistance` | gap `n*0.002 + obs*0.002`, 600 s | – |
| [ICRA13] Table I | 20x15 grids, 1000 s | – | – | – | no generator or driver |
| [ICRA13] Table II | 32x32, 20% obstacles, 10..50 robots | `32x32` [exact] | `run32x32PerformanceMetric` | 1800 s | – |
| [ICRA13] Table III | local-repair heuristic, 25..150 robots | `32x32-heuristic` | `run32x32PerformanceMetricHeuristic` | n 20..200 step 20, 10 runs (labelled 100), no time limit | legacy, file names never generated |

Legacy drivers without a published figure: `32x32-splitting` (`run32x32PerformanceMetricSplitting`),
`32x32-8c` (`run32x32PerformanceMetric8Connected`, 3600 s), `32x32-splitting-8c`,
`full` and `full-extras` (`runFullPerformanceMetric[Extras]`, 3600 s; robot counts in steps of 25
that the generator never writes).
