# Bugs in the original Java code

This note lists every bug, crash, hang and source of nondeterminism we confirmed in the
original Java implementation of our papers
(<https://github.com/arc-l/optimal-mrppg-journal>, commit `3dd2e23`; Yu and LaValle, T-RO 2016,
and the ICRA 2013 heuristic), and what `mppflow` does about each.

* `compat="original"` reproduces the original: a Java exception becomes the matching
  `mppflow.errors.Java*` exception (raised at the same point, after the same random draws), and
  a Java infinite loop raises `mppflow.errors.OriginalWouldHang` as soon as it is detected.
* `compat="fixed"` (the default) corrects the bug; everywhere else it gives the same results as
  the original.

Locations are `file:line` in `source/projects/multipath/` of that repository (`ILP/` and
`advanced/`). We verified each item against the compiled original (JDK 17, Gurobi 13.0.2); most
are also covered by the golden fixtures in `tests/golden/`. The reproductions are in
[`repro/`](repro/), use HiGHS unless they say otherwise, and the output shown is what they print.
HiGHS versions can return different, equally valid optimal plans, so a script that prints a plan
uses the pure-Python `builtin` backend (the same output on every platform) or prints only facts
that do not depend on the plan chosen; output that is specific to one HiGHS version starts with a
`scipy` version line (scipy bundles HiGHS).
Unless noted, the paper's experiments are not affected: the bug needs inputs or call sequences
the experiment drivers never produce. Errors in the papers themselves are in
[paper-errata.md](paper-errata.md).

| id | area | item | kind |
|---|---|---|---|
| [B1](#b-infeasible) | solvers | no upper bound on T: infeasible input loops forever | hang |
| [B2](#b-timelimit) | solvers | time-limit accounting subtracts elapsed time repeatedly; limit <= 0 | wrong result |
| [B3](#b-nan) | solvers | TT / DIST reject an optimum of 0 (0/0 gap) | wrong result |
| [B4](#b-dup) | solvers | duplicate starts are not rejected (collision at t = 0); duplicate goals hang | wrong result, hang |
| [B5](#b-unreach) | path finder | unreachable goal: NullPointerException | crash |
| [B6](#b-timer) | solvers | TimeR: int overflow drops one-move-per-step rows for vertex ids >= 22 | wrong result |
| [B7](#b-keys) | solvers | packed int/long model keys collide (T >= 10000; `getEdgeId` from ~1600 vertices and T ~ 50) | latent wrong model |
| [B8](#b-oneway) | solvers | reachability pruning measures the goal side forward | latent wrong result |
| [B9](#b-exact) | solvers | path extraction tests X == 1.0 exactly | latent wrong result |
| [B10](#b-splitloop) | k-way split | splitPaths widens its window forever | hang |
| [B11](#b-endpoints) | k-way split | splitPaths never picks a robot's own start or goal | non-optimal plans |
| [B12](#b-nocycle) | no-cycle split | unbounded retries; NPE on a failed retry; leaked print flag | hang, crash |
| [B13](#b-splitdist) | split + distance | NPE on a failed piece; always null without a time limit | crash, wrong result |
| [B14](#b-mt) | k-way split | worker exception hangs; stale pieces on a reused planner; lost pieces (HashMap race) | hang, wrong result, nondeterminism |
| [B15](#b-bysize) | split by size (dead code) | endless loop; getLongestPathLength off by one | hang, wrong result |
| [B16](#b-utils) | path utilities | empty / ragged / null path sets | crash, wrong result |
| [B17](#b-puzzle) | PuzzleSolver | deadlock when both models fail; stale state on a second call | hang, wrong result |
| [B18](#b-heurcrash) | heuristics | direct planHeuristicPath call; zero robots; edge ids >= 10000 | crash, wrong result |
| [B19](#b-bheur) | Main + heuristic | PathFinder.bHeuristic leaks; heuristic statistics off by one | wrong result |
| [B20](#b-case5) | Main | case 5 falls through into case 6; bad arguments | wrong behaviour, crash |
| [B21](#b-subopt) | Main | solveProblemSuboptimal prints a null plan (NPE) | crash |
| [B22](#b-ttmain) | Main | TT / DIST drivers: limit <= 0 returns null; unchecked split result | wrong result, crash |
| [B23](#b-stats) | Main | median over a TreeSet; no row without a success; heuristic `ave time` per success | wrong statistic |
| [B24](#b-files) | Main / Problem | swallowed I/O errors, Windows paths, legacy file names, .dat writer limits | crash |
| [B25](#b-grids) | Graph | narrow grids lose edges; too many agents / holes | wrong graph, hang, crash |
| [B26](#b-nondet) | several | nondeterminism: wall clock, threads, identity hashes | nondeterminism |
| [B27](#b-zeroleft) | heuristics | an exactly spent attempt budget (timeLeft == 0.0) solves without any limit | hang |

---

## Solvers (`ILP/MultiagentGraphSolverGurobi*.java`)

<a id="b-infeasible"></a>
### B1. No upper bound on T: an infeasible instance loops forever

* **Java:** `MultiagentGraphSolverGurobiTime.java:60-127` (also `TimeR`, and `PuzzleSolver.java:46-108`):
  `while(true)` (LP relaxation) and `while(val != starts.length)` (ILP) only exit on success or,
  with `hasTimeLimit`, on time-out. No feasibility check, no `T_max` ([ICRA13] Prop. 3; [TRO16]
  Sec. IV-E has both).
* **Behaviour:** without a time limit, an infeasible instance (e.g. two robots swapping on a
  path, or two robots with the same goal) never returns; with one, it returns `null` when the
  limit expires.
* **original:** after the first failed horizon, an exact joint-state search decides
  feasibility; if the instance is infeasible, `OriginalWouldHang` (no limit) or the `null`
  result at once (with a limit, `extra["shortcut"]`).
* **fixed:** raises `Infeasible` when the proof succeeds; `t_max=` bounds the search. The proof
  is the joint-state search (practical only for small joint state spaces) plus polynomial
  certificates that it tries first: robots that must pass each other on a component that is a
  path, or change their cyclic order on a cycle; a robot that must cross a bridge of a fully
  occupied component (only rotations along cycles are possible there); one connected component
  whose own search fails. The search is repeated with a larger budget at the 2nd, 4th, 8th, ...
  failed horizon, and also whenever the horizons solved so far took longer than all searches,
  since the ILP's infeasibility proofs get slow as `T` grows (a 16-vertex corridor in which 5
  robots must reverse their order: horizons up to `T = 22` took under 1 s each, `T = 23` over
  90 s, and the first conclusive search came only at `T = 30`; the path certificate now settles
  it at the first failed horizon). An infeasible instance that none of this settles (a large
  joint state space, outside the certificates; say, many robots that must reorder on a large tree
  with too few free vertices) still makes the loop run until `t_max` or the time limit, or for a
  very long time without either. We do not implement the polynomial feasibility test [TRO16]
  Sec. IV-E cites.

`python3 docs/notes/repro/minmakespan_no_tmax.py`

```text
original: OriginalWouldHang - MultiagentGraphSolverGurobiTime.solve: the instance is infeasible for every T (proved by joint-state search); the original increases T forever
fixed:    Infeasible - the instance has no solution for any horizon (joint-state search)
fixed, 5x5 grid + separate edge: Infeasible - the instance has no solution for any horizon (joint-state search)
fixed, corridor: Infeasible - the instance has no solution for any horizon (joint-state search)
```


<a id="b-timelimit"></a>
### B2. Time-limit accounting

* **Java:** `MultiagentGraphSolverGurobiTime.java:36, 61, 69, 77, 92, 112` and `prepareModel`
  `:232-237` (identical in `TimeR`); `PathPlanner.java:1114`.
* **Behaviour:** `timeLimit = timeLimit - (now - startTime)/1000.0` runs at every iteration with a
  fixed `startTime`, so iteration *k* gets `L − Σ_{j≤k} E_j` instead of `L − E_k`, and the time-out
  test `timeLimit - elapsed < 0` subtracts once more: runs time out early. A remaining limit
  `<= 0` is passed to Gurobi as "no limit" (`1e10`), so a limit shorter than the setup time
  makes the first model run unbounded (and possibly succeed). With `hasTimeLimit = true` and a
  limit `<= 0` (the code base's "no limit") the first failed horizon returns `null`; on the
  LP-relaxation path even a feasible relaxation returns `null` ("relaxed model not feasible,
  time limit reached."). `multiThreadedSplitPlanning` always passes `hasTimeLimit = true`
  (`PathPlanner.java:1114`), so with `-1` every piece that is infeasible at its first horizon fails.
* **original:** the same arithmetic, on mppflow's clock (`clock=` makes runs deterministic).
* **fixed:** `remaining = limit − elapsed`; no time left means time-out (never "unlimited"); a
  feasible relaxation is accepted. The limit the caller passes: `plan`, the `min_*` functions
  and the CLI accept only `None` (no limit) or a finite number > 0 and raise `ValueError` for 0,
  a negative limit or NaN (which would otherwise mean no limit, the opposite of a tiny
  positive limit); the solvers with the original's signature keep its sentinel, a limit `<= 0`
  means no limit, and reject NaN.
* **Experiments:** every driver passes a positive limit; the early time-outs shorten the
  effective limits of the timed runs, by an amount that depends on the machine.

`python3 docs/notes/repro/bug_time_limits.py`

```text
(a) limits printed per horizon, time limit 10 s:
  original: ['9.5', '8.0', '5.5'] -> makespan 4
  fixed: ['9.5', '8.5', '7.5'] -> makespan 4
(b) time limit 3.2 s:
  original: status time_limit, makespan None
  fixed: status optimal, makespan 4
(c) time limit 0.2 s, shorter than the setup (first remaining limit -0.3 s):
  original: status optimal, makespan 4, models solved 1
  fixed: status time_limit, makespan None, models solved 0
(d) hasTimeLimit = true with timeLimit = -1 (the code base's 'no limit'):
  original: status time_limit, makespan None
  fixed: status optimal, makespan 4
  original, LP relaxation first: status time_limit, makespan None, log: ['Solving relaxed model first. ', 'Trying T = 4 with max time limit -1.0 ... relaxed model not feasible, time limit reached.']
  fixed, LP relaxation first: status optimal, makespan 4, log: ['Solving relaxed model first. ', 'Trying T = 4 ... relaxed model feasible.']
(e) multiThreadedSplitPlanning(timeLimit = -1, splits = 1) always passes hasTimeLimit = true:
  original: null
  fixed: makespan 6
```


<a id="b-nan"></a>
### B3. TT / DIST reject an optimum of 0

* **Java:** `MultiagentGraphSolverGurobiTotalTime.java:76-80` (`agap = bnd/val - 1`),
  `MultiagentGraphSolverGurobiDistance.java:58` (`agap = 1 - bnd/objVal`), then
  `if (agap <= gap || bReturnAnySolution)`.
* **Behaviour:** an objective of 0 gives `0/0 = NaN`, and `NaN <= gap` is false: the optimal
  solution is discarded, " time out." is printed and `null` is returned. TT: every robot needs the
  whole horizon; DIST: nobody has to move. `(int)` truncation of `ObjVal` can cause the same.
* **original:** reproduced (status `"rejected"`). **fixed:** accepted when the solver proves
  optimality or the gap (TT: on the total time) is within `gap`.
* **Experiments:** not affected (random instances with distinct starts and goals); the
  fixed-time drivers set `bReturnAnySolution`.

`python3 docs/notes/repro/bug_nan_gap.py`

```text
original TT:   status rejected, total time None, paths None, ' time out.' printed: True
original DIST: status rejected, distance None, paths None, ' time out.' printed: True
fixed TT:   status optimal, total time 2, paths [[0, 1, 2]], ' time out.' printed: False
fixed DIST: status optimal, distance 0, paths [[0, 0, 0, 0], [3, 3, 3, 3]], ' time out.' printed: False
```


<a id="b-dup"></a>
### B4. Duplicate starts or goals

* **Java:** `MultiagentGraphSolverGurobiTime.prepareModel` (`:300-318`): the vertex-capacity rows
  cover `(v, t)` for `t = 1..T` only (at `t = 0` only the loopbacks enter); `solve` never checks
  the input.
* **Behaviour:** duplicate starts produce a plan that collides at `t = 0` (the `PathPlanner`
  wrappers then reject it with `isPathSetValid`); duplicate goals make every horizon infeasible
  (B1).
* **original:** reproduced. **fixed:** `ValueError` (duplicate starts), `Infeasible` (duplicate goals).

`python3 docs/notes/repro/bug_duplicate_starts_goals.py`

```text
original: starts [0, 0] -> goals [2, 3]: [[0, 1, 2, 2], [0, 0, 1, 3]]
   validity: Path 0 and 1 meet on vertex 0 at time step 0
original: starts [0, 2] -> goals [3, 3]: OriginalWouldHang: MultiagentGraphSolverGurobiTime.solve: the instance is infeasible for every T (proved by joint-state search); the original increases T forever
fixed: starts [0, 0] -> goals [2, 3]: ValueError: two robots share a start vertex
fixed: starts [0, 2] -> goals [3, 3]: Infeasible: two robots share a goal vertex
```


<a id="b-unreach"></a>
### B5. Unreachable goal

* **Java:** `PathFinder.findOneShortestPath` returns `null` (`PathFinder.java:233`); every caller
  reads `paths[i].vertexList` (lower bounds, all solvers, `PathPlanner.splitPaths`,
  `HeuristicPathPlanner.java:106`, `Main.java:341`).
* **original:** `JavaNullPointerException` after the same draws. **fixed:** `Infeasible`.
* **Experiments:** not affected (the generators keep the grids connected).

`python3 docs/notes/repro/bug_unreachable_goal.py`

```text
original: getMakespanLowerBound: JavaNullPointerException: Cannot read field "vertexList" because "<local3>[<local5>]" is null
original: Time.solve: JavaNullPointerException: Cannot read field "vertexList" because "<local13>[<local14>]" is null
fixed: getMakespanLowerBound: Infeasible: goal of robot 0 is unreachable from its start
fixed: Time.solve: Infeasible: goal of robot 0 is unreachable from its start
```


<a id="b-timer"></a>
### B6. TimeR: int overflow in getEdgeId drops the one-move rows

* **Java:** `MultiagentGraphSolverGurobiTimeR.java:173-175` (`v1 * 100000000` in `int`) and the
  rows at `:293-302` (`getTimeFromEdgeId(edgeId) == t`).
* **Behaviour:** for a tail vertex id `>= 22` the product wraps, the decoded step is wrong, and the
  arc is missing from its "at most one move at step t" row: several robots move at once. (With
  one move per step, the swap below needs 12 steps: the only shortest paths stay in the robots'
  row, so one robot detours by two moves. A makespan of 10 therefore means simultaneous moves.) Only
  `PathPlanner.planPathsAdvancedPMG` / `Main.solveProblemPMG` use this solver, and nothing calls them.
* **original:** reproduced (same wrapped keys and row contents). **fixed:** every move arc is in its row.

`python3 docs/notes/repro/bug_timer_overflow.py`

```text
getEdgeId(0, 21, 20, 3) = 2100200003, getTimeFromEdgeId = 3
getEdgeId(0, 22, 21, 3) = -2094757293, getTimeFromEdgeId = -7293
getEdgeId(1, 22, 21, 3) = 997905242707, getTimeFromEdgeId = 2707
original: swap [30, 35]: makespan 10, more than one mover at some step: yes
original: swap [0, 5]: makespan 12, more than one mover at some step: no
fixed: swap [30, 35]: makespan 12, more than one mover at some step: no
fixed: swap [0, 5]: makespan 12, more than one mover at some step: no
```


<a id="b-keys"></a>
### B7. Packed model keys collide

* **Java:** `MultiagentGraphSolverGurobiTime.java:162-181` (and the other solvers):
  `getVertexTimeId = v*10000 + t`, `getEdgeId = a*10^12 + v1*10^8 + v2*10^4 + t`,
  `getEdgeTimeId = min*10^9 + max*10^4 + t`, with `int` products. Keys collide once `T >= 10000`
  (or for vertex ids `>= 214749`), and `getEdgeTimeId` also on large graphs with long horizons.
  `getEdgeId` collides much earlier: its `v1*10^8` wraps from tail id `v1 = 22` on, so on a graph
  with about 1600 vertices two arcs of one robot can share a key only 48 steps apart
  (`getEdgeId(0, 1589, 1379, t) = getEdgeId(0, 0, 0, t + 48)`; such a model collides from
  `T = 49`). With vertex ids below 1024 two `getEdgeId` keys are equal only for time steps at
  least 1328 apart. Colliding rows and variables are merged in the `TreeMap`s.
* **original:** the same keys and merges (`MPPModel.key_collisions` counts them). In
  `compat="original"` the two-robot swap model even becomes feasible at `T = 10000`. **fixed:**
  colliding groups are kept apart (the head-on rows keep the original's order).
* **Experiments:** not affected (no collision on any instance size used: the largest graphs
  have at most 1024 vertex ids, and every horizon is far below 1328).

`python3 docs/notes/repro/bug_key_collisions.py`

```text
getVertexTimeId(0, 10000) = 10000 = getVertexTimeId(1, 0) = 10000
getEdgeId(0, 5, 0, 10000) = 500010000 = getEdgeId(0, 5, 1, 0) = 500010000
getEdgeTimeId(4, 29501, 0) = 42704 = getEdgeTimeId(0, 4, 2704) = 42704
getEdgeId(0, 1589, 1379, 10) = 58 = getEdgeId(0, 0, 0, 58) = 58
smallest time difference of colliding getEdgeId keys, ids < 1024: 1328 ; ids < 1590: 48
T = 48: edgeVar collisions {'original': 0, 'fixed': 0}
T = 49: edgeVar collisions {'original': 1, 'fixed': 1}
T = 60: edgeVar collisions {'original': 12, 'fixed': 12}
```


<a id="b-oneway"></a>
### B8. Reachability pruning on one-way graphs

* **Java:** `updateReachableSets`, `MultiagentGraphSolverGurobiTime.java:135-160` (same in
  `TimeR`, `TotalTime`, `Distance`, `PuzzleSolver`): `distFromGoal` is a BFS from the goal,
  `dist(g, v)`, where `dist(v, g)` is meant.
* **Behaviour:** on a directed graph this prunes vertices a robot must pass: non-optimal
  makespans, single-horizon TT/DIST models that are wrongly infeasible, and (without wait loops)
  no solution at any horizon. Every graph the original builds is symmetric, so this is latent.
* **original:** reproduced; a never-ending search is detected (`OriginalWouldHang`), also when
  robots start at their goals: with `extraSteps >= 1` the loop starts at `T >= 1`, where such a
  robot needs a closed walk (on a one-way cycle without wait loops the pruning leaves none).
  **fixed:** `dist(v, g)` by a BFS over the reversed arcs (identical on symmetric graphs). The
  split windows of `splitPaths` measure the goal side the same way (B11).

`python3 docs/notes/repro/bug_one_way_pruning.py`

```text
original: robot 0 -> 3: [[0, 1, 2, 3, 3, 3]]
fixed: robot 0 -> 3: [[0, 1, 2, 3]]
```


<a id="b-exact"></a>
### B9. Path extraction tests X == 1.0

* **Java:** `isEdgeVarSetToTrue`, `MultiagentGraphSolverGurobiTime.java:209-216`, used by
  `retrievePaths` (`:183-207`): an arc counts only if `X == 1.0`; when none matches, the path
  silently continues from `paths[a][t] = 0`. `ObjVal` is truncated with `(int)` (`:102`).
* **Behaviour:** correct with Gurobi in practice (it returns exact binaries); with a solver that
  returns `0.9999999999999998` the plan is invalid.
* **original:** reproduced; mppflow issues `InexactSolutionWarning` (run `compat="original"`
  with Gurobi). **fixed:** `X > 0.5`, `ObjVal` rounded. See also [solver-notes.md](solver-notes.md).

Which binaries come back inexact depends on the HiGHS version. With scipy 1.17.1:

`python3 docs/notes/repro/highs_inexact_original.py`

```text
scipy 1.17.1
original/highs: makespan 12, InexactSolutionWarning x1, plan: Path 2 is not valid at t = 0
fixed/highs: makespan 12, InexactSolutionWarning x0, plan: valid
original/gurobi: makespan 12, InexactSolutionWarning x0, plan: valid
```


## Split heuristics (`ILP/PathPlanner.java`)

<a id="b-splitloop"></a>
### B10. splitPaths widens its window forever

* **Java:** `PathPlanner.java:577-648` and `PathFinder.findGoalsInDistanceRange`
  (`PathFinder.java:61-102`), which starts at distance 1 and never returns its centre.
* **Behaviour:** robot *i*'s candidates are at most `V \ {s_i, g_i}`; when the robots processed
  before it hold all of them as intermediate goals, `while(!foundVertex)` never ends. Needs about
  as many robots as vertices (2x2 grid with 3 robots: 20 of 576 instances; random 9-puzzles: 4 of
  20). Reached from `planPathsAdvancedSplit`, `...SplitDist`, `...SplitNoCycle` and
  `multiThreadedSplitPlanning`.
* **original:** `OriginalWouldHang` once the window covers the component. **fixed:** a robot's own
  start or goal may be its intermediate goal (B11); a saturated window falls back to a free
  vertex.

`python3 docs/notes/repro/bug_split_paths.py`

```text
original: splitPaths(2x2, [0,2,3] -> [3,1,2]): OriginalWouldHang: splitPaths: every vertex at any distance from robot 2's start and goal is already another robot's intermediate goal; the original widens its window forever
fixed: splitPaths(2x2, [0,2,3] -> [3,1,2]): [1, 0, 2]
original: 3x3 [0, 4] -> [1, 4]: intermediate goals [2, 1], 2-way split makespan 3
original: 3x3 [4] -> [4]: intermediate goals [1], 2-way split makespan 2
original: 3x3 [0, 2] -> [1, 2]: intermediate goals [2, 1], 2-way split makespan 6
fixed: 3x3 [0, 4] -> [1, 4]: intermediate goals [0, 4], 2-way split makespan 1
fixed: 3x3 [4] -> [4]: intermediate goals [4], 2-way split makespan 0
fixed: 3x3 [0, 2] -> [1, 2]: intermediate goals [0, 2], 2-way split makespan 1
```


<a id="b-endpoints"></a>
### B11. splitPaths never picks a robot's own start or goal

* **Java:** as B10. [TRO16] Sec. V-B takes the middle vertex of the robot's path; the code takes
  any vertex at about half the distance from both ends, excluding the ends themselves.
* **Behaviour:** a robot already at its goal (path of length 0) or one step from it is sent away
  and back: the split's makespan grows (0 → 2, 1 → 3, 1 → 6 in the examples of B10's output). Dense
  random instances have many such robots.
* **Also:** the code's first window is `d = L/2` from both ends (`L` = vertices on the path,
  `PathPlanner.java:612-613`), i.e. `ceil(D/2)` for `D` edges. For an odd `D` it holds only
  vertices with `dist(s, v) + dist(v, g) = D + 1`, one step off every shortest path. A
  bipartite graph (a 4-connected grid) has none, and the next window gives the middle
  vertices; on other graphs, such as the 8-connected grids, a lone robot gets a one-step
  detour (3x4 8-connected grid, `0 -> 3`: split makespan 4, optimum 3). The goal side is a
  forward BFS from the goal, `dist(g, v)`, as in B8: on a one-way graph it can offer a vertex
  from which the goal is unreachable, and the next level of the split then fails.
* **original:** reproduced. **fixed:** the window includes the endpoints at distance 0; the goal
  side is `dist(v, g)` (reversed arcs on a one-way graph); for an odd `D` the middle vertices
  of the shortest paths (`dist(s, v)` in `{floor(D/2), ceil(D/2)}`, `dist(s, v) + dist(v, g) =
  D`) come first. Otherwise the choice is the original's, including the `HashSet` order and
  the draws; on symmetric bipartite graphs only the endpoint rule changes it.
* **Experiments:** affects, on instances with such robots, the split results of Figs. 11, 12,
  16, 17, 19 and 20 (bottom, `solveProblemDistanceSplit` → `planPathsAdvancedSplitDist`,
  `PathPlanner.java:683`) and of [TRO16] footnote 10 (the 8-connected 32x32 runs,
  `multiThreadedSplitPlanning`, `:1062`, where the odd-`D` window above also applies), and the
  total-time horizons of Figs. 18 and 20 (top), which are the makespan of a 4-way
  `planPathsAdvancedSplit` (`Main.java:226` → `:656`).

<a id="b-nocycle"></a>
### B12. planPathsAdvancedSplitNoCycle

* **Java:** `PathPlanner.java:716-774`.
  1. Retries: `while((hasCycles(...) > 0 ...) && (tries-- > 0 || top))` (`:730, :737, :740`) never
     stops at the top level while the plan has rotations, and every retry gets the same
     `timeLimit/2`, so the caller's limit does not bound the run.
  2. Only the first pair of pieces is null-checked (`:728`); a retried piece that is `null` (a
     time-out or an infeasible piece) makes the next `hasCycles(null)` throw
     (`PathPlanner.java:1152`). The commented-out loop at `:744-748` had the guard.
  3. `bPrintPaths` is cleared at entry (`:718-721`) and not restored on the `splits == 0` return
     (`:767`) or the early `return null` (`:728`).
* **Experiments:** Fig. 17's split runs use this planner (`Main.java:611`); an instance without a
  rotation-free split there runs until it is killed, and a piece time-out aborts the sweep.
* **original:** the retry loop is analysed; if no retry can give a rotation-free plan,
  `OriginalWouldHang`. A `null` retry raises `JavaNullPointerException`. The flag leak is
  reproduced (`planner.flags`). **fixed:** stops when no retry can help, after 1000 extra solves,
  or when a positive time limit has elapsed, and returns the plan with rotations; a failed retry
  keeps the last complete pair; the flag is restored.

`python3 docs/notes/repro/bug_nocycle.py`

```text
original: ring, splits 1, top: OriginalWouldHang: the top-level retry loop (tries-- > 0 || top) never ends: every plan a retry can produce contains rotations (hasCycles > 0)
   leaf solves: 40; []
original: ring, splits 2, top: JavaNullPointerException: Cannot load from object array because "<parameter1>" is null
fixed: ring, splits 1, top: makespan 2, rotations (hasCycles) 2
   leaf solves: 40; ['no retry can give a plan without rotations; keeping the plan with rotations']
fixed: ring, splits 2, top: null
original: splits 0 -> bPrintPaths afterwards False, paths printed: False
fixed: splits 0 -> bPrintPaths afterwards True, paths printed: True
```


<a id="b-splitdist"></a>
### B13. planPathsAdvancedSplitDist

* **Java:** `PathPlanner.java:680-713`.
  1. The leaf `int[][] tp = planPathsAdvanced(...); return planPathsAdvancedTD(..., tp[0].length - 1, timeLimit);`
     (`:708-709`) has no null check: a makespan time-out throws `NullPointerException`, which ends
     the experiment loop (`Main.solveProblemDistanceSplit`, Fig. 19) instead of recording a failure.
     The distance solve also gets the full limit again.
  2. With `timeLimit <= 0` and `splits > 0`, `timeLimit -= elapsed` keeps it `<= 0` and
     `if (timeLimit > 0)` skips the second half: always `null`.
* **original:** reproduced. **fixed:** `None` on a failed piece; a limit `<= 0` means no limit; the
  distance solve gets the time that is left.

`python3 docs/notes/repro/bug_splitdist.py`

```text
original: star swap, timeLimit 1, splits 0: JavaNullPointerException: Cannot load from object array because "<local9>" is null
original: 3x3 grid, timeLimit -1, splits 1: null
fixed: star swap, timeLimit 1, splits 0: null
fixed: 3x3 grid, timeLimit -1, splits 1: makespan 6
```


<a id="b-mt"></a>
### B14. multiThreadedSplitPlanning

* **Java:** `PathPlanner.java:1040-1131`.
  1. A worker sets `mdone[i] = true` only after `solve` returns (`:1125`); a `GRBException` is
     caught and printed (`:1127`), any other exception kills the thread, and the main thread polls
     `mdone[]` forever (`:1077-1091`; seen with a missing Gurobi license).
  2. `mpath` is an instance field that is never cleared: on a reused planner a piece that fails
     keeps the previous call's piece, and `mergePaths` (which does not check continuity) glues it in.
  3. With `timeLimit <= 0`, B2 (pieces fail at their first infeasible horizon).
  4. `mpath` (a `HashMap`) and `mdone[]` (not `volatile`) are shared without synchronization.
     Workers that finish at about the same moment can lose a `mpath.put` (`:1117`): the race
     is on the first puts into the new map (its table is created on demand). `mergePaths` then
     gets `null` for the missing piece, and the method returns `null` with nothing printed and
     no exception. This happens in a sizeable share of runs when pieces are short. Example:
     two robots on a 4x6 grid with 20% holes, starts `[9, 6]`, goals `[1, 2]`, `splits = 2`,
     time limit 1e6, `Math.random` seeded 610148160. The split goals are
     `[[9,6],[3,7],[2,1],[3,7],[1,2]]`, so two of the pieces are one step long. 5-24% of the
     runs returned `null` (16 of 300 to 109 of 600, depending on the machine's load). Any
     piece can be lost, also two at once: the kept key sets were `[0,2,3]`, `[1,2,3]`,
     `[0,3]`, `[2,3]` and others. Every other run returned `[[9,3,2,3,2,1],[6,7,1,7,8,2]]`.
     With a `ConcurrentHashMap` in place of `mpath`, none of 600 runs failed. With a reused
     planner (item 2), a stale piece of an earlier call can hide the lost one.
* **original:** a worker exception raises `OriginalWouldHang`; `planner.mpath` persists as in Java.
  The pieces run sequentially (identical plans; see B26). Item 4 is not reproduced: the lost
  `put` depends on thread timing, so `mppflow` always returns the plan that the runs without a
  lost piece return (the golden differential skips such runs as `mt-hashmap-race`).
  **fixed:** exceptions propagate, the pieces of each call are kept apart.
* **Experiments:** `Main` always creates a fresh planner. Item 4 can make a split run of the
  experiments fail at random.

`python3 docs/notes/repro/bug_mt_split.py`

```text
original: worker throws: OriginalWouldHang: multiThreadedSplitPlanning: worker 0 threw java.lang.NullPointerException; mdone[0] is never set and the main thread waits forever
fixed: worker throws: Infeasible: goal of robot 0 is unreachable from its start
original: B on the reused planner: [[8, 5, 2, 5, 8], [6, 3, 0, 3, 6], [1, 4, 3, 4, 1]]
   valid for B: Goal location for agent 0 is incorrect in the path set.; B on a fresh planner: None
fixed: B on the reused planner: [[8, 5, 2, 1, 4, 3, 0], [6, 3, 0, 0, 0, 1, 2], [1, 4, 3, 3, 3, 6, 6]]
   valid for B: True; B on a fresh planner: [[8, 5, 2, 1, 4, 3, 0], [6, 3, 0, 0, 0, 1, 2], [1, 4, 3, 3, 3, 6, 6]]
```


Item 4 needs the live original, so it has a Java driver instead
([`repro/java/MtSplitRace.java`](repro/java/MtSplitRace.java); compile it against the original
classes and Gurobi, and put a `gurobi.env` with `Threads 1` in the working directory):

```text
$ java --add-opens java.base/java.lang=ALL-UNNAMED -cp CLASSES:gurobi.jar:. projects.multipath.ILP.MtSplitRace 300 hm
HashMap (original): 16 of 300 runs returned null; pieces kept in those runs: [[0, 2, 3], [0, 3], [1, 2, 3], [1, 3], [2, 3]]; plan otherwise: [[9, 3, 2, 3, 2, 1], [6, 7, 1, 7, 8, 2]]
$ java ... projects.multipath.ILP.MtSplitRace 300 chm
ConcurrentHashMap: 0 of 300 runs returned null; pieces kept in those runs: []; plan otherwise: [[9, 3, 2, 3, 2, 1], [6, 7, 1, 7, 8, 2]]
```


<a id="b-bysize"></a>
### B15. Split by size (dead code)

* **Java:** `splitPathBySize`, `PathPlanner.java:788-858`: the goal-side window is off by one
  (vertex count instead of edges) and only `dMin` moves, so the loop can spin forever.
  `getLongestPathLength`, `:777-786`, compares `size() - 1 > length` but stores `size()`, so the
  result depends on the robot order. The only callers (`planPathsAdvancedSplitBySize[NoCycle]`)
  are commented out in `Main.java:139-140`.
* **original:** `OriginalWouldHang`; the order-dependent length. **fixed:** a correct window that
  widens on both sides; the largest vertex count.
* `planPathsAdvancedSplitBySizeNoCycle` also retries a piece with rotations by a random re-split
  (`splitPaths(..., true)`) and recurses into both halves, which split and retry again. A half need
  not be shorter than its parent, so nothing bounds the descent; only an exact repeat of a
  sub-instance on the stack is caught. **fixed:** a half splits again only when its longest path is
  shorter than its parent's (else it is solved directly), so the depth is at most the longest path
  length. Before this our port ran into Python's `RecursionError` after minutes on a 3x5 grid
  (`create_grid_problem(3, 5, 0.2, 7, MathRandom(842709))`, size 2); it now returns a plan in
  about 3 s (`tests/test_planner.py::test_split_by_size_no_cycle_retries_shrink`).

`python3 docs/notes/repro/bug_split_by_size.py`

```text
original: splitPathBySize(3x3, [0] -> [8], firstSize 2): OriginalWouldHang: splitPathBySize: no free vertex for robot 0 and the window no longer changes (dMax never grows); the original loops forever
original: getLongestPathLength([0,1] -> [4,6]): 5
original: getLongestPathLength([1,0] -> [6,4]): 6
fixed: splitPathBySize(3x3, [0] -> [8], firstSize 2): [1]
fixed: getLongestPathLength([0,1] -> [4,6]): 6
fixed: getLongestPathLength([1,0] -> [6,4]): 6
```


<a id="b-utils"></a>
### B16. Path utilities on empty, ragged or null input

* **Java:** `printPaths` (`PathPlanner.java:40-41`), `hasCycles` (`:1150-1152`), `isPathSetValid`
  (`:1184-1186`) read `paths[0].length` and use it as the horizon of every row.
* **Behaviour:** `int[0][0]` throws; rows of length 0 pass validation without checking the starts
  and goals; a longer second row is cut off silently; `null` throws.
* **original:** reproduced. **fixed:** empty input is valid (no robots), ragged or short rows are
  invalid, `None` raises `ValueError`.

`python3 docs/notes/repro/bug_empty_paths.py`

```text
original: isPathSetValid(int[0][0]): JavaArrayIndexOutOfBoundsException: Index 0 out of bounds for length 0
original: hasCycles(int[0][0]): JavaArrayIndexOutOfBoundsException: Index 0 out of bounds for length 0
original: isPathSetValid(int[2][0], starts {0,3}, goals {3,0}): True
original: isPathSetValid({{0,1},{3,3,1,0}}, goals {1,0}): True
original: printPaths(null): JavaNullPointerException: Cannot read the array length because "<parameter2>" is null
fixed: isPathSetValid(int[0][0]): True
fixed: hasCycles(int[0][0]): 0
fixed: isPathSetValid(int[2][0], starts {0,3}, goals {3,0}): False
fixed: isPathSetValid({{0,1},{3,3,1,0}}, goals {1,0}): False
fixed: printPaths(null): ValueError: paths is None
```


## PuzzleSolver (`ILP/PuzzleSolver.java`)

<a id="b-puzzle"></a>
### B17. Deadlock, and stale state on a second call

* **Java:** `PuzzleSolver.java:46-108` (main loop), `:110-130` (workers), `callback`.
  1. When both models fail at a horizon (instance 1 with value `< n`, instance 2 infeasible) and
     both workers finish before the main thread's 1 s poll, the main thread waits in
     `while(!allThreadFinished)` (`:63-76`) for a flag that only a Gurobi message callback sets:
     it never returns, whatever the time limit. Typical for small instances.
  2. A worker that throws `GRBException` (`:127`; e.g. its log file `C:\Temp\25puzzle-instance-<k>`
     cannot be written) leaves its flag unset: the main thread hangs at `:89`, or returns `null`
     at the time limit if both fail.
  3. Every exit leaves `optimizationDone == true`, so a second `solve()` on the same object skips
     its first horizon (a plan one step longer than optimal), or returns the plan the previous,
     timed-out call dropped.
  4. Races: a stale `failedAttempt` can abort the next horizon's workers and skip a `T`; which
     model's plan is returned depends on timing.
* **original:** 1 raises `OriginalWouldHang`; 3 is reproduced. 2 cannot occur (no log files are
  written) and 4 is not emulated (both models run in sequence; instance 2's plan is returned).
  **fixed:** no deadlock (a failed horizon moves on), each call searches afresh.
* **Experiments:** `Main.runN2PuzzlePerformanceMetric` uses `PuzzleSolver` for the 25-puzzle only
  (more than 16 robots); `main` never calls it.

`python3 docs/notes/repro/bug_puzzle.py`

```text
original: star swap, solve(10): OriginalWouldHang: PuzzleSolver.solve: both models failed at T = 2 before the main thread's poll noticed it; it waits forever for a Gurobi callback that never comes (original-bugs B17)
fixed: star swap, solve(10): 4
original: 3x3 [0, 6] -> [2, 8]: first solve() makespan 2, second solve() makespan 3
fixed: 3x3 [0, 6] -> [2, 8]: first solve() makespan 2, second solve() makespan 2
```


## Heuristics (`ILP/HeuristicPathPlanner.java`, `ILP/PathPlanner.java`)

<a id="b-heurcrash"></a>
### B18. Direct planHeuristicPath calls, zero robots, edge ids >= 10000

* **Java:**
  1. `distanceToGoals` (`PathPlanner.java:65`) is assigned only in `planHeuristicPaths`
     (`HeuristicPathPlanner.java:51`); `createSubProblem` reads it
     (`HeuristicPathPlanner.java:396`, `PathPlanner.java:386`). A direct `planHeuristicPath` call
     throws at the first conflict; after an earlier call on another instance it silently uses the
     stale table.
  2. `planHeuristicPath` reads `pathArray[0]` (`HeuristicPathPlanner.java:102`): zero robots throw.
  3. Edge ids `max*10000 + min` in `int` (`HeuristicPathPlanner.java:130, 267`;
     `PathPlanner.java:157, 452`) collide once a vertex id is `>= 10000` (the class Javadoc states a
     10000-vertex limit), giving false head-on conflicts and non-optimal plans.
* **original:** reproduced. **fixed:** the table is computed when missing or stale, zero robots
  give `[]`, edge ids are tuples.

`python3 docs/notes/repro/bug_heuristic_planner.py`

```text
original: LocalRepairPlanner.plan_heuristic_path (direct): JavaNullPointerException: Cannot load from object array because "this.distanceToGoals" is null
original: HeuristicPathPlanner.plan_heuristic_path (direct): JavaNullPointerException: Cannot load from object array because "this.distanceToGoals" is null
original: zero robots: JavaArrayIndexOutOfBoundsException: Index 0 out of bounds for length 0
original: existsEdgeCollision([[10001, 20001], [1, 20002]], 0): [0, 1]
original: [10001, 1] -> [20001, 20002]: makespan 2, [[10001, 10001, 20001], [1, 20002, 20002]]
original: [5001, 1] -> [9001, 9002]: makespan 1, [[5001, 9001], [1, 9002]]
fixed: LocalRepairPlanner.plan_heuristic_path (direct): feasible
fixed: HeuristicPathPlanner.plan_heuristic_path (direct): feasible
fixed: zero robots: []
fixed: existsEdgeCollision([[10001, 20001], [1, 20002]], 0): None
fixed: [10001, 1] -> [20001, 20002]: makespan 1, [[10001, 20001], [1, 20002]]
fixed: [5001, 1] -> [9001, 9002]: makespan 1, [[5001, 9001], [1, 9002]]
```


<a id="b-bheur"></a>
### B19. Main.solveProblemWithHeuristic: leaked flag and off-by-one statistics

* **Java:** `Main.java:323-358`; `PathFinder.java:253` (`addToQueue`).
  1. `PathFinder.bHeuristic = true` is never reset. With the flag, `findOneShortestPath` inserts at
     a random queue position: it is no longer a BFS, the "lower bounds" are too large, and every
     later exact solve in the same JVM starts above `T_opt` and can return a non-optimal makespan.
  2. It returns `{paths[0].length, time, goals, length - 1, leastPossibleLength - 1}`: the first entry
     is the vertex count (makespan + 1), the two totals are one too small.
* **Experiments:** only the legacy driver `run32x32PerformanceMetricHeuristic` ("do not use!",
  never called) uses it.
* **original:** the flag is emulated per `Experiments` instance (`b_heuristic`); the stat vector is
  reproduced. **fixed:** no leak; `[makespan, ms, goals, moves, lb]`.

`python3 docs/notes/repro/bug_heuristic_flag_stats.py`

```text
getMakespanLowerBound(0 -> 63) - 1, seeds 15..24: [14, 14, 14, 14, 14, 16, 14, 14, 14, 14]
seed 1000 original: heuristic stat [16, 6, 41, 39] (true makespan, moves, lb: [15, 42, 40]); bHeuristic left set: True; exact solve afterwards: makespan 16, lb 14 (optimum 14)
seed 1000 fixed: heuristic stat [15, 6, 42, 40] (true makespan, moves, lb: [15, 42, 40]); bHeuristic left set: False; exact solve afterwards: makespan 14, lb 14 (optimum 14)
seed 1005 original: heuristic stat [16, 6, 41, 39] (true makespan, moves, lb: [15, 42, 40]); bHeuristic left set: True; exact solve afterwards: makespan 16, lb 16 (optimum 14)
seed 1005 fixed: heuristic stat [15, 6, 42, 40] (true makespan, moves, lb: [15, 42, 40]); bHeuristic left set: False; exact solve afterwards: makespan 14, lb 14 (optimum 14)
```


<a id="b-zeroleft"></a>
### B27. An exactly spent attempt budget solves without any limit

* **Java:** `HeuristicPathPlanner.java:177-194`, `PathPlanner.java:214-231`
  (`planHeuristicPath`). Before each sub-problem the attempt computes
  `timeLeft = timeLimit - (currentTimeMillis() - start)/1000.` and returns only if
  `timeLeft < 0`. When the budget is exactly spent (`timeLeft == 0.0`), it calls
  `planPathsAdvanced(sub, true, 0, true, timeLeft)`, which passes `hasTimeLimit = timeLeft > 0`,
  i.e. `false` (`PathPlanner.java:940`). The sub-solver's T loops then run with no limit and
  no maximum T (B1), and on a sub-problem without a solution the attempt never returns.
  The clock has millisecond steps and the budgets are exact at a millisecond. For 6 robots
  `HeuristicPathPlanner`'s budgets `1.5 + (5 + 4i) n/120` are 1.75, 1.95 and 2.15 s, and
  `1.75 - 1750/1000. == 0.0`. So this happens whenever a test lands on that millisecond.
  Seen in the differential test (`heuristic/001`, seed 1): Java hung at
  `MultiagentGraphSolverGurobiTime.solve:99 < planPathsAdvanced:940 < planHeuristicPath:194`
  and printed "Trying T = ..." without "with max time limit". The hung sub-problem was 3 robots
  on the path 0-1-6-5-4-3-2, starts `[4, 5, 3]`, goals `[1, 2, 0]`, which has no solution.
  Our first analysis of B2 had assumed that the heuristics always pass a positive limit.
* **original:** reproduced. With a clock that reads exactly the budget, the sub-solver runs without
  a limit and raises `OriginalWouldHang` once it proves the sub-problem infeasible. The default
  virtual clock never expires, so the default runs are not affected. **fixed:** `timeLeft <= 0`
  ends the attempt (and a fixed-mode sub-solver proves infeasibility and stops anyway).
* **Experiments:** the heuristic experiments can hang at random.

`python3 docs/notes/repro/bug_heuristic_zero_time_left.py`

```text
original, budget 1.75 s, 1.75 s elapsed: OriginalWouldHang: MultiagentGraphSolverGurobiTime.solve: the instance is infeasible for every T (proved by joint-state search); the original increases T forever
original, budget 1.751 s, 1.75 s elapsed: stop 'stalled', sub-problems solved 0 of 6
fixed, budget 1.75 s, 1.75 s elapsed: stop 'time_limit', sub-problems solved 0 of 1
```

Live Java on the hung sub-problem ([`repro/java/ZeroTimeLeft.java`](repro/java/ZeroTimeLeft.java),
the call `planHeuristicPath` makes): with `timeLeft = 0.5` it returns `null` within 0.5 s.
With `timeLeft = 0.0` it prints "Trying T = 2, 3, ... for the integer model" with no limit
and was still at T = 22 when `timeout 25` killed it.


## Experiment drivers and I/O (`ILP/Main.java`, `advanced/Problem.java`)

<a id="b-case5"></a>
### B20. main: case 5 falls through into case 6; argument errors

* **Java:** `Main.java:73-77`: case 5 (16x16 grid, 60 robots) has no `break`, so it continues with
  case 6 (the 17-vertex tree with 8 robots). `Main.java:16`: `Integer.parseInt(argv[0])`;
  `Main.java:67`: case 4 reads `argv[1]`. The `switch` has no `default`, so a case number
  outside 0..6 does nothing and exits normally: a mistyped case looks like a successful run.
* **original:** reproduced (`NumberFormatException`, `ArrayIndexOutOfBoundsException`, silent
  no-op). **fixed:** case 5 stops; bad arguments (a non-integer, a case outside 0..6, case 4
  without a file) raise `ValueError`.

`bug_case5_fallthrough.py` uses Gurobi (`Threads=1`) when a full license is available; it takes
about 4 minutes. On HiGHS (no license, or pip's size-limited one) the original line reads
`'Agent' lines printed: 60, case 6 ran: True`: case 6 still runs, but its path set is
rejected ("Goal location for agent 2 is incorrect in the path set."), because the original
needs exact binaries ([B9](#b-exact)).

`python3 docs/notes/repro/bug_case5_fallthrough.py`

```text
original: 'Agent' lines printed: 68, case 6 ran: True
fixed: 'Agent' lines printed: 60, case 6 ran: False
```

`python3 docs/notes/repro/bug_main_cli.py`

```text
original: main(['x']): JavaNumberFormatException: For input string: "x"
original: main(['4']): JavaArrayIndexOutOfBoundsException: Index 1 out of bounds for length 1
original: main(['7']) prints: ''
fixed: main(['x']): ValueError: case must be an integer, not 'x'
fixed: main(['4']): ValueError: case 4 needs a problem file
fixed: main(['7']) prints: ValueError: case must be 0..6, not 7
```


<a id="b-subopt"></a>
### B21. solveProblemSuboptimal prints a null plan

* **Java:** `Main.java:148`: `PathPlanner.printPaths(p.graph, paths)` runs before the
  `paths != null` test of line 151, and `printPaths` reads `paths.length` (`PathPlanner.java:41`).
* **Behaviour:** any failure of the k-way split (a time-out) throws `NullPointerException` out of the
  whole driver, so the `stat == null` branches of the drivers (stop the row, move to the next
  obstacle level) never run and no statistics are written for the sweep.
* **Experiments:** every driver that uses it (the 24x18 sweeps of Figs. 10–12, the 8x8/16x16 sweeps
  of Fig. 16, the 32x32 split sweeps of Fig. 17) ends with this exception at its first time-out,
  instead of recording the failure and moving on.
* **original:** reproduced. **fixed:** returns `None`, and the drivers take their intended branch.

`python3 docs/notes/repro/bug_subopt_npe.py`

```text
original: solveProblemSuboptimal(p, false, true, 0, 1e-4, 0, false): JavaNullPointerException: Cannot read the array length because "<parameter2>" is null
fixed: solveProblemSuboptimal(p, false, true, 0, 1e-4, 0, false): None
   failures: ['solveProblemSuboptimal: no plan']
```


<a id="b-ttmain"></a>
### B22. Main.solveProblemTotalTime / solveProblemDistance

* **Java:** `Main.java:221-262`: `timeLimit -= elapsed; if (timeLimit <= 0) return null;` after the
  4-way split, so a limit `<= 0` always fails; `paths[0].length` of the split is read without a null
  check (`:229`, `:259`).
* **original:** reproduced (`JavaNullPointerException` for a failed split). **fixed:** a limit `<= 0`
  means no limit; a failed split returns `None`. (The single horizon of these functions is a
  deviation from the paper, see [paper-vs-code.md](paper-vs-code.md).)

`python3 docs/notes/repro/bug_tt_td_main.py`

```text
original: solveProblemTotalTime(p, 0, -1): None
original: solveProblemDistance(p, 0, -1): None
fixed: solveProblemTotalTime(p, 0, -1): [11, 11]
fixed: solveProblemDistance(p, 0, -1): [11, 11]
```


<a id="b-stats"></a>
### B23. writeStats: median over a TreeSet; no row without a success; heuristic `ave time`

* **Java:** `Main.java:369` (`SortedSet<Double> finishingTimeSet = new TreeSet<Double>()`),
  `:400/:414` (add), `:429-430` and `:442-443` (`ft[ft.length/2]`, `ft[0]`).
* **Behaviour:** equal times collapse, so the reported median is the upper median of the
  distinct times (times are whole milliseconds, so ties are common). With no
  success, `ft[0]` throws `ArrayIndexOutOfBoundsException`, which is caught and printed, and the row
  is not written. `ave time` is correct in `writeStats` (every processed run is a success), but not
  in `writeHeuristicStats`: `processHeuristicStats` (`Main.java:409-425`) adds the time of every
  run to `subTotal` and counts a success only when every goal is reached, and `:443` prints
  `subTotal / successCount`, the time of all runs per success (one run of 1 s with all goals and
  one partial run of 9 s give `ave time: 10.0`). It affects `run32x32PerformanceMetricHeuristic`
  ([ICRA13] Table III, approximately) when some runs are partial.
* **Experiments:** the medians in the result files; none of our papers reports a median.
* **original:** reproduced. **fixed:** median over all times (`sorted[n // 2]`); a row with
  `Total success: 0` and `NaN` fields; the heuristic row's `ave time` is the mean over all runs,
  the same runs as its median, min and max (5.0 in the example above).

`python3 docs/notes/repro/bug_write_stats.py`

```text
original: 'times 1,1,1,5 s: Total success: 4, ave time: 2.0, median:5.0, min: 1.0, max: 5.0, ave solution: 10.0, min possible: 7.0\n'
original: None; stderr: ['java.lang.ArrayIndexOutOfBoundsException: Index 0 out of bounds for length 0', '\tat projects.multipath.ILP.Main.writeStats(Main.java:430)']
fixed: 'times 1,1,1,5 s: Total success: 4, ave time: 2.0, median:1.0, min: 1.0, max: 5.0, ave solution: 10.0, min possible: 7.0\n'
fixed: 'no success: Total success: 0, ave time: NaN, median:NaN, min: NaN, max: NaN, ave solution: NaN, min possible: NaN\n'; stderr: []
original: '1 s all goals, 9 s partial: Total success: 1, goals reached: 8, ave time: 10.0, median:9.0, min: 1.0, max: 9.0, steps: 40, exp: 40, ratio: 1.0\n'
fixed: '1 s all goals, 9 s partial: Total success: 1, goals reached: 8, ave time: 5.0, median:9.0, min: 1.0, max: 9.0, steps: 40, exp: 40, ratio: 1.0\n'
```


<a id="b-files"></a>
### B24. Files: swallowed errors, Windows paths, legacy names, writer limits

* **Java:**
  1. `Problem.readFromFile` (`Problem.java:97-125`) catches every I/O error, prints it and returns
     a `Problem` with `graph = sg = null`; the caller's `p.sg[0]` then throws (`Main.java:69, 136, 226`).
  2. Paths are `basePath + "\\<dir>\\<file>"` / `FILE_FOLDER + "\\<dir>\\" + name`
     (`Problem.java:16, 195-266`; `Main.java` everywhere). On Linux `\` is a file-name character:
     `Problem.main` throws `NullPointerException` at `new File(name).getParentFile().mkdirs()`
     (`Problem.java:46`) unless given a folder ending in `/`, and then writes flat files whose names
     contain backslashes.
  3. Legacy drivers ask for files the generator never writes: `run32x32PerformanceMetricHeuristic`
     drops `-agts-` (`Main.java:966-967`), `run32x32PerformanceMetricSplitting8Connected` reads the
     text `.txt` as serialized data (`Main.java:982`), `runFullPerformanceMetric` asks for robot
     counts in steps of 25 (the generator writes steps of 10).
  4. `Graph.writeObject` (`Graph.java:346-363`) stores a vertex in a fixed `int[13]` (9 adjacency
     entries including the self-loop): a vertex with 9 neighbours throws
     `ArrayIndexOutOfBoundsException`. A vertex without coordinates (`addVertex(int, int[])`) makes
     both writers throw `NullPointerException` (`Graph.java:351`, `Problem.java:55`). Both leave a
     truncated `.dat` (227 bytes) or an empty `.txt`.
* **Experiments:** 2 matters for running the original on Linux at all; the others need inputs the
  generators never produce.
* **original:** reproduced (file names with backslashes on POSIX, the 227-byte prefix).
  **fixed:** `os.path.join`, `FileNotFoundError` / `ValueError`, the generator's names, longer rows
  for high-degree vertices.

`python3 docs/notes/repro/bug_dat_io.py`

```text
original: write .dat, vertex with 9 neighbours: JavaArrayIndexOutOfBoundsException: Index 13 out of bounds for length 13
   file size 227
original: write .dat, no coordinates: JavaNullPointerException: Cannot invoke "java.awt.geom.Point2D.getX()" because "this.point" is null
original: write .txt, no coordinates: JavaNullPointerException: Cannot invoke "java.awt.geom.Point2D.getX()" because "this.point" is null
   .dat size 227, .txt size 0
original: read a missing file: Problem(graph=None, starts=None, goals=None)
   stderr: java.io.FileNotFoundException: missing.dat (No such file or directory)
   graph=None sg=None
original: write .txt named '\8x8-grid\x.txt' (no '/'): JavaNullPointerException: Cannot invoke "java.io.File.mkdirs()" because the return value of "java.io.File.getParentFile()" is null
fixed: write .dat, vertex with 9 neighbours: None
   file size 937
fixed: write .dat, no coordinates: ValueError: vertex 0 has no coordinates; the .dat format requires them
fixed: write .txt, no coordinates: ValueError: vertex 0 has no coordinates; the file format requires them
   .dat size None, .txt size None
fixed: read a missing file: FileNotFoundError: [Errno 2] No such file or directory: 'missing.dat'
fixed: write .txt named '\8x8-grid\x.txt' (no '/'): None
```

`python3 docs/notes/repro/bug_legacy_drivers.py`

```text
original 32x32-heuristic: JavaNullPointerException: Cannot load from object array because "<parameter1>.sg" is null
   stderr: java.io.FileNotFoundException: ./\32x32-grid\20-pct-obs-20-1001.dat (No such file or directory)
original 32x32-splitting-8c: JavaNullPointerException: Cannot load from object array because "<parameter1>.sg" is null
   stderr: java.io.FileNotFoundException: ./\32x32-grid-8c\20-pct-obs-25-agts-1001.txt (No such file or directory)
fixed 32x32-heuristic: FileNotFoundError: ./32x32-grid/20-pct-obs-20-agts-1001.dat: no such data file (generate the data set with 'python -m mppflow generate' / mppflow.experiments.generate)
fixed 32x32-splitting-8c: FileNotFoundError: ./32x32-grid-8c/20-pct-obs-25-agts-1001.dat: no such data file (generate the data set with 'python -m mppflow generate' / mppflow.experiments.generate)
```


## Graph generators (`advanced/Graph.java`)

<a id="b-grids"></a>
### B25. Narrow grids, too many agents or holes

* **Java:**
  1. `create2DGridGraph` (`Graph.java:44-87`) and `create2DGridGraph8Connected` (`:96-146`) remove
     boundary neighbours by id arithmetic (`id = y*cols + x + 1`): with `cols == 1`, `id ± 1 == id ± cols`
     and the vertical edges go; the 8-connected builder also aliases `id ± cols ∓ 1` (asymmetric
     adjacency for `cols == 2`, no self-loops for `cols == 1`).
  2. `getRandomStartGoal` (`:211-246`) and `createGeneric2DGridGraphWithHoles[8Connected]`
     (`:161`, `:186`) retry forever when asked for more agents (holes) than there are vertices;
     `getRandomStartGoalMany` (`:256-272`) throws `IndexOutOfBoundsException` (`:267`).
* **Experiments:** all grids have at least 8 columns and enough vertices.
* **original:** reproduced (`OriginalWouldHang` for the loops, after the same draws).
  **fixed:** correct neighbourhoods; `ValueError` before any draw.

`python3 docs/notes/repro/bug_grid_narrow.py`

```text
original
  4-connected 3x1: {1: [1], 2: [2], 3: [3]}
  8-connected 2x2: {1: [1, 3, 4], 2: [2, 4], 3: [1, 3], 4: [1, 2, 4]}
  8-connected 3x1: {1: [], 2: [], 3: []}
fixed
  4-connected 3x1: {1: [1, 2], 2: [1, 2, 3], 3: [2, 3]}
  8-connected 2x2: {1: [1, 2, 3, 4], 2: [1, 2, 3, 4], 3: [1, 2, 3, 4], 4: [1, 2, 3, 4]}
  8-connected 3x1: {1: [1, 2], 2: [1, 2, 3], 3: [2, 3]}
```

`python3 docs/notes/repro/bug_random_sg_overflow.py`

```text
original getRandomStartGoal(2x2, 5): OriginalWouldHang: getRandomStartGoal loops forever: cannot draw 5 distinct starts/goals from 4 vertices
original getRandomStartGoalMany(2x2, 5): JavaIndexOutOfBoundsException: Index: 0, Size: 0
original holes(3x3, 150%): OriginalWouldHang: createGeneric2DGridGraphWithHoles loops forever: no vertex is left to remove
fixed getRandomStartGoal(2x2, 5): ValueError: cannot draw 5 distinct starts/goals from 4 vertices
fixed getRandomStartGoalMany(2x2, 5): ValueError: cannot assign 5 agents on 4 vertices
fixed holes(3x3, 150%): ValueError: cannot remove 13 vertices from a graph with 9 vertices (percent_obstacle too large)
```


## Nondeterminism

<a id="b-nondet"></a>
### B26. Wall clock, threads and identity hashes

These make the original's output depend on the machine or the run. They are not
reproducible bit for bit, and `mppflow` makes each of them deterministic.

1. **Wall-clock budgets.** Every time limit (B2) and the local-repair heuristic's attempt budgets
   (`HeuristicPathPlanner.java:58, 61`: `1.5 + (5 + 4i) n/120` s per attempt; `PathPlanner`'s table by
   robot count) are wall clock. On a loaded machine the same seed gives a different plan. `mppflow`
   reads time from `clock=`; the heuristics default to a virtual clock that never advances, plus a
   deterministic `max_subproblems` budget.
2. **Threads.** `multiThreadedSplitPlanning` runs one thread per piece, all drawing from the shared
   `Math.random()` (and sharing an unsynchronized `HashMap` that can lose a piece, B14 item 4). The number of draws of a breadth-first search depends on the values drawn, so
   the generator state after the call varies from run to run (12 different states in 20 runs);
   the returned plans do not (the draws only break ties among shortest paths). Concurrent Gurobi
   runs also compete for the time limit. `PuzzleSolver` races two models (B17). `mppflow` runs the
   pieces and the models sequentially, in thread order.
3. **Identity hashes.** `Vertex` has no `equals`/`hashCode`, so `Graph.verticeSet` (a
   `HashSet<Vertex>`) iterates in identity-hash order, which leaks into the raw `vertices[]` of
   unconverted grids (`Graph.java:83, 147, 175, 200, 568, 595`). No reachable output depends on it
   (every consumer sorts or ignores the order); `mppflow` uses id order.

`python3 docs/notes/repro/bug_wallclock_heuristic.py`

```text
idle (virtual clock): feasible, makespan 53, attempts 1, goals reached 40/40
loaded (0.05 s per reading): partial, makespan 35, attempts 3, goals reached 20/40, same plan as idle: no
```


Also see [paper-vs-code.md](paper-vs-code.md#reproducibility) for the `HashSet` iteration order
of JDK 6/7 versus JDK 8 and the Gurobi version.
