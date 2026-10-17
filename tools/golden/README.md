# Golden fixtures from the original Java code

`tests/golden/*.jsonl` hold input/output records of the original Java implementation
(<https://github.com/arc-l/optimal-mrppg-journal>, directory `source/`), recorded by the tools in
this directory. They are the reference for `compat="original"` (and, wherever the original is
correct, for the default `compat="fixed"`).

| file | purpose |
|---|---|
| `Harness.java` | Java driver compiled into the package `projects.multipath.ILP` next to the unmodified original sources. Runs one case per input line and writes one JSON line per case. Contains its own minimal JSON parser and emitter (no external jars). |
| `build.sh` | Copies the original sources to a temporary directory, rewrites `import gurobi.` to `import com.gurobi.gurobi.` when the jar uses the new package name (Gurobi >= 12), and compiles them together with `Harness.java`. |
| `cases.py` | Deterministic generation of the case list (two phases, see below). |
| `record.py` | Builds the harness, runs it, restarts it after hangs, and writes `tests/golden/<group>.jsonl` and `tests/golden/META.json`. |
| `core/` | `CoreGolden.java`, `CoreAccess.java` and `record_core.sh`: the core fixtures `tests/data/core/{graph,problem,pathfinder,families}.json.gz` (see "Core fixtures" below). |

`tests/test_golden_core.py` runs every `graph`, `problem` and `pathfinder` record through the
port; the other groups are compared by the `test_*.py` module of their area.

## Regenerating

Requirements: a JDK (17 was used), Gurobi with its Java jar and a license, Python 3.10+, and a
POSIX shell with bash, `find` and `sed` (GNU or BSD/macOS).

```
git clone https://github.com/arc-l/optimal-mrppg-journal ~/src/optimal-mrppg-journal
python3 tools/golden/record.py \
    --original-src ~/src/optimal-mrppg-journal/source \
    --gurobi-jar "$GUROBI_HOME/lib/gurobi.jar" \
    --jdk /path/to/jdk-17            # optional; default: java/javac on PATH
    [--work DIR]                     # keep the build, logs and run directory
    [--only planner,solvers]         # record some groups only
    [--filter 'planner/split']       # re-record matching phase-2 cases, merged into the files
    [--check-determinism]            # run phase 2 twice and list records that differ
```

`build.sh` can also be used on its own:
`ORIGINAL_SRC=... GUROBI_JAR=... JAVAC=... OUT=classes bash tools/golden/build.sh`
(it prints the classes directory). The harness is then run as

```
java --add-opens java.base/java.lang=ALL-UNNAMED -cp classes:gurobi.jar \
     projects.multipath.ILP.Harness cases.jsonl out.jsonl progress.txt workdir
```

with `LD_LIBRARY_PATH` containing Gurobi's `lib` directory. The `--add-opens` flag is needed to
seed `java.lang.Math`'s private generator by reflection.

### Determinism

* **Gurobi**: the harness runs in a directory whose `gurobi.env` contains `Threads 1`; Gurobi
  reads it whenever the original code creates a `GRBEnv`. With one thread, Gurobi's results do
  not depend on the machine's core count. They still depend on the Gurobi version:
  `META.json` records the version (13.0.2) and the JDK used. Every model dump also records the
  `Threads` parameter the model saw.
* **`Math.random()`**: before each case the harness calls
  `Math.RandomNumberGeneratorHolder.randomNumberGenerator.setSeed(seed)`. The stream is then
  identical to `new java.util.Random(seed).nextDouble()` (`mppflow._java.MathRandom(seed)`).
* **Static state**: before each case every static flag of the original is reset to its declared
  default (`MultiagentGraphSolverGurobiTime.bPrintPaths/bDebugInfo = true`,
  `...TimeR.bPrintPaths/bDebugInfo = true`, `...TotalTime.bPrintPaths/bDebugInfo = true`,
  `...TotalTime.bReturnAnySolution = false`, `...Distance.bPrintPaths = false`,
  `...Distance.bDebugInfo = true`, `...Distance.bReturnAnySolution = false`,
  `PathFinder.bHeuristic = false`), `Problem.FILE_FOLDER` to `"D:\temp\data\dmrpp\"`, all of
  `Main`'s statistics fields to their initial values, and `Main.basePath` to a fresh, empty
  per-case directory (`$CASEDIR/`). Then the case's `statics` are applied. Each call gets fresh
  instances of `PathPlanner`, `HeuristicPathPlanner` and the solvers (except inside `Sequence`).
* **Wall clock**: time limits are either non-positive ("no limit", where the original supports
  it) or `1e6` s, so that they never bind. Wall-clock numbers that the original prints
  (`found in X seconds`, `with max time limit X`, `Time spent = X seconds.`,
  `Total time spent: X seconds.`, `, time: X`) are replaced by `<t>` in `stdout`, `stderr`,
  exception messages and file texts, unless the case has `"normalize": false` (cases whose
  times are injected). `Main.solve*` stat vectors have their time entry (index 1) replaced by
  `null`. Elapsed times are not stored.
* **Threads**: `multiThreadedSplitPlanning` and `PuzzleSolver` run Java threads. Their paths are
  deterministic (except `PuzzleSolver`'s race, see `nondet`), but the order in which the
  threads draw from the shared `Math.random()` is not, so the number of draws is not either.
  Such fields are listed in the record's `nondet`; tests must ignore them.
* Records were checked with `--check-determinism` (two full runs of phase 2, identical apart
  from `nondet` fields and `hang_stack`). The remaining run-to-run variation comes from the
  original itself: thread interleaving (`multiThreadedSplitPlanning`, `PuzzleSolver`) and its
  test `timeLimit - elapsed < 0` with `timeLimit = 0`, which depends on whether a millisecond
  has elapsed (the `-1` variants of those cases are deterministic). The local-repair heuristic
  (`planHeuristicPaths`) has wall-clock attempt budgets; the recorded instances finish well
  within them on an idle machine.

### Phases

`cases.phase1()` builds instances with the original generators (grids, grids with holes,
random problems, N^2 puzzles, the hand-made graphs of `Graph`/`Problem`/`Main`).
`cases.phase2(results)` builds every other case from those results, embedding the instances in
the arguments. Every record is therefore self-contained: its `args` are the complete input.

## Record format

One JSON object per line. Keys, in this order (optional keys are omitted when empty):

| key | meaning |
|---|---|
| `id` | unique, `<group>/<name>` |
| `fn` | the Java method, `Class.method`, or `Class.method(ParamTypes)` for overloads; see below |
| `args` | named arguments (JSON encodings below) |
| `seed` | `Math.random()` seed set immediately before the call |
| `statics` | static fields set after the reset, e.g. `{"MultiagentGraphSolverGurobiTime.bDebugInfo": false}` |
| `budget` | seconds after which the harness declared a hang (default 120) |
| `tags` | e.g. `"crash"`, `"hang"`, `"defect:<name>"` (the defect of the original this case reproduces; see "Crash and hang cases") |
| `note` | free text |
| `nondet` | fields of this record that differ from run to run (ignore them) |
| `result` | the return value (`null` for `void`) |
| `exception` | instead of `result` when the call threw: `{"class": "java.lang.NullPointerException", "message": ..., "at": ["PathPlanner.hasCycles:1152", ...]}` (`at`: up to 8 frames of the original code, innermost first). Compare `class` (and possibly the first frame); messages of helpful NPEs mention JVM-internal local names. |
| `hang` | `true` instead of `result` when the call did not return within `budget`; `hang_stack` then lists the frames (original code and `java.*`) of the case thread at that moment (informational: where a busy loop is caught varies). A port raises `OriginalWouldHang` here. |
| `mutated` | arguments that the call changed, re-encoded after the call (e.g. `convertGraph`'s `start`/`goal`, `TotalTime.solve`'s `opt`, graphs changed by `isConnectedWithoutVertex`) |
| `extra` | additional state, e.g. `multiThreadedSplitPlanning`'s `msg` (the split goals) and `mpath_keys` |
| `stdout` | text the call wrote to `System.out` (truncated at 20000 characters). Gurobi's own log goes to the native stdout and is not included. |
| `stderr` | text written to `System.err` (stack traces printed and swallowed by the original), keeping only exception lines and frames of the original code |
| `rng_next` | the next `Math.random()` value after the call (absent after a hang) |
| `draws` | number of `Math.random()` draws the call consumed (position of `rng_next` in the seeded stream; -1 if more than 2e7) |
| `statics_after` | static flags whose value differs after the call (e.g. `PathFinder.bHeuristic` left `true`) |
| `files` | files the call wrote under `$CASEDIR` (`Main.basePath`, `.dat`/`.txt` writers): `{name: {"size", "sha256", "text" | "base64"}}` (`text` if UTF-8 and at most 20000 bytes, else `base64` if at most 4096 bytes; `size` and `sha256` are dropped when the text was normalized). More than 20 files are summarized as `{"count", "sha256"}`. |

Paths printed in messages are written relative to `$CASEDIR` (the case's directory, which is
also `Main.basePath` minus the trailing `/`) and `$WORKDIR`.

### JSON encodings

* `int`, `long`, `boolean`, `String`: JSON scalars. `double`: JSON number (Java
  `Double.toString`; `NaN`, `Infinity`, `-Infinity` as strings).
* `int[]`, `int[][]`, `long[]`, `Integer[]`: (nested) arrays. `Path`: its `vertexList`;
  `Path[]`: array of those (`null` entries allowed, e.g. an unreachable goal).
* `Set`/`Collection` results: array **in Java iteration order** (`HashSet<Integer>` order matters
  for the callers of `findGoalsInDistanceRange`; `getEdgeSet` returns a `HashSet<Long>`).
* `Map` results with non-string keys (e.g. `Problem.vidNewIdMap`): array of `[key, value]` in
  iteration order.
* **Graph**:
  ```
  {"vertices": [[id, x, y] | [id], ...],   // g.vertices in array order; [id] when point == null
   "adj": [[id, [neighbours in iteration order]], ...],   // adjacencySetMap, keys ascending
   "adj_type": "TreeSet" | "HashSet",      // class of the neighbour sets (HashSet in sub-problems)
   "rows": r, "cols": c,                   // the package-private rows/columns fields
   "idVertexMap_extra": [[id, x, y]...],   // only if idVertexMap has vertices missing from the array
   "idVertexMap_missing": [id, ...],       // only if the array has vertices missing from idVertexMap
   "verticeSet_size": n}                   // only if verticeSet disagrees with idVertexMap
  ```
  The `vertices` array of graphs that the original builds from its `HashSet<Vertex>`
  (`create2DGridGraph*` with `update=false`, the `...WithHoles` generators, `getv4TestGraph`)
  has an unspecified order in Java (identity hash codes; it varies between runs and has no
  effect on any result, see original-bugs B26). It is always stored sorted by id.
  As input, the harness builds `Vertex(point, id)` (or `Vertex(id)`), fills `verticeSet`,
  `idVertexMap`, `vertices` in the given order, and one `TreeSet` (or `HashSet`) per `adj` entry.
* **Problem**: `{"graph": Graph, "sg": [[starts], [goals]], "vidNewIdMap": [[k, v], ...],
  "newIdVidMap": [[k, v], ...]}` (the maps only when non-null, i.e. for sub-problems).
* **Model dump** (`*.prepareModel` results):
  `{"ncols", "nrows", "nnz", "sense", "edgeVarMap_size", "sha256", "dump"?, "params":
  {"TimeLimit", "MIPGap", "OutputFlag", "Threads"}}`. `dump` (the canonical text below) is stored
  only when it has at most 20000 characters; `sha256` is always stored.

### Canonical model dump

The model as Gurobi holds it after `prepareModel` (and `model.update()`), as UTF-8 text with
`\n` line endings and a final newline; `sha256` is the SHA-256 of exactly this text.
`mppflow.lp.linear.LinearModel.to_canonical_text(names)` produces exactly this text when
`names` are the column names of `mppflow.model.MPPModel.column_names()` (which is what
`MPPModel.canonical_text()` and `mppflow.model.canonical_text(lm, names)` pass); `sha256` is
`LinearModel.sha256(names)`.

```
SENSE max|min
NCOLS <n>
NROWS <m>
C<i> <lb> <ub> <type> <obj> <name>          one line per column, i = 0..n-1 (Gurobi index order)
R<i> <sense> <rhs> : (<col>,<coef>) ...     one line per row, i = 0..m-1 (insertion order)
```

* `<type>`: `B` (binary), `C` (continuous), `I` (integer).
* `<sense>`: `<=`, `>=`, `=`.
* Row terms are sorted by column index; duplicate columns are summed and zero coefficients
  dropped (this is what Gurobi stores: e.g. the `T = 0` flow row `-x + x` becomes an empty row
  `R4 = 0 :`). A row with no terms ends with `" :"`.
* Numbers: an integral value with magnitude below 1e15 is printed as an integer (`0`, `1`,
  `-1`, `10`); values >= 1e100 as `inf` (<= -1e100 as `-inf`); anything else with Java's
  `Double.toString` (`mppflow._java.java_double_str`). `-0.0` prints as `0`.
* Column names:
  * `x[a,u,v,t]` for the arc of agent `a` from vertex `u` at time `t` to vertex `v` at time
    `t+1` (`u == v` is a wait), and `x[a,g,s,T]` for agent `a`'s loopback arc from its goal at
    the horizon `T` to its start at time 0. They are recovered from the original's
    `edgeVarMap` by recomputing its (int-wrapping) keys `getEdgeId(a,u,v,t)` for all
    `t < T`, all `u` and all `v` in `adj[u]`, plus the loopback key `getEdgeId(a,g,s,T)`. If two
    tuples ever mapped to the same key the name would read `x[...]|x[...]`; this does not occur
    in the recorded models.
  * `yT[a]` and `y[a,t]` for `MultiagentGraphSolverGurobiTotalTime`'s variables
    `yTVars[a]` and `yVars[a][t]`.
* Example (`models/Time/star_T3`, Main case 3's star graph, starts `[0,3]`, goals `[3,0]`, T=3):

  ```
  SENSE max
  NCOLS 16
  NROWS 18
  C0 0 1 B 0 x[0,0,0,0]
  ...
  C14 1 1 B 1 x[0,3,0,3]
  C15 1 1 B 1 x[1,0,3,3]
  R0 <= 1 : (1,1) (7,1)
  ...
  R4 = 0 : (0,1) (1,1) (14,-1)
  ...
  ```

The objective constant is always 0 in these models and is not printed. Parameters other than
those under `params` are not part of the model.

## Groups and functions

Counts are in `META.json` (`groups`). Argument names follow the Java parameter names.

### `graph.jsonl` (Graph, Vertex, Path)

| fn | args | notes |
|---|---|---|
| `Graph.create2DGridGraph` | `rows, cols, update` | `update=true` converts (ids 0..n-1, rows=cols=0) |
| `Graph.create2DGridGraph8Connected` | `rows, cols, update` | |
| `Graph.createGeneric2DGridGraphWithHoles` | `rows, cols, percentObstacle` | random |
| `Graph.createGeneric2DGridGraphWithHoles8Connected` | `rows, cols, percentObstacle` | random |
| `Graph.getRandomStartGoal` | `g, number` | random; result `[[starts],[goals]]` |
| `Graph.getRandomStartGoalMany` | `g, number` | random |
| `Graph.convertGraph` | `graph, start, goal` | protected; `start`/`goal` are mutated |
| `Graph.manual` | `neighbors: [[vId, [nbrs]], ...]` | harness helper: `addVertex(vId, nbrs)` for each entry, then `finishBuildingGraph()` |
| `Graph.getEdgeSet` | `g` | `HashSet<Long>` in iteration order |
| `Graph.getEdgeId` | `pairs: [[u, v], ...]` | per pair `[getEdgeId(u,v), getFirstVertexOfEdge(e), getSecondVertexOfEdge(e)]` |
| `Graph.removeVertex(Vertex)` | `g, vId` | `g.removeVertex(g.idVertexMap.get(vId))`; graph in `mutated.g` |
| `Graph.removeVertex(int,int)` | `g, x, y` | private |
| `Graph.addVertex(Vertex)` | `g, id, x, y` (`x: null` for no point) | |
| `Graph.addVertex(int,int[])` | `g, vId, neighbors` | |
| `Graph.finishBuildingGraph` | `g` | |
| `Graph.getId` | `g, x, y` | private |
| `Graph.updateAdjList` | `g` | removes self-loops |
| `Graph.isConnectedWithoutVertex` | `g, vId` | |
| `Graph.areVerticesConnectedWithoutV3` | `g, vId1, vId2, vId3` | private |
| `Graph.getv4TestGraph`, `Graph.getTestGraphForBridge` | none | |
| `Graph.printGraphAsAdjacenyList` | `g` | stdout |
| `Vertex.print`, `Vertex.printVertex` | `g, vId` | stdout |
| `Vertex.printVertex(StringBuffer)` | `g, vId` | result: the appended text |
| `Path.printPath` | `g, path` | stdout |

Graph serialization (`writeObject`/`readObject`) is covered through `Problem` below.
`Graph.paint`, `Path.drawPaths/drawRobots` and `Vertex.draw` are GUI-only and not covered.

### `problem.jsonl` (Problem)

| fn | args | notes |
|---|---|---|
| `Problem.createGridProblem`, `Problem.createGridProblem8Connected` | `col, row, fracObs, n` | random |
| `Problem.createN2Puzzle`, `Problem.createN2M1Puzzle` | `n` | random |
| `Problem.getA9Puzzle`, `Problem.getLongStraightWithOneGarageProblem` | none | |
| `Problem.writeToFile(String)` | `p` | private text format, written to `out/problem.txt` (see `files`) |
| `Problem.writeToFile(Problem,String)` | `p` | Java serialization, written to `problem.dat` (see `files`) |
| `Problem.readFromFile` | `source: "missing" \| "text" \| "base64", data` | |
| `Problem.roundTrip` | `p` | harness helper: `writeToFile(p, f)` then `readFromFile(f)` |
| `Problem.createN2Puzzles`, `createCrowdedHardProblems`, `create32x32Problems`, `create32x32Problems8Connected`, `create24x18PerformanceTestingProblems`, `createCrowdedHardProblems8Connected` | `folder: "case" \| "default", samples` | batch generators, `FILE_FOLDER` = a fresh directory (`"case"`) or the original default. Result: `{"nfiles", "ntxt", "ndat", "sha256_txt", "sha256_dat", "sha256_names", "first_names", "samples"}` where `sha256_txt`/`sha256_dat` hash `name \0 size \0 bytes` over the files sorted by name, `sha256_names` hashes the sorted names joined by `\n` (each followed by `\n`), and `samples` holds the text of the first `samples` `.txt` files by name. File names keep the original's literal backslashes, e.g. `\n2puzzle\9-puzzle-1001.txt`. |
| `Problem.main` | `argv` (`"$BATCH"` = that directory + `/`), `samples` | same digest |

### `pathfinder.jsonl` (PathFinder)

| fn | args |
|---|---|
| `PathFinder.getDistancesToAllVertices` | `g, start, inputDistances` (`null` or an array, which is mutated) |
| `PathFinder.findGoalsInDistanceRange` | `g, start, dMin, dMax` (result in `HashSet` order) |
| `PathFinder.findShortestPaths` | `g, start, goal` (random) |
| `PathFinder.findOneShortestPath` | `g, start, goal, alreadyVisitedVertices` (`null` or array -> `HashSet`) |
| `PathFinder.getTotalDistance(int[][])`, `PathFinder.getTotalDistance(Path[])` | `paths` |
| `PathFinder.getMakespanLowerBound`, `PathFinder.getTotalTimeLowerBound` | `g, start, goal` (random) |

(`addToQueue`/`deQueue` are private helpers of `findOneShortestPath`.)

### `models.jsonl` (model builders)

| fn | args |
|---|---|
| `MultiagentGraphSolverGurobiTime.prepareModel`, `MultiagentGraphSolverGurobiTimeR.prepareModel` | `graph, starts, goals, integer, setOptimal, timeSteps, timeLimit` |
| `MultiagentGraphSolverGurobiTimePuzzle.prepareModel` | `graph, starts, goals, integer, setOptimal, timeSteps` (reachability from `PuzzleSolver.updateReachableSets`) |
| `MultiagentGraphSolverGurobiTotalTime.prepareModel`, `MultiagentGraphSolverGurobiDistance.prepareModel` | `graph, starts, goals, integer, timeSteps, timeLimit`, optional `gap` (set as `MIPGap` on the environment, as `solve` does) |
| `<Solver>.updateReachableSets` (`Time`, `TotalTime`, `Distance`, `TimeR`, `PuzzleSolver`) | `graph, starts, goals, timeSteps`; result `[agent][t] -> [reachable vertices]` |
| `MultiagentGraphSolverGurobiTime.keyHelpers`, `MultiagentGraphSolverGurobiTimeR.keyHelpers` | `tuples: [[a, v1, v2, t], ...]`; per tuple `{"edgeId", "vertexTimeId" (v1,t), "agentVertexTimeId" (a,v1,t), "edgeTimeId" (v1,v2,t)}`, plus TimeR's `"timeFromEdgeId"`, `"isV1V2Same"` |

The harness calls `updateReachableSets(graph, starts, goals, timeSteps)` and then
`prepareModel(env, ...)` on a fresh solver, with a fresh `GRBEnv` (`OutputFlag 0`; the flag
is then set by `prepareModel` from `bDebugInfo`, which these cases set to `false`).

### `solvers.jsonl`

| fn | args |
|---|---|
| `MultiagentGraphSolverGurobiTime.solve`, `MultiagentGraphSolverGurobiTimeR.solve` | `graph, starts, goals, solveRelaxedFirst, setOptimal, extraSteps, hasTimeLimit, timeLimit` |
| `MultiagentGraphSolverGurobiTotalTime.solve` | `graph, starts, goals, opt, gap, timeSteps, hasTimeLimit, timeLimit` (`opt` mutated) |
| `MultiagentGraphSolverGurobiDistance.solve` | `graph, starts, goals, gap, expansion, timeLimit` |
| `PuzzleSolver.solve(double)` | `graph, starts, goals, timeLimit` (`new PuzzleSolver(...).solve(timeLimit)`) |
| `PuzzleSolver.solve(Problem,double)` | `p, timeLimit` |

The result is the path array `paths[agent][t]` (makespan = `len(paths[0]) - 1`) or `null`.
`retrievePaths`, `isEdgeVarSetToTrue` and the `storeVarFor*` helpers are exercised through these.
`MultiagentGraphSolverGurobiTimePuzzle.solve` only runs inside `PuzzleSolver`. The `callback`
methods are dead code or only reachable through `PuzzleSolver`.

### `planner.jsonl` (PathPlanner)

| fn | args |
|---|---|
| `PathPlanner.printPaths`, `PathPlanner.printPathToString` | `g, paths` |
| `PathPlanner.printStartAndGoals` | `g, start, goal` |
| `PathPlanner.numberOfGoalsReached` | `paths, goal` |
| `PathPlanner.parsePaths` | `paths` (Path[]) |
| `PathPlanner.hasCycles` | `paths` |
| `PathPlanner.isPathSetValid` | `paths, graph, starts, goals` |
| `PathPlanner.mergePaths` | `pathFromStart, pathFromMiddle` |
| `PathPlanner.moveAgentsIndependently` | `paths` |
| `PathPlanner.moveAgentsIndependentlyWhenCan` | `g, start, goal, pathArray` (`start`, `pathArray` mutated) |
| `PathPlanner.createSubProblem` | `g, paths, svs, goal, distance, attempt, distanceToGoals` (`"fromGoals"` = computed as `planHeuristicPaths` does, or a matrix, or `null`) |
| `PathPlanner.splitPaths` | `g, start, goal, random` |
| `PathPlanner.splitPathBySize` | `g, start, goal, random, firstSize` |
| `PathPlanner.getLongestPathLength` | `g, start, goal` |
| `PathPlanner.planPathsAdvanced`, `PathPlanner.planPathsAdvancedPMG` | `g, start, goal, solveRelaxed, extraSteps, setOptimal, timeLimit` |
| `PathPlanner.planPathsAdvancedTT` | `g, start, goal, opt, gap, timeSteps, timeLimit` |
| `PathPlanner.planPathsAdvancedTD` | `g, start, goal, gap, timeSteps, timeLimit` |
| `PathPlanner.planPathsAdvancedSplit` | `g, start, goal, setOptimal, timeLimit, splits` |
| `PathPlanner.planPathsAdvancedSplitDist` | `g, start, goal, gap, timeLimit, splits` |
| `PathPlanner.planPathsAdvancedSplitNoCycle` | `g, start, goal, setOptimal, timeLimit, splits, top` |
| `PathPlanner.planPathsAdvancedSplitBySize` | `g, start, goal, setOptimal, timeLimit, size` |
| `PathPlanner.planPathsAdvancedSplitBySizeNoCycle` | `g, start, goal, setOptimal, timeLimit, size, top` |
| `PathPlanner.multiThreadedSplitPlanning` | `g, start, goal, timeLimit, splits` (`extra.msg`, `extra.mpath_keys`) |
| `Sequence` | `calls: [{"fn", "args"}, ...]`: several calls on **one** planner instance; result: per call `{"fn", "result" \| "exception", "mutated"?, "extra"?}` |

`createThread` (the worker of `multiThreadedSplitPlanning`) is covered through it.

### `heuristic.jsonl` (the local-repair heuristic: PathPlanner and HeuristicPathPlanner)

| fn | args |
|---|---|
| `PathPlanner.planHeuristicPaths`, `HeuristicPathPlanner.planHeuristicPaths` | `graph, start, goal` |
| `PathPlanner.planHeuristicPath` | `g, start, goal, timeLimit`, optional `attempt`, `distanceToGoals` |
| `HeuristicPathPlanner.planHeuristicPath` | `g, s, goal, timeLimit`, optional `distanceToGoals` |
| `PathPlanner.main`, `HeuristicPathPlanner.main` | none |
| `PathPlanner.numberOfGoalsReached`, `HeuristicPathPlanner.numberOfGoalsReached` | `paths, goal` |
| `HeuristicPathPlanner.printPaths` | `g, paths` |
| `HeuristicPathPlanner.existsEdgeCollision`, `HeuristicPathPlanner.existsVertexCollision` | `paths, t` |
| `HeuristicPathPlanner.moveAgentsIndependentlyOneStep` | `pathArray, paths, t` (`pathArray` mutated) |
| `HeuristicPathPlanner.createSubProblem` | `g, paths, ct, svs, goal, distance, distanceToGoals` |
| `Sequence` | as above |

The heuristic's attempts have wall-clock budgets (1.5 s + ...); the recorded instances are small
enough that no budget binds (see original-bugs B26).

### `main.jsonl` (Main)

| fn | args |
|---|---|
| `Main.main` | `argv` (`"$CASEDIR"` is replaced by the case directory) |
| `Main.solveProblem`, `Main.solveProblemPMG` | `p, solveLP, timeLimit` |
| `Main.solveProblemSuboptimal` | `p, solveLP, setOptimal, extraSteps, timeLimit, splitLevel, noCycle` |
| `Main.solveProblemTotalTime`, `Main.solveProblemDistance` | `p, gap, timeLimit` |
| `Main.solveProblemDistanceSplit` | `p, gap, timeLimit, splits` |
| `Main.solvePuzzleProblem` | `instance, p, timeLimit` |
| `Main.solveProblemWithHeuristic` | `p` |
| `Main.statsSequence` | `kind: "plain" \| "heuristic", test, stats: [stat vector \| null, ...], n` (heuristic only), `write` (default true): harness helper calling `resetStats(test)` / `resetHeuristicStats(test)`, then `processStats(stat)` / `processHeuristicStats(stat, n)` for each injected vector, then `writeStats()` / `writeHeuristicStats()`. The result holds `Main`'s statistics fields afterwards; the written text is under `files` (`\n2puzzle-result.txt`). |
| `Main.writePuzzleStats` | `instance, time` |
| `Main.writeString` | `str` |
| `Main.writePuzzleResult` | `instance, time, g, paths` |
| `Main.run*` (all 18 `run...` drivers) | `params`: the positional arguments (e.g. `[10, 20, 1800.0, 1, false]`) |

`Main.solve*` results are the stat vectors with the time entry replaced by `null`. The `run*`
drivers read `.dat` files from `Main.basePath`, which is empty here; these records capture
how each driver reacts to missing data (printed exceptions, crash, or silent end).

## Core fixtures (`tests/data/core`)

`tests/data/core/{graph,problem,pathfinder,families}.json.gz` (read by `tests/test_graph.py`,
`test_problem.py` and `test_pathfinder.py`) come from `core/CoreGolden.java`, a second recorder
compiled next to the original classes (`CoreAccess.java` sits in the package
`projects.multipath.advanced` to reach package-private members):

```
ORIGINAL_SRC=~/src/optimal-mrppg-journal/source GUROBI_JAR=$GUROBI_HOME/lib/gurobi.jar \
JAVAC=/path/to/jdk-17/bin/javac JAVA=/path/to/jdk-17/bin/java \
    bash tools/golden/core/record_core.sh        # OUT=dir to write elsewhere, WHAT=graph,problem
```

A re-recording equals the committed files except for the `vertices` order of graphs built
without `update` (unconverted grids, grids with holes): that array is filled from a
`HashSet<Vertex>`, whose order follows `Object.hashCode()` and changes from run to run; the
tests compare those arrays as sets.

`dat_streams.json.gz`, `dat_rows.json.gz`, `writers.json.gz` and `treebins.json.gz` were
recorded once with small probes (JDK 17, the original classes) that are not included here. Each
case stores its input next to the original's outcome (the serialized stream in base64 for
`Problem.readFromFile`, the writer's problem and output bytes, the `HashSet` operation sequence
or grid sweep), so the tests need nothing else, and any case can be checked again by feeding
its input to the original (`Problem.readFromFile`, the writers, `PathFinder`).

## Crash and hang cases

Cases tagged `crash` (the original throws) or `hang` (it never returns) reproduce defects of
the original; the tag `defect:<name>` names the defect, and the table maps the names to the
entries of our notes:

| entry | tags `defect:<name>` |
|---|---|
| [original-bugs.md](../../docs/notes/original-bugs.md) B1 | `minmakespan-no-tmax` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B2 | `hastimelimit-nonpositive` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B3 | `dist-nan-gap-zero-objective`, `tt-nan-gap-zero-objective` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B4 | `vertex-cap-t0-unconstrained` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B5 | `unreachable-goal-npe` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B6 | `timer-overflow-single-move` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B10 | `splitpaths-infinite-loop` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B12 | `nocycle-printpaths-leak`, `nocycle-unbounded-retry-hang` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B13 | `splitdist-null-without-limit` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B14 | `mt-grbexception-hang`, `mt-nondeterminism-scope`, `mt-nonpositive-timelimit`, `mt-split-race`, `mt-stale-mpath` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B15 | `longest-path-off-by-one`, `splitbysize-infinite-loop` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B16 | `empty-input-crash` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B17 | `puzzle-deadlock-both-fail` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B18 | `distanceToGoals-null-npe`, `zero-agents-aioobe` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B19 | `bheuristic-leak-nonoptimal`, `heuristic-stats-off-by-one` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B20 | `case5-fallthrough` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B21 | `subopt-npe-on-failure` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B22 | `caller-npe-null-makespan` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B23 | `median-treeset-dedup` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B24 | `dat-write-degree-aioobe`, `dat-write-null-point-npe`, `legacy-missing-files`, `problem-main-linux-npe`, `readfromfile-swallow` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B25 | `grid4-cols1-no-edges`, `grid8-narrow-wrap`, `random-sg-infinite-loop`, `random-sg-many-ioobe` |
| [original-bugs.md](../../docs/notes/original-bugs.md) B26 | `wallclock-nondeterminism` |
| [paper-errata.md](../../docs/notes/paper-errata.md) E6 | `fig8-split-child-needs-3` |

Not recorded, because they need wall-clock timeouts or a broken environment to trigger:
`nocycle-npe` (B12: a leaf ILP timing out during a retry), `puzzle-hang-on-exception` (B17:
unwritable log file), the GRBException variant of `mt-grbexception-hang` (B14: missing license;
the recorded case triggers the same hang with an exception from bad input),
`puzzle-stale-failedattempt` (B17: needs injected delays), `timelimit-*` (B2: cumulative
subtraction of elapsed time: only the amount of work before a timeout changes), and `PuzzleSolver.solve(Problem, ...)` with more than
16 agents (a 25-puzzle race of 20+ s whose outcome depends on timing).

## Differential testing on fresh random inputs (`differential.py`)

`differential.py` complements the fixed golden records: it generates **fresh** random inputs
(seeded; the `Math.random()` seeds are drawn from `[1000, 2^31)`, so they never coincide with
those of `cases.py`), runs every case once through the live original (`Harness.java`, built by
`build.sh`, Gurobi `Threads 1`) and once through `mppflow` with `compat="original"`,
`backend="gurobi"`, `params={"Threads": 1}`, and compares the two records field by field. Cases
use the record format above, so a mismatch's `repro` (`fn`, `args`, `seed`, `statics`) can be
replayed with the harness directly.

```
python3 tools/golden/differential.py --n 30 --seed 1 --report diff.json \
    --classes CLASSES                 # or --original-src ~/src/optimal-mrppg-journal/source (builds)
    [--gurobi-jar $GUROBI_HOME/lib/gurobi.jar] [--jdk /path/to/jdk-17]
    [--groups generators,bfs,...]     # default: all groups
    [--ids REGEX]                     # rerun some cases of the same generated list
    [--java-results cache.json]       # reuse / store the Java records
    [--work DIR] [--list]
```

`--classes` defaults to `$MPPFLOW_JAVA_HARNESS`, `--gurobi-jar` to `$GUROBI_JAR` (or
`$GUROBI_HOME/lib/gurobi.jar`), `--jdk` to `$MPPFLOW_JDK`. `LD_LIBRARY_PATH` must contain Gurobi's
`lib` directory. The case list is a deterministic function of `--seed` and `--n` (each group draws
from its own stream). The exit status is 0 when every compared case matches.

| group | functions (`--n` cases each) |
|---|---|
| `generators` | `Graph.create2DGridGraph(8Connected)`, `createGeneric2DGridGraphWithHoles(8Connected)`, `getRandomStartGoal(Many)` (`Many` sometimes with more robots than vertices), `Problem.createGridProblem(8Connected)`, `createN2Puzzle` |
| `bfs` | `PathFinder.getDistancesToAllVertices`, `findShortestPaths` (draws; sometimes `bHeuristic`), `findOneShortestPath`, `findGoalsInDistanceRange` (HashSet order) |
| `lowerbounds` | `PathFinder.getMakespanLowerBound`, `getTotalTimeLowerBound` |
| `split` | `PathPlanner.splitPaths` (`random` both ways), `splitPathBySize`, `getLongestPathLength` |
| `models` | `prepareModel` of Time, TimeR, TimePuzzle, TotalTime, Distance (counts, sense, `edgeVarMap` size, canonical-dump SHA-256, `TimeLimit`/`MIPGap`/`OutputFlag`) |
| `solvers` | `MultiagentGraphSolverGurobi{Time,TimeR,TotalTime,Distance}.solve` (paths, printed text, draws, mutated `opt`) |
| `planner` | `PathPlanner.planPathsAdvanced`, `planPathsAdvancedSplitNoCycle`, `planPathsAdvancedSplitDist` |
| `mtsplit` | `PathPlanner.multiThreadedSplitPlanning`, splits 1..3 (`extra.msg`, `extra.mpath_keys`) |
| `heuristic` | `PathPlanner.planHeuristicPaths`, `HeuristicPathPlanner.planHeuristicPaths` |
| `validity` | `PathPlanner.isPathSetValid`, `hasCycles` on optimal plans, padded shortest paths, perturbed sets (moved, jumped, met, swapped, truncated, extended, reversed, duplicated, empty rows) and full or partial rotations around grid cycles |
| `main` | `Main.solveProblem`, `solveProblemPMG`, `solveProblemSuboptimal`, `solveProblemTotalTime`, `solveProblemDistance`, `solveProblemDistanceSplit`, `solveProblemWithHeuristic` (stat vectors without the time entry, printed text, files, static flags left changed) |

Instances are random grid problems from the port's own generators (4- or 8-connected, 0-30 %
obstacles, a fresh seed), sometimes cut in two components (unreachable goals) or with every goal
equal to its start (the NaN-gap cases); both sides receive the same embedded instance.

Compared fields: the outcome (result, exception class, hang = `OriginalWouldHang`), the result,
`stdout` (wall-clock numbers replaced by `<t>` on both sides; a Java text truncated at 20000
characters is compared as a prefix), `draws`, `rng_next`, `mutated`, `extra` and, for `main`,
`statics_after` and the written `.txt` files. Not compared: the fields listed in `nondet`
(`draws`/`rng_next` of `multiThreadedSplitPlanning` and of `solveProblemSuboptimal` with
`splitLevel > 0` and `noCycle = false`: thread interleaving), exception messages, `hang_stack`,
and `statics_after` outside `main` (the heuristic planners do not report it).

A difference is reported as `skipped` (with its reason) instead of a mismatch when it has a
documented cause:

* `time-limited-endless-loop`: the original hung (exceeded the case budget) in a `T++` loop on an
  instance that the port proved infeasible under a time limit. The port returns the original's
  eventual `null` at once (`mppflow.solvers`, "Infinite loops"), so its callers continue (e.g.
  `planPathsAdvancedSplitDist` then throws the original's `NullPointerException`); the original
  gets there only when its limit (here 10^6 s) expires.
* `wallclock`: the local-repair heuristic's attempt budgets bound in the Java run (its output
  contains `time limit reached` / ` time out.`, or it hung). The port runs the heuristic on a
  frozen clock (an infinitely fast machine); see original-bugs B26.
* `mt-hashmap-race`: `multiThreadedSplitPlanning` returned `null` in Java, with some keys missing
  from `extra.mpath_keys` and nothing printed, while every piece succeeded in the port (or a piece
  failed in both runs, so both results are `null` with the same messages, and Java's keys are a
  strict subset of the port's: Java also lost the `put` of a piece that succeeded; `--n 150 --seed
  5550123`, case `diff/mtsplit/035_...`, kept `[0,2,4,5,6,7]` against the port's
  `[0,2,3,4,5,6,7]` and matched on 3 of 3 reruns). Worker
  threads that finish at the same moment can lose a `put` into the unsynchronized `HashMap mpath`
  (original-bugs B14, item 4). Tiny pieces make this frequent: on the 2-robot 4x6 instance of
  `--n 30 --seed 3` case `diff/mtsplit/007_...` (splits 2, two one-step pieces) about 12 % of the Java
  runs return `null`. Rerunning the Java case usually gives the port's plan.

`tests/test_differential.py` wraps the tool for pytest (markers `java`, `gurobi`, `slow`; one
test per group, `MPPFLOW_DIFF_N` cases each, default 4, seed `MPPFLOW_DIFF_SEED`). It is skipped
unless `MPPFLOW_JAVA_HARNESS` is set:

```
MPPFLOW_JAVA_HARNESS=CLASSES GUROBI_JAR=$GUROBI_HOME/lib/gurobi.jar MPPFLOW_JDK=/path/to/jdk-17 \
    python3 -m pytest -q -m java tests/test_differential.py
```
