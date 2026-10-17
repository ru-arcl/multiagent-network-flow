# Solver notes

`mppflow` builds every ILP as a neutral `mppflow.lp.LinearModel` and solves it with one of three
backends (`backend=` in every solver, planner and driver):

| backend | requires | use |
|---|---|---|
| `"highs"` (default) | scipy >= 1.15.3 (`scipy.optimize.milp`, HiGHS) | open-source default |
| `"gurobi"` | gurobipy and a license | exact reproduction of the original's plans (`params={"Threads": 1}`) |
| `"builtin"` | nothing | pure-Python simplex and branch and bound; small models only (the browser demo runs the same algorithm) |

The reproductions are in [`repro/`](repro/); the output shown is what they print on our machine
(Python 3.11, scipy 1.17.1, Gurobi 13.0.2). Output that depends on the HiGHS version starts with
a `scipy` version line (scipy bundles HiGHS); `tests/test_notes_repro.py` compares such a block
only on that scipy version.

## 1. HiGHS: presolve defects in scipy 1.17.1 and 1.18.1 (and one in 1.15.3)

**What we observed.** With scipy 1.17.1 (its bundled HiGHS), the MIP presolve gives wrong answers
or crashes on some small *general-integer* models (integer columns with non-binary bounds),
found by a random stress generator. scipy 1.18.1, the newest release at the time of writing
(the one a fresh `pip install .` picked on Python 3.13 in our check), behaves the same on all four
models (output below):

* an integer column with fractional bounds (`[0.5, 1.2]`, `[-1.5, -0.8]`): "infeasible" for a
  feasible model, or "optimal" for a suboptimal point;
* status 4 "Solve error" without a solution on an infeasible model;
* a segmentation fault (the Python process dies) on a 6-column model with integral bounds
  (m320). It is nondeterministic: in some runs (2-15% in our counts) the same model is
  instead reported "infeasible", which is wrong (the optimum is 1.0);
* a stray debug line on standard output, `HighsMipSolverData::transformNewIntegerFeasibleSolution tmpSolver.run();`,
  printed by native code on some MIPs (harmless; the repro scripts discard native output).

With presolve off, all four models solve correctly. We saw the same kind of failure with highspy
1.15.1. So moving to the newest scipy does not avoid these defects. With scipy 1.15.3 (the newest scipy for Python 3.10) we saw only the fractional-bound
defect of m390: presolve calls the suboptimal -2.75 optimal, as in scipy 1.17.1. The other three
models solve correctly there, and mppflow's bound rounding (below) fixes m390 in both versions.

`python3 docs/notes/repro/highs_presolve_bug.py`

```text
scipy 1.17.1
m293 raw milp, presolve on : objective None, The problem is infeasible. (HiGHS Status 8: model_status is Infeasible; primal_status is None)
m293 raw milp, presolve off: objective 0.10000000000000009, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m293 mppflow highs         : optimal 0.10000000000000009
m293 mppflow highs isolate : optimal 0.10000000000000009
m390 raw milp, presolve on : objective -2.75, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m390 raw milp, presolve off: objective -0.75, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m390 mppflow highs         : optimal -0.75
m390 mppflow highs isolate : optimal -0.75
m968 raw milp, presolve on : objective None, (HiGHS Status 4: Solve error)
m968 raw milp, presolve off: objective None, The problem is infeasible. (HiGHS Status 8: model_status is Infeasible; primal_status is None)
m968 mppflow highs         : infeasible None (first attempt: HiGHS status 4; retried with presolve off)
m968 mppflow highs isolate : infeasible None (first attempt: HiGHS status 4; retried with presolve off)
m320 raw milp, presolve on : process died (exit code -11)
m320 raw milp, presolve off: objective 1.0, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m320 mppflow highs         : process died (exit code -11)
m320 mppflow highs isolate : optimal 1.0 (first attempt: HiGHS status -2; retried with presolve off)
```

With scipy 1.18.1 (Python 3.13, numpy 2.5.4), the same as 1.17.1 apart from the version line:

`python3 docs/notes/repro/highs_presolve_bug.py`

```text
scipy 1.18.1
m293 raw milp, presolve on : objective None, The problem is infeasible. (HiGHS Status 8: model_status is Infeasible; primal_status is None)
m293 raw milp, presolve off: objective 0.10000000000000009, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m293 mppflow highs         : optimal 0.10000000000000009
m293 mppflow highs isolate : optimal 0.10000000000000009
m390 raw milp, presolve on : objective -2.75, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m390 raw milp, presolve off: objective -0.75, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m390 mppflow highs         : optimal -0.75
m390 mppflow highs isolate : optimal -0.75
m968 raw milp, presolve on : objective None, (HiGHS Status 4: Solve error)
m968 raw milp, presolve off: objective None, The problem is infeasible. (HiGHS Status 8: model_status is Infeasible; primal_status is None)
m968 mppflow highs         : infeasible None (first attempt: HiGHS status 4; retried with presolve off)
m968 mppflow highs isolate : infeasible None (first attempt: HiGHS status 4; retried with presolve off)
m320 raw milp, presolve on : process died (exit code -11)
m320 raw milp, presolve off: objective 1.0, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m320 mppflow highs         : process died (exit code -11)
m320 mppflow highs isolate : optimal 1.0 (first attempt: HiGHS status -2; retried with presolve off)
```

With scipy 1.15.3 (Python 3.10):

`python3 docs/notes/repro/highs_presolve_bug.py`

```text
scipy 1.15.3
m293 raw milp, presolve on : objective 0.10000000000000009, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m293 raw milp, presolve off: objective 0.10000000000000009, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m293 mppflow highs         : optimal 0.10000000000000009
m293 mppflow highs isolate : optimal 0.10000000000000009
m390 raw milp, presolve on : objective -2.75, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m390 raw milp, presolve off: objective -0.75, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m390 mppflow highs         : optimal -0.75
m390 mppflow highs isolate : optimal -0.75
m968 raw milp, presolve on : objective None, The problem is infeasible. (HiGHS Status 8: model_status is Infeasible; primal_status is None)
m968 raw milp, presolve off: objective None, The problem is infeasible. (HiGHS Status 8: model_status is Infeasible; primal_status is None)
m968 mppflow highs         : infeasible None
m968 mppflow highs isolate : infeasible None (first attempt: HiGHS status 8; retried with presolve off)
m320 raw milp, presolve on : objective 1.0, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m320 raw milp, presolve off: objective 1.0, Optimization terminated successfully. (HiGHS Status 7: Optimal)
m320 mppflow highs         : optimal 1.0
m320 mppflow highs isolate : optimal 1.0
```

(The m968 isolate line reports a retry because in isolate mode an "infeasible" answer on a model
with general-integer columns is confirmed with presolve off; see point 3 of "What mppflow does"
below.)

(The m320 lines of scipy 1.17.1 and 1.18.1 vary between runs. In a minority of runs presolve answers
"infeasible" instead of crashing, which is a wrong answer: the `raw milp, presolve on` line then
reads `objective None, The problem is infeasible. (HiGHS Status 8: ...)` and the `mppflow highs`
line `infeasible None`. In isolate mode a crash and a wrong "infeasible" both lead to the retry
with presolve off, so that line always ends with `optimal 1.0`, but its first attempt is then
`HiGHS status 8` instead of `-2`. In 40 runs of each line with scipy 1.17.1 we saw 1, 4 and 2
such runs; with scipy 1.18.1, one of two full runs of the script had the "infeasible" answer on
the `raw milp, presolve on` line.
`tests/test_notes_repro.py` accepts either outcome on these three lines. Repeat with
`python3 docs/notes/repro/highs_presolve_bug.py m320 on` (or `mppflow`, `isolate`).)


**What mppflow does** (`mppflow/lp/highs.py`):

1. The bounds of integer and binary columns are rounded inward (`ceil(lb − 1e-6)`,
   `floor(ub + 1e-6)`) before the call, as the builtin solver does. This does not change the
   feasible set, and integral bounds (every MPP model) are passed unchanged. `relax=True` keeps
   the fractional bounds (the true LP relaxation).
2. If HiGHS ends with an error or unknown status and no solution, the model is solved once more
   with presolve off (with the time that is left); the first attempt is recorded in
   `solution.extra["highs_retry"]`. If that also fails, `RuntimeError` is raised.
3. A segmentation fault cannot be caught in-process, and neither 1 nor 2 prevents it (m320 has
   integral bounds, and a crash never returns a status). The wrong "infeasible" on the same
   model is a definite status, so it is not retried either. `params={"isolate": True}` (or
   `MPPFLOW_HIGHS_ISOLATE=1`) runs every `milp` call in a worker process (a fresh
   interpreter, reused between solves). A crash then counts as an error without a solution
   and is retried with presolve off in a new worker. On models with general-integer columns,
   an "infeasible" answer is also confirmed with presolve off. An interrupted call
   (`KeyboardInterrupt`, or an exception from a signal handler while the call waits) kills
   its worker before the exception propagates; the next call starts a new one (a worker
   left running would answer the interrupted request, and the next call would read that
   answer as its own). Sending the model to the worker has a cost, and starting a worker
   takes about a second, so this is off by default.
   Otherwise pass `params={"presolve": False}`, use scipy 1.15.3 (where only m390's defect
   appears, and the bound rounding fixes it), or use Gurobi. scipy 1.17.1 and 1.18.1 both have
   the m320 defect.

**MPP models are not affected, as far as we have seen.** Our models have binary arc columns and,
for total time and max distance, integer counters with bounds `[0, T]`, all integral. In our stress
test of 4668 MPP models (389 random grid instances of 3–6 × 3–6 vertices with 0–20% obstacles; the
makespan, total-time, total-distance and max-distance models at `T_min`, `T_min + 1` and
`T_min + 3`), HiGHS with presolve on agreed with Gurobi 13.0.2 on every model, with no crash. (The
only disagreements came from the raw `milp` call on 8 degenerate `T = 0` models, where every robot
is already at its goal and every column is fixed; it reports "Model error", while `mppflow`'s
backend solves them.) A reduced version runs in about a minute:

`python3 docs/notes/repro/highs_mpp_stress.py`

```text
600 models from 50 instances; solvers compared: HiGHS presolve on/off and Gurobi; mismatches: 0
```

**Why the floor is scipy 1.15.3, not 1.15.0.** scipy 1.15.0-1.15.2 copy the column duals out of
HiGHS one element at a time, fetching the whole dual list again for each element, which takes
time quadratic in the number of columns. Small models do not notice, but the test model of 40000
columns (a 2-vertex instance at `T = 10000`, in `tests/test_model.py`) had not returned after
150 s with scipy 1.15.0 on our machine; 1.15.3 solves it in about 7 s and 1.18.1 in under 1 s.
The floor used to be declared as `scipy>=1.15`, although only 1.15.3 and newer had been tested;
it is now 1.15.3, and an older scipy warns at the first solve.


**Speed: the makespan model on large sparse grids.** With presolve on (our default), HiGHS can be
one to two orders of magnitude slower than with presolve off, or than Gurobi, on the makespan
ILP of a large grid with few robots. On a 20x20 grid without obstacles and 4 robots
(`create_grid_problem(20, 20, 0.0, 4, MathRandom(3))`, optimal makespan 27), the one ILP at
`T = 27` takes 8-10 s with scipy 1.17.1 (40-100 s on a loaded machine) and 21-45 s with scipy
1.15.3; with `params={"presolve": False}` it takes 0.1-0.8 s, and Gurobi 0.4-0.6 s (seed 5 is
similar). The answer is the same. Presolve off is not faster in general: on an 8x8 grid with
20% obstacles and 30 robots it took 22 s against 32 s, on the 8-puzzle 2.7 s against 1.8 s, on a
16x16 grid with 40 robots 17.5 s against 16.5 s (scipy 1.17.1). So we keep the default and
suggest `params={"presolve": False}` (or `backend="gurobi"`) for makespan runs on large open
grids. The LP relaxation at `T = 27`, which the makespan loop solves first, is already integral
here, but the loop follows the original and solves the ILP at that horizon anyway: taking the
LP's point when it is integral would change the plans that `compat="fixed"` returns on every
backend, so we leave it.

`python3 docs/notes/repro/highs_makespan_presolve_speed.py`

```text
presolve on : optimal 27 in 9.9 s (ILP: 9.7 s)
presolve off: optimal 27 in 0.1 s (ILP: 0.1 s)
LP relaxation at T = 27: optimal 4.0, 0 fractional of 14683 columns
```

## 2. Gurobi: determinism

* The original sets only `OutputFlag`, `TimeLimit` (`1e10` when there is no limit) and, for total
  time and total distance, `MIPGap`. Its puzzle model (`MultiagentGraphSolverGurobiTimePuzzle`)
  sets only logging parameters (`OutputFlag` 1, `LogToConsole` 0, `LogFile`
  `C:\Temp\25puzzle-instance-<k>`) and no `TimeLimit`. `mppflow` sets the same solver
  parameters (it passes no time limit instead of `1e10`) and writes no log file (`OutputFlag` 0;
  see [B17](original-bugs.md#b-puzzle)).
* Many MPP instances have several optimal plans; which one a solver returns depends on the solver,
  its version and its threads. With the same Gurobi version (13.0.2 for `tests/golden/`) and
  `Threads=1`, `mppflow` builds every model column for column and row for row as the original does,
  and gets the original's plans exactly; the golden tests compare plans only under this setting.
  With more threads, Gurobi's result can depend on timing. Other backends are checked on objective
  values and plan validity.
* Time limits are wall clock in every backend, so a run that hits a limit is not reproducible
  across machines.

Below, Main case 3 as `Main.solveProblem` runs it: Gurobi (`Threads=1`) returns the plan the live
original prints, HiGHS returns another optimal plan; then HiGHS and `builtin` on growing grids.

`python3 docs/notes/repro/solver_backends.py`

```text
highs: 1 distinct plan(s) in 3 runs: ['[[0, 1, 2, 1, 3], [3, 3, 1, 0, 0]]']
builtin: 1 distinct plan(s) in 3 runs: ['[[0, 0, 1, 3, 3], [3, 1, 2, 1, 0]]']
gurobi Threads=1: 1 distinct plan(s) in 3 runs: ['[[0, 0, 1, 3, 3], [3, 1, 2, 1, 0]]']
3x3, 3 robots, highs: optimal, makespan 3, 0 s
3x3, 3 robots, builtin: optimal, makespan 3, 0 s
5x5, 8 robots, highs: optimal, makespan 5, 0 s
5x5, 8 robots, builtin: optimal, makespan 5, 0 s
6x6, 12 robots, highs: optimal, makespan 6, 0 s
6x6, 12 robots, builtin: time_limit, makespan None, 20 s
```


## 3. compat="original" needs exact binaries

The original extracts paths with `X == 1.0` and silently continues from vertex 0 when no arc
matches ([original-bugs.md, B9](original-bugs.md#b-exact)). Gurobi returns exact binaries; HiGHS
sometimes returns values such as `0.9999999999999998`, and the extracted plan is then invalid.
`mppflow` issues `mppflow.model.InexactSolutionWarning` when this happens; run
`compat="original"` with `backend="gurobi"` when exact agreement matters. `compat="fixed"` tests
`X > 0.5` and is unaffected. Which binaries come back inexact depends on the HiGHS version, and
so does the first check the broken plan fails:

`python3 docs/notes/repro/highs_inexact_original.py`

```text
scipy 1.17.1
original/highs: makespan 12, InexactSolutionWarning x1, plan: Path 2 is not valid at t = 0
fixed/highs: makespan 12, InexactSolutionWarning x0, plan: valid
original/gurobi: makespan 12, InexactSolutionWarning x0, plan: valid
```

`python3 docs/notes/repro/highs_inexact_original.py`

```text
scipy 1.15.3
original/highs: makespan 12, InexactSolutionWarning x1, plan: Goal location for agent 0 is incorrect in the path set.
fixed/highs: makespan 12, InexactSolutionWarning x0, plan: valid
original/gurobi: makespan 12, InexactSolutionWarning x0, plan: valid
```


## 4. builtin: limits

`backend="builtin"` (`mppflow/lp/builtin.py`) is a dense bounded-variable simplex with depth-first
branch and bound, in pure Python (the same algorithm, line by line, as the browser demo's
`docs/js/lp.js`). It needs no dependency, but:

* it is fast enough only for small models (the 3x3–5x5 examples above; a 6x6 grid with 12 robots
  already hits a 20 s limit that HiGHS needs a fraction of a second for);
* the time limit is checked between nodes and before every simplex pivot, so a solve overruns it
  by at most one pivot (a dense pivot costs `O(rows × columns)` Python operations);
  `params={"node_limit": ..., "work_limit": ...}` give deterministic budgets;
* its gap-based pruning is valid but looser than Gurobi's, and it can dive for a long time on
  integer columns with an infinite upper bound (MPP models have none);
* depth-first branch and bound proves an infeasible makespan horizon by enumeration, and the
  work grows exponentially with `T` even when the joint state space is tiny: one piece of a
  2-way split of a tree with 8 vertices and 4 robots (1680 configurations) fails at `T = 2`
  and has a plan only at `T = 6`; proving `T = 2, 3, 4, 5` infeasible took 1, 19, 353 and 2971
  nodes (12 s at `T = 5`), and finding the plan at `T = 6` took 25180 nodes (260 s in Python,
  about 25 s in Node). The joint-state breadth-first search that the makespan loop already runs
  after a failed horizon (the infeasibility check, within `feasibility_budget`) finds the
  smallest feasible horizon and a plan there; with `backend="builtin"` the loop returns that
  plan (`extra["proof"] = "joint-state search"`, and an attempt with phase `"joint"` at its
  horizon, which `plan()` lists in `stats["horizons"]` / `stats["phases"]` but does not count
  in `stats["solves"]`) instead of solving the horizons up to it. The
  makespan model is a feasibility problem, so the plan is as good as the ILP's. HiGHS and
  Gurobi keep solving every horizon (a fraction of a second here), so their plans stay those of
  the ILP, which `compat="fixed"` shares with the original wherever the original is right;
* numerical safeguards. The MPP models are highly degenerate, and on models of a few hundred
  rows (3x3 grids, 500-1400 columns) the dense tableau used to lose all accuracy:
  entries reached 1e21. Noise then broke the ties of Bland's rule and was accepted as
  pivots. The solver either hit its iteration cap (`RuntimeError`) or returned "optimal" for a
  point around 1e14 that violated its own bounds, which the branch and bound accepted as an
  integer solution. Three changes fix this. Ratios equal up to `TIE_TOL` count as ties, and
  values within `ZERO_TOL` of a bound count as at the bound. The tableau is recomputed from the
  original rows every `REFRESH` (100) iterations. Every LP solution is checked against the
  model, and a violation raises `RuntimeError` instead of being returned. For example, the LP
  relaxation of the total-time model on a 3x3 grid with starts 8 2 0 7 6 1, goals
  8 7 2 5 0 4 and T = 6 (532 columns, 361 rows; the `tt532` case of
  `tests/test_oracles.py`) used to hit the iteration cap. It now takes 32635 iterations, as
  in exact rational arithmetic, and about a minute.

`python3 docs/notes/repro/builtin_tt532.py`

```text
columns 532, rows 361
highs:   optimal 23.333333
builtin: optimal 23.333333, 32635 iterations (57 s, varies)
```

* these safeguards stop the numerical failures, but not slow progress. Bland's rule can stall on a
  degenerate vertex for a very long time. The max-distance model at `T = 8` of an 11-vertex
  graph with 5 robots (638 columns, 404 rows; `min_max_distance(..., T=8, horizon="single")`)
  has a node LP whose phase 1 needs 268876 pivots, almost all of them degenerate (in the first
  144797, 6 were not). The bases were all distinct, so it did not cycle. The cap of
  `max(100000, 100 * (rows + columns))` iterations per simplex phase stopped it with
  `RuntimeError("iteration cap exceeded (numerical cycling?)")`, after 6-8 minutes in Python
  and 30-54 s in Node, although HiGHS solves the model at once (optimum 5). The cap is now
  `max(1000000, 1000 * (rows + columns))`, a guard only: the solve returns the optimum 5 (about
  9 minutes in Node, 2.3 hours in Python). A reached cap no longer raises; the solve stops as at a work limit, with
  status `"time_limit"` and `raw_status "iteration_cap"` (also in the solver's attempt record),
  so the horizon counts as undecided (`tests/test_lp.py::test_builtin_iteration_cap_is_a_limit`).
  Runs that stay under the old cap are unchanged. The default searches did not reach such a
  model in 40 random medium instances (the joint-state search answers first), but an explicit
  `T`, `horizon="single"` or a skipped joint-state stage can; use a time limit there. The
  browser demo's live cap (`docs/js/README.md`) looks only at the makespan model at the
  lower-bound horizon, so it does not exclude these models either.


## 5. Limits that decide nothing

A solve that stops at a limit proves nothing about its model: the horizon is neither feasible
nor infeasible. Earlier versions treated every result that was not a time limit, and had no
accepted plan, as a proof of infeasibility. Node limits are a documented option (`params={"node_limit":
100}` for HiGHS and `builtin`, `NodeLimit` for Gurobi), so this was reachable:

* `min_total_time(..., T=8, horizon="single", params={"node_limit": 0})` on a 5x5 grid with
  10% obstacles and 10 robots (`create_grid_problem(5, 5, 0.1, 10, MathRandom(1))`, optimal
  makespan 6; `tests/test_solver_limits.py`) raised `Infeasible("no plan with makespan <= T = 8")`,
  on every backend, and so did Gurobi's `SolutionLimit=1`, whose incumbent is outside the gap;
* the makespan loop moved on to `T + 1` after such a result: with Gurobi `NodeLimit=1`,
  `Heuristics=0`, `Cuts=0`, 14 of the seeds 1-59 of a 5x5 grid with 10% obstacles and 14 robots
  (`create_grid_problem(5, 5, 0.1, 14, MathRandom(seed))`) returned
  `"optimal"` with a makespan one too large, and with `NodeLimit=0` the loop never ended;
* the total-time search reported `"optimal"` although its solve at `T_hi` had ended at a node
  limit (a 3x4 grid without obstacles and 6 robots, `create_grid_problem(3, 4, 0.0, 6,
  MathRandom(35))`: 14 instead of 13 once the joint-state stage is skipped).

Now, in `compat="fixed"`, only a backend status `infeasible` (or `optimal` below `n` for a
makespan model) proves a horizon infeasible. Any other result without an accepted plan stops
the search with status `"time_limit"` (a limit stopped it), `extra["solver_status"]` holds the
backend's status for the makespan loop, and a plan already found is returned as `"feasible"`.
The same holds in the JS engine (`lp: {nodeLimit}`). `compat="original"` keeps the original's
behaviour.

The Gurobi backend mapped every status without a solution that is not a limit to
`infeasible`, including `INTERRUPTED`: gurobipy turns Ctrl-C into that status instead of an
exception, so an interrupted horizon counted as infeasible and the makespan search went on
(the 4x4 puzzle, seed 1, interrupted twice, returned `"optimal"` 8; the optimum is 7). It now
raises `KeyboardInterrupt` for `INTERRUPTED` and `RuntimeError` for any other stop without a
solution (`NUMERIC`, `MEM_LIMIT`, `CUTOFF`, ...), as the HiGHS backend does for a solver error.

`time_limit` is the total budget of a call. Model building did not count: on a 30x30 grid with
10% obstacles and 120 robots (seed 5), the first makespan model has 5 million columns and took
20 s to build in Python, and HiGHS then spent 10 s at a 1 s limit (much of it converting the
model and in presolve), so `min_makespan(..., time_limit=1.0)` returned `"time_limit"` after 30
to 60 s. The builders now poll the limit (every 4096 columns or rows) and stop; the backends get
what is left after the build, and HiGHS and Gurobi subtract their own model conversion. The
same call now returns after about 1.1 s. A backend can still overrun its own share, because
HiGHS does not check its limit inside presolve.

