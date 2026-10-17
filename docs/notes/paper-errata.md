# Errata for our papers

This note lists the errors we found in our three papers on multi-robot path planning via
network flow, while porting the original Java code to `mppflow`:

* **[TRO16]** J. Yu and S. M. LaValle, "Optimal Multirobot Path Planning on Graphs: Complete
  Algorithms and Effective Heuristics," *IEEE Transactions on Robotics* 32(5):1163–1177, 2016.
* **[ICRA13]** J. Yu and S. M. LaValle, "Planning Optimal Paths for Multiple Robots on Graphs,"
  *IEEE ICRA 2013*, pp. 3612–3617.
* **[STAR13]** J. Yu and S. M. LaValle, "Multi-agent Path Planning and Network Flow,"
  *Algorithmic Foundations of Robotics X* (WAFR 2012), STAR vol. 86, pp. 157–173, 2013.

Page numbers are journal/proceedings pages. None of these errors affects `mppflow`, which
implements the corrected form in every case. Only E11 (no bound on T) also affects the
original Java code (<https://github.com/arc-l/optimal-mrppg-journal>), as bug
[B1](original-bugs.md#b-infeasible). No other entry affects the original, and E13-E15
concern statements of [STAR13] that neither code implements. Code-side issues are in
[original-bugs.md](original-bugs.md), and differences between the papers' experiments and the code are in
[paper-vs-code.md](paper-vs-code.md).

Every entry has a reproduction script in [`repro/`](repro/). The scripts run from a checkout
of this repository (no installation needed; they use HiGHS through scipy, or the pure-Python
`builtin` backend where a script prints a plan, since HiGHS versions can return different, equally
valid optimal plans), and the output shown below is what they print.

| id | paper | item | severity |
|---|---|---|---|
| [E1](#e1) | TRO16 | eq. (15) is off by one; minimizing it can return a non-optimal plan | error in a formula |
| [E2](#e2) | TRO16, ICRA13 | distance objectives (12), (13), (16) / (7) are constant with the stated arc costs | error in a formula |
| [E3](#e3) | TRO16 | "x_{i,j} = 0 for j < n" after eq. (12) | typo |
| [E4](#e4) | TRO16, ICRA13, STAR13 | Theorem 1 (Thm. 2, Thm. 3): the flow-solution map is not a bijection | error in a proof, results unaffected |
| [E5](#e5) | TRO16, ICRA13 | Fig. 5 draws loopback e2 from the wrong row | figure |
| [E6](#e6) | TRO16 | Fig. 8: the split example needs 3 + 2 steps, not 2 + 2 | error in an example |
| [E7](#e7) | TRO16 | collision definition binds k but uses t | typo |
| [E8](#e8) | TRO16 | eq. (5) binds i twice | typo |
| [E9](#e9) | TRO16, ICRA13 | 16-puzzle branching factor is 950, not "over 1000" | wrong number |
| [E10](#e10) | TRO16 | Algorithm 1 keeps looping after MINMAKESPAN is solved | inefficiency |
| [E11](#e11) | ICRA13 | MinMakespan has no upper bound on T (Prop. 3) | incompleteness |
| [E12](#e12) | ICRA13 | the diagonal gadget of Fig. 6 admits head-on swaps | error in a construction |
| [E13](#e13) | STAR13 | Theorem 8: "necessary and sufficient" should be "sufficient, and tight in the worst case" | error in a statement |
| [E14](#e14) | STAR13 | planar graphs do not have ell = O(V^{1/2}) | error in a remark |
| [E15](#e15) | STAR13 | "unit edges and no acute angles" does not make a graph a CUG | error in a remark |
| [E16](#e16) | TRO16 | Sec. VI-B writes n (robots) for N (puzzle side) | notation |

---

<a id="e1"></a>
## E1. [TRO16] eq. (15) is off by one

**Paper** (Sec. IV-C, eqs. (14)–(15), p. 1168). The blue arcs `(v(t), v(t)')` exist for
`t = 1..T`, and `x_i^t` is robot `r_i`'s blue arc at its goal at time `t`. Eq. (14) sets
`y_i^T = x_i^T` and `y_i^t = x_i^t ∧ y_i^{t+1}`; eq. (15) minimizes `nT − Σ_{i, 1≤t≤T} y_i^t`,
and the text says that `T − Σ_t y_i^t` is the time robot `r_i` spends before it stays at its goal.

**What is wrong.** For a robot whose final arrival time is `t_f ≥ 1`, `y_i^t = 1` exactly for
`t = t_f..T`, so `Σ_{t=1}^{T} y_i^t = T − t_f + 1` and `T − Σ y = t_f − 1`, not `t_f`. For
`t_f = 0` the term is 0. Hence eq. (15) equals `Σ_i t_f^i − |{i : t_f^i ≥ 1}|`. The subtracted
count is a constant for robots that start away from their goals, but a robot that starts at
its goal earns a one-step discount when it leaves and comes back, so the minimizer of (15)
can be a plan whose total arrival time is not minimal.

**Correction.** Index `y` over `t = 0..T−1` and tie `y_i^t` to "at the goal during `[t, t+1]`"
(the goal's wait arc `g_i(t) → g_i(t+1)`), with `y_i^{T−1}` equal to that arc and
`y_i^t = (wait arc at t) ∧ y_i^{t+1}`. Then `T − Σ_{t=0}^{T−1} y_i^t = t_f^i` exactly. This is what
the original Java (`MultiagentGraphSolverGurobiTotalTime.prepareModel`) and `mppflow`
(`build_total_time_model`, and `build_full_network_model(objective="total_time")`) implement.

**Impact.** Only the paper's formula. The code has always computed the exact total arrival
time. A literal implementation of (15) would return non-optimal plans whenever a robot starts
at its goal and has to move out of the way.

**Reproduction.** The script builds the full network of Sec. III, adds eq. (14) literally and
minimizes eq. (15) on a small "trap": robots 1 and 2 sit at their goals in two pockets of the
shortcut 0-1-2-3, robot 0 goes 0 → 3, and a 7-step detour avoids the pockets. A second instance
(a 3x4 grid without vertex 6, starts [1, 7, 8, 3, 10], goals [11, 1, 8, 3, 10], T = 8, true
optimum 21, every minimizer of (15) has 22) is in `tests/test_solvers.py`.

`python3 docs/notes/repro/eq15_off_by_one.py`

```text
minimize eq. (15):  eq15 = 5  arrivals = [3, 2, 3]  true total time = 8
exact total time:   value = 7  arrivals = [7, 0, 0]
single robot, arrival 3 -> T - sum y = 2
```


<a id="e2"></a>
## E2. [TRO16, ICRA13] the distance objectives are constant with the stated costs

**Paper.** [TRO16] Sec. III (p. 1167): "For all green arcs, we assign them unit capacity and
cost"; the middle arc of each merge-split gadget also costs 1, all other arcs cost 0. Eqs. (12)
and (13) (MINMAXDIST, Sec. IV-B) and (16) (MINTOTALDIST, Sec. IV-D, p. 1168) sum `c_2(e_j) x_{i,j}`
with this single cost function. [ICRA13] Sec. III-B (p. 3614) and eq. (7) (p. 3615) are the same.

**What is wrong.** Every unit of flow crosses each of the `T` layers through exactly one
cost-1 arc: a green (wait) arc or a gadget, whose middle arc costs 1. So every feasible flow of
one robot costs exactly `T`: eq. (16) equals `nT` and the left side of (12) equals `T` for every
feasible solution, and the objectives cannot tell solutions apart.

**Correction.** Waiting is free for the distance objectives: green (holdover) arcs cost 0,
only the gadget middle arc (a move) costs 1. [STAR13] Sec. 5.1 (Objective 3, p. 169) says so
("setting the cost of the holdover edge ... to zero"), and the completeness argument of [TRO16]
Sec. IV-E (each step, some robot must move a distance of one) assumes it.

**Impact.** Only the papers. The original `MultiagentGraphSolverGurobiDistance` adds an
objective term only for `curV != nextV` (moves), as do `mppflow`'s models. In `mppflow`,
`FullNetwork.paper_cost` holds the literal costs and `FullNetwork.arc_cost` the corrected ones.

**Reproduction.** Fig. 4(a) instance (robot 1: s1+ → s1−, robot 2: s2+ → s2−), minimized and
maximized on the full network.

`python3 docs/notes/repro/distance_cost_constant.py`

```text
T=4: eq16 paper costs min=8 max=8 (nT=8) | free waits min=6 | eq13 paper costs min x_max=4 | free waits min x_max=3
T=5: eq16 paper costs min=10 max=10 (nT=10) | free waits min=6 | eq13 paper costs min x_max=5 | free waits min x_max=3
T=6: eq16 paper costs min=12 max=12 (nT=12) | free waits min=6 | eq13 paper costs min x_max=6 | free waits min x_max=3
T=8: eq16 paper costs min=16 max=16 (nT=16) | free waits min=6 | eq13 paper costs min x_max=8 | free waits min x_max=3
mppflow compact models: total distance 6 max distance 3
```


<a id="e3"></a>
## E3. [TRO16] "x_{i,j} = 0 for j < n" after eq. (12)

**Paper** (Sec. IV-B, p. 1168): "For a fixed i, the left side of (12) represents the distance
traveled by robot r_i. Note that x_{i,j} = 0 for j < n."

**What is wrong.** The loopbacks are `e_1..e_n`, `e_j` belonging to `r_j`. The constraint after
(8) sets `x_{i,j} = 0` for `i ≠ j`, but eq. (11) sets `x_{i,i} = 1`; e.g. `n = 2, i = j = 1` gives
`x_{1,1} = 1` with `1 < n`. The case `j = n` is also left out.

**Correction.** "Note that for `j ≤ n`, `x_{i,j} = 0` when `j ≠ i`, and `x_{i,i} = 1`." Since the
loopbacks cost 0, the restriction `j > n` in (12) and (16) is redundant anyway.

**Impact.** None (wording; the sums are unchanged).

`python3 docs/notes/repro/eq12_index.py`

```text
x_{1,1} in [1, 1]
x_{1,2} in [0, 0]
x_{2,1} in [0, 0]
x_{2,2} in [1, 1]
loopback costs: [0, 0] (paper costs: [0, 0] )
```


<a id="e4"></a>
## E4. [TRO16] Theorem 1 (also [ICRA13] Thm. 2, [STAR13] Thm. 3) is not a bijection

**Paper** ([TRO16] Sec. III, Theorem 1, p. 1167): there is a bijection between the solutions of
an MPP instance with horizon `T` and the integer maximum multiflows of value `n` on the
time-expanded network. The proof's injectivity part says the flow built from a path is unique.

**What is wrong.** A robot that waits at `u` during `[t, t+1]` can use the green arc
`u(t)' → u(t+1)`, or enter any merge-split gadget of an edge `{u, w}` at `u(t)'` and leave it
toward `u(t+1)`: nothing in the gadget (Fig. 4(b)) or the ILP forces it to exit at `w(t+1)`. Each
wait step therefore has `1 + deg(u)` flow realizations, all of which decode to the same path.
The map from flows to solutions is onto, and every solution has a flow, but it is not one-to-one.

**Correction.** State the theorem as "every integer multiflow of value `n` yields a solution,
and every solution yields such a flow" (a surjection from flows onto solutions); or forbid
same-side exits from the gadget.

**Impact.** The reduction remains correct for solutions: feasibility, the optimal makespan
and the optimal objective values are unchanged, since every flow decodes to a valid plan and
every plan has a flow. A gadget wait costs 1 (its middle arc), like a green arc, so the costs
of (12) and (16) are consistent too; with the corrected costs of E2, a gadget wait costs 1 while
a green wait costs 0, so a minimizer never uses one. The paper-literal ILP merely has more
equivalent optima. The compact model of [TRO16] Sec. V-A (Figs. 6–7), which the code
implements, has a single wait variable per robot, vertex and step, and no duplicates.

**Reproduction.** Enumerates every unit flow of one robot on the full network and decodes it.

`python3 docs/notes/repro/theorem1_not_bijection.py`

```text
edge u-w, stay at u, T=1: unit flows=2, distinct paths=1
edge u-w, move u->w, T=1: unit flows=1, distinct paths=1
Fig. 4(a), r1 s1+ -> s1-, T=3: unit flows=1, distinct paths=1
Fig. 4(a), r1 s1+ -> s1-, T=4: unit flows=12, distinct paths=4
Fig. 4(a), r1 s1+ -> s1-, T=5: unit flows=97, distinct paths=15
```


<a id="e5"></a>
## E5. [TRO16, ICRA13] Fig. 5 draws loopback e2 from the wrong row

**Paper.** Fig. 4(a) ([TRO16] p. 1166, [ICRA13] p. 3614): `s2−` is the pendant vertex on the
junction A (and `s1+` the pendant on B). Fig. 5 ([TRO16] p. 1167, [ICRA13] p. 3614) draws the
time-expanded network for `T = 2` with rows, top to bottom, `s1−, s2−, A, s1+, B, s2+` (the
pendant rows are offset and connect to one other row only).

**What is wrong.** Loopback `e2` (and, in [TRO16], `r2`'s bold blue goal arcs) leaves the row
of A, a degree-3 vertex, instead of `s2−(2)'` on the pendant row; Fig. 6 ([TRO16] p. 1169) repeats
this placement. `e1` is drawn correctly. Separately, the drawn instance has no solution with
`T = 2` (its optimal makespan is 4); the figure only illustrates the construction.

**Correction.** Draw `e2` (and `r2`'s goal arcs) from the second row (`s2−`).

**Impact.** None (figure only; the text, `S− = {v(T)'}`, is correct).

`python3 docs/notes/repro/fig5_loopback_row.py`

```text
degrees (without the wait loop): {'s1-': 1, 's2-': 1, 'A': 3, 's1+': 1, 'B': 3, 's2+': 1}
loopback e1: s1-(2)' -> s1+(0)
loopback e2: s2-(2)' -> s2+(0)
maximum multiflow by T: {0: 0, 1: 0, 2: 0, 3: 1, 4: 2, 5: 2}
```


<a id="e6"></a>
## E6. [TRO16] Fig. 8: the split example needs 3 + 2 steps

**Paper** (Sec. V-B, Fig. 8, p. 1170): splitting the two-robot instance at the intermediate
goals (1,1) and (2,2) "yields two child instances both requiring a time expansion with two
steps each". The text calls the graph a "3 × 3 grid".

**What is wrong.** In the first child, robot 1 goes (3,1) → (1,1) and robot 2 goes
(2,0) → (2,2); each one's only 2-step path passes (2,1) at `t = 1`, a meet collision. The first
child needs 3 steps, so this split gives makespan 3 + 2 = 5 against the unsplit optimum 4. The
figure has 4 × 4 vertices (3 × 3 cells), while the 9-puzzle's "3 × 3 grid" counts vertices.

**Correction.** "... two child instances requiring 3 and 2 steps" (or choose intermediate goals
that give 2 + 2, e.g. robot 1 → (2,0)); "a 4 × 4 grid".

**Impact.** The illustration only. It is a nice example of the split losing optimality, which
the heuristic does not claim to preserve.

`python3 docs/notes/repro/fig8_split.py`

```text
intermediate goals chosen by the code: [5, 10] (paper: (1,1) = 5, (2,2) = 10)
child 1 optimal makespan: 3 [[13, 9, 5, 5], [8, 8, 9, 10]]
child 2 optimal makespan: 2
unsplit optimal makespan: 4
2-way split plan: makespan 5 intermediate goals [[5, 10]]
```


<a id="e7"></a>
## E7. [TRO16] the collision definition binds k but uses t

**Paper** (Sec. II-A, p. 1165): two paths `p_i, p_j` collide "if there exists `k ∈ Z+` such that
`p_i(t) = p_j(t)` (meet collision) or `(p_i(t) = p_j(t+1)) ∧ (p_i(t+1) = p_j(t))` (head-on collision)".

**What is wrong / correction.** The bound variable is `k` and the body uses the free `t`; it should
read "there exists `t ∈ Z+`". [ICRA13] and [STAR13] use `k` consistently.

**Impact.** None; the intended definition is unambiguous and is what every model and validator
uses.

`python3 docs/notes/repro/collision_definition.py`

```text
Fig. 2(a) follow: valid
Fig. 2(b) head-on: Path 0 and 1 cross the same edge at time step 1
Fig. 2(c) meet: Path 0 and 1 meet on vertex 1 at time step 1
rotation on a 4-cycle: valid
```


<a id="e8"></a>
## E8. [TRO16] eq. (5) binds i twice

**Paper** (Sec. II-C, eq. (5), p. 1166): `∀i ∈ C, ∀e ∈ E, Σ_i f_i(e) ≤ c_1(e)`.

**What is wrong / correction.** The outer `∀i ∈ C` re-binds the index of the sum. The intended
constraint (it generalizes the edge capacity (1)) is the joint capacity
`∀e ∈ E, Σ_{i∈C} f_i(e) ≤ c_1(e)`, i.e. eq. (8) of the ILP. Read as a per-commodity bound it would
let robots share vertices and swap.

**Impact.** None; the code implements the joint form.

`python3 docs/notes/repro/eq5_joint_capacity.py`

```text
joint capacity rows (eq. 8): sum over both robots per arc, e.g. [(0, 1.0), (66, 1.0)]
max multiflow with joint capacity: 1
max multiflow with per-commodity bounds only: 2
```


<a id="e9"></a>
## E9. [TRO16, ICRA13] the 16-puzzle branching factor is 950

**Paper.** [TRO16] Sec. VI-B (p. 1173): bidirectional search on the 16-puzzle "must reach a depth
of 3, which requires storing over 5 × 10^8 states (the branching factor is over 1000)".
[ICRA13] Sec. VI-A (p. 3616) has the same sentence ("over 1000 per step").

**What is wrong.** On a fully occupied grid a joint move is a permutation `σ` with `σ(v) ∈ N[v]`
and no 2-cycles. A 4 × 4 grid has exactly 951 of them, 950 without the all-stay move; "over 1000"
holds only if head-on swaps were allowed (32000), which the papers forbid. The depth-3 figure
counts sequences: 951³ ≈ 8.6 × 10^8.

**Correction.** "the branching factor is about 950". The 36-puzzle statement ("well over 10^6") is
correct (41,921,394, counted by the row-profile dynamic program of the script below).

**Impact.** None; the order of magnitude and the memory argument stand.

`python3 docs/notes/repro/npuzzle_branching.py`

```text
2x2: joint moves incl. stay = 3, non-trivial = 2, with head-on swaps allowed = 9
3x3: joint moves incl. stay = 27, non-trivial = 26, with head-on swaps allowed = 229
4x4: joint moves incl. stay = 951, non-trivial = 950, with head-on swaps allowed = 32000
5x5: joint moves incl. stay = 107151, non-trivial = 107150
6x6 (row-profile DP): joint moves incl. stay = 41921395, non-trivial = 41921394
```


<a id="e10"></a>
## E10. [TRO16] Algorithm 1 keeps looping after MINMAKESPAN is solved

**Paper** (Sec. IV-E, Algorithm 1 MPP-ILP-OPTIMIZATION, p. 1168): "for T from T_min to T_max do
... P ← best solution found so far ... end; return P".

**What is wrong.** Nothing is incorrect, but the loop has no early exit. For MINMAKESPAN the
first feasible `T` is optimal; for MINTOTALTIME, MINTOTALDIST and MINMAXDIST the optimum is
non-increasing in `T` (a plan for `T` extends to `T + 1` by waiting at the goals), so a single
solve at `T_max = nT_opt` gives the optimum and the smaller horizons only add work.

**Correction.** MINMAKESPAN: stop at the first feasible `T`. Other objectives: solve at `T_max`
(or search upwards and stop once the objective reaches its lower bound).

**Impact.** Efficiency only. The original code never loops over `T` for these objectives (see
[paper-vs-code.md](paper-vs-code.md)). `mppflow`'s `horizon="search"` solves at the optimal
makespan `T0` and stops there if the value meets the lower bound. Otherwise it proves
optimality over all horizons with an exact joint-state search (instances with at most 10^6
configurations), or else by one more solve at a horizon `T_hi` that is provably large enough
and derived from the `T0` plan (for total time, `TT(T0) - Σd + max d`), capped by
`T_max = nT_opt` (see [DESIGN.md](../DESIGN.md) §4). The monotonicity needs a wait loop at every
goal, as in the paper's model; on a graph where a goal has none, `mppflow` solves every horizon
from `T_opt` to `T_hi` instead.

`python3 docs/notes/repro/alg1_monotone.py`

```text
T_opt = 3  T_max = 9
T=3: total distance 9, total time 9, max distance 3
T=4: total distance 9, total time 9, max distance 3
T=5: total distance 7, total time 9, max distance 3
T=6: total distance 7, total time 9, max distance 3
T=7: total distance 7, total time 9, max distance 3
T=8: total distance 7, total time 9, max distance 3
T=9: total distance 7, total time 9, max distance 3
```


<a id="e11"></a>
## E11. [ICRA13] MinMakespan has no upper bound on T

**Paper** (Sec. IV-A, Prop. 3, p. 3615): "If the model is not feasible, we increase T and try
again ... The algorithm is complete: since the problem is discrete ... for some sufficiently
large T, there must either be a feasible solution or we can pronounce that none can exist."

**What is wrong.** The finiteness argument shows that a bound exists, but the algorithm never
states one, so on an infeasible instance it never terminates. [TRO16] Sec. IV-E fixes this: it
checks feasibility first (Yu and Rus) and takes `T_max = O(|V|³)` from the same work.

**Correction.** Check feasibility first, or stop at an explicit `T_max` (for example the number of
joint configurations, `|V|!/(|V|−n)!`).

**Impact.** The original Java follows the [ICRA13] loop and never returns on infeasible
instances without a time limit ([original-bugs.md](original-bugs.md#b-infeasible)). `mppflow`
raises `Infeasible` (`compat="fixed"`) or `OriginalWouldHang` (`compat="original"`).

`python3 docs/notes/repro/minmakespan_no_tmax.py`

```text
original: OriginalWouldHang - MultiagentGraphSolverGurobiTime.solve: the instance is infeasible for every T (proved by joint-state search); the original increases T forever
fixed:    Infeasible - the instance has no solution for any horizon (joint-state search)
fixed, 5x5 grid + separate edge: Infeasible - the instance has no solution for any horizon (joint-state search)
fixed, corridor: Infeasible - the instance has no solution for any horizon (joint-state search)
```


<a id="e12"></a>
## E12. [ICRA13] the diagonal gadget of Fig. 6 admits head-on swaps

**Paper** (Sec. III-C, p. 3614; Fig. 6, p. 3615): for grids whose cells allow one diagonal crossing, all
four corners `v_k(t)'` of a cell feed one unit-capacity middle arc that splits to all four
corners `v_l(t+1)`; the extra paths are said to be harmless because "no two robots can go
through a single vertex at the same time step".

**What is wrong.** The gadget also connects corners that share a cell edge. Robot A can go
`v0 → v1` through the diagonal gadget while robot B goes `v1 → v0` through the ordinary edge
gadget: a head-on swap, which Problem 1 forbids; no arc or vertex is used twice, so the
capacities do not stop it.

**Correction.** Restrict the gadget to the two diagonals, e.g. two Fig. 4(b) gadgets (one per
diagonal) plus a side constraint that their middle arcs carry at most one unit in total.

**Impact.** Only this optional extension of [ICRA13]. [TRO16] and the code do not use the
gadget (the code's 8-connected grids add diagonals as ordinary edges).

`python3 docs/notes/repro/icra_fig6_gadget.py`

```text
swap v0 <-> v1 (cell edge), without the gadget: 1
swap v0 <-> v1 (cell edge), with the gadget:    2
crossing diagonals v0 -> v3, v1 -> v2, with the gadget: 1
```


<a id="e13"></a>
## E13. [STAR13] Theorem 8 is sufficient, not necessary

**Paper** (Sec. 4, Theorem 8, p. 167): for the unlabeled problem, a time-expanded network with
`T = n + ℓ − 1` steps (`ℓ` the largest start-goal distance) is "necessary and sufficient" for a
feasible solution to exist.

**What is wrong.** The bound is sufficient for every instance and tight in the worst case (some
instances need exactly `n + ℓ − 1`), but it is not necessary for a given instance: on the path
0-1-2-3-4 with starts {0, 4} and goals {1, 3}, `n + ℓ − 1 = 4` while one step suffices. The
conclusion (p. 170, "a tight bound on the number of time steps necessary and sufficient")
shows that worst-case tightness was meant.

**Correction.** "... is sufficient for a feasible solution to exist, and the bound is tight."

**Impact.** None on the results built on it (Cor. 9, 12 and 13 use sufficiency only) or on the
code, which does not use this bound.

`python3 docs/notes/repro/star_thm8.py`

```text
path 0-1-2-3-4, S={0,4}, G={1,3}: ell=3, n+ell-1=4, optimal makespan=1
STAR Fig. 3 / T-RO Fig. 4(a): ell=3, n+ell-1=4, optimal makespan=4
```


<a id="e14"></a>
## E14. [STAR13] planar graphs do not have ell = O(V^{1/2})

**Paper** (Sec. 4, remark after Thm. 10, p. 168): "If G is a planar graph, we have E ~ O(V) and
ℓ ~ O(V^{1/2})", giving `O(n²V + nV^{3/2})`.

**What is wrong.** Planarity bounds `E`, not the diameter: the path `P_V` is planar with
`ℓ = V − 1`, and its optimal makespan is `V − n`. The `O(V^{1/2})` diameter holds for
square-grid-like graphs, not for planar graphs in general. (The "−2" inside the
`MF(n, V(n + V^{1/2} − 2), ...)` expression does not follow from `T = n + ℓ − 1` either; it is a
constant.)

**Correction.** For planar graphs, `O(nV(n + ℓ)) ⊆ O(n²V + nV²)`; `O(n²V + nV^{3/2})` for graphs of
diameter `O(V^{1/2})`, such as grids.

**Impact.** None on Thm. 10 (strongly polynomial, `O(nVE)`) or on the code.

`python3 docs/notes/repro/star_planar_ell.py`

```text
V=9 n=1: ell=8 (sqrt(V)=3), n+ell-1=8, optimal makespan=8
V=16 n=2: ell=15 (sqrt(V)=4), n+ell-1=16, optimal makespan=14
V=36 n=4: ell=35 (sqrt(V)=6), n+ell-1=38, optimal makespan=32
V=64 n=4: ell=63 (sqrt(V)=8), n+ell-1=66, optimal makespan=60
```


<a id="e15"></a>
## E15. [STAR13] "unit edges and no acute angles" does not make a graph a CUG

**Paper** (Sec. 2, pp. 160–161): a collision-free unit-distance graph (CUG) has unit edges, and
for any two edges `(u1, v1), (u2, v2)` with `u1 ≠ u2`, `v1 ≠ v2`, discs of radius `< √2/4`
traversing them at the same time do not collide; "one can easily verify that any graph with
unit edge length and no acute angles between adjacent edges is a CUG".

**What is wrong.** The condition concerns all pairs of edges, while the angle condition only
constrains adjacent ones. A polygonal spiral with unit edges and only obtuse angles (about
170°) has successive turns 0.3 apart, so discs of any radius in (0.15, √2/4) collide.

**Correction.** Add that non-adjacent edges (and vertices) are at least √2/2 apart, or state the
claim only for subgraphs of the 2D grid (which are CUGs: non-adjacent grid edges are at least
1 apart).

**Impact.** None on the results, which only use grids, or on the code.

`python3 docs/notes/repro/star_cug_spiral.py`

```text
73 vertices, 72 edges, edge length 1.000000..1.000000
angle between adjacent edges: 169.4..170.4 degrees (none acute)
closest approach of two discs on qualifying edges: 0.300 at edges (13, 14, 51, 50)
required: >= 2r, i.e. any r in (0.150, sqrt(2)/4 = 0.354) collides
```


<a id="e16"></a>
## E16. [TRO16] Sec. VI-B writes n for N

**Paper** (Sec. VI-B, p. 1173): "minimum makespan solutions to the N²-puzzle for n = 3, 4, 5, and 6",
"always solvable for n ≥ 3", "instances for n = 3, 4, 5".

**What is wrong / correction.** `n` is the number of robots (Sec. II); the side length `N` is meant.
The N²-puzzle has `n = N²` robots.

**Impact.** None (notation).

`python3 docs/notes/repro/notation_n_N.py`

```text
N=3: 9 vertices, n=9 robots
N=4: 16 vertices, n=16 robots
N=5: 25 vertices, n=25 robots
N=6: 36 vertices, n=36 robots
```


---

## Checked, and not errors

These came up while we verified the papers and turned out to be correct, or only a matter of
wording:

* **[STAR13] abstract and intro: makespan at most `n + V − 1`.** True; since `ℓ ≤ V − 1`, the
  slightly tighter `n + V − 2` (more precisely `n + ℓ − 1`) also holds.
* **[TRO16] Sec. IV-E, MINTOTALDIST: "the optimal solution for MINTOTALTIME will not require more than
  nT_opt total number of moves".** True as written (moves ≤ total time ≤ `nT_opt`); citing the
  MINMAKESPAN plan would be more direct. The bound `T_max = nT_opt` is correct.
* **[TRO16] Sec. V-B remark** mixes "total distance" and "total arrival time" in one argument; the
  argument holds for both.
* **[TRO16] footnote 4** ("radii of no more than √2/4") versus [STAR13] ("less than √2/4"): at
  exactly √2/4 two discs on an L-shaped following move touch at one instant; which is right
  depends on whether contact counts as a collision.
* **[STAR13] requires disjoint start and goal sets**; [ICRA13], [TRO16] and the code do not. A
  brute-force check found no instance violating the Theorem 8 bound when they overlap.
* **[TRO16] Sec. VI-C, "8 × 8 grids, 16 × 16 grids, and 32 × 32 grids with 20% obstacles"**: the
  20% applies to the 32 × 32 grids only (see [paper-vs-code.md](paper-vs-code.md)).
* **N²-puzzle timings** differ between [ICRA13] and [TRO16] because the experiments were re-run on
  different hardware (Intel Q6600 vs. Core i7-4850HQ).
