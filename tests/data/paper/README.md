# Worked examples from the papers

These are the data files for `tests/test_paper_examples.py`. Each file is one worked example
(or a small group of them) taken from the papers behind `mppflow`, together with the values
that were verified for it.

## Papers

* **TOR 2016** (cited as `YuLav16TOR` in the files): J. Yu, S. M. LaValle, "Optimal Multirobot
  Path Planning on Graphs: Complete Algorithms and Effective Heuristics", *IEEE Transactions
  on Robotics* 32(5):1163-1177, 2016. Page numbers refer to the journal (pp. 1163-1177), and
  "PDF p." refers to the page of the 15-page article.
* **ICRA 2013**: J. Yu, S. M. LaValle, "Planning Optimal Paths for Multiple Robots on Graphs",
  *IEEE ICRA 2013*.
* **STAR 2013**: J. Yu, S. M. LaValle, "Multi-agent Path Planning and Network Flow",
  *Algorithmic Foundations of Robotics X* (WAFR 2012), Springer STAR vol. 86, 2013.

## Conventions

* A grid vertex id is `row * cols + col`, with row 0 at the **top** of the figure and
  column 0 at the left. This is the figure read row by row, and it is also the id that
  `mppflow.graph.create_2d_grid_graph(rows, cols)` assigns. Grid files also store the visual
  layout (`visual_layout_*`, rows from top to bottom, holding robot labels).
* Robot index `i` is the paper's label `i + 1`. `starts[i]` and `goals[i]` are robot `i`'s
  vertices. In the N^2-puzzles the goal is row-major, so robot `i`'s goal is vertex `i`.
* Plans are `paths[i][t]` (or lists of configurations `cfg[t][i]`). A step drawn as arrows
  is stored as `(from, to)` pairs: the robot on `from` moves to `to`.
* Grids with removed vertices (`removed`) keep the row-major ids in the file. The tests
  relabel the remaining vertices to `0..n-1` in increasing order.
* Motion model (TOR Sec. II-A): in each step a robot stays or moves along one edge. Two
  robots may not end a step on the same vertex (a meet) or swap along an edge (head-on).
  Following a robot into the vertex it is leaving is allowed, and so is rotating along a
  fully occupied cycle of length 3 or more.

## How the data was obtained

1. **Transcription.** Every page of each paper was read, and each figure was rendered at
   300-800 dpi. Instances were read off the figures. Fig. 3 and Fig. 15 of TOR 2016 were read
   arrow by arrow: each arrow sits at an edge midpoint and points the way the robot at its
   tail moves. The 134 arrows of Fig. 15 were read from an 800-dpi render, and the arrow
   counts per panel are stored as well (20, 22, 16, 16, 22, 16, 22).
2. **Independent verification.** Every value was recomputed by programs written from the
   papers' definitions only. Neither the original Java code nor `mppflow` was used for
   this. The programs were:
   * a plan validator for the motion model above;
   * exhaustive joint-state breadth-first search (all 9! = 362,880 states of the 9-puzzle,
     and smaller instances elsewhere);
   * exact dynamic programming or lexicographic Dijkstra for total time and distance on
     small instances;
   * independent ILPs: the paper's full Sec. III network (eqs. (8)-(10)) and the compact
     Sec. V-A model (eqs. (17)-(18)), with the objectives of eq. (15) read literally, true
     total time, and total distance;
   * max-flow computations for the STAR network examples;
   * counts of full-occupancy joint moves, by brute force and by a transfer-matrix count.

   A transcription counts as confirmed when its plan is valid and ends exactly at the goal.
   For Fig. 3 and Fig. 15 this is strong evidence that the arrows were read correctly.
3. **The original code.** `tor16_java_original_outputs.json` records what the original Java
   implementation (github.com/arc-l/optimal-mrppg-journal, run with JDK 17 and Gurobi 13.0.2)
   returns on the same instances: makespans, model sizes and split goals, the original's
   9-puzzle and 25-puzzle plans (with their rotation counts and distances), and the seed-1
   Fig. 9 instance from `Problem.createGridProblem`. Plans depend on the Gurobi version and
   the thread count, so the 9-puzzle plan is compared exactly only under Gurobi.

`tests/test_paper_examples.py` repeats an independent check for each example (its own
joint-state BFS, DP, validator, max flow and a small ILP solved with scipy) next to the
`mppflow` check. Workspace-only fields (the names of the verification scripts, timings)
were removed when the files were copied here.

## Files

| file | paper, location | content |
|---|---|---|
| `tor16_fig1_fig3_9puzzle.json` | TOR p.1165 (PDF p.3), Fig. 1, Fig. 3, Sec. II-B | 9-puzzle instance. Optimal makespan 4, and all 4 optimal plans (Fig. 3 is index 3). Fig. 3 transcribed as 4 rotations. Full-network max flow 8 at T=3 and 9 at T=4. The original's `getA9Puzzle` is a different instance (optimum 5). |
| `icra_9puzzle_fig2_fig3_fig7.json` | ICRA Fig. 2, 3, 7 | The same 9-puzzle as directed cycles, plus Fig. 7 (three rotations on a 2x3 grid exchange robots 8 and 9). |
| `tor16_fig2_moves.json` | TOR p.1165, Fig. 2, Sec. II-A | Fig. 2 (a) follow is valid, (b) head-on swap is invalid, (c) meet is invalid, plus rotation, chain and blocked-move cases. |
| `tor16_fig4_fig7_network.json` | TOR pp.1166-1167, 1169, Fig. 4-7 | The Fig. 4(a) tree. Sizes of the full (Fig. 5) and compact (Fig. 6/7) networks at T=2. Max flow for T=0..5 (the drawn T=2 network is infeasible; the optimum is 4). Eq. (16) with the paper's costs is the constant nT. Theorem 1 flow and path counts (12 flows, 4 paths). |
| `icra_fig4a_fig5_network.json` | ICRA Fig. 4(a)/5 (= STAR Fig. 3, TOR Fig. 4(a)/5) | The same network sizes. Minimum total time 7 and distance 6. The unlabeled optimum attains STAR's bound. |
| `icra_bijection_duplicate_flows.json` | ICRA Thm. 2 / STAR Thm. 3 / TOR Thm. 1 | One edge, T=1: two unit flows map to one stay path. |
| `tor16_fig8_split.json` | TOR p.1170 (PDF p.8), Fig. 8, Sec. V-B | 4x4 grid. The 2-way split's intermediate goals are [5, 10]. The children need 3 and 2 steps; the paper says 2 and 2 (see `docs/notes/paper-errata.md`). |
| `tor16_fig14_fig15_25puzzle.json` | TOR p.1173 (PDF p.11), Fig. 14, 15, Sec. VI-B | 25-puzzle instance and the transcribed 7-step plan. The plan is valid and matches the lower bound of 7 (robot 10), so it is optimal. |
| `tor16_bounds_objectives.json` | TOR Sec. IV-C (eqs. (14)-(15)), IV-E, VI-B | Eq. (15) is off by one. Strict counterexample: the true optimum is 21, and every eq. (15) minimiser has 22. Also a tie case and the value on the Fig. 3 plan. The garage corridor: makespan lower bound 8, optimal makespan T_opt = 10 and T_max = n T_opt = 20; the minimum total distance 18 and total time 19 are the same at every T in 10..20 (eq. (15) gives 17). Also the star graph and state-space sizes. |
| `tor16_puzzle_branching.json`, `icra_npuzzle_state_space.json` | TOR Sec. VI-B, ICRA Sec. VI-A | Legal joint moves of a full grid: 27 (3x3) and 951 (4x4; the paper says "over 1000"). State counts. |
| `tor16_16puzzle_random_check.json` | TOR Sec. VI-B ("generally 6 steps") | Seeded random 16-puzzles with independently computed optimal makespans (a sanity check, not a figure). |
| `icra_fig1_time_vs_distance.json` | ICRA Sec. II-A, Fig. 1 | Time-optimal plans: makespan 1.5t+2, distance 2.5t+4. Distance-optimal plans: makespan 2t+3, distance 2t+4. |
| `star_fig1_escape.json` | STAR Fig. 1 | Escape problem on a 6x6 grid: (a) all 10 escape; (b) 10 of 11. |
| `star_fig2_dynamic_flow.json` | STAR Fig. 2 | Arc counts of the time-expanded dynamic-flow network and its max flow by T. |
| `star_theorem8_bound.json` | STAR Thm. 8, Cor. 9, 12 | T = n + ell - 1 is sufficient and tight on a family of instances, but not necessary for a given instance. |
| `star_cug_radius.json` | STAR Sec. 2, TOR footnote 4 | The minimum centre distance on non-conflicting grid moves is sqrt(2)/2, so radius sqrt(2)/4. |
| `tor16_java_original_outputs.json` | (original code) | The original's makespans, model sizes by T on Fig. 4(a), split goals on Fig. 8, its Fig. 1 and Fig. 14 plans (4 and 19 rotations; distances 24 and 144), and the seed-1 Fig. 9 generator instance (`fig9_generator`). |
