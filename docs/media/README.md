# Media

Figures and animations used in the documentation, with the scripts that make them.

## Concepts figure (`concepts.png`, `gadget.png`)

`concepts.png` / `concepts.svg` illustrate the key ideas in four panels, each drawn from
real `mppflow` output:

* (a) the two-robot tree of T-RO 2016 Fig. 4(a), with starts shaded and goals unshaded;
* (b) its compact time-expanded network (`mppflow.network.CompactNetwork`, T = 4). The
  figure shows the arcs that reachability pruning removes, the arcs left in the ILP, the
  loopback arcs, and the optimal flow that `mppflow.min_makespan` finds (T = 3 is
  infeasible);
* (c) the same plan as one snapshot per time step;
* (d) the k-way split. On the Fig. 8 instance, the intermediate goals [5, 10] give a plan
  of 3 + 2 = 5 steps, against the unsplit optimum of 4 (erratum E6 in
  `docs/notes/paper-errata.md`). The two charts give the makespan and the solve time of
  k = 1, 2, 4, 8, 16 on the 24x18 demo map.

Robot 1 is teal and robot 2 is magenta, the colours the browser demo gives these two robots
(and the colours of `demo.gif`).

`gadget.png` / `gadget.svg` compare the 5-arc merge-split gadget of Fig. 4(b) with the
compact 2-arc gadget of Fig. 7.

Regenerate both figures from the repository root (matplotlib, a few seconds):

```sh
python3 docs/media/make_concepts.py               # default backend (HiGHS via SciPy)
python3 docs/media/make_concepts.py --backend gurobi
python3 docs/media/make_concepts.py --no-svg      # PNG only
```

The script prints the plans, model sizes and backend it used. The footer of the figure
records the `mppflow` version and the backend. Panels (a) to (c) and the Fig. 8 split are
solved on every run. The 24x18 makespans and solve times are not: they are read from
`docs/data/demo-data.js`, built with Gurobi 13.0.2, and the solve times are wall-clock
seconds of that build. The committed figures use HiGHS. Gurobi returns other optimal
plans for the Fig. 8 split (still 3 + 2 = 5 steps through [5, 10]), so the drawn paths change
with the backend. The SVG output is deterministic, so a rerun with the same library
versions writes the same file.

## Demo animation (`demo.gif`)

`demo.gif` (800 x 640, about 24 s, loops) is made of screenshots of the browser demo
(`docs/index.html`). Each robot, ring and label in it is drawn by the page from the
precomputed plans in `docs/data/demo-data.js`. It has three scenes:

1. The 9-puzzle of T-RO 2016 Fig. 1 and our optimal 4-step plan, which is the plan of Fig. 3.
   The goal configuration is shown on the right.
2. The two-robot tree of Fig. 4(a) above its compact time-expanded network (Figs. 6 and 7),
   which fills the width, with the network's legend. The network shows the plan's two unit
   flows (each loopback dashed in its robot's colour), and the time cursor, which the legend
   also lists, sweeps across it. Each robot is drawn in the colour of its flow
   (the page's "color robots" option).
3. The 24 x 18 grid with 50 robots. The exact plan (k = 1, makespan 29, 108 s) is on the left
   and the 4-way split plan (k = 4, makespan 35, 2.7 s) is on the right; the solve times are
   the Gurobi 13.0.2 wall-clock times of the demo-data build. Both are shown at the same time
   step, one frame per step. The brown dotted rings are the intermediate goals of the current
   piece.

Regenerate it from the repository root. You need Node 22 or later, Chrome or Chromium, and
Python with Pillow and NumPy. It takes about a minute.

```sh
frames="$(mktemp -d)"
node docs/media/make_demo_gif.mjs --out "$frames"   # headless Chrome: PNG frames + frames.json
python3 docs/media/make_demo_gif.py "$frames"       # writes docs/media/demo.gif
rm -r "$frames"
```

`make_demo_gif.mjs` has no dependencies. It reuses the Chrome DevTools Protocol helpers of
`tools/check_demo.mjs`. Set the browser with `--chrome PATH` or `CHROME`. The script opens
the page from `file://` at a 1280 x 900 viewport in the light theme. It selects each map and
plan through the page's URL hash (`#map=...&plan=...`) and sets the time with the page's own
time slider, so the frames are evenly spaced on any machine. Each clip is captured at 2x.
It changes the page for the capture only, and nothing in `docs/` is modified: it replaces each
plan's subtitle by a plain one written from `demo-data.js` (for example "4-way split: makespan
35, 2.7 s"), in the text colour; in the network scene it draws the network 600 CSS pixels wide
and stacks its legend, turns on "color robots", and draws the legend's loopback swatch in the two
robots' loopback colours (the page draws one red dashed line). The page's legend already lists
the time cursor, a translucent band.
The script exits with an error if the page logs an error, or if a plan does not end with
every robot on its goal. `--probe` writes only one full-page screenshot per scene, for
checking the layout.

`make_demo_gif.py` crops the page's own elements out of the frames (the heading, the graph
view, the time slider and its label, the goal configuration, and the network with its title and
legend). The play buttons are left out. It scales the crops and arranges them on the canvas.
Each graph view is cropped with one box per scene (the union of the drawn content over all
frames), so the tiles do not move. It draws no robots and no text. The only thing it draws is the rounded tile,
in the view's colour, behind each cropped graph view. The GIF uses one palette of 96 colours with no
dithering. The palette keeps the exact flat colours of every scene, so the page's colours
survive. Unchanged pixels are transparent in the delta frames. The script fails if the
file exceeds 1.8 MB (`--max-bytes`). Most of the size comes from the moving robots of
scene 3 (the 50-robot grid), so that scene uses one frame per time step. `PACE` in the `.mjs` and `--colors` in
the `.py` trade smoothness and colour against size. `--keep-png DIR` also writes the
composed frames as PNG files.
