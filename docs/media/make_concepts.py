"""Draw the key concepts of mppflow from real solver output.

    python3 docs/media/make_concepts.py            # from the repository root

writes ``docs/media/concepts.png`` / ``concepts.svg`` (four panels) and
``docs/media/gadget.png`` / ``gadget.svg`` (the two head-on gadgets). Every plan, network
and count in the figures is computed here with ``mppflow`` (default backend: HiGHS via
SciPy). The only numbers that are not recomputed are the k-way split timings of the 24x18
instance, which are read from ``docs/data/demo-data.js`` (built with Gurobi; a fresh solve
of the unsplit 24x18 model takes minutes).

Panels:

(a) the two-robot tree of T-RO 2016 Fig. 4(a): starts shaded, goals unshaded;
(b) its compact time-expanded network (Sec. V-A, Figs. 6-7) at the optimal horizon T = 4,
    with the arcs that survive reachability pruning and the optimal flow highlighted;
(c) the same plan as snapshots t = 0..4;
(d) the k-way split heuristic: the Fig. 8 instance split at intermediate goals [5, 10]
    (makespan 3 + 2 = 5 against the unsplit optimum 4), and the makespan / solve-time
    trade-off of k on the 24x18 demo instance.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
if ROOT not in sys.path:  # run from a checkout without installing the package
    sys.path.insert(0, ROOT)

import matplotlib  # noqa: E402
import matplotlib.ticker  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch  # noqa: E402

import mppflow  # noqa: E402
from mppflow import model as M  # noqa: E402
from mppflow import network as N  # noqa: E402

# ------------------------------------------------------------------ style
INK = "#1f2328"        # primary text
INK2 = "#59636e"       # secondary text
FAINT = "#d8dee4"      # unused network arcs, grid lines
MID = "#8c959f"        # pruned (model) arcs, graph edges
CARD = "#ffffff"
PANEL = "#f6f8fa"      # panel background
# robot 1 teal, robot 2 magenta: the browser demo's colours for these robots (docs/js/demo.js,
# light theme: flowColor = hsl(hue, 70%, L) for flows and outlines, L = 40% lowered per hue until the
# line has 3:1 contrast on the view (33% for this teal), robotFill = hsl(hue, 68%, 74%) for solid
# robot discs, hue = 190 + 137.508 i), so the figure and the demo GIF match
ROBOT = ["#197b8f", "#ad1f6c"]
ROBOT_FILL = ["#90dbea", "#ea90c0"]
MIDGOAL = "#b07a3c"    # intermediate goals (the paper's brown)
CHART = "#3d4651"      # single-series chart marks


def tint(hex_color: str, a: float) -> str:
    """Blend ``hex_color`` with white (``a`` = share of the color)."""
    c = [int(hex_color[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(255 - a * (255 - x)):02x}" for x in c)


plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 9.5,
    "text.color": INK,
    "axes.edgecolor": MID,
    "axes.labelcolor": INK2,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "svg.hashsalt": "mppflow-concepts",
    "svg.fonttype": "path",
})

# ------------------------------------------------------------------ instances
# T-RO 2016 Fig. 4(a): vertices s1-, s2-, A, s1+, B, s2+ (tests/data/paper/tor16_fig4_fig7_network.json)
TREE_ADJ = [[2], [2], [0, 1, 4], [4], [2, 3, 5], [4]]
TREE_XY = [(1, 0), (0, 1), (1, 1), (0, 2), (1, 2), (1, 3)]
TREE_NAMES = [r"$s_1^-$", r"$s_2^-$", r"$A$", r"$s_1^+$", r"$B$", r"$s_2^+$"]
TREE_STARTS, TREE_GOALS = [3, 5], [0, 1]

# T-RO 2016 Fig. 8: 4 x 4 grid, vertex = row * 4 + col (tests/data/paper/tor16_fig8_split.json)
FIG8_STARTS, FIG8_GOALS = [13, 8], [0, 7]


def backend_label(backend: str) -> str:
    if backend == "highs":
        import scipy
        return f"HiGHS (SciPy {scipy.__version__})"
    if backend == "gurobi":
        import gurobipy
        return "Gurobi " + ".".join(map(str, gurobipy.gurobi.version()))
    return "builtin simplex"


def compute(backend: str) -> dict:
    """Solve the instances; return everything the figures draw."""
    g = mppflow.from_adjacency(TREE_ADJ, xy=TREE_XY)
    r = mppflow.min_makespan(g, TREE_STARTS, TREE_GOALS, backend=backend)
    assert mppflow.validate(mppflow.Problem(g, TREE_STARTS, TREE_GOALS), r.paths) is None
    T = r.T
    net = N.CompactNetwork.build(g, TREE_STARTS, TREE_GOALS, T, prune=True)
    mm = M.build_time_model(g, TREE_STARTS, TREE_GOALS, T).model
    tree = dict(graph=g, paths=r.paths, T=T, lower_bound=r.lower_bound, net=net,
                attempts=[(a["T"], a["status"]) for a in r.attempts],
                columns=mm.num_columns, rows=mm.num_rows)

    G = mppflow.create_2d_grid_graph(4, 4, True)
    prob = mppflow.Problem(G, FIG8_STARTS, FIG8_GOALS)
    p1 = mppflow.plan(prob, backend=backend)
    p2 = mppflow.plan(prob, split=2, backend=backend, rng=mppflow.MathRandom(1))
    for p in (p1, p2):
        assert mppflow.validate(prob, p.paths) is None
    mids = p2.intermediate_goals[0]
    # the time the robots reach the intermediate goals (end of the first piece)
    t_split = next(t for t in range(len(p2.paths[0]))
                   if all(p2.paths[i][t] == mids[i] for i in range(2)))
    fig8 = dict(graph=G, unsplit=p1.makespan, split=p2.makespan, paths=p2.paths,
                unsplit_paths=p1.paths, mids=mids, t_split=t_split)
    return dict(tree=tree, fig8=fig8, backend=backend_label(backend))


def read_demo_k_tradeoff() -> dict:
    """Makespan and wall-clock seconds of the 24x18 makespan plans for k = 1..16."""
    path = os.path.join(ROOT, "docs", "data", "demo-data.js")
    with open(path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"root\.MPP_DEMO\s*=\s*(\{.*\});\s*\n\s*if \(typeof module", text, re.S)
    data = json.loads(m.group(1))
    mp = next(x for x in data["maps"] if x["id"] == "tor16-fig9-24x18-seed1")
    ks, makespan, secs, cols = [], [], [], []
    for k in (1, 2, 4, 8, 16):
        p = mp["plans"][f"makespan/k{k}"]
        ks.append(k)
        makespan.append(p["value"])
        secs.append(p["seconds"])
        cols.append(p["model"]["columns"])
    return dict(k=ks, makespan=makespan, seconds=secs, columns=cols,
                lower_bound=mp["lower_bounds"]["makespan"], robots=len(mp["starts"]),
                n=mp["graph"]["n"], backend=data.get("backend_version", data.get("backend")))


# ------------------------------------------------------------------ drawing helpers
def robot_disc(ax, x, y, i, r=0.2, kind="start", z=5, fs=9.5, lw=1.5):
    """``kind``: start (shaded), goal (white), robot (solid), mid (dashed, intermediate goal)."""
    col = ROBOT[i]
    if kind == "start":
        fc, ec, tc, ls = tint(col, 0.35), col, INK, "-"
    elif kind == "goal":
        fc, ec, tc, ls = "white", col, INK, "-"
    elif kind == "mid":
        fc, ec, tc, ls = tint(MIDGOAL, 0.35), MIDGOAL, INK, (0, (2, 1.5))
    else:
        fc, ec, tc, ls = ROBOT_FILL[i], col, INK, "-"
    ax.add_patch(Circle((x, y), r, fc=fc, ec=ec, lw=lw, ls=ls, zorder=z))
    ax.text(x, y, str(i + 1), ha="center", va="center", fontsize=fs, color=tc, zorder=z + 1,
            fontweight="bold" if kind == "robot" else "normal")


def panel_title(ax, letter, text, x=0.0, y=1.0):
    ax.annotate(f"({letter})", (x, y), xycoords="axes fraction", ha="left", va="bottom", fontsize=11,
                fontweight="bold", color=INK)
    ax.annotate(text, (x, y), xycoords="axes fraction", xytext=(21, 0), textcoords="offset points",
                ha="left", va="bottom", fontsize=11, color=INK)


def bare(ax):
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_facecolor("none")


def draw_tree(ax, g, scale=1.0, names=True, edge_lw=3.0):
    for u in range(6):
        for w in g.adj[u]:
            if u < w:
                (x1, y1), (x2, y2) = TREE_XY[u], TREE_XY[w]
                ax.plot([x1 * scale, x2 * scale], [y1 * scale, y2 * scale], color=MID, lw=edge_lw,
                        solid_capstyle="round", zorder=1)
    for v, (x, y) in enumerate(TREE_XY):
        ax.add_patch(Circle((x * scale, y * scale), 0.07 * scale, fc=MID, ec="none", zorder=2))


# ------------------------------------------------------------------ panels
def panel_instance(ax, d):
    tree = d["tree"]
    bare(ax)
    draw_tree(ax, tree["graph"])
    for v, (x, y) in enumerate(TREE_XY):
        dx = -0.3 if x == 0 else 0.3
        ax.text(x + dx, y, TREE_NAMES[v], ha="right" if x == 0 else "left", va="center",
                fontsize=11, color=INK2)
    for i, (s, gl) in enumerate(zip(TREE_STARTS, TREE_GOALS)):
        robot_disc(ax, *TREE_XY[s], i, r=0.17, kind="start")
        robot_disc(ax, *TREE_XY[gl], i, r=0.17, kind="goal")
    ax.set_xlim(-1.0, 2.0)
    ax.set_ylim(4.75, -0.4)
    ax.set_aspect("equal")
    panel_title(ax, "a", "An MPP instance")
    # legend: shaded = start, white = goal
    lx, ly = -0.75, 2.85
    ax.add_patch(Circle((lx, ly), 0.12, fc=tint(ROBOT[0], 0.35), ec=ROBOT[0], lw=1.2))
    ax.text(lx + 0.2, ly, "start", va="center", fontsize=9, color=INK2)
    ax.add_patch(Circle((lx, ly + 0.36), 0.12, fc="white", ec=ROBOT[0], lw=1.2))
    ax.text(lx + 0.2, ly + 0.36, "goal", va="center", fontsize=9, color=INK2)
    ax.text(-0.95, 4.7, f"T-RO Fig. 4(a). Both robots must pass\nA and B, so the lower bound "
            f"{tree['lower_bound']} cannot\nbe met: the optimal makespan is {tree['T']}.",
            fontsize=8.6, color=INK2, va="bottom", linespacing=1.35)


def _route(ax, pts, color, lw=1.3):
    xs, ys = zip(*pts)
    ax.plot(xs, ys, color=color, lw=lw, ls=(0, (4, 2.5)), zorder=3, solid_joinstyle="round",
            dash_joinstyle="round")
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>", mutation_scale=10,
                                 color=color, lw=lw, zorder=3, shrinkA=0, shrinkB=7))


def panel_network(ax, d):
    tree = d["tree"]
    net, T, paths = tree["net"], tree["T"], tree["paths"]
    bare(ax)
    DX = 1.55  # horizontal spacing of the time copies

    def pos(v, t):
        return (t * DX, v)

    pruned = set()
    for arcs in net.agent_arcs:
        for (_, u, w, t) in arcs:
            pruned.add((u, w, t))
    flow = {}
    for i, p in enumerate(paths):
        for t in range(T):
            flow[(p[t], p[t + 1], t)] = i
    for (u, w, t, kind) in net.arcs:
        if kind == "loop":
            continue
        key = (u, w, t)
        (x1, y1), (x2, y2) = pos(u, t), pos(w, t + 1)
        if key in flow:
            continue
        on = key in pruned
        ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>" if on else "-",
                                     mutation_scale=7, color=MID if on else FAINT,
                                     lw=1.05 if on else 0.9, shrinkA=5, shrinkB=5, zorder=1 if not on else 2))
    for key, i in flow.items():
        u, w, t = key
        ax.add_patch(FancyArrowPatch(pos(u, t), pos(w, t + 1), arrowstyle="-|>", mutation_scale=11,
                                     color=ROBOT[i], lw=2.6, shrinkA=5, shrinkB=5, zorder=4))
    # nodes
    on_flow = {(p[t], t): i for i, p in enumerate(paths) for t in range(T + 1)}
    for (v, t) in net.nodes:
        x, y = pos(v, t)
        i = on_flow.get((v, t))
        ax.scatter([x], [y], s=46 if i is not None else 34, color=ROBOT[i] if i is not None else "white",
                   edgecolor=ROBOT[i] if i is not None else MID, linewidth=1.0, zorder=5)
    # loopbacks g_i(T) -> s_i(0), routed around the network as in Fig. 5
    xr = T * DX
    _route(ax, [(xr + 0.1, 0), (xr + 0.45, 0), (xr + 0.45, -0.7), (-0.55, -0.7), (-0.55, 3), (-0.1, 3)],
           ROBOT[0])
    _route(ax, [(xr + 0.1, 1), (xr + 0.7, 1), (xr + 0.7, 5.7), (-0.8, 5.7), (-0.8, 5), (-0.1, 5)], ROBOT[1])
    ax.text(xr + 0.78, 3.3, "loopback\n" + r"$g_2(T)\to s_2(0)$", fontsize=8.3, color=INK2, va="center")
    ax.text(T * DX / 2, -0.82, "loopback " + r"$g_1(T)\to s_1(0)$", fontsize=8.3, color=INK2,
            ha="center", va="bottom")
    # labels
    for v in range(6):
        ax.text(-1.0, v, TREE_NAMES[v], ha="right", va="center", fontsize=10.5, color=INK2)
    for t in range(T + 1):
        ax.text(t * DX, 6.15, f"t = {t}", ha="center", va="center", fontsize=9, color=INK2)
    c = net.counts()
    ax.set_xlim(-1.75, xr + 2.45)
    ax.set_ylim(6.55, -1.25)
    panel_title(ax, "b", f"Time-expanded network (compact, T = {T}) and the optimal flow")
    # legend
    lx, ly = xr + 0.95, -0.2
    items = [(FAINT, 0.9, "arc pruned by\nreachability"), (MID, 1.05, "arc in the ILP"),
             (ROBOT[0], 2.6, "flow of robot 1"), (ROBOT[1], 2.6, "flow of robot 2")]
    for k, (col, lw, lab) in enumerate(items):
        y = ly + k * 0.62
        ax.plot([lx, lx + 0.4], [y, y], color=col, lw=lw, solid_capstyle="round")
        ax.text(lx + 0.52, y, lab, va="center", fontsize=8.3, color=INK2, linespacing=1.1)
    infeasible = [Tt for (Tt, s) in tree["attempts"] if s != "optimal"]
    note = (f"{c['vertices']} nodes, {c['arcs_total']} arcs + {c['loopback_arcs']} loopbacks; "
            f"after pruning {c['pruned_arcs_per_agent'][0]} + {c['pruned_arcs_per_agent'][1]} arcs.\n"
            f"ILP at T = {T}: {tree['columns']} columns, {tree['rows']} rows"
            + (f" (T = {', '.join(map(str, infeasible))} infeasible)." if infeasible else ".")
            + " Maximizing the loopback flow\nroutes every robot, and the capacity rows keep the "
              "flows collision-free.")
    ax.text(-1.7, 6.5, note, fontsize=8.6, color=INK2, va="top", linespacing=1.35)


def panel_snapshots(fig, spec, d):
    tree = d["tree"]
    paths, T = tree["paths"], tree["T"]
    sub = spec.subgridspec(1, T + 1, wspace=0.0)
    axes = [fig.add_subplot(sub[0, t]) for t in range(T + 1)]
    for t, ax in enumerate(axes):
        bare(ax)
        draw_tree(ax, tree["graph"], edge_lw=2.4)
        for i, gl in enumerate(TREE_GOALS):
            if paths[i][t] != gl:
                robot_disc(ax, *TREE_XY[gl], i, r=0.2, kind="goal", z=4, fs=8.5, lw=1.2)
        for i, p in enumerate(paths):
            if t > 0 and p[t] != p[t - 1]:
                # where the robot was at t - 1 (faded) and the move it made
                ax.add_patch(Circle(TREE_XY[p[t - 1]], 0.2, fc=tint(ROBOT[i], 0.22), ec="none", zorder=3))
                ax.add_patch(FancyArrowPatch(TREE_XY[p[t - 1]], TREE_XY[p[t]], arrowstyle="-|>",
                                             mutation_scale=12, color=ROBOT[i], lw=2.4, shrinkA=0,
                                             shrinkB=9, zorder=6))
            robot_disc(ax, *TREE_XY[p[t]], i, r=0.2, kind="robot", z=7, fs=8.5)
        ax.set_xlim(-1.3, 1.6)
        ax.set_ylim(3.75, -0.35)
        ax.set_aspect("equal")
        ax.text(0.5, 3.62, f"t = {t}", ha="center", va="center", fontsize=9.5, color=INK2)
        waits = [i + 1 for i, p in enumerate(paths) if t > 0 and p[t] == p[t - 1] and p[t] != TREE_GOALS[i]]
        if waits:
            ax.text(-0.55, 2.0, "robot " + ", ".join(map(str, waits)) + "\nwaits", ha="right",
                    va="center", fontsize=8.3, color=INK2, linespacing=1.15)
    panel_title(axes[0], "c", "The plan read off the flow, one snapshot per time step "
                "(faded: position at t - 1)", x=0.0, y=1.0)


def panel_split(ax, d):
    f8 = d["fig8"]
    bare(ax)
    for r in range(4):
        ax.plot([0, 3], [r, r], color=FAINT, lw=1.6, zorder=0, solid_capstyle="round")
        ax.plot([r, r], [0, 3], color=FAINT, lw=1.6, zorder=0, solid_capstyle="round")

    def xy(v):
        return (v % 4, v // 4)

    offs = [(-0.07, -0.07), (0.07, 0.07)]
    ts = f8["t_split"]
    for i, p in enumerate(f8["paths"]):
        ox, oy = offs[i]
        for (a, b, ls) in ((0, ts, "-"), (ts, len(p) - 1, (0, (3, 2)))):
            pts = []
            for v in p[a:b + 1]:
                if not pts or pts[-1] != v:
                    pts.append(v)
            xs = [xy(v)[0] + ox for v in pts]
            ys = [xy(v)[1] + oy for v in pts]
            ax.plot(xs, ys, color=ROBOT[i], lw=2.2, ls=ls, zorder=2, solid_capstyle="round",
                    dash_capstyle="round")
    for i in range(2):
        robot_disc(ax, *xy(FIG8_STARTS[i]), i, r=0.2, kind="start")
        robot_disc(ax, *xy(FIG8_GOALS[i]), i, r=0.2, kind="goal")
        robot_disc(ax, *xy(f8["mids"][i]), i, r=0.2, kind="mid")
    ax.set_xlim(-0.45, 3.45)
    ax.set_ylim(4.75, -0.75)
    ax.set_aspect("equal")
    n1, n2 = ts, f8["split"] - ts
    ax.text(-0.42, 3.42,
            f"T-RO Fig. 8, k = 2: intermediate goals\n{f8['mids']} (dashed). Pieces: "
            f"{n1} + {n2} = {f8['split']}\nsteps; unsplit optimum: {f8['unsplit']}.",
            fontsize=8.6, color=INK2, va="top", linespacing=1.35)
    # mini legend for the two pieces
    ax.plot([2.2, 2.55], [-0.52, -0.52], color=INK2, lw=1.8)
    ax.text(2.62, -0.52, "piece 1", fontsize=8, color=INK2, va="center")
    ax.plot([2.2, 2.55], [-0.2, -0.2], color=INK2, lw=1.8, ls=(0, (3, 2)))
    ax.text(2.62, -0.2, "piece 2", fontsize=8, color=INK2, va="center")


def small_chart(ax, ks, ys, ylabel, fmt, log=False, ref=None, ref_label=None,
                label_side=lambda j: (0, 10, "center")):
    xs = list(range(len(ks)))
    ax.plot(xs, ys, color=CHART, lw=2, zorder=3)
    ax.scatter(xs, ys, s=34, color=CHART, zorder=4, edgecolor="white", linewidth=1.2)
    for j, (x, y) in enumerate(zip(xs, ys)):
        dx, dy, ha = label_side(j)
        ax.annotate(fmt(y), (x, y), textcoords="offset points", xytext=(dx, dy), ha=ha, va="center",
                    fontsize=8.3, color=INK)
    if ref is not None:
        ax.axhline(ref, color=MID, lw=1, ls=(0, (3, 2)), zorder=1)
        ax.annotate(ref_label, (len(ks) - 0.6, ref), xytext=(-2, 3), textcoords="offset points",
                    fontsize=8, color=INK2, va="bottom", ha="right")
    ax.set_xticks(xs)
    ax.set_xticklabels([str(k) for k in ks])
    ax.set_xlabel("k (pieces)", fontsize=8.6)
    ax.set_title(ylabel, fontsize=9.2, color=INK, loc="left", pad=6)
    if log:
        ax.set_yscale("log")
    ax.set_xlim(-0.4, len(ks) - 0.6)
    ax.set_facecolor("none")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_color(FAINT)
    ax.spines["bottom"].set_color(FAINT)
    ax.tick_params(labelsize=8.3, length=0)
    ax.grid(axis="y", color=FAINT, lw=0.6)
    ax.set_axisbelow(True)


# ------------------------------------------------------------------ figures
def make_concepts(d, k_data, out_base, svg=True):
    fig = plt.figure(figsize=(12, 11.0), facecolor=CARD)
    outer = fig.add_gridspec(3, 1, height_ratios=[4.4, 2.1, 3.0], hspace=0.36,
                             left=0.035, right=0.975, top=0.955, bottom=0.075)
    top = outer[0].subgridspec(1, 2, width_ratios=[1.0, 2.6], wspace=0.04)
    ax_a = fig.add_subplot(top[0, 0])
    panel_instance(ax_a, d)
    panel_network(fig.add_subplot(top[0, 1]), d)
    panel_snapshots(fig, outer[1], d)
    bottom = outer[2].subgridspec(1, 3, width_ratios=[1.25, 1, 1], wspace=0.3)
    ax_split = fig.add_subplot(bottom[0, 0])
    panel_split(ax_split, d)

    ax1 = fig.add_subplot(bottom[0, 1])
    ax2 = fig.add_subplot(bottom[0, 2])
    kd = k_data
    small_chart(ax1, kd["k"], kd["makespan"], "Makespan", lambda y: f"{y}", ref=kd["lower_bound"],
                ref_label=f"lower bound {kd['lower_bound']}",
                label_side=lambda j: (0, 11, "center") if j == 0 else (-7, 6, "right"))
    ax1.set_ylim(kd["lower_bound"] - 6, max(kd["makespan"]) * 1.1)
    small_chart(ax2, kd["k"], kd["seconds"], "Solve time (s, log scale)",
                lambda y: f"{y:.0f}" if y >= 10 else f"{y:.1f}", log=True,
                label_side=lambda j: (0, -12, "center") if j == 3 else (7, 7, "left"))
    ax2.set_ylim(1, 400)
    ax2.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
    y_head = outer[2].get_position(fig).y1 + 0.04
    ax_a.apply_aspect()
    x_d = ax_a.get_position().x0
    fig.text(x_d, y_head, "(d)", fontsize=11, fontweight="bold", color=INK, ha="left", va="bottom")
    fig.text(x_d + 0.024, y_head, "k-way split: smaller models, longer plans", fontsize=11, color=INK,
             ha="left", va="bottom")
    fig.text(ax1.get_position().x0, y_head - 0.016,
             f"The 24 × 18 demo grid (25% obstacles, {kd['robots']} robots, seed 1), k = "
             f"{', '.join(map(str, kd['k']))}", fontsize=9.5, color=INK2, ha="left", va="bottom")
    foot = (f"Panels (a)-(c) and the Fig. 8 split: computed by mppflow {mppflow.__version__} "
            f"(python3 docs/media/make_concepts.py, backend {d['backend']}). "
            f"24 × 18 curves: docs/data/demo-data.js ({kd['backend']}, wall clock).")
    fig.text(0.035, 0.022, foot, fontsize=8, color=INK2, ha="left", va="bottom")
    fig.savefig(out_base + ".png", dpi=170, facecolor=CARD)
    if svg:
        fig.savefig(out_base + ".svg", facecolor=CARD, metadata={"Date": None})
    plt.close(fig)


def make_gadget(out_base, svg=True):
    """The merge-split gadget of Fig. 4(b) (5 arcs) next to the compact 2-arc version of Fig. 7,
    built from mppflow.network on a single edge {u, v} with T = 1."""
    g = mppflow.from_adjacency([[1], [0]])
    full = N.FullNetwork.build(g, [0], [1], 1)
    comp = N.CompactNetwork.build(g, [0], [1], 1)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 3.1), facecolor=CARD,
                                 gridspec_kw=dict(wspace=0.15, left=0.02, right=0.98, top=0.85, bottom=0.1))
    for ax in (a1, a2):
        bare(ax)
        ax.set_aspect("equal")
    # full gadget: u(0)'=u(0), v(0), merge, split, u(1), v(1); positions from to_json cols/rows
    pos = {}
    for nd in full.to_json()["nodes"]:
        if nd["kind"] in ("copy", "copy'") and nd["t"] <= 1 and nd["kind"] == "copy":
            pos[nd["id"]] = (nd["col"] * 1.2, nd["row"] * 1.6)
        elif nd["kind"] in ("merge", "split"):
            pos[nd["id"]] = (nd["col"] * 1.2 - 0.3, 0.8)
    lab = {}
    for i, key in enumerate(full.nodes):
        if key[0] == "v" and key[3] == 0 and key[2] <= 1 and i in pos:
            lab[i] = "$" + "uv"[key[1]] + ("(t)'" if key[2] == 0 else "(t+1)") + "$"
    for (ta, he, kind) in full.arcs:
        if ta in pos and he in pos and kind in ("g_in", "g_mid", "g_out"):
            a1.add_patch(FancyArrowPatch(pos[ta], pos[he], arrowstyle="-|>", mutation_scale=11,
                                         color=INK if kind == "g_mid" else INK2,
                                         lw=2.6 if kind == "g_mid" else 1.4, shrinkA=7, shrinkB=7))
    for i, (x, y) in pos.items():
        a1.add_patch(Circle((x, y), 0.1, fc="white" if i in lab else INK2, ec=INK2, lw=1.2, zorder=5))
        if i in lab:
            a1.text(x, y + (-0.3 if y < 0.8 else 0.32), lab[i], ha="center", va="center", fontsize=11)
    a1.text(1.5, 0.55, "capacity 1", ha="center", fontsize=9, color=INK2)
    a1.set_xlim(-0.5, 3.0)
    a1.set_ylim(2.15, -0.55)
    a1.set_title("Merge-split gadget (5 arcs)", fontsize=10.5, color=INK, loc="left")
    # compact: two crossing move arcs with a shared capacity row
    cp = {}
    for i, (v, t) in enumerate(comp.nodes):
        cp[(v, t)] = (t * 2.4, v * 1.6)
    for (u, w, t, kind) in comp.arcs:
        if kind == "move":
            a2.add_patch(FancyArrowPatch(cp[(u, t)], cp[(w, t + 1)], arrowstyle="-|>", mutation_scale=11,
                                         color=INK2, lw=1.8, shrinkA=7, shrinkB=7))
        elif kind == "stay":
            a2.add_patch(FancyArrowPatch(cp[(u, t)], cp[(w, t + 1)], arrowstyle="-|>", mutation_scale=9,
                                         color=MID, lw=1.1, shrinkA=7, shrinkB=7))
    for (v, t), (x, y) in cp.items():
        a2.add_patch(Circle((x, y), 0.1, fc="white", ec=INK2, lw=1.2, zorder=5))
        a2.text(x, y + (-0.3 if v == 0 else 0.32), "$" + "uv"[v] + ("(t)" if t == 0 else "(t+1)") + "$",
                ha="center", va="center", fontsize=11)
    a2.text(1.2, 0.8, r"$x_{u\to v}+x_{v\to u}\leq 1$", ha="center", va="center", fontsize=10,
            color=INK, bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=FAINT))
    a2.set_xlim(-0.5, 3.0)
    a2.set_ylim(2.15, -0.55)
    a2.set_title("Compact gadget (2 move arcs + 1 row)", fontsize=10.5, color=INK, loc="left")
    fig.text(0.02, 0.025, "No head-on collision: at most one robot crosses the edge {u, v} per step "
             "(T-RO Figs. 4(b) and 7).", fontsize=8.5, color=INK2)
    fig.savefig(out_base + ".png", dpi=170, facecolor=CARD)
    if svg:
        fig.savefig(out_base + ".svg", facecolor=CARD, metadata={"Date": None})
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--backend", default=None, help="ILP backend (default: mppflow's default, highs)")
    ap.add_argument("--out", default=HERE, help="output directory (default: docs/media)")
    ap.add_argument("--no-svg", action="store_true", help="write PNG only")
    args = ap.parse_args(argv)
    backend = args.backend or mppflow.available_backends()[0]
    d = compute(backend)
    k_data = read_demo_k_tradeoff()
    os.makedirs(args.out, exist_ok=True)
    make_concepts(d, k_data, os.path.join(args.out, "concepts"), svg=not args.no_svg)
    make_gadget(os.path.join(args.out, "gadget"), svg=not args.no_svg)
    t = d["tree"]
    f8 = d["fig8"]
    print(f"backend: {d['backend']}")
    print(f"tree: T = {t['T']} (lower bound {t['lower_bound']}), paths {t['paths']}, "
          f"model {t['columns']} x {t['rows']}, attempts {t['attempts']}")
    print(f"fig8: unsplit {f8['unsplit']}, split {f8['split']} at t = {f8['t_split']}, "
          f"intermediate goals {f8['mids']}, paths {f8['paths']}")
    print(f"24x18 (demo-data.js): k {k_data['k']} makespan {k_data['makespan']} seconds {k_data['seconds']}")
    for name in ("concepts", "gadget"):
        for ext in ("png", "svg"):
            p = os.path.join(args.out, f"{name}.{ext}")
            if os.path.exists(p):
                print(f"{name}.{ext}: {os.path.getsize(p) / 1024:.0f} KiB")


if __name__ == "__main__":
    main()
