#!/usr/bin/env python3
"""Static plot of an LA-RRT 2D demo solution.

Usage:
    python3 plot_solution.py <solution.json> [out.png]

Handles both demo flavours, detected from the JSON "demo" field:
  * "configspace": 2D configuration space (x = object 0, y = object 1). The
    path is drawn as connected axis-aligned segments, each labelled by which
    group (object) moves. The total action count is annotated.
  * "workspace": 2D workspace with two pucks. Each puck's trajectory is drawn
    in its own colour with start/goal markers.
"""
import json
import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def changed_group(groups, a, b, tol=1e-9):
    for gi, g in enumerate(groups):
        for idx in g:
            if abs(a[idx] - b[idx]) > tol:
                return gi
    return -1


def draw_obstacles(ax, obstacles):
    for r in obstacles:
        ax.add_patch(Rectangle((r["xmin"], r["ymin"]),
                               r["xmax"] - r["xmin"], r["ymax"] - r["ymin"],
                               facecolor="0.55", edgecolor="0.3", hatch="//",
                               alpha=0.7, zorder=1))


def draw_band(ax, band, bounds):
    """Shade the forbidden diagonal band |x - y| < gap while x + y < sum_limit."""
    if not band:
        return
    import numpy as np
    gap, smax = band["gap"], band["sum_limit"]
    lo0, hi0 = bounds[0]
    lo1, hi1 = bounds[1]
    xs = np.linspace(lo0, hi0, 400)
    ys = np.linspace(lo1, hi1, 400)
    X, Y = np.meshgrid(xs, ys)
    mask = (np.abs(X - Y) < gap) & ((X + Y) < smax)
    ax.contourf(X, Y, mask.astype(float), levels=[0.5, 1.5],
                colors=["0.55"], alpha=0.7, zorder=1)
    ax.contour(X, Y, mask.astype(float), levels=[0.5], colors=["0.3"],
               linewidths=1.0, zorder=1)


def plot_configspace(data, out_png):
    groups = data["groups"]
    path = data["path"]
    bounds = data["bounds"]
    labels = data.get("axis_labels", ["object 0", "object 1"])
    seg_colors = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_obstacles(ax, data["obstacles"])
    draw_band(ax, data.get("forbidden_band"), bounds)

    # Each segment moves exactly one group -> axis-aligned. Colour by group.
    seen = set()
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        g = changed_group(groups, a, b)
        c = seg_colors[g % len(seg_colors)] if g >= 0 else "0.5"
        lbl = None
        if g not in seen:
            lbl = f"move object {g}"
            seen.add(g)
        ax.plot([a[0], b[0]], [a[1], b[1]], "-", color=c, lw=3,
                solid_capstyle="round", label=lbl, zorder=3)
        mx, my = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
        if g >= 0:
            ax.annotate(f"obj{g}", (mx, my), fontsize=8, color=c,
                        ha="center", va="center",
                        bbox=dict(boxstyle="round,pad=0.15", fc="white",
                                  ec=c, alpha=0.8), zorder=4)

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    ax.plot(xs, ys, "o", color="black", ms=4, zorder=5)
    ax.plot(data["start"][0], data["start"][1], "*", color="green", ms=22,
            label="start", zorder=6)
    ax.plot(data["goal"][0], data["goal"][1], "X", color="red", ms=16,
            label="goal", zorder=6)

    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] + 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] + 0.5)
    ax.set_xlabel(labels[0])
    ax.set_ylabel(labels[1])
    ax.set_aspect("equal")
    ax.set_title(f"LA-RRT config space  |  actions = {data['action_count']}  "
                 f"(each segment moves one object)")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, ls=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print("wrote", out_png)


def plot_workspace(data, out_png):
    path = data["path"]
    bounds = data["bounds"]
    names = data.get("object_names", ["puck A", "puck B"])
    colA, colB = "#1f77b4", "#d62728"

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_obstacles(ax, data["obstacles"])

    ax_x = [p[0] for p in path]
    ax_y = [p[1] for p in path]
    bx = [p[2] for p in path]
    by = [p[3] for p in path]

    ax.plot(ax_x, ax_y, "-o", color=colA, lw=2.5, ms=4, label=names[0], zorder=3)
    ax.plot(bx, by, "-o", color=colB, lw=2.5, ms=4, label=names[1], zorder=3)

    s, g = data["start"], data["goal"]
    ax.plot(s[0], s[1], "*", color=colA, ms=22, mec="black", zorder=6)
    ax.plot(g[0], g[1], "X", color=colA, ms=16, mec="black", zorder=6)
    ax.plot(s[2], s[3], "*", color=colB, ms=22, mec="black", zorder=6)
    ax.plot(g[2], g[3], "X", color=colB, ms=16, mec="black", zorder=6)
    ax.annotate("A start", (s[0], s[1]), fontsize=8, xytext=(4, 4),
                textcoords="offset points")
    ax.annotate("A goal", (g[0], g[1]), fontsize=8, xytext=(4, 4),
                textcoords="offset points")
    ax.annotate("B start", (s[2], s[3]), fontsize=8, xytext=(4, 4),
                textcoords="offset points")
    ax.annotate("B goal", (g[2], g[3]), fontsize=8, xytext=(4, 4),
                textcoords="offset points")

    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] + 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] + 0.5)
    ax.set_xlabel("workspace x")
    ax.set_ylabel("workspace y")
    ax.set_aspect("equal")
    ax.set_title(f"LA-RRT workspace rearrangement  |  actions = "
                 f"{data['action_count']}  (one puck moves per action)")
    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, ls=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print("wrote", out_png)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    json_path = sys.argv[1]
    with open(json_path) as f:
        data = json.load(f)
    default_png = os.path.splitext(json_path)[0] + ".png"
    out_png = sys.argv[2] if len(sys.argv) > 2 else default_png

    if data["demo"] == "configspace":
        plot_configspace(data, out_png)
    elif data["demo"] == "workspace":
        plot_workspace(data, out_png)
    else:
        raise SystemExit(f"unknown demo type: {data['demo']}")


if __name__ == "__main__":
    main()
