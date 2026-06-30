#!/usr/bin/env python3
"""Animate an LA-RRT 2D demo solution and save a GIF.

Usage:
    python3 animate_solution.py <solution.json> [out.gif]

For "configspace": a point moves through the 2D configuration space, one
axis-aligned segment (one object) at a time.
For "workspace": two pucks move in the workspace, only one puck moving during
any given action while the other stays fixed.
"""
import json
import sys
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation, PillowWriter

STEPS = 18  # interpolation frames per path segment


def changed_group(groups, a, b, tol=1e-9):
    for gi, g in enumerate(groups):
        for idx in g:
            if abs(a[idx] - b[idx]) > tol:
                return gi
    return -1


def densify(path, steps):
    """Linearly interpolate between consecutive states; tag each frame with the
    moving group index for that segment."""
    frames = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        for k in range(steps):
            t = k / float(steps)
            frames.append(([a[j] + t * (b[j] - a[j]) for j in range(len(a))], i))
    frames.append((list(path[-1]), len(path) - 2))
    return frames


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
    xs = np.linspace(bounds[0][0], bounds[0][1], 400)
    ys = np.linspace(bounds[1][0], bounds[1][1], 400)
    X, Y = np.meshgrid(xs, ys)
    mask = (np.abs(X - Y) < gap) & ((X + Y) < smax)
    ax.contourf(X, Y, mask.astype(float), levels=[0.5, 1.5],
                colors=["0.55"], alpha=0.7, zorder=1)


def animate_configspace(data, out_gif):
    groups = data["groups"]
    path = data["path"]
    bounds = data["bounds"]
    labels = data.get("axis_labels", ["object 0", "object 1"])
    frames = densify(path, STEPS)

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_obstacles(ax, data["obstacles"])
    draw_band(ax, data.get("forbidden_band"), bounds)
    ax.plot([p[0] for p in path], [p[1] for p in path], "-", color="0.8",
            lw=1.5, zorder=2)
    ax.plot(data["start"][0], data["start"][1], "*", color="green", ms=20, zorder=4)
    ax.plot(data["goal"][0], data["goal"][1], "X", color="red", ms=14, zorder=4)
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] + 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] + 0.5)
    ax.set_xlabel(labels[0]); ax.set_ylabel(labels[1])
    ax.set_aspect("equal"); ax.grid(True, ls=":", alpha=0.4)

    trace, = ax.plot([], [], "-", color="#1f77b4", lw=3, zorder=3)
    dot, = ax.plot([], [], "o", color="black", ms=10, zorder=5)
    title = ax.set_title("")
    xs, ys = [], []

    def update(fi):
        pt, seg = frames[fi]
        g = changed_group(groups, path[seg], path[seg + 1])
        xs.append(pt[0]); ys.append(pt[1])
        trace.set_data(xs, ys)
        dot.set_data([pt[0]], [pt[1]])
        title.set_text(f"LA-RRT config space  |  action {seg + 1}: move object {g}"
                       f"   (total actions = {data['action_count']})")
        return trace, dot, title

    anim = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    anim.save(out_gif, writer=PillowWriter(fps=20))
    print("wrote", out_gif)


def animate_workspace(data, out_gif):
    groups = data["groups"]
    path = data["path"]
    bounds = data["bounds"]
    names = data.get("object_names", ["puck A", "puck B"])
    colA, colB = "#1f77b4", "#d62728"
    frames = densify(path, STEPS)

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_obstacles(ax, data["obstacles"])
    s, g = data["start"], data["goal"]
    ax.plot(s[0], s[1], "*", color=colA, ms=20, mec="black", zorder=4)
    ax.plot(g[0], g[1], "X", color=colA, ms=14, mec="black", zorder=4)
    ax.plot(s[2], s[3], "*", color=colB, ms=20, mec="black", zorder=4)
    ax.plot(g[2], g[3], "X", color=colB, ms=14, mec="black", zorder=4)
    ax.set_xlim(bounds[0][0] - 0.5, bounds[0][1] + 0.5)
    ax.set_ylim(bounds[1][0] - 0.5, bounds[1][1] + 0.5)
    ax.set_xlabel("workspace x"); ax.set_ylabel("workspace y")
    ax.set_aspect("equal"); ax.grid(True, ls=":", alpha=0.4)

    traceA, = ax.plot([], [], "-", color=colA, lw=1.5, alpha=0.6, zorder=2)
    traceB, = ax.plot([], [], "-", color=colB, lw=1.5, alpha=0.6, zorder=2)
    puckA = Circle((s[0], s[1]), 0.3, color=colA, zorder=5)
    puckB = Circle((s[2], s[3]), 0.3, color=colB, zorder=5)
    ax.add_patch(puckA); ax.add_patch(puckB)
    title = ax.set_title("")
    axu, ayu, bxu, byu = [], [], [], []

    def update(fi):
        pt, seg = frames[fi]
        gidx = changed_group(groups, path[seg], path[seg + 1])
        axu.append(pt[0]); ayu.append(pt[1])
        bxu.append(pt[2]); byu.append(pt[3])
        traceA.set_data(axu, ayu)
        traceB.set_data(bxu, byu)
        puckA.center = (pt[0], pt[1])
        puckB.center = (pt[2], pt[3])
        moving = names[gidx] if gidx >= 0 else "-"
        title.set_text(f"LA-RRT workspace  |  action {seg + 1}: move {moving}"
                       f"   (total actions = {data['action_count']})")
        return traceA, traceB, puckA, puckB, title

    anim = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    anim.save(out_gif, writer=PillowWriter(fps=20))
    print("wrote", out_gif)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    json_path = sys.argv[1]
    with open(json_path) as f:
        data = json.load(f)
    default_gif = os.path.splitext(json_path)[0] + ".gif"
    out_gif = sys.argv[2] if len(sys.argv) > 2 else default_gif

    if data["demo"] == "configspace":
        animate_configspace(data, out_gif)
    elif data["demo"] == "workspace":
        animate_workspace(data, out_gif)
    else:
        raise SystemExit(f"unknown demo type: {data['demo']}")


if __name__ == "__main__":
    main()
