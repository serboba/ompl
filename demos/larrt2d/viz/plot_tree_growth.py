#!/usr/bin/env python3
"""Visualise LA-RRT tree growth for the door puzzle.

Reads a tree dump (LARRT_TREE_DUMP) whose nodes are [Ax, Ay, doorAngle] with a
parent index, plus the scene JSON for walls. Renders two panels:
  (left)  workspace: box-A positions of every start/goal node + tree edges;
  (right) door-angle coverage of each tree (this is where the bug shows up).

Usage: plot_tree_growth.py <tree_dump.json> <scene.json> <out.png> [title]
"""
import sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

tree = json.load(open(sys.argv[1]))
scene = json.load(open(sys.argv[2]))
out = sys.argv[3]
title = sys.argv[4] if len(sys.argv) > 4 else ""

fig, (axw, axd) = plt.subplots(1, 2, figsize=(15, 6))

# --- left: workspace, box-A positions -------------------------------------
for o in scene["obstacles"]:
    axw.add_patch(plt.Rectangle((o["xmin"], o["ymin"]), o["xmax"]-o["xmin"],
                                o["ymax"]-o["ymin"], color="0.35", zorder=1))
for key, col, lbl in [("start", "#1f77b4", "start tree"), ("goal", "#d62728", "goal tree")]:
    nodes = tree[key]
    pts = np.array([[n["v"][0], n["v"][1]] for n in nodes])
    segs = [[(nodes[i]["v"][0], nodes[i]["v"][1]),
             (nodes[n["p"]]["v"][0], nodes[n["p"]]["v"][1])]
            for i, n in enumerate(nodes) if n["p"] >= 0]
    axw.add_collection(LineCollection(segs, colors=col, linewidths=0.25, alpha=0.35, zorder=2))
    axw.scatter(pts[:, 0], pts[:, 1], s=2, c=col, alpha=0.5, label=f"{lbl} (A pos), n={len(nodes)}", zorder=3)
axw.set_xlim(scene["world"]["xmin"], scene["world"]["xmax"])
axw.set_ylim(scene["world"]["ymin"], scene["world"]["ymax"])
axw.set_aspect("equal"); axw.set_title("box-A positions reached by each tree")
axw.legend(loc="upper center", fontsize=8)

# --- right: door-angle coverage vs A-x ------------------------------------
di = len(tree["start"][0]["v"]) - 1  # door dim = last
for key, col, lbl in [("start", "#1f77b4", "start tree"), ("goal", "#d62728", "goal tree")]:
    nodes = tree[key]
    ax_ = np.array([n["v"][0] for n in nodes])
    dr = np.array([n["v"][di] for n in nodes])
    axd.scatter(ax_, dr, s=3, c=col, alpha=0.4, label=lbl)
axd.axhline(1.5708, color="k", ls="--", lw=1, label="door closed (π/2)")
axd.axhline(0.0, color="g", ls=":", lw=1, label="door open (0)")
axd.set_xlabel("box-A x"); axd.set_ylabel("door angle (rad)")
axd.set_title("door angle explored (the diagnosis)")
axd.legend(loc="upper right", fontsize=8)

fig.suptitle(title, fontsize=13)
fig.tight_layout()
fig.savefig(out, dpi=110)
print("wrote", out)
