#!/usr/bin/env python3
"""Visualize an LA-RRT 2D rearrangement-puzzle solution.

Draws obstacles as grey rectangles and each object as a coloured oriented
rectangle (OBB). Animates the path one object at a time and also writes a
static keyframe strip (one panel per action).

Usage:
    python3 viz_puzzle.py <solution.json> [out.gif]

Default out.gif = the solution path with its extension replaced by .gif.
The keyframe strip is written as <solution_stem>_strip.png.

See demos/larrt2d/PUZZLE_PIPELINE_SPEC.md (the contract):
  §1 geometry, §2 state layout, §4 solution JSON, §5 action counting.
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
from matplotlib.animation import FuncAnimation, PillowWriter

STEPS = 18  # interpolation frames per path segment


def changed_group(groups, a, b, tol=1e-9):
    """Return the group index whose dims differ between states a and b; -1 if none."""
    for gi, g in enumerate(groups):
        for idx in g:
            if abs(a[idx] - b[idx]) > tol:
                return gi
    return -1


def action_runs(groups, path):
    """Merge consecutive path segments that move the SAME object into one action
    (spec §5). Returns a list of [action_index (1-based), group_index, end_waypoint],
    where end_waypoint is the path index at which that action finishes. Zero-move
    segments (changed_group == -1) are ignored, matching the validator's recount."""
    runs = []
    prev = None
    for i in range(len(path) - 1):
        g = changed_group(groups, path[i], path[i + 1])
        if g == -1:
            continue
        if g != prev:
            runs.append([len(runs) + 1, g, i + 1])
            prev = g
        else:
            runs[-1][2] = i + 1
    return runs


def segment_action_index(groups, path):
    """For each segment i, the 1-based action index it belongs to (same merging as
    action_runs), so the animation title reflects actions, not raw segments."""
    idx = [1] * max(len(path) - 1, 1)
    prev = None
    a = 0
    for i in range(len(path) - 1):
        g = changed_group(groups, path[i], path[i + 1])
        if g != -1 and g != prev:
            a += 1
            prev = g
        idx[i] = max(a, 1)
    return idx


def densify(path, steps):
    """Linearly interpolate between consecutive states; tag each frame with the
    segment index it belongs to."""
    frames = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        for k in range(steps):
            t = k / float(steps)
            frames.append(([a[j] + t * (b[j] - a[j]) for j in range(len(a))], i))
    frames.append((list(path[-1]), max(len(path) - 2, 0)))
    return frames


def obb_corners(x, y, theta, hx, hy):
    """Return the 4 OBB corners: C = (x,y) + R(theta)*(±hx, ±hy)."""
    c, s = math.cos(theta), math.sin(theta)
    out = []
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        lx, ly = sx * hx, sy * hy
        out.append((x + c * lx - s * ly, y + s * lx + c * ly))
    return out


def obb_from_dof(obj, v):
    """OBB (cx, cy, hx, hy, theta) of an object from its own DOF vector v.

    Mirrors the driver's ObjSpec::obb for every kind (box/door/revolute/slider);
    v is the object's DOF (1 for door/slider/revolute, 2-3 for a box)."""
    t = obj.get("type", "box")
    if t == "box":
        theta = v[2] if len(v) >= 3 else obj.get("angle", 0.0)
        return (v[0], v[1], obj["hx"], obj["hy"], theta)
    if t == "door":
        a = v[0]
        L, W = obj["length"], obj["width"]
        hxp, hyp = obj["hinge"]
        return (hxp + 0.5 * L * math.cos(a), hyp + 0.5 * L * math.sin(a),
                0.5 * L, 0.5 * W, a)
    if t == "revolute":
        a = v[0]
        ax, ay = obj["anchor"]
        cx0, cy0 = obj["com"]
        c, s = math.cos(a), math.sin(a)
        return (ax + c * cx0 - s * cy0, ay + s * cx0 + c * cy0,
                obj["hx"], obj["hy"], a)
    if t == "slider":
        d = v[0]
        bx, by = obj["base"]
        axx, axy = obj["axis"]
        return (bx + d * axx, by + d * axy, obj["hx"], obj["hy"], obj.get("angle", 0.0))
    raise ValueError("unknown object type: %r" % t)


def object_dof(state, obj):
    """The object's own DOF values sliced out of a full path state."""
    return [state[d] for d in obj["dims"]]


def obj_corners(obj, v):
    """OBB corners of an object from its DOF vector v (ready for a Polygon)."""
    cx, cy, hx, hy, th = obb_from_dof(obj, v)
    return obb_corners(cx, cy, th, hx, hy)


def draw_obstacles(ax, obstacles):
    for r in obstacles:
        ax.add_patch(Rectangle((r["xmin"], r["ymin"]),
                               r["xmax"] - r["xmin"], r["ymax"] - r["ymin"],
                               facecolor="0.55", edgecolor="0.3",
                               alpha=0.85, zorder=1))


def object_colors(n):
    cmap = plt.get_cmap("tab10" if n <= 10 else "hsv")
    if n <= 10:
        return [cmap(i) for i in range(n)]
    return [cmap(i / float(n)) for i in range(n)]


def draw_reference_outlines(ax, objs, colors):
    """Dashed start outline for every object; solid (unfilled) goal outline only for
    TARGET objects. Non-targets have no goal (they may end anywhere), so we mark them
    as free obstacles rather than drawing a goal."""
    for obj, col in zip(objs, colors):
        ax.add_patch(Polygon(obj_corners(obj, obj["start"]), closed=True,
                             fill=False, edgecolor=col, ls="--", lw=1.0,
                             alpha=0.8, zorder=2))
        is_target = obj.get("target", obj.get("goal") is not None)
        if is_target and obj.get("goal") is not None:
            ax.add_patch(Polygon(obj_corners(obj, obj["goal"]), closed=True,
                                 fill=False, edgecolor=col, ls="-", lw=2.0,
                                 alpha=0.95, zorder=2))


def world_limits(world, pad=0.3):
    return (world["xmin"] - pad, world["xmax"] + pad,
            world["ymin"] - pad, world["ymax"] + pad)


def setup_axes(ax, data):
    xlo, xhi, ylo, yhi = world_limits(data["world"])
    ax.set_xlim(xlo, xhi)
    ax.set_ylim(ylo, yhi)
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.3)


def animate(data, out_gif):
    world = data["world"]
    objs = data["objects"]
    groups = data["groups"]
    path = data["path"]
    colors = object_colors(len(objs))
    frames = densify(path, STEPS)
    action_total = data.get("action_count", -1)
    seg_action = segment_action_index(groups, path)

    fig, ax = plt.subplots(figsize=(9, 9 * (world["ymax"] - world["ymin"]) /
                                    max(world["xmax"] - world["xmin"], 1e-6)))
    draw_obstacles(ax, data["obstacles"])
    draw_reference_outlines(ax, objs, colors)
    setup_axes(ax, data)

    # One filled polygon patch per object, updated each frame.
    patches = []
    for obj, col in zip(objs, colors):
        p = Polygon(obj_corners(obj, object_dof(path[0], obj)), closed=True,
                    facecolor=col, edgecolor="black", lw=1.2, alpha=0.9, zorder=5)
        ax.add_patch(p)
        patches.append(p)
    title = ax.set_title("")

    def update(fi):
        state, seg = frames[fi]
        for obj, p in zip(objs, patches):
            p.set_xy(obj_corners(obj, object_dof(state, obj)))
        gidx = changed_group(groups, path[seg], path[seg + 1]) if seg + 1 < len(path) else -1
        moving = objs[gidx]["name"] if gidx >= 0 else "-"
        act = seg_action[seg] if seg < len(seg_action) else action_total
        title.set_text(f"action {act}: move {moving}   "
                       f"(total actions = {action_total})")
        return patches + [title]

    anim = FuncAnimation(fig, update, frames=len(frames), interval=60, blit=False)
    anim.save(out_gif, writer=PillowWriter(fps=20))
    plt.close(fig)
    print("wrote", out_gif)


def draw_scene(ax, data, state, objs, colors, highlight_gidx):
    draw_obstacles(ax, data["obstacles"])
    draw_reference_outlines(ax, objs, colors)
    for gi, (obj, col) in enumerate(zip(objs, colors)):
        lw = 2.5 if gi == highlight_gidx else 1.0
        edge = "black" if gi == highlight_gidx else col
        ax.add_patch(Polygon(obj_corners(obj, object_dof(state, obj)),
                             closed=True, facecolor=col, edgecolor=edge,
                             lw=lw, alpha=0.9, zorder=5))
    setup_axes(ax, data)


def keyframe_strip(data, out_png):
    objs = data["objects"]
    groups = data["groups"]
    path = data["path"]
    colors = object_colors(len(objs))

    # One panel per ACTION (consecutive same-object segments merged, spec §5), showing
    # the state at the moment that action finishes with the just-moved object highlighted.
    runs = action_runs(groups, path)
    if not runs:  # degenerate (start == goal): show the single state
        runs = [[1, -1, len(path) - 1]]
    n_panels = len(runs)
    ncols = min(4, n_panels)
    nrows = math.ceil(n_panels / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 4.2 * nrows),
                             squeeze=False)

    for k, (act_idx, gidx, end_wp) in enumerate(runs):
        ax = axes[k // ncols][k % ncols]
        moving = objs[gidx]["name"] if gidx >= 0 else "-"
        draw_scene(ax, data, path[end_wp], objs, colors, gidx)
        ax.set_title(f"action {act_idx}: {moving}")

    # Hide any unused panels.
    for k in range(n_panels, nrows * ncols):
        axes[k // ncols][k % ncols].axis("off")

    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
    print("wrote", out_png)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    json_path = sys.argv[1]
    with open(json_path) as f:
        data = json.load(f)

    if not data.get("solved", False):
        print(f"solution not solved (solved=false): {json_path} — nothing to animate.")
        sys.exit(0)

    if not data.get("path"):
        print(f"solution has empty path: {json_path} — nothing to animate.")
        sys.exit(0)

    stem = os.path.splitext(json_path)[0]
    out_gif = sys.argv[2] if len(sys.argv) > 2 else stem + ".gif"
    out_png = stem + "_strip.png"

    animate(data, out_gif)
    keyframe_strip(data, out_png)


if __name__ == "__main__":
    main()
