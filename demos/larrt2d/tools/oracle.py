#!/usr/bin/env python3
"""Exact small-n rearrangement oracle (HANDOFF T5b).

Computes a GROUND-TRUTH optimal action count by A* over a discretized
mode graph, to resolve cases where the sound lower bound (monotonicity.py) is
loose or the LA-RRT planner is suboptimal. Intended for n<=4 objects.

Model (matches the pick-place move model of the rest of the pipeline):
  * A *configuration* assigns every object to one of its CANDIDATE poses
    (its start, its goal, and collision-free buffer poses); a config is valid
    iff all objects are mutually + environmentally collision-free (touching ok).
  * An *action* moves ONE object from its current candidate pose to another,
    provided a collision-free single-object path exists with all OTHER objects
    frozen at the current config (grid-flood reachability for boxes; interval
    sweep for 1-DOF door/slider/revolute -- same geometry as monotonicity.py).
  * Cost = number of actions. A* from the all-start config to the all-goal
    config; admissible heuristic = #objects not already at their goal pose.

The returned count is an EXACT optimum over the candidate discretization, i.e.
a real (validatable) UPPER bound on the continuous optimum. Combined with the
sound LB it tightens the Q5 bracket: oracle == LB => certified; oracle < planner
UB => the planner was suboptimal; oracle > LB (with a rich candidate set) =>
the LB is genuinely loose.

Usage (from demos/larrt2d):
    python3 tools/oracle.py scenes/<scene>.json [--cand-step 0.5] [--grid 0.1]
                            [--samples1d 400] [--max-expand 200000] [--json]
"""
import argparse
import heapq
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_puzzle import (obb_corners, sat_overlap, obb_from_dof,
                             obstacle_to_obb)
from monotonicity import object_dofs


def _obb_tuple_corners(obb):
    return obb_corners(*obb), obb[4]


def overlaps(a, b):
    ca, ta = _obb_tuple_corners(a)
    cb, tb = _obb_tuple_corners(b)
    return sat_overlap(ca, ta, cb, tb)


def collision_free(obb, static_obbs):
    return not any(overlaps(obb, s) for s in static_obbs)


# ---------------------------------------------------------------------------
# Candidate pose generation
# ---------------------------------------------------------------------------

def _seg_dist(p, a, b):
    """Distance from point p to segment a-b."""
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def candidate_poses(obj, all_objs, obst_obbs, cand_step, samples1d, max_buffers=8,
                    prune=True):
    """Candidate DOF vectors for `obj`: start, goal, and BUFFER pockets.

    prune=True (default, LEAN): a box only ever *parks* at start/goal or a pocket
    off the through paths; it slides through lane positions continuously (handled
    by reachability), so lane-interior grid points are dropped -- buffers are the
    collision-free grid positions clear of every object's straight start->goal
    corridor, farthest-point-sampled to `max_buffers`. Keeps the config space small
    enough for n up to ~5, and the oracle result is a valid UPPER bound.

    prune=False (FULL): keep the entire collision-free grid over the object's bounds
    (+ every object's start/goal slot). Needed to find near-lane *stacking* optima
    (e.g. swap_3_1's 5-plan stacks two boxes in one niche column), which the lean
    prune removes. Only tractable for small n; use when the lean oracle > sound LB."""
    t = obj.get("type", "box")
    s, g = object_dofs(obj)
    cands = []

    def add(dof):
        for c in cands:
            if len(c) == len(dof) and all(abs(a - b) < 1e-6 for a, b in zip(c, dof)):
                return True
        if collision_free(obb_from_dof(obj, dof), obst_obbs):
            cands.append(list(dof)); return True
        return False

    add(s)
    if g is not None:
        add(g)

    if t == "box":
        theta = s[2] if len(s) >= 3 else obj.get("angle", 0.0)
        xb = obj.get("xbounds"); yb = obj.get("ybounds")
        # every object's straight start->goal corridor (to exclude through-lane)
        segs = []
        for o in all_objs:
            os_, og_ = object_dofs(o)
            if os_ is not None and og_ is not None and len(os_) >= 2 and len(og_) >= 2:
                segs.append(((os_[0], os_[1]), (og_[0], og_[1])))
        excl_r = math.hypot(obj["hx"], obj["hy"]) + 0.15   # ~box half-diagonal + margin
        pool = []
        if xb and yb:
            nx = max(1, int(round((xb[1] - xb[0]) / cand_step)))
            ny = max(1, int(round((yb[1] - yb[0]) / cand_step)))
            for i in range(nx + 1):
                for j in range(ny + 1):
                    x = xb[0] + (xb[1] - xb[0]) * i / nx
                    y = yb[0] + (yb[1] - yb[0]) * j / ny
                    if not collision_free(obb_from_dof(obj, [x, y, theta]), obst_obbs):
                        continue
                    if prune and any(_seg_dist((x, y), a, b) < excl_r for a, b in segs):
                        continue          # on a through-corridor -> not a parking pocket
                    pool.append((x, y))
        if not prune:
            # FULL: also let a box occupy any object's start/goal slot, keep all pool
            for o in all_objs:
                os_, og_ = object_dofs(o)
                for d in (os_, og_):
                    if d is not None and len(d) >= 2:
                        add([d[0], d[1], theta])
            for x, y in pool:
                add([x, y, theta])
        else:
            # LEAN: farthest-point sample the pocket pool down to max_buffers
            anchors = [(c[0], c[1]) for c in cands]
            chosen = []
            while pool and len(chosen) < max_buffers:
                ref = anchors + chosen
                best, bd = None, -1.0
                for p in pool:
                    d = min(math.hypot(p[0] - r[0], p[1] - r[1]) for r in ref) if ref else 1e9
                    if d > bd:
                        bd, best = d, p
                chosen.append(best); pool.remove(best)
            for x, y in chosen:
                add([x, y, theta])
    elif t in ("door", "revolute", "slider"):
        lo, hi = (obj.get("angle_bounds") or obj.get("disp_bounds") or [s[0], s[0]])
        k = max(2, samples1d // 40)
        for i in range(k + 1):
            add([lo + (hi - lo) * i / k])
    return cands


# ---------------------------------------------------------------------------
# Single-object reachability with all other objects frozen
# ---------------------------------------------------------------------------

class BoxReach(object):
    """Grid flood for a 2-DOF (x,y) box: cell free iff the box OBB at the cell
    centre is collision-free vs (static obstacles + frozen others). Returns the
    set of candidate indices reachable from a given start candidate."""

    def __init__(self, obj, static_all, h):
        self.obj = obj
        self.static = static_all
        self.theta = None
        xb, yb = obj["xbounds"], obj["ybounds"]
        # pad by one cell so bound-touching poses land inside the grid
        self.x0, self.x1 = xb[0], xb[1]
        self.y0, self.y1 = yb[0], yb[1]
        self.h = h
        self.nx = max(1, int(math.ceil((self.x1 - self.x0) / h)))
        self.ny = max(1, int(math.ceil((self.y1 - self.y0) / h)))
        self._free = {}

    def _cell(self, x, y):
        i = min(self.nx, max(0, int(round((x - self.x0) / self.h))))
        j = min(self.ny, max(0, int(round((y - self.y0) / self.h))))
        return i, j

    def _center(self, i, j):
        return (self.x0 + i * self.h, self.y0 + j * self.h)

    def _is_free(self, i, j, theta):
        x, y = self._center(i, j)
        return collision_free(obb_from_dof(self.obj, [x, y, theta]), self.static)

    def candidate_components(self, cand_dofs, static_all, theta):
        """Label the free grid into connected components (8-connected) and return
        a list comp[k] = component id of candidate k's cell (None if that cell is
        blocked). Two candidates are mutually reachable iff comp values match.
        Computed ONCE per (object, frozen-others) -- cache by others upstream."""
        self.static = static_all
        cand_cells = [self._cell(d[0], d[1]) for d in cand_dofs]
        needed = set(cand_cells)
        # flood every candidate cell's component (lazy over the grid)
        label = {}
        comp_id = 0
        for cell in cand_cells:
            if cell in label:
                continue
            i0, j0 = cell
            if not self._is_free(i0, j0, theta):
                label[cell] = None
                continue
            stack = [cell]
            label[cell] = comp_id
            while stack:
                i, j = stack.pop()
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    ni, nj = i + di, j + dj
                    if 0 <= ni <= self.nx and 0 <= nj <= self.ny and (ni, nj) not in label:
                        if self._is_free(ni, nj, theta):
                            label[(ni, nj)] = comp_id
                            stack.append((ni, nj))
                        else:
                            label[(ni, nj)] = None
            comp_id += 1
        return [label.get(c) for c in cand_cells]


def dof1_candidate_components(obj, cand_dofs, static_all, samples):
    """1-DOF analogue of BoxReach.candidate_components: comp[k] = id of the
    contiguous collision-free interval containing candidate k (None if blocked)."""
    lo, hi = (obj.get("angle_bounds") or obj.get("disp_bounds") or
              [cand_dofs[0][0], cand_dofs[0][0]])
    n = samples
    if hi <= lo:
        return [0 for _ in cand_dofs]
    free = [collision_free(obb_from_dof(obj, [lo + (hi - lo) * k / n]), static_all)
            for k in range(n + 1)]
    # label contiguous free runs
    run = [None] * (n + 1)
    cid = 0
    k = 0
    while k <= n:
        if free[k]:
            while k <= n and free[k]:
                run[k] = cid
                k += 1
            cid += 1
        else:
            k += 1

    def val_to_k(v):
        return min(n, max(0, int(round((v - lo) / (hi - lo) * n))))

    return [run[val_to_k(d[0])] for d in cand_dofs]


# ---------------------------------------------------------------------------
# A* over configurations
# ---------------------------------------------------------------------------

def solve(scene, cand_step=0.5, grid_h=0.1, samples1d=400, max_expand=200000,
          max_buffers=8, prune=True, verbose=False):
    objs = scene["objects"]
    obst_obbs = [obstacle_to_obb(o) for o in scene.get("obstacles", [])]
    names = [o["name"] for o in objs]

    cands = [candidate_poses(o, objs, obst_obbs, cand_step, samples1d, max_buffers, prune)
             for o in objs]
    # index of the start and goal candidate for each object
    start_idx, goal_idx = [], []
    for oi, o in enumerate(objs):
        s, g = object_dofs(o)
        si = _find(cands[oi], s)
        cands[oi][si]  # ensure exists
        start_idx.append(si)
        goal_idx.append(_find(cands[oi], g) if g is not None else si)
    start_cfg = tuple(start_idx)
    goal_cfg = tuple(goal_idx)

    reachers = []
    for o in objs:
        if o.get("type", "box") == "box":
            reachers.append(BoxReach(o, obst_obbs, grid_h))
        else:
            reachers.append(None)

    def config_valid(cfg):
        obbs = [obb_from_dof(objs[i], cands[i][cfg[i]]) for i in range(len(objs))]
        for i in range(len(obbs)):
            if not collision_free(obbs[i], obst_obbs):
                return False
            for j in range(i + 1, len(obbs)):
                if overlaps(obbs[i], obbs[j]):
                    return False
        return True

    if not config_valid(start_cfg):
        raise RuntimeError("start configuration is in collision")
    if not config_valid(goal_cfg):
        raise RuntimeError("goal configuration is in collision")

    def frozen_static(cfg, moving):
        obbs = list(obst_obbs)
        for i in range(len(objs)):
            if i != moving:
                obbs.append(obb_from_dof(objs[i], cands[i][cfg[i]]))
        return obbs

    # cache the connected-component labels of object i's candidates, keyed by
    # (i, the OTHER objects' candidate indices) -- object i's reachability
    # depends only on where everything else is, so this is reused across every
    # config that differs only in object i's own pose.
    comp_cache = {}

    def comp_labels(cfg, i):
        others = cfg[:i] + cfg[i + 1:]
        key = (i, others)
        hit = comp_cache.get(key)
        if hit is None:
            static_all = frozen_static(cfg, i)
            if reachers[i] is not None:
                theta = (cands[i][cfg[i]][2] if len(cands[i][cfg[i]]) >= 3
                         else objs[i].get("angle", 0.0))
                hit = reachers[i].candidate_components(cands[i], static_all, theta)
            else:
                hit = dof1_candidate_components(objs[i], cands[i], static_all, samples1d)
            comp_cache[key] = hit
        return hit

    def neighbors(cfg):
        for i in range(len(objs)):
            labels = comp_labels(cfg, i)
            cur = labels[cfg[i]]
            if cur is None:
                continue
            for k, lab in enumerate(labels):
                if k == cfg[i] or lab != cur:
                    continue
                ncfg = cfg[:i] + (k,) + cfg[i + 1:]
                if config_valid(ncfg):
                    yield i, k, ncfg

    def h(cfg):
        return sum(1 for i in range(len(objs)) if cfg[i] != goal_cfg[i])

    # A*
    g_cost = {start_cfg: 0}
    parent = {start_cfg: None}
    pq = [(h(start_cfg), 0, start_cfg)]
    expanded = 0
    while pq:
        f, gc, cfg = heapq.heappop(pq)
        if cfg == goal_cfg:
            return _reconstruct(parent, cfg, names, cands), gc, len(pq), expanded, cands
        if gc > g_cost.get(cfg, math.inf):
            continue
        expanded += 1
        if expanded > max_expand:
            raise RuntimeError("A* exceeded max_expand=%d (raise it or trim candidates)"
                               % max_expand)
        for i, k, ncfg in neighbors(cfg):
            ng = gc + 1
            if ng < g_cost.get(ncfg, math.inf):
                g_cost[ncfg] = ng
                parent[ncfg] = (cfg, i, k)
                heapq.heappush(pq, (ng + h(ncfg), ng, ncfg))
    raise RuntimeError("no plan found within the candidate discretization "
                       "(scene may be infeasible, or candidates too coarse)")


def _find(cand_list, dof):
    for idx, c in enumerate(cand_list):
        if dof is not None and len(c) == len(dof) and all(abs(a - b) < 1e-6
                                                          for a, b in zip(c, dof)):
            return idx
    # dof not present (shouldn't happen: start/goal are always added first)
    cand_list.append(list(dof))
    return len(cand_list) - 1


def _reconstruct(parent, cfg, names, cands):
    steps = []
    while parent[cfg] is not None:
        pcfg, i, k = parent[cfg]
        steps.append((names[i], cands[i][k]))
        cfg = pcfg
    steps.reverse()
    return steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--cand-step", type=float, default=0.5)
    ap.add_argument("--grid", type=float, default=0.1)
    ap.add_argument("--samples1d", type=int, default=400)
    ap.add_argument("--max-expand", type=int, default=200000)
    ap.add_argument("--max-buffers", type=int, default=8)
    ap.add_argument("--no-prune", action="store_true",
                    help="keep the full candidate grid (finds near-lane stacking optima; small n only)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    with open(a.scene) as f:
        scene = json.load(f)
    plan, cost, _, expanded, cands = solve(scene, a.cand_step, a.grid, a.samples1d,
                                           a.max_expand, a.max_buffers, not a.no_prune)
    if a.json:
        print(json.dumps({"scene": scene.get("name"), "oracle_optimum": cost,
                          "plan": [[n, [round(x, 3) for x in d]] for n, d in plan],
                          "expanded": expanded,
                          "candidates_per_object": [len(c) for c in cands]}, indent=2))
    else:
        print("scene: %s" % scene.get("name"))
        print("candidates/object: %s" % [len(c) for c in cands])
        print("A* expanded: %d nodes" % expanded)
        print("ORACLE OPTIMUM: %d actions" % cost)
        for t, (n, d) in enumerate(plan, 1):
            print("  %2d. move %-6s -> %s" % (t, n, [round(x, 3) for x in d]))


if __name__ == "__main__":
    main()
