#!/usr/bin/env python3
"""Generalized park-once feasibility check for the sound-LB MRB refinement (T4).

Given a scene and its dependency-graph classification, decide whether a plan of
length equal to the FVS-based sound LB (the "park-once" plan: each object moves
exactly its base number of runs) is geometrically realizable. Generalizes
`swap_optimum_check` beyond the single-crosser reversal family:

  * ANY number of movers / crossers, and any interleaving of their moves -- via a
    BFS over "progress" states (state = how many of each object's base runs are
    done; ~prod(base+1) states, tiny), each transition gated by collision-free
    single-object reachability with the others frozen. Stacking order needs NO
    special handling: a bottom box trapped under a top box is simply unreachable
    upward in the grid flood.
  * BOXES (buffer poses = off-lane niche positions, incl. stacked levels) AND
    1-DOF DOORS / revolute / slider (buffer pose = an "open" value that clears a
    path; the object returns to its start value). So door_chain-style scenes are
    handled, not skipped.

Soundness: if NO assignment of buffer slots + ordering yields a valid length-LB
plan, then (provided the buffer-slot set is COMPLETE) no length-LB plan exists,
so the true optimum is > LB -- a sound +1 bump. Completeness holds for corridor /
niche / articulated scenes (finitely many buffer equivalence classes); it does
NOT hold for open rooms (buffers anywhere), so the caller must guard against those
(`has_structural_buffers`). This module only ever reports feasibility; the sound
bump decision lives in monotonicity.mrb_refine.

Returns feasible(bool). Standalone:
    python3 tools/park_once_check.py scenes/gen/swap_4_2.json
"""
import collections
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_puzzle import obb_corners, sat_overlap, obb_from_dof, obstacle_to_obb
from monotonicity import (object_dofs, build_graphs, analyse, SWEEP_SAMPLES,
                          must_move)

LANE_MARGIN = 0.4          # a buffer must sit this far (in y) off the start/goal row
MAX_SLOTS = 6              # cap buffer candidates per object (tractability)


def _ov(a, b):
    return sat_overlap(obb_corners(*a), a[4], obb_corners(*b), b[4])


def _box_reachable(obj, static, frm, to, h=0.06):
    """4-connected grid reachability for a non-rotating box, others frozen."""
    xb, yb = obj["xbounds"], obj["ybounds"]

    def free(x, y):
        return not any(_ov(obb_from_dof(obj, [x, y, 0.0]), s) for s in static)

    def cell(x, y):
        return (round((x - xb[0]) / h), round((y - yb[0]) / h))

    nx, ny = int((xb[1] - xb[0]) / h), int((yb[1] - yb[0]) / h)
    s, g = cell(frm[0], frm[1]), cell(to[0], to[1])
    if not free(frm[0], frm[1]) or not free(to[0], to[1]):
        return False
    seen = {s}
    dq = collections.deque([s])
    while dq:
        i, j = dq.popleft()
        if (i, j) == g:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni <= nx and 0 <= nj <= ny and (ni, nj) not in seen \
                    and free(xb[0] + ni * h, yb[0] + nj * h):
                seen.add((ni, nj))
                dq.append((ni, nj))
    return False


def _dof1_reachable(obj, static, frm, to, n=400):
    """Interval reachability for a 1-DOF object (door/revolute/slider)."""
    lo, hi = (obj.get("angle_bounds") or obj.get("disp_bounds") or [frm, frm])
    if hi <= lo:
        return abs(frm - to) < 1e-9

    def free(v):
        return not any(_ov(obb_from_dof(obj, [v]), s) for s in static)

    def k(v):
        return min(n, max(0, int(round((v - lo) / (hi - lo) * n))))

    kf = k(frm)
    if not free(lo + (hi - lo) * kf / n):
        return abs(frm - to) < 1e-9
    left = kf
    while left - 1 >= 0 and free(lo + (hi - lo) * (left - 1) / n):
        left -= 1
    right = kf
    while right + 1 <= n and free(lo + (hi - lo) * (right + 1) / n):
        right += 1
    return left <= k(to) <= right


def _reachable(obj, static, frm, to):
    if obj.get("type", "box") == "box":
        return _box_reachable(obj, static, frm, to)
    return _dof1_reachable(obj, static, frm[0] if isinstance(frm, (list, tuple)) else frm,
                           to[0] if isinstance(to, (list, tuple)) else to)


def _pose_dof(obj, pose):
    """Normalize a pose (xy for box, scalar for 1-DOF) to a full DOF vector."""
    if obj.get("type", "box") == "box":
        theta = obj["start"][2] if len(obj["start"]) >= 3 else obj.get("angle", 0.0)
        return [pose[0], pose[1], theta]
    return [pose if not isinstance(pose, (list, tuple)) else pose[0]]


def has_structural_buffers(scene):
    """Corridor/niche or articulated scene => buffer slots are a complete finite
    set (safe to bump). Open rooms (no floor stubs, no 1-DOF objects) => not."""
    objs = scene["objects"]
    if any(o.get("type", "box") in ("door", "revolute", "slider") for o in objs):
        return True
    # floor stub = obstacle resting on the ground that does NOT span full height
    world = scene["world"]
    lane_ys = [o["start"][1] for o in objs if o.get("type", "box") == "box"]
    lane_y = sum(lane_ys) / len(lane_ys) if lane_ys else world["ymax"] / 2
    for ob in scene.get("obstacles", []):
        if abs(ob["ymin"] - world["ymin"]) < 1e-6 and ob["ymax"] < lane_y - LANE_MARGIN \
                and (ob["xmax"] - ob["xmin"]) < (world["xmax"] - world["xmin"]) - 1e-6:
            return True          # a pocket/niche opening exists above this stub
    return False


def _buffer_slots(obj, obst, scene):
    """Candidate buffer poses for a buffered object (few, structural).

    For a box: at each y-level BELOW the lane (down to the niche floor, one per
    ~box height so stacking levels are represented), finely scan x and take the
    CENTRE of every maximal collision-free x-interval -- i.e. one slot per niche
    opening per level. This lands a slot squarely inside each niche (a coarse
    uniform grid can miss the ~box-wide free band entirely)."""
    t = obj.get("type", "box")
    if t == "box":
        xb, yb = obj["xbounds"], obj["ybounds"]
        lane_y = obj["start"][1]
        y_hi = lane_y - LANE_MARGIN
        step = 2 * obj["hy"]                         # one box height => bottom + stack levels
        slots = []
        yl = yb[0]
        while yl <= y_hi + 1e-9:
            xs = []
            x = xb[0]
            while x <= xb[1] + 1e-9:
                if not any(_ov(obb_from_dof(obj, [x, yl, 0.0]), o) for o in obst):
                    xs.append(x)
                x += 0.1
            if xs:                                  # group into maximal intervals
                grp = [xs[0]]
                for xv in xs[1:]:
                    if xv - grp[-1] <= 0.15:
                        grp.append(xv)
                    else:
                        slots.append((sum(grp) / len(grp), yl))
                        grp = [xv]
                slots.append((sum(grp) / len(grp), yl))
            yl += step
        return slots[:MAX_SLOTS * 3]                # generous cap (few per niche)
    # 1-DOF: sample "open" values (the object returns to start), keep collision-free
    lo, hi = (obj.get("angle_bounds") or obj.get("disp_bounds") or [0, 0])
    start_v = object_dofs(obj)[0][0]
    slots = []
    K = 10
    for i in range(K + 1):
        v = lo + (hi - lo) * i / K
        if abs(v - start_v) < 1e-6:
            continue
        if not any(_ov(obb_from_dof(obj, [v]), o) for o in obst):
            slots.append(v)
    # keep a spread of at most MAX_SLOTS
    if len(slots) > MAX_SLOTS:
        step = len(slots) / MAX_SLOTS
        slots = [slots[int(i * step)] for i in range(MAX_SLOTS)]
    return slots


def _templates(scene, movers, forced, buffered_movers):
    """Per-object (template_poses, buffered) for the park-once plan, given which
    movers are the ones that buffer (`buffered_movers`).

    template_poses: list of target poses AFTER start (a 'BUF' entry means the
    variable buffer slot). Total runs of v = len(template)."""
    objs = {o["name"]: o for o in scene["objects"]}
    buffered_movers = set(buffered_movers)
    tpl = {}
    for name, o in objs.items():
        s, g = object_dofs(o)
        spose = _startpose(o, s)
        gpose = _startpose(o, g) if g is not None else spose
        if name in movers:
            if name in buffered_movers:
                tpl[name] = (["BUF", gpose], True)      # start -> buffer -> goal
            else:
                tpl[name] = ([gpose], False)            # start -> goal
        elif name in forced:
            if g is not None:                            # parked target: out and back
                tpl[name] = (["BUF", spose], True)
            else:                                        # free blocker: aside, stay
                tpl[name] = (["BUF"], True)
        else:
            tpl[name] = ([], False)                      # never moves
    return tpl


def _startpose(obj, dof):
    if obj.get("type", "box") == "box":
        return (dof[0], dof[1])
    return dof[0]


def feasible(scene, movers, forced, fvs, verbose=False):
    """True iff a length-(sound LB) park-once plan is geometrically realizable.

    The FVS fixes only the NUMBER of movers that must buffer (q); WHICH movers
    buffer is a choice (any q-subset), so we try them all -- feasible if ANY
    choice of buffered movers admits a valid plan."""
    q = len([v for v in fvs if v in movers])
    for buffered_movers in itertools.combinations(movers, q):
        if _feasible_templates(scene, movers, forced, set(buffered_movers)):
            return True
    return False


def _feasible_templates(scene, movers, forced, buffered_movers):
    objs = {o["name"]: o for o in scene["objects"]}
    obst = [obstacle_to_obb(o) for o in scene["obstacles"]]
    tpl = _templates(scene, movers, forced, buffered_movers)
    buffered = [n for n in objs if tpl[n][1]]
    slot_opts = {n: _buffer_slots(objs[n], obst, scene) for n in buffered}
    if any(len(slot_opts[n]) == 0 for n in buffered):
        return False          # a buffered object has no valid buffer slot

    names = list(objs.keys())
    order = names            # fixed index order for state tuples
    maxp = {n: len(tpl[n][0]) for n in names}

    def resolve(name, assign):
        """Concrete pose list for `name` given a buffer-slot assignment."""
        poses = []
        for p in tpl[name][0]:
            poses.append(_pose_dof(objs[name], assign[name]) if p == "BUF"
                         else _pose_dof(objs[name], p))
        return poses

    # reachability cache shared across assignments/states: a move's validity
    # depends only on the moving object, its from/to poses, and where the OTHERS
    # currently are -- which recurs heavily across the search.
    rcache = {}

    def _pk(p):
        return (round(p[0], 2), round(p[1], 2) if len(p) > 1 else 0.0)

    def rc(name, frm, to, cur):
        others = tuple(sorted((m, _pk(cur[m])) for m in order if m != name))
        key = (name, _pk(frm), _pk(to), others)
        if key in rcache:
            return rcache[key]
        static = obst + [obb_from_dof(objs[m], cur[m]) for m in order if m != name]
        r = _reachable(objs[name], static, frm, to)
        rcache[key] = r
        return r

    # iterate buffer-slot assignments (Cartesian product over buffered objects)
    for combo in itertools.product(*[slot_opts[n] for n in buffered]):
        assign = dict(zip(buffered, combo))
        resolved = {n: resolve(n, assign) for n in names}
        start_pose = {n: _pose_dof(objs[n], _startpose(objs[n], object_dofs(objs[n])[0]))
                      for n in names}

        def pose_at(name, prog):
            return start_pose[name] if prog == 0 else resolved[name][prog - 1]

        start_state = tuple(0 for _ in order)
        goal_state = tuple(maxp[n] for n in order)
        seen = {start_state}
        dq = collections.deque([start_state])
        ok = False
        while dq:
            st = dq.popleft()
            if st == goal_state:
                ok = True
                break
            cur = {order[i]: pose_at(order[i], st[i]) for i in range(len(order))}
            for i, name in enumerate(order):
                if st[i] >= maxp[name]:
                    continue
                frm = cur[name]
                to = pose_at(name, st[i] + 1)
                if rc(name, frm, to, cur):
                    ns = list(st)
                    ns[i] += 1
                    ns = tuple(ns)
                    if ns not in seen:
                        seen.add(ns)
                        dq.append(ns)
        if ok:
            return True
    return False


def check_scene(scene):
    names = [o["name"] for o in scene["objects"]]
    info, ep_arcs, _sw, _w = build_graphs(scene, SWEEP_SAMPLES)
    res = analyse(names, info, ep_arcs)
    return res["action_lower_bound"], feasible(scene, res["movers"],
                                               res["forced_blockers"], res["fvs"])


def main():
    import json
    with open(sys.argv[1]) as f:
        scene = json.load(f)
    lb, feas = check_scene(scene)
    print("scene: %s  sound LB=%d  structural-buffers=%s" %
          (scene.get("name"), lb, has_structural_buffers(scene)))
    print("park-once plan feasible: %s%s" %
          (feas, "" if feas else "  => sound LB is LOOSE (+1 sound bump)"))


if __name__ == "__main__":
    main()
