#!/usr/bin/env python3
"""A-priori monotone / non-monotone classifier for 2D rearrangement scenes.

Implements the dependency-graph recipe of research/01_monotone_vs_nonmonotone.md §5
and the tooling plan of research/SCENARIOS.md:

  1. Build the OBB dependency graph from start/goal (and optionally swept-volume)
     overlaps, using the same SAT geometry as validate_puzzle.py.
  2. Tarjan SCC: DAG => "monotone-plausible" (+ topological one-shot order);
     cycle => "non-monotone: buffer required" (+ SCCs + exact minimum FVS).
  3. Derive a structural lower bound on the action count and, when a solution
     JSON is supplied, bracket it against the planner's (upper-bound) count (Q5).

Two graphs are reported:
  * endpoint graph  — arcs only from goal/start endpoint overlaps. Under the
    idealized pick-place model these are NECESSARY ordering constraints, so a
    cycle here soundly certifies non-monotonicity and the lower bound holds.
  * swept graph     — endpoint arcs plus arcs from the straight-line transfer
    sweep of each mover. Better predictor of what a straight-line planner must
    do, but can flag phantom cycles a curved transfer would avoid (incomplete).

Caveats (research/01_...md §5 "Why this cheap test can be WRONG") apply: no arm
model, straight-line sweeps, endpoint OBBs only, no buffer-existence check.

Usage:
    python3 monotonicity.py <scene.json> [--solution <solution.json>]
                            [--samples N] [--json]
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_puzzle import (obb_corners, sat_overlap, obb_from_dof,
                             obstacle_to_obb)

MOVE_TOL = 1e-6          # DOF delta below which an object "does not move"
SWEEP_SAMPLES = 64       # interior samples along a straight-line transfer


# ---------------------------------------------------------------------------
# Scene parsing: per-object DOF vectors (schema of scenes/*.json, mirroring
# the driver's parsing in LARRT2D_Puzzle.cpp)
# ---------------------------------------------------------------------------

def object_dofs(obj):
    """Return (start_dof, goal_dof_or_None) in obb_from_dof conventions.

    Boxes always get a 3-vector [x, y, theta] (theta fixed from start[2] when
    the box does not rotate); door/revolute -> [angle]; slider -> [disp].
    """
    t = obj.get("type", "box")
    target = obj.get("target", False)
    if t == "box":
        s = list(obj["start"])
        if len(s) < 3:
            s.append(obj.get("angle", 0.0))
        g = None
        if target:
            g = list(obj["goal"])
            if len(g) < 3:
                g.append(s[2])          # non-rotating box keeps its angle
        return s, g
    if t in ("door", "revolute"):
        s = [obj["start_angle"]]
        g = [obj["goal_angle"]] if target else None
        return s, g
    if t == "slider":
        s = [obj["start_disp"]]
        g = [obj["goal_disp"]] if target else None
        return s, g
    raise ValueError("unknown object type: %r" % t)


def obb_at(obj, dof):
    cx, cy, hx, hy, th = obb_from_dof(obj, dof)
    return (obb_corners(cx, cy, hx, hy, th), th)


def overlaps(A, B):
    """A, B are (corners, theta) pairs."""
    return sat_overlap(A[0], A[1], B[0], B[1])


def must_move(s, g):
    return g is not None and any(abs(a - b) > MOVE_TOL for a, b in zip(s, g))


# ---------------------------------------------------------------------------
# Graph machinery
# ---------------------------------------------------------------------------

def tarjan_sccs(nodes, adj):
    """Tarjan strongly connected components. Returns list of node lists."""
    index = {}
    low = {}
    onstack = {}
    stack = []
    sccs = []
    counter = [0]

    def strongconnect(v):
        index[v] = low[v] = counter[0]
        counter[0] += 1
        stack.append(v)
        onstack[v] = True
        for w in adj.get(v, ()):
            if w not in index:
                strongconnect(w)
                low[v] = min(low[v], low[w])
            elif onstack.get(w):
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            comp = []
            while True:
                w = stack.pop()
                onstack[w] = False
                comp.append(w)
                if w == v:
                    break
            sccs.append(comp)

    for v in nodes:
        if v not in index:
            strongconnect(v)
    return sccs


def is_acyclic(nodes, adj):
    sccs = tarjan_sccs(nodes, adj)
    return all(len(c) == 1 for c in sccs) and \
        not any(v in adj.get(v, ()) for v in nodes)


def topo_order(nodes, adj):
    """Kahn topological order; assumes acyclic. Arc u->v = u before v."""
    indeg = {v: 0 for v in nodes}
    for u in nodes:
        for v in adj.get(u, ()):
            indeg[v] += 1
    queue = sorted(v for v in nodes if indeg[v] == 0)
    order = []
    while queue:
        u = queue.pop(0)
        order.append(u)
        for v in sorted(adj.get(u, ())):
            indeg[v] -= 1
            if indeg[v] == 0:
                queue.append(v)
    return order


def min_fvs(scc, adj, cost):
    """Minimum-COST feedback vertex set of the subgraph induced by `scc`.

    cost[v] is the extra actions buffering v implies; minimizing total cost
    (not cardinality) keeps the derived action bound a true lower bound.
    Brute force over subsets — fine for the handful of objects our scenes have.
    """
    sub = {u: [v for v in adj.get(u, ()) if v in scc] for u in scc}
    if is_acyclic(scc, sub):
        return []
    from itertools import combinations
    best, best_cost = list(scc), sum(cost[v] for v in scc)
    for k in range(1, len(scc) + 1):
        for cand in combinations(sorted(scc), k):
            c = sum(cost[v] for v in cand)
            if c >= best_cost:
                continue
            rem = [v for v in scc if v not in cand]
            radj = {u: [v for v in sub[u] if v not in cand] for u in rem}
            if is_acyclic(rem, radj):
                best, best_cost = list(cand), c
    return best


# ---------------------------------------------------------------------------
# Sound blocking arcs via free-space disconnection
#
# Freeze ONE pose of object B as an extra obstacle, remove every other movable
# (removal only enlarges free space), and test whether mover A's start and goal
# are still connected in A's own DOF space. If not, EVERY path of A must wait
# for / precede B at that pose => a sound ordering arc, valid for curved
# transfers too (unlike the straight-line swept heuristic).
#
# Soundness of the grid test (2-DOF box movers): a cell is free only if the
# box SHRUNK by the cell half-diagonal delta = h*sqrt(2)/2 is collision-free at
# the cell centre. A true pose inside the cell displaces the shrunk box by at
# most delta, so shrunk-at-centre is contained in full-at-pose: every truly
# free pose lies in a free cell, and a continuous path induces an 8-connected
# free-cell chain. Grid disconnection therefore PROVES C-space disconnection
# (grid connection proves nothing). 1-DOF movers use a fine interval sweep
# (sound up to angular/displacement resolution; ~mm tip motion per step).
# ---------------------------------------------------------------------------

DISC_GRID = 0.05          # box-mover grid step
DISC_STEPS_1D = 2000      # samples across a 1-DOF mover's start->goal interval


class MoverGrid:
    """Over-approximated free-space grid of one 2-DOF box mover."""

    def __init__(self, obj, start_dof, goal_dof, obst_obbs, h):
        self.ok = obj.get("type", "box") == "box" and not obj.get("rotates", False)
        if not self.ok:
            return
        self.h = h
        self.delta = h * math.sqrt(2.0) / 2.0
        self.hx = obj["hx"] - self.delta
        self.hy = obj["hy"] - self.delta
        self.theta = start_dof[2]
        if self.hx <= 0 or self.hy <= 0:
            self.ok = False
            return
        self.xlo, self.xhi = obj["xbounds"]
        self.ylo, self.yhi = obj["ybounds"]
        self.nx = max(2, int(round((self.xhi - self.xlo) / h)) + 1)
        self.ny = max(2, int(round((self.yhi - self.ylo) / h)) + 1)
        self.start = self._cell(start_dof[0], start_dof[1])
        self.goal = self._cell(goal_dof[0], goal_dof[1])
        # base free grid vs static obstacles only (computed once per mover)
        self.base = [[self._free_at(i, j, obst_obbs)
                      for j in range(self.ny)] for i in range(self.nx)]

    def _cell(self, x, y):
        i = min(self.nx - 1, max(0, int(round((x - self.xlo) / self.h))))
        j = min(self.ny - 1, max(0, int(round((y - self.ylo) / self.h))))
        return (i, j)

    def _centre(self, i, j):
        return (self.xlo + i * self.h, self.ylo + j * self.h)

    def _free_at(self, i, j, obbs):
        cx, cy = self._centre(i, j)
        me = (obb_corners(cx, cy, self.hx, self.hy, self.theta), self.theta)
        return not any(overlaps(me, ob) for ob in obbs)

    def disconnected(self, frozen):
        """True iff start and goal are provably disconnected with `frozen`
        ((corners, theta)) added as an obstacle."""
        if not self.ok:
            return False
        # only cells near the frozen OBB need rechecking against it
        xs = [c[0] for c in frozen[0]]
        ys = [c[1] for c in frozen[0]]
        margin = math.hypot(self.hx, self.hy) + self.delta + self.h
        ilo, _ = self._cell(min(xs) - margin, 0)
        ihi, _ = self._cell(max(xs) + margin, 0)
        _, jlo = self._cell(0, min(ys) - margin)
        _, jhi = self._cell(0, max(ys) + margin)

        def free(i, j):
            if not self.base[i][j]:
                return False
            if ilo <= i <= ihi and jlo <= j <= jhi:
                return self._free_at(i, j, [frozen])
            return True

        if not free(*self.start) or not free(*self.goal):
            return False        # cannot prove anything from a blocked endpoint
        # BFS, 8-connected
        from collections import deque
        seen = [[False] * self.ny for _ in range(self.nx)]
        dq = deque([self.start])
        seen[self.start[0]][self.start[1]] = True
        while dq:
            i, j = dq.popleft()
            if (i, j) == self.goal:
                return False    # connected -> no proof of blocking
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ii, jj = i + di, j + dj
                    if 0 <= ii < self.nx and 0 <= jj < self.ny and \
                            not seen[ii][jj] and free(ii, jj):
                        seen[ii][jj] = True
                        dq.append((ii, jj))
        return True


class Mover1D:
    """Interval sweep of a 1-DOF mover (door / revolute / slider).

    Sound because a 1-DOF path from a to b within joint limits must visit every
    value in [min(a,b), max(a,b)] — UNLESS the joint's bounds span a full circle
    (the path could wrap the other way), in which case we refuse to certify.
    """

    def __init__(self, obj, start_dof, goal_dof, obst_obbs):
        t = obj.get("type")
        self.ok = t in ("door", "revolute", "slider")
        if not self.ok:
            return
        if t in ("door", "revolute"):
            lo, hi = obj["angle_bounds"]
            if hi - lo >= 2.0 * math.pi - 1e-6:
                self.ok = False      # wrap-around possible -> interval not forced
                return
        a, b = start_dof[0], goal_dof[0]
        n = DISC_STEPS_1D
        self.obbs = [obb_at(obj, [a + (b - a) * k / n]) for k in range(n + 1)]
        # blocked by statics alone => scene relaxation already infeasible; the
        # disconnection is then not attributable to any single blocker B.
        self.base_blocked = any(
            any(overlaps(ob, o) for o in obst_obbs) for ob in self.obbs[1:-1])

    def disconnected(self, frozen):
        if not self.ok or self.base_blocked:
            return False
        if overlaps(self.obbs[0], frozen) or overlaps(self.obbs[-1], frozen):
            return False             # endpoint overlap: handled by endpoint arcs
        return any(overlaps(ob, frozen) for ob in self.obbs[1:-1])


def make_mover_grid(name, info, obst_obbs, h, warnings):
    ia = info[name]
    obj = ia["obj"]
    t = obj.get("type", "box")
    if t == "box":
        if obj.get("rotates", False):
            warnings.append("disconnection test skipped for rotating box %s "
                            "(3-DOF grid not implemented)" % name)
            return None
        g = MoverGrid(obj, ia["start_dof"], ia["goal_dof"], [o for _, o in obst_obbs], h)
        return g if g.ok else None
    g = Mover1D(obj, ia["start_dof"], ia["goal_dof"], [o for _, o in obst_obbs])
    return g if g.ok else None


# ---------------------------------------------------------------------------
# Dependency-graph construction
# ---------------------------------------------------------------------------

def build_graphs(scene, samples, disc_grid=DISC_GRID, use_disc=True):
    """Return (info, sound_arcs, swept_arcs, warnings).

    sound arcs = endpoint arcs + disconnection arcs (both are necessary
    ordering constraints under the pick-place model => usable for the Q5
    certificate). swept arcs = endpoint + straight-line-sweep heuristic.

    Arc (u, v, reason) means "u must move before v acts/is placed".
    endpoint arcs:  start(B) ∩ goal(A)  =>  B -> A   (B vacates before A lands)
    swept arcs add: start(B) ∩ sweep(A) =>  B -> A   (B blocks A's transfer)
                    goal(B)  ∩ sweep(A) =>  A -> B   (A crosses B's goal spot)
    """
    objects = scene["objects"]
    obstacles = scene.get("obstacles", [])
    obst_obbs = []
    for i, ob in enumerate(obstacles):
        cx, cy, hx, hy, th = obstacle_to_obb(ob)
        obst_obbs.append((i, (obb_corners(cx, cy, hx, hy, th), th)))

    info = {}
    for obj in objects:
        s, g = object_dofs(obj)
        info[obj["name"]] = {
            "obj": obj,
            "start_dof": s,
            "goal_dof": g,
            "start_obb": obb_at(obj, s),
            "goal_obb": obb_at(obj, g) if g is not None else None,
            "moves": must_move(s, g),
        }

    warnings = []
    names = [o["name"] for o in objects]

    # scene sanity
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = info[names[i]], info[names[j]]
            if overlaps(a["start_obb"], b["start_obb"]):
                warnings.append("starts of %s and %s overlap (invalid scene?)"
                                % (names[i], names[j]))
            if a["goal_obb"] and b["goal_obb"] and \
                    overlaps(a["goal_obb"], b["goal_obb"]):
                warnings.append("goals of %s and %s overlap (infeasible?)"
                                % (names[i], names[j]))

    endpoint = []
    for a in names:                       # a = the mover being placed
        ga = info[a]["goal_obb"]
        if ga is None:
            continue
        for b in names:
            if b == a:
                continue
            if overlaps(ga, info[b]["start_obb"]):
                endpoint.append((b, a, "start(%s) blocks goal(%s)" % (b, a)))

    swept = list(endpoint)
    for a in names:
        ia = info[a]
        if not ia["moves"]:
            continue
        s, g = ia["start_dof"], ia["goal_dof"]
        sweep = []
        for k in range(1, samples):
            t = k / samples
            sweep.append(obb_at(ia["obj"],
                                [s[d] + t * (g[d] - s[d]) for d in range(len(s))]))
        hit_start = set(u for (u, v, _) in endpoint if v == a)
        hit_goal = set()
        hit_obst = set()
        for smp in sweep:
            for b in names:
                if b == a:
                    continue
                if b not in hit_start and overlaps(smp, info[b]["start_obb"]):
                    hit_start.add(b)
                    swept.append((b, a,
                                  "start(%s) blocks transfer sweep of %s" % (b, a)))
                gb = info[b]["goal_obb"]
                if gb is not None and b not in hit_goal and overlaps(smp, gb):
                    hit_goal.add(b)
                    swept.append((a, b,
                                  "transfer sweep of %s crosses goal(%s)" % (a, b)))
            for i, ob in obst_obbs:
                if i not in hit_obst and overlaps(smp, ob):
                    hit_obst.add(i)
        if hit_obst:
            warnings.append(
                "straight transfer of %s hits obstacle(s) %s - real transfer "
                "must curve; swept arcs involving %s may be spurious"
                % (a, sorted(hit_obst), a))

    # sound arcs: endpoint + provable disconnection arcs
    sound = list(endpoint)
    if use_disc:
        for a in names:
            ia = info[a]
            if not ia["moves"]:
                continue
            grid = make_mover_grid(a, info, obst_obbs, disc_grid, warnings)
            if grid is None:
                continue
            for b in names:
                if b == a:
                    continue
                ib = info[b]
                blocks_start = grid.disconnected(ib["start_obb"])
                if blocks_start:
                    sound.append((b, a, "start(%s) disconnects %s start->goal "
                                        "(provable: must clear first)" % (b, a)))
                if ib["goal_obb"] is not None:
                    same_pose = (ib["goal_dof"] == ib["start_dof"])
                    blocks_goal = blocks_start if same_pose \
                        else grid.disconnected(ib["goal_obb"])
                    if blocks_goal:
                        sound.append((a, b, "goal(%s) disconnects %s start->goal "
                                            "(provable: %s must finish first)"
                                     % (b, a, a)))

    # dedupe arcs (keep first reason)
    def dedupe(arcs):
        seen = {}
        for u, v, r in arcs:
            seen.setdefault((u, v), r)
        return [(u, v, r) for (u, v), r in seen.items()]

    return info, dedupe(sound), dedupe(swept), warnings


# ---------------------------------------------------------------------------
# Analysis of one graph
# ---------------------------------------------------------------------------

def analyse(names, info, arcs):
    adj = {}
    for u, v, _ in arcs:
        adj.setdefault(u, []).append(v)

    sccs = [c for c in tarjan_sccs(names, adj)]
    cyclic = [sorted(c) for c in sccs
              if len(c) > 1 or (c[0] in adj.get(c[0], ()))]

    movers = [n for n in names if info[n]["moves"]]
    # a non-target (or non-moving) object with an outgoing arc must still clear
    forced = [n for n in names if not info[n]["moves"] and adj.get(n)]
    forced_set = set(forced)

    # BASE cost = the minimum number of runs each object must make in ANY valid
    # solution (a sound per-object lower bound):
    #   * mover (start != goal)                              -> 1 (>=1 run)
    #   * forced blocker that is a PARKED TARGET (start==goal
    #     but pinned to return) -> 2: a disconnection/ordering
    #     arc proves it must leave its pose, and start==goal
    #     forces it to come back  => >=2 runs (out AND back)
    #   * forced blocker that is a FREE object (no goal)      -> 1 (move aside,
    #     may stay)
    #   * otherwise                                           -> 0
    # This fixes a looseness: the old model charged forced blockers only +1 and
    # relied on a +2 FVS surcharge to catch the out-and-back, but the min-COST
    # FVS dodged it by buffering a cheaper shared mover instead (e.g. blocked_
    # goal_chain_k / door_chain_d gave LB k+2 / d+2 instead of the true 2k+1 /
    # 2d+1). The out-and-back of a parked target is mandatory, so it belongs in
    # the base, not in an avoidable surcharge.
    def base_cost(n):
        if info[n]["moves"]:
            return 1
        if n in forced_set:
            return 2 if info[n]["goal_dof"] is not None else 1
        return 0
    base = {n: base_cost(n) for n in names}

    # FVS surcharge = the EXTRA runs (beyond base) that buffering v costs when v
    # is chosen to break a dependency cycle. Only a mover pays it (base 1 -> 2
    # runs = +1); a parked target already double-moves in its base, and a free
    # blocker's single aside-move already is the buffering  => both cost +0. So
    # min_fvs happily includes the free/parked vertices to break cycles for 0.
    fvs_extra = {n: (1 if info[n]["moves"] else 0) for n in names}
    fvs = []
    for c in cyclic:
        fvs.extend(min_fvs(c, adj, fvs_extra))

    # lower bound: mandatory per-object runs + the extra buffer visits that a
    # minimum-cost feedback vertex set forces on the movers.
    lb = sum(base.values()) + sum(fvs_extra[v] for v in fvs)

    res = {
        "arcs": arcs,
        "cyclic_sccs": cyclic,
        "monotone_plausible": not cyclic,
        "fvs": sorted(fvs),
        "movers": movers,
        "forced_blockers": forced,
        "action_lower_bound": lb,
    }
    if not cyclic:
        res["topo_order"] = [n for n in topo_order(names, adj)
                             if n in movers or n in forced]
    return res


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("scene")
    ap.add_argument("--solution", help="driver solution JSON to bracket (Q5)")
    ap.add_argument("--samples", type=int, default=SWEEP_SAMPLES)
    ap.add_argument("--grid", type=float, default=DISC_GRID,
                    help="grid step for the disconnection proof (default %g)" % DISC_GRID)
    ap.add_argument("--no-disc", action="store_true",
                    help="skip disconnection arcs (endpoint-only sound graph)")
    ap.add_argument("--json", action="store_true", dest="as_json")
    args = ap.parse_args(argv[1:])

    with open(args.scene) as f:
        scene = json.load(f)
    names = [o["name"] for o in scene["objects"]]

    info, ep_arcs, sw_arcs, warnings = build_graphs(
        scene, args.samples, disc_grid=args.grid, use_disc=not args.no_disc)
    ep = analyse(names, info, ep_arcs)
    sw = analyse(names, info, sw_arcs)

    upper = None
    if args.solution:
        with open(args.solution) as f:
            sol = json.load(f)
        upper = sol.get("action_count") if sol.get("solved") else None

    if args.as_json:
        out = {"scene": scene.get("name"), "warnings": warnings,
               "sound": ep, "swept": sw, "planner_action_count": upper}
        json.dump(out, sys.stdout, indent=2)
        print()
        return 0

    print("scene: %s   objects: %s" % (scene.get("name"), ", ".join(names)))
    for w in warnings:
        print("WARNING: %s" % w)

    for label, res in (("sound graph (endpoint + disconnection arcs)", ep),
                       ("swept graph (straight-line transfer predictor)", sw)):
        print("\n== %s ==" % label)
        if res["arcs"]:
            for u, v, r in sorted(res["arcs"]):
                print("  %-6s -> %-6s  [%s]" % (u, v, r))
        else:
            print("  (no arcs)")
        if res["monotone_plausible"]:
            print("  VERDICT: monotone-plausible (DAG)")
            print("  one-shot order: %s" % " -> ".join(res["topo_order"]))
        else:
            print("  VERDICT: NON-MONOTONE (buffer required)")
            print("  cyclic SCCs: %s" % res["cyclic_sccs"])
            print("  min feedback vertex set: %s  (>=%d object(s) must buffer)"
                  % (res["fvs"], len(res["fvs"])))
        print("  movers: %s%s" % (res["movers"],
              ("  forced blockers: %s" % res["forced_blockers"])
              if res["forced_blockers"] else ""))
        print("  action lower bound: %d" % res["action_lower_bound"])

    if upper is not None:
        lb = ep["action_lower_bound"]
        print("\n== Q5 bracket ==")
        print("  structural lower bound (sound graph):    %d" % lb)
        print("  planner action count (upper bound):      %d" % upper)
        if upper == lb:
            print("  bounds MEET -> action count %d is CERTIFIED optimal "
                  "(under the pick-place OBB model)" % upper)
        else:
            print("  gap = %d -> planner may be suboptimal (or the model "
                  "under-counts)" % (upper - lb))
    elif args.solution:
        print("\nsolution %s is unsolved; no upper bound." % args.solution)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
