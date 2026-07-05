#!/usr/bin/env python3
"""Does a `swap_k_m` reversal admit the 2k-1 action 'park-once' optimum?

For a k-box single-file reversal, the ONLY way to reach the sound lower bound
2k-1 is the park-once structure: the crosser A moves once (start->goal) while the
other k-1 movers each park in a niche once and then go to their goal (2 each), and
all k-1 must be parked SIMULTANEOUSLY (each blocks A's lane). This exhaustively
searches that structure -- every assignment of movers to niche slots (bottom
y=1.75 or stacked top y=2.5 per niche) and every park/unpark ordering, respecting
the stacking constraints (bottom parked before top; top unparked before bottom) --
checking each move's collision-free single-object reachability (grid flood, others
frozen).

Result:
  * FOUND  => a 2k-1 plan exists => optimum = 2k-1 = sound LB (LB tight).
  * NONE   => no 2k-1 plan => optimum > 2k-1 => the niche-count-independent sound
              LB is LOOSE under buffer scarcity (the MRB phenomenon; motivates T4).

Validated: FINDS the 7-plan for swap_4_3 (3 niches, oracle-confirmed optimum 7);
finds NONE for swap_4_2 (2 niches) => swap_4_2 optimum >= 8 > LB 7.

Usage (from demos/larrt2d):
    python3 tools/swap_optimum_check.py scenes/gen/swap_4_2.json [--grid 0.06]
"""
import argparse
import collections
import itertools
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from validate_puzzle import obb_corners, sat_overlap, obb_from_dof, obstacle_to_obb

NICHE_BOTTOM_Y = 1.75
NICHE_TOP_Y = 2.5           # "stacked" level (deep niches only)


def _ov(a, b):
    return sat_overlap(obb_corners(*a), a[4], obb_corners(*b), b[4])


def _reachable(obj, obst, frm, to, frozen, h):
    """4-connected grid reachability for a non-rotating box, others frozen."""
    xb, yb = obj["xbounds"], obj["ybounds"]
    stat = obst + frozen

    def free(x, y):
        o = obb_from_dof(obj, [x, y, 0.0])
        return not any(_ov(o, s) for s in stat)

    def cell(x, y):
        return (round((x - xb[0]) / h), round((y - yb[0]) / h))

    nx = int((xb[1] - xb[0]) / h)
    ny = int((yb[1] - yb[0]) / h)
    s, g = cell(*frm), cell(*to)
    if not free(*frm) or not free(*to):
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


def check(scene, grid_h=0.06):
    objs = {o["name"]: o for o in scene["objects"]}
    obst = [obstacle_to_obb(o) for o in scene["obstacles"]]
    niches = sorted(round(o["xmin"] + (o["xmax"] - o["xmin"]) / 2, 3)
                    for o in scene["obstacles"]
                    if o["ymin"] == 0.0 and o["ymax"] < 1.5)
    st = {n: (objs[n]["start"][0], objs[n]["start"][1]) for n in objs}
    gl = {n: (objs[n]["goal"][0], objs[n]["goal"][1]) for n in objs}
    crosser = max(objs, key=lambda n: abs(gl[n][0] - st[n][0]))
    movers = [n for n in objs if n != crosser]

    slots = {}
    for nx_ in niches:
        slots[(nx_, "b")] = (nx_, NICHE_BOTTOM_Y)
        slots[(nx_, "t")] = (nx_, NICHE_TOP_Y)

    def obb(n, x, y):
        return obb_from_dof(objs[n], [x, y, 0.0])

    def valid(assign, po, uo):
        pos = dict(st)
        seq = [(w, slots[assign[w]]) for w in po] + [(crosser, gl[crosser])] \
            + [(w, gl[w]) for w in uo]
        for nx_ in niches:
            bs = [m for m in movers if assign[m] == (nx_, "b")]
            ts = [m for m in movers if assign[m] == (nx_, "t")]
            if ts and not bs:
                return False
            if bs and ts:
                if po.index(bs[0]) > po.index(ts[0]):
                    return False
                if uo.index(ts[0]) > uo.index(bs[0]):
                    return False
        for w, to in seq:
            frozen = [obb(n, *p) for n, p in pos.items() if n != w]
            if not _reachable(objs[w], obst, pos[w], to, frozen, grid_h):
                return False
            pos[w] = to
        return all(abs(pos[x][0] - gl[x][0]) < 1e-6 and abs(pos[x][1] - gl[x][1]) < 1e-6
                   for x in pos)

    for combo in itertools.permutations(slots.keys(), len(movers)):
        assign = dict(zip(movers, combo))
        for po in itertools.permutations(movers):
            for uo in itertools.permutations(movers):
                if valid(assign, po, uo):
                    return True, (assign, po, uo)
    return False, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--grid", type=float, default=0.06)
    a = ap.parse_args()
    with open(a.scene) as f:
        scene = json.load(f)
    k = len(scene["objects"])
    found, plan = check(scene, a.grid)
    print("scene: %s  (k=%d, target 2k-1=%d)" % (scene.get("name"), k, 2 * k - 1))
    if found:
        print("2k-1 PARK-ONCE PLAN: FOUND  => optimum = %d = sound LB (TIGHT)" % (2 * k - 1))
    else:
        print("2k-1 PARK-ONCE PLAN: NONE   => optimum > %d => sound LB is LOOSE "
              "(buffer scarcity / MRB; motivates T4)" % (2 * k - 1))


if __name__ == "__main__":
    main()
