#!/usr/bin/env python3
"""Parameterized scene-family generator for the LA-RRT 2D benchmark (HANDOFF T1).

Emits schema-correct scene JSON (see research/HANDOFF.md §2.3) for four families,
three of which have a *derivable* action optimum that we can cross-check against
the sound lower bound from tools/monotonicity.py:

  swap_k_m            reverse k single-file boxes using m side niches.
                      Generalizes buffer_swap (k=2,m=1) and two_buffer (k=3,m=2).
                      CONJECTURED optimum 2k-1 (needs m>=k-1 niches); verified
                      against the sound LB for small k -- do NOT trust blindly.

  door_chain_d        d swing doors in series, each goal_angle==start_angle
                      (must be re-closed). One box A crosses all of them.
                      PROVEN optimum 2d+1 (open all, cross once, close all);
                      generalizes door_relock (d=1).

  blocked_goal_chain_k  k parked boxes (start==goal) sitting on A's only path,
                      each with its own niche to dip into. A crosses once.
                      PROVEN optimum 2k+1; generalizes blocked_goal (k=1).

  random_room         n boxes with random non-overlapping start/goal in an open
                      walled room (seeded). No closed form -> exact oracle (T5b).

The three structured families reproduce the geometry conventions of the anchor
scenes exactly: box half-size 0.35, corridor lane y in [2.5,3.6], niche opening
width 1.4 with a stub floor at y=1.4, wall thickness 0.3, world height 6.0.

Usage:
    python3 gen_scenes.py swap        --k 3 --m 2 [-o out.json]
    python3 gen_scenes.py door_chain  --d 2       [-o out.json]
    python3 gen_scenes.py blocked     --k 2       [-o out.json]
    python3 gen_scenes.py random_room --n 4 --seed 7 [--rho 0.15] [-o out.json]
    python3 gen_scenes.py suite --outdir scenes/gen   # emit the whole benchmark set

Design decisions the generator owns (kept out of subagent hands): the family
geometry, the derived optima, and the feasibility margins. Batch solving/plotting
of the emitted scenes is delegated downstream (see HANDOFF T1 step 2).
"""

import argparse
import json
import math
import os

# ---- global geometry conventions (match the anchor scenes exactly) ----------
H_BOX = 0.35            # box half-extent (0.7 x 0.7)
WALL = 0.3             # wall / stub thickness
WORLD_H = 6.0           # world height
LANE_Y0, LANE_Y1 = 2.5, 3.6     # corridor lane (free strip one box tall)
NICHE_FLOOR = 1.4       # stub floor height inside a niche opening
NICHE_W = 1.4          # niche opening width (fits a 0.7 box with margin)
LANE_CY = 3.0          # box centre-y in the lane
NICHE_CY = 1.75        # box centre-y parked in a niche (rests on stub floor 1.4)
BOX_YB = [1.75, 3.25]   # box y-bounds: lane down to niche


def _rect(xmin, xmax, ymin, ymax):
    return {"xmin": round(xmin, 4), "xmax": round(xmax, 4),
            "ymin": round(ymin, 4), "ymax": round(ymax, 4)}


def _box(name, cx, cy, gx, gy, W, target=True):
    """A non-rotating target box at (cx,cy) -> (gx,gy) in a width-W world."""
    o = {"name": name, "type": "box", "hx": H_BOX, "hy": H_BOX,
         "rotates": False, "target": target,
         "start": [round(cx, 4), round(cy, 4), 0.0],
         "xbounds": [WALL + H_BOX, round(W - WALL - H_BOX, 4)],
         "ybounds": list(BOX_YB), "tbounds": [0.0, 0.0]}
    if target:
        o["goal"] = [round(gx, 4), round(gy, 4), 0.0]
    return o


def _corridor_obstacles(W, niche_centers):
    """Top wall + side walls + floor broken by niche openings.

    Reproduces buffer_swap / two_buffer exactly for the matching niche centres.
    """
    obs = [_rect(0.0, W, LANE_Y1, WORLD_H),        # top wall
           _rect(0.0, WALL, 0.0, WORLD_H),          # left wall
           _rect(W - WALL, W, 0.0, WORLD_H)]        # right wall
    cursor = 0.0
    for c in sorted(niche_centers):
        x0, x1 = c - NICHE_W / 2, c + NICHE_W / 2
        if x0 > cursor + 1e-9:
            obs.append(_rect(cursor, x0, 0.0, LANE_Y0))   # solid floor segment
        obs.append(_rect(x0, x1, 0.0, NICHE_FLOOR))       # niche stub
        cursor = x1
    if W > cursor + 1e-9:
        obs.append(_rect(cursor, W, 0.0, LANE_Y0))
    return obs


# ---------------------------------------------------------------------------
# swap_k_m : reverse k single-file boxes with m niches. Optimum conjecture 2k-1.
# ---------------------------------------------------------------------------
def gen_swap(k, m, slot_gap=3.0, margin=1.7):
    """k boxes evenly spaced; goal = reversed order. m niches spread in the gaps.

    slot_gap: centre-to-centre box spacing. margin: clearance from the side walls
    to the first/last slot centre. Niches are placed at slot midpoints (canonical
    m=k-1) or spread as evenly as possible across the k-1 inter-slot gaps.
    """
    if k < 2:
        raise ValueError("swap needs k>=2")
    x0 = WALL + margin
    slots = [x0 + i * slot_gap for i in range(k)]
    W = slots[-1] + margin + WALL
    # candidate niche x-positions = midpoints of consecutive slots (k-1 of them)
    mids = [(slots[i] + slots[i + 1]) / 2 for i in range(k - 1)]
    if m > len(mids):
        raise ValueError("swap_k_m needs m<=k-1 niches (only %d gaps)" % len(mids))
    # pick m of the k-1 gaps, spread evenly
    if m == len(mids):
        chosen = mids
    else:
        idx = [round(j * (len(mids) - 1) / (m - 1)) if m > 1 else (len(mids) - 1) // 2
               for j in range(m)]
        chosen = [mids[i] for i in sorted(set(idx))]
    objs = []
    names = [chr(ord('A') + i) for i in range(k)]
    for i in range(k):
        cx = slots[i]
        gx = slots[k - 1 - i]        # reversed
        objs.append(_box(names[i], cx, LANE_CY, gx, LANE_CY, W))
    scene = {"name": "swap_%d_%d" % (k, m),
             "world": {"xmin": 0.0, "xmax": round(W, 4), "ymin": 0.0, "ymax": WORLD_H},
             "obstacles": _corridor_obstacles(W, chosen),
             "objects": objs,
             "_meta": {"family": "swap", "k": k, "m": m,
                       "optimum_conjecture": 2 * k - 1,
                       "optimum_status": "conjectured (verify vs sound LB)",
                       "mrb_conjecture": k - 1}}
    return scene


# ---------------------------------------------------------------------------
# blocked_goal_chain_k : k parked blockers (start==goal) on A's only path.
# ---------------------------------------------------------------------------
def gen_blocked(k, slot_gap=3.0, margin=1.7):
    """A crosses left->right; k blockers B_i sit in the lane, each above a niche.

    Proven optimum 2k+1: each B_i dips into its niche and returns (2), A crosses (1).
    Needs k niches (MRB=k) so all blockers can be parked simultaneously.
    """
    if k < 1:
        raise ValueError("blocked needs k>=1")
    x0 = WALL + margin
    # blockers occupy k interior slots; A starts left of them, goal right of them
    blockers = [x0 + (i + 1) * slot_gap for i in range(k)]
    a_start = x0
    a_goal = blockers[-1] + slot_gap
    W = a_goal + margin + WALL
    niches = list(blockers)          # one niche directly under each blocker
    objs = [_box("A", a_start, LANE_CY, a_goal, LANE_CY, W)]
    for i in range(k):
        bx = blockers[i]
        objs.append(_box("B%d" % (i + 1), bx, LANE_CY, bx, LANE_CY, W))  # start==goal
    scene = {"name": "blocked_goal_chain_%d" % k,
             "world": {"xmin": 0.0, "xmax": round(W, 4), "ymin": 0.0, "ymax": WORLD_H},
             "obstacles": _corridor_obstacles(W, niches),
             "objects": objs,
             "_meta": {"family": "blocked", "k": k,
                       "optimum": 2 * k + 1, "optimum_status": "proven",
                       "mrb": k}}
    return scene


# ---------------------------------------------------------------------------
# door_chain_d : d swing doors in series, each must be re-closed.
# ---------------------------------------------------------------------------
DOOR_L = 2.85           # blade length (<= gap 3.0 - 0.15, the wedge rule)
DOOR_W = 0.3
GAP_Y0, GAP_Y1 = 2.5, 5.5    # vertical door gap (height 3.0)
STATION_GAP = 4.0       # centre-to-centre spacing of door stations


def gen_door_chain(d, first_x=3.0):
    """d door stations in series; box A crosses all of them left->right.

    Each station: lower stub [X-.3,X+.3]x[.3,2.5], upper stub [.,.]x[5.5,5.7],
    hinge just right of the stub at (X+.05, 2.5), blade length 2.85, closed at
    angle pi/2 (fills the gap up to y=5.35, leaving a 0.15 sliver). goal==start
    (re-close). Proven optimum 2d+1: open all, A crosses once high, close all.
    """
    if d < 1:
        raise ValueError("door_chain needs d>=1")
    stations = [first_x + STATION_GAP * j for j in range(d)]
    # A's goal must sit BEYOND the last door's swing reach (hinge + blade =
    # X_last+0.35+2.85 = X_last+3.2), else the door cannot re-close with A parked
    # at goal (this is exactly the door_relock lesson: keep the goal clear of the
    # swing disc). Give ~0.4 margin and size the world to fit.
    a_goal = stations[-1] + 3.6
    W = stations[-1] + 4.5
    obs = [_rect(0.0, W, 0.0, WALL),               # floor
           _rect(0.0, W, WORLD_H - WALL, WORLD_H),  # ceiling
           _rect(0.0, WALL, 0.0, WORLD_H),          # left wall
           _rect(W - WALL, W, 0.0, WORLD_H)]        # right wall
    objs = []
    for j, X in enumerate(stations):
        obs.append(_rect(X - WALL, X + WALL, WALL, GAP_Y0))            # lower stub
        obs.append(_rect(X - WALL, X + WALL, GAP_Y1, WORLD_H - WALL))  # upper stub
        # hinge sits just RIGHT of the stub's right edge (X+WALL) so the blade
        # swings away from the stub, never into it (cf. door_relock: stub edge
        # 5.3, hinge 5.35). Placing it inside the stub makes the door unopenable.
        objs.append({"name": "door%d" % (j + 1), "type": "door", "target": True,
                     "hinge": [round(X + WALL + 0.05, 4), GAP_Y0],
                     "length": DOOR_L, "width": DOOR_W,
                     "start_angle": round(math.pi / 2, 4),
                     "goal_angle": round(math.pi / 2, 4),
                     "angle_bounds": [-0.5, round(math.pi / 2, 4)]})
    a_start = first_x - 1.5
    a_cy = 4.0            # cross high, above the horizontal open doors (y~2.5)
    A = {"name": "A", "type": "box", "hx": H_BOX, "hy": H_BOX, "rotates": False,
         "target": True, "start": [round(a_start, 4), a_cy, 0.0],
         "goal": [round(a_goal, 4), a_cy, 0.0],
         "xbounds": [WALL + H_BOX, round(W - WALL - H_BOX, 4)],
         "ybounds": [WALL + H_BOX, round(WORLD_H - WALL - H_BOX, 4)],
         "tbounds": [0.0, 0.0]}
    objs.insert(0, A)
    scene = {"name": "door_chain_%d" % d,
             "world": {"xmin": 0.0, "xmax": round(W, 4), "ymin": 0.0, "ymax": WORLD_H},
             "obstacles": obs, "objects": objs,
             "_meta": {"family": "door_chain", "d": d,
                       "optimum": 2 * d + 1, "optimum_status": "proven"}}
    return scene


# ---------------------------------------------------------------------------
# random_room : n boxes, random non-overlapping start/goal (seeded).
# ---------------------------------------------------------------------------
def gen_random_room(n, seed, W=8.0, Hh=8.0, rho=None):
    """Open walled room, n target boxes with random non-overlapping start & goal.

    Deterministic in `seed` (LCG, no external RNG import for reproducibility).
    No closed-form optimum -> exact oracle (T5b) supplies ground truth.
    """
    # tiny self-contained LCG so results are reproducible & dependency-free
    state = (seed * 2654435761 + 12345) & 0xFFFFFFFF

    def rnd():
        nonlocal state
        state = (1103515245 * state + 12345) & 0x7FFFFFFF
        return state / 0x7FFFFFFF

    lo_x, hi_x = WALL + H_BOX, W - WALL - H_BOX
    lo_y, hi_y = WALL + H_BOX, Hh - WALL - H_BOX
    min_sep = 2 * H_BOX + 0.05        # non-overlap (touching allowed, +eps)

    def sample_free(placed):
        for _ in range(2000):
            x = lo_x + rnd() * (hi_x - lo_x)
            y = lo_y + rnd() * (hi_y - lo_y)
            if all(max(abs(x - px), abs(y - py)) >= min_sep for px, py in placed):
                return x, y
        raise RuntimeError("could not place box (room too dense for n=%d)" % n)

    starts, goals = [], []
    for _ in range(n):
        starts.append(sample_free(starts))
    for _ in range(n):
        goals.append(sample_free(goals))
    objs = []
    for i in range(n):
        sx, sy = starts[i]
        gx, gy = goals[i]
        o = {"name": chr(ord('A') + i), "type": "box", "hx": H_BOX, "hy": H_BOX,
             "rotates": False, "target": True,
             "start": [round(sx, 4), round(sy, 4), 0.0],
             "goal": [round(gx, 4), round(gy, 4), 0.0],
             "xbounds": [round(lo_x, 4), round(hi_x, 4)],
             "ybounds": [round(lo_y, 4), round(hi_y, 4)], "tbounds": [0.0, 0.0]}
        objs.append(o)
    obs = [_rect(0.0, W, 0.0, WALL), _rect(0.0, W, Hh - WALL, Hh),
           _rect(0.0, WALL, 0.0, Hh), _rect(W - WALL, W, 0.0, Hh)]
    scene = {"name": "random_room_n%d_s%d" % (n, seed),
             "world": {"xmin": 0.0, "xmax": W, "ymin": 0.0, "ymax": Hh},
             "obstacles": obs, "objects": objs,
             "_meta": {"family": "random_room", "n": n, "seed": seed,
                       "optimum_status": "unknown (needs exact oracle, T5b)"}}
    return scene


# ---------------------------------------------------------------------------
def _write(scene, path):
    with open(path, "w") as f:
        json.dump(scene, f, indent=2)
    print("wrote %s  (%s)" % (path, scene["_meta"]))


def gen_suite(outdir):
    os.makedirs(outdir, exist_ok=True)
    made = []
    for k in range(2, 6):                       # swap_2_1 .. swap_5_4
        for m in range(1, k):
            made.append(gen_swap(k, m))
    for d in range(1, 5):                       # door_chain_1 .. 4
        made.append(gen_door_chain(d))
    for k in range(1, 5):                       # blocked_goal_chain_1 .. 4
        made.append(gen_blocked(k))
    for n in range(2, 5):                       # random_room n=2..4, seeds 0..9
        for s in range(10):
            made.append(gen_random_room(n, s))
    for sc in made:
        _write(sc, os.path.join(outdir, sc["name"] + ".json"))
    print("\nsuite: %d scenes -> %s" % (len(made), outdir))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("swap"); p.add_argument("--k", type=int, required=True); p.add_argument("--m", type=int, required=True); p.add_argument("-o")
    p = sub.add_parser("door_chain"); p.add_argument("--d", type=int, required=True); p.add_argument("-o")
    p = sub.add_parser("blocked"); p.add_argument("--k", type=int, required=True); p.add_argument("-o")
    p = sub.add_parser("random_room"); p.add_argument("--n", type=int, required=True); p.add_argument("--seed", type=int, default=0); p.add_argument("--rho", type=float, default=None); p.add_argument("-o")
    p = sub.add_parser("suite"); p.add_argument("--outdir", default="scenes/gen")
    a = ap.parse_args()

    if a.cmd == "suite":
        gen_suite(a.outdir); return
    if a.cmd == "swap":
        sc = gen_swap(a.k, a.m)
    elif a.cmd == "door_chain":
        sc = gen_door_chain(a.d)
    elif a.cmd == "blocked":
        sc = gen_blocked(a.k)
    elif a.cmd == "random_room":
        sc = gen_random_room(a.n, a.seed, rho=a.rho)
    out = a.o or (sc["name"] + ".json")
    _write(sc, out)


if __name__ == "__main__":
    main()
