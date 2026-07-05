#!/usr/bin/env python3
"""Independent validator for the embodied counterexample (research/09 §Q-E2).

Proves, using ONLY the SAT/OBB primitives of validate_puzzle.py, that
scenes/embodied_ce.json exhibits "object-level feasibility does NOT imply
embodied feasibility":
  (a) the box start and goal are collision-free;
  (b) the box's own straight-line start->goal path is collision-free;
  (c) the contact-support set at the goal is EMPTY -- no collision-free base
      placement (a square, ANY orientation, anywhere in-world) puts the 2R
      arm's shoulder within reach (L1+L2) of the goal push-face contact point,
      so no coupled base+arm trajectory can deliver the box to its goal by
      pushing, even though the box could sit there.

This is the chair-verification artifact for briefing 09; run from demos/larrt2d:
    python3 tools/embodied_ce_check.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from validate_puzzle import obb_corners, sat_overlap, obstacle_to_obb

# --- scene (must match scenes/embodied_ce.json) ---
OBST = [obstacle_to_obb({"xmin": 4.5, "xmax": 7.5, "ymin": 0.0, "ymax": 2.64}),
        obstacle_to_obb({"xmin": 4.5, "xmax": 7.5, "ymin": 3.36, "ymax": 6.0})]
BOXH = 0.35
BASEH = 0.40
L1 = L2 = 0.6
REACH = L1 + L2
CONTACT = (6.0, 3.0)          # box rear-face centre at goal (6.35 - 0.35)
WORLD = (0.0, 12.0, 0.0, 6.0)


def _ov(A, B):
    return sat_overlap(obb_corners(*A), A[4], obb_corners(*B), B[4])


def box_clear(cx, cy):
    b = (cx, cy, BOXH, BOXH, 0.0)
    return not any(_ov(b, o) for o in OBST)


def base_clear(cx, cy, th):
    if not (BASEH <= cx <= WORLD[1] - BASEH and BASEH <= cy <= WORLD[3] - BASEH):
        return False
    base = (cx, cy, BASEH, BASEH, th)
    return not any(_ov(base, o) for o in OBST)


def main(step=0.05, nth=24):
    ok_a = box_clear(2.0, 3.0) and box_clear(6.35, 3.0)
    bad = sum(1 for t in range(6001)
              if not box_clear(2.0 + (6.35 - 2.0) * t / 6000, 3.0))
    ok_b = (bad == 0)

    feasible = 0
    best = float("inf")
    bestcfg = None
    tested = nbc = 0
    cx = 0.0
    while cx <= WORLD[1] + 1e-9:
        cy = 1.5
        while cy <= 4.5 + 1e-9:
            for i in range(nth):
                th = 2 * math.pi * i / nth
                tested += 1
                if not base_clear(cx, cy, th):
                    continue
                nbc += 1
                shx = cx + BASEH * math.cos(th)
                shy = cy + BASEH * math.sin(th)
                d = math.hypot(shx - CONTACT[0], shy - CONTACT[1])
                if d < best:
                    best = d
                    bestcfg = (round(cx, 3), round(cy, 3), round(th, 3), round(d, 4))
                if d <= REACH + 1e-9:
                    feasible += 1
            cy += step
        cx += step
    ok_c = (feasible == 0)

    print("(a) box start & goal collision-free:", ok_a)
    print("(b) box straight-line path collision-free (6001 samples):", ok_b)
    print("(c) tested=%d base_clear=%d within_reach=%d" % (tested, nbc, feasible))
    print("    min |shoulder-contact| over collision-free bases = %.4f (reach %.2f)"
          % (best, REACH))
    print("    achieved at (cx,cy,th,d)=%s" % (bestcfg,))
    verdict = ok_a and ok_b and ok_c
    print("VERDICT:", "COUNTEREXAMPLE CONFIRMED" if verdict else "FAILED")
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
