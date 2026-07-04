#!/usr/bin/env python3
"""Independent validator for the 2D rearrangement-puzzle pipeline.

Re-checks a driver-produced solution JSON with its OWN geometry code so it can
catch bugs in the C++ planner/driver. It deliberately does NOT import or reuse
any driver code.

Usage:
    python3 validate_puzzle.py <solution.json>

Contract: see demos/larrt2d/PUZZLE_PIPELINE_SPEC.md
  - SAT collision on oriented rectangles (OBB), obstacles axis-aligned rects.
  - EPS = 1e-6, touching (shared edge) is NOT a collision.
  - Object i owns path dims [3i, 3i+1, 3i+2] = x, y, theta.
"""

import json
import math
import sys

EPS = 1e-6

# tolerances
START_GOAL_TOL = 1e-3      # (a) start/goal match
MOVE_TOL = 1e-9            # (d) which dims changed within a segment
ACTION_TOL = 1e-9         # (e) action counting (spec §5: |Delta|>1e-9)
EDGE_SAMPLES = 200        # (c) minimum samples per segment


# ---------------------------------------------------------------------------
# Geometry: oriented-rectangle (OBB) corners + Separating Axis Theorem overlap
# ---------------------------------------------------------------------------

def obb_corners(cx, cy, hx, hy, theta):
    """4 corners of an oriented rectangle: (x,y) + R(theta)*(+-hx, +-hy)."""
    c = math.cos(theta)
    s = math.sin(theta)
    corners = []
    for sx, sy in ((+hx, +hy), (+hx, -hy), (-hx, -hy), (-hx, +hy)):
        corners.append((cx + c * sx - s * sy,
                        cy + s * sx + c * sy))
    return corners


def _axes_for(theta):
    """The 2 edge normals of a rect at angle theta."""
    c = math.cos(theta)
    s = math.sin(theta)
    return ((c, s), (-s, c))


def _project(corners, ax, ay):
    dots = [px * ax + py * ay for px, py in corners]
    return min(dots), max(dots)


def sat_overlap(cornersA, thetaA, cornersB, thetaB):
    """SAT overlap test for two convex rectangles.

    Candidate axes = the 4 edge normals (2 from each rect). Separated on an
    axis iff maxA <= minB + EPS or maxB <= minA + EPS. Overlap iff no axis
    separates. Touching (shared edge) is NOT a collision.
    """
    for ax, ay in (_axes_for(thetaA) + _axes_for(thetaB)):
        minA, maxA = _project(cornersA, ax, ay)
        minB, maxB = _project(cornersB, ax, ay)
        if maxA <= minB + EPS or maxB <= minA + EPS:
            return False  # separating axis found -> no overlap
    return True


# ---------------------------------------------------------------------------
# Scene model
# ---------------------------------------------------------------------------

def obstacle_to_obb(o):
    cx = 0.5 * (o["xmin"] + o["xmax"])
    cy = 0.5 * (o["ymin"] + o["ymax"])
    hx = 0.5 * (o["xmax"] - o["xmin"])
    hy = 0.5 * (o["ymax"] - o["ymin"])
    return (cx, cy, hx, hy, 0.0)


def obb_from_dof(obj, v):
    """OBB (cx,cy,hx,hy,theta) of an object from its own DOF vector v.

    Mirrors the driver's ObjSpec::obb (LARRT2D_Puzzle.cpp) for every kind:
      box     : v=[x,y] or [x,y,theta]; theta from v[2] or the fixed "angle".
      door    : v=[a]; centre = hinge + (L/2)(cos a, sin a), half=(L/2,W/2).
      revolute: v=[a]; centre = anchor + R(a)*com, half=(hx,hy).
      slider  : v=[d]; centre = base + d*axis, half=(hx,hy), theta=fixed angle.
    """
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


def object_obb(state, obj):
    return obb_from_dof(obj, object_dof(state, obj))


def dof_is_angle(obj, k):
    """Is DOF k of this object an angle (SO(2), shortest-arc)?"""
    t = obj.get("type", "box")
    if t in ("door", "revolute"):
        return k == 0
    if t == "box":
        return k == 2   # theta, only present when the box rotates (ndof==3)
    return False        # slider displacement is linear


def config_collisions(state, objects, obstacles):
    """Return list of collision descriptions for a full path state.

    Empty list => valid configuration.
    """
    # Precompute each object's corners.
    obj_data = []  # (name, theta, corners)
    for obj in objects:
        cx, cy, hx, hy, th = object_obb(state, obj)
        corners = obb_corners(cx, cy, hx, hy, th)
        obj_data.append((obj["name"], th, corners))

    obst_data = []
    for i, ob in enumerate(obstacles):
        cx, cy, hx, hy, th = obstacle_to_obb(ob)
        obst_data.append((i, th, obb_corners(cx, cy, hx, hy, th)))

    collisions = []
    # object vs obstacle
    for oname, oth, ocorners in obj_data:
        for idx, wth, wcorners in obst_data:
            if sat_overlap(ocorners, oth, wcorners, wth):
                collisions.append("object %s overlaps obstacle #%d" % (oname, idx))
    # object vs object
    n = len(obj_data)
    for a in range(n):
        for b in range(a + 1, n):
            na, tha, ca = obj_data[a]
            nb, thb, cb = obj_data[b]
            if sat_overlap(ca, tha, cb, thb):
                collisions.append("object %s overlaps object %s" % (na, nb))
    return collisions


# ---------------------------------------------------------------------------
# Action counting (spec §5)
# ---------------------------------------------------------------------------

def changed_group(s0, s1, groups, tol):
    """Index of the group whose dims differ (|Delta|>tol); -1 if none.

    (Spec §5 assumes at most one group changes; if several change, this returns
    the first changed group, matching a single-index reference.)
    """
    for g, dims in enumerate(groups):
        for d in dims:
            if abs(s1[d] - s0[d]) > tol:
                return g
    return -1


def action_count(path, groups, tol):
    count = 0
    prev = None
    for k in range(len(path) - 1):
        cg = changed_group(path[k], path[k + 1], groups, tol)
        if cg == -1:
            # no movement: does not start a new action, keeps prev run.
            continue
        if cg != prev:
            count += 1
            prev = cg
    return count


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

def approx(a, b, tol):
    return abs(a - b) <= tol


def dof_close(obj, k, want, got, tol):
    """Match DOF k of an object; angle DOFs compare on the shortest arc (2*pi)."""
    if dof_is_angle(obj, k):
        d = math.fmod(want - got, 2.0 * math.pi)
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        return abs(d) <= tol
    return approx(want, got, tol)


def check_start_goal(path, objects):
    """Every object at the path start, and every TARGET at the path goal, DOF by DOF.

    Works for any object kind: object i owns the state indices obj["dims"] (1 DOF for
    door/slider/revolute, 2-3 for a box) and its start/goal arrays hold exactly those
    DOF. Non-targets have no goal (they may end anywhere valid)."""
    ok = True
    msgs = []
    start = path[0]
    goal = path[-1]
    for obj in objects:
        dims = obj["dims"]
        s_dof = object_dof(start, obj)
        for k in range(len(dims)):
            want, got = obj["start"][k], s_dof[k]
            if not dof_close(obj, k, want, got, START_GOAL_TOL):
                ok = False
                msgs.append("  object %s start dof[%d]: expected %g got %g"
                            % (obj["name"], k, want, got))
        if obj.get("target", obj.get("goal") is not None) and obj.get("goal") is not None:
            g_dof = object_dof(goal, obj)
            for k in range(len(dims)):
                want, got = obj["goal"][k], g_dof[k]
                if not dof_close(obj, k, want, got, START_GOAL_TOL):
                    ok = False
                    msgs.append("  object %s goal dof[%d]: expected %g got %g"
                                % (obj["name"], k, want, got))
    return ok, msgs


def check_waypoints(path, objects, obstacles):
    ok = True
    msgs = []
    for k, state in enumerate(path):
        cols = config_collisions(state, objects, obstacles)
        if cols:
            ok = False
            msgs.append("  waypoint %d: %s" % (k, "; ".join(cols)))
    return ok, msgs


def check_edges(path, objects, obstacles, samples=EDGE_SAMPLES):
    ok = True
    msgs = []
    ndim = len(path[0])
    for k in range(len(path) - 1):
        s0 = path[k]
        s1 = path[k + 1]
        seg_fail = None
        # sample inclusive of endpoints; >= samples interior resolution.
        for i in range(samples + 1):
            t = i / samples
            state = [s0[d] + t * (s1[d] - s0[d]) for d in range(ndim)]
            cols = config_collisions(state, objects, obstacles)
            if cols:
                seg_fail = (t, cols)
                break
        if seg_fail is not None:
            ok = False
            t, cols = seg_fail
            msgs.append("  segment %d->%d at t=%.4f: %s"
                        % (k, k + 1, t, "; ".join(cols)))
    return ok, msgs


def check_single_object(path, groups):
    ok = True
    msgs = []
    for k in range(len(path) - 1):
        s0 = path[k]
        s1 = path[k + 1]
        moved = []
        for g, dims in enumerate(groups):
            if any(abs(s1[d] - s0[d]) > MOVE_TOL for d in dims):
                moved.append(g)
        if len(moved) > 1:
            ok = False
            msgs.append("  segment %d->%d moves %d groups: %s"
                        % (k, k + 1, len(moved), moved))
    return ok, msgs


def check_action_recount(path, groups, declared):
    recount = action_count(path, groups, ACTION_TOL)
    ok = (recount == declared)
    msgs = []
    if not ok:
        msgs.append("  recomputed action_count=%d but JSON declares %s"
                    % (recount, declared))
    return ok, msgs, recount


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv):
    if len(argv) != 2:
        print("usage: python3 validate_puzzle.py <solution.json>")
        return 2

    with open(argv[1]) as f:
        data = json.load(f)

    if not data.get("solved", False):
        print("solved = false; nothing to validate.")
        return 0

    objects = data["objects"]
    obstacles = data.get("obstacles", [])
    groups = data["groups"]
    path = data["path"]

    if not path:
        print("FAIL: solved=true but path is empty.")
        print("VALIDATION: FAIL (1 checks failed)")
        return 1

    failed = 0

    def report(label, ok, msgs):
        nonlocal failed
        print("[%s] %s" % ("PASS" if ok else "FAIL", label))
        if not ok:
            failed += 1
            for m in msgs:
                print(m)

    ok, msgs = check_start_goal(path, objects)
    report("(a) start/goal match", ok, msgs)

    ok, msgs = check_waypoints(path, objects, obstacles)
    report("(b) waypoint validity (collision-free)", ok, msgs)

    ok, msgs = check_edges(path, objects, obstacles)
    report("(c) edge validity (>=%d samples/segment)" % EDGE_SAMPLES, ok, msgs)

    ok, msgs = check_single_object(path, groups)
    report("(d) single object moves per segment", ok, msgs)

    declared = data.get("action_count")
    ok, msgs, recount = check_action_recount(path, groups, declared)
    report("(e) action recount (declared=%s recount=%d)" % (declared, recount),
           ok, msgs)

    if failed == 0:
        print("VALIDATION: PASS")
        return 0
    print("VALIDATION: FAIL (%d checks failed)" % failed)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
