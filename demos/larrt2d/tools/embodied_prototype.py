#!/usr/bin/env python3
"""Pure-Python reference prototype for embodied 2D contact-manipulation planning.

Implements research/09b's ROUTE 1 literally:
  - transit  = robot moves alone (objects frozen) -> a basic RRT in the robot's
               SE(2) x T^2 space.
  - transfer = coupled robot+object push -> an explicit full-state waypoint
               sweep (robot IK per sample, object DOF advanced in lockstep),
               built directly by this module, NEVER by generic interpolation.

Reuses the SAT/OBB primitives of tools/validate_puzzle.py (imported, not
reimplemented): obb_corners, sat_overlap, obstacle_to_obb, obb_from_dof.

Usage:
    python3 tools/embodied_prototype.py <scene.json>

See research/09b_embodied_prototype_blueprint.md for the design this follows,
and research/09_embodied_contact_manipulation.md §2, §8.5 for the model and
the Q-E2 counterexample this is built to reproduce dynamically.
"""

import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from validate_puzzle import obb_corners, sat_overlap, obstacle_to_obb, obb_from_dof  # noqa: E402

TWO_PI = 2.0 * math.pi


# ---------------------------------------------------------------------------
# small geometry helpers
# ---------------------------------------------------------------------------

def wrap_angle(a):
    """Wrap an angle (radians) to (-pi, pi]."""
    a = math.fmod(a, TWO_PI)
    if a > math.pi:
        a -= TWO_PI
    elif a <= -math.pi:
        a += TWO_PI
    return a


def rot(theta, vx, vy):
    c, s = math.cos(theta), math.sin(theta)
    return (c * vx - s * vy, s * vx + c * vy)


def in_world(obb, world):
    cx, cy, hx, hy, theta = obb
    xmin, xmax = world["xmin"], world["xmax"]
    ymin, ymax = world["ymin"], world["ymax"]
    for x, y in obb_corners(cx, cy, hx, hy, theta):
        if x < xmin - 1e-9 or x > xmax + 1e-9 or y < ymin - 1e-9 or y > ymax + 1e-9:
            return False
    return True


def obbs_collide(obb, others):
    for other in others:
        if sat_overlap(obb_corners(*obb), obb[4], obb_corners(*other), other[4]):
            return True
    return False


def within_bounds(x, bounds):
    lo, hi = bounds
    return lo - 1e-9 <= x <= hi + 1e-9


# ---------------------------------------------------------------------------
# RobotSpec: base cube + 2R arm, FK -> 3 OBBs
# ---------------------------------------------------------------------------

class RobotSpec(object):
    """Mobile base (cube) + front-mounted 2R stick arm.

    State v = [xb, yb, thetab, q1, q2]. shoulder = base_center + R(thetab)*shoulder_off.
    theta1 = thetab + q1 (link1 direction, world frame). theta2 = theta1 + q2 (link2).
    """

    def __init__(self, robot_obj):
        self.base_hx = robot_obj["base_hx"]
        self.base_hy = robot_obj["base_hy"]
        self.shoulder_off = tuple(robot_obj["shoulder_off"])
        self.L1 = robot_obj["L1"]
        self.W1 = robot_obj["W1"]
        self.L2 = robot_obj["L2"]
        self.W2 = robot_obj["W2"]
        self.angle_bounds_base = tuple(robot_obj.get("angle_bounds_base", (-math.pi, math.pi)))
        qb = robot_obj.get("q_bounds", [[-math.pi, math.pi], [-math.pi, math.pi]])
        self.q1_bounds = tuple(qb[0])
        self.q2_bounds = tuple(qb[1])

    def shoulder_pos(self, xb, yb, thetab):
        sx, sy = self.shoulder_off
        dx, dy = rot(thetab, sx, sy)
        return (xb + dx, yb + dy)

    def fk(self, state):
        xb, yb, thetab, q1, q2 = state
        shoulder = self.shoulder_pos(xb, yb, thetab)
        theta1 = thetab + q1
        elbow = (shoulder[0] + self.L1 * math.cos(theta1),
                  shoulder[1] + self.L1 * math.sin(theta1))
        theta2 = theta1 + q2
        tip = (elbow[0] + self.L2 * math.cos(theta2),
               elbow[1] + self.L2 * math.sin(theta2))
        return shoulder, elbow, tip, theta1, theta2

    def obbs(self, state):
        """Return (base_obb, link1_obb, link2_obb), each (cx,cy,hx,hy,theta)."""
        xb, yb, thetab, q1, q2 = state
        shoulder, elbow, tip, theta1, theta2 = self.fk(state)
        base_obb = (xb, yb, self.base_hx, self.base_hy, thetab)
        link1_c = ((shoulder[0] + elbow[0]) * 0.5, (shoulder[1] + elbow[1]) * 0.5)
        link1_obb = (link1_c[0], link1_c[1], self.L1 * 0.5, self.W1 * 0.5, theta1)
        link2_c = ((elbow[0] + tip[0]) * 0.5, (elbow[1] + tip[1]) * 0.5)
        link2_obb = (link2_c[0], link2_c[1], self.L2 * 0.5, self.W2 * 0.5, theta2)
        return base_obb, link1_obb, link2_obb

    def joint_ok(self, thetab, q1, q2):
        return (within_bounds(thetab, self.angle_bounds_base)
                and within_bounds(q1, self.q1_bounds)
                and within_bounds(q2, self.q2_bounds))


# ---------------------------------------------------------------------------
# 2R inverse kinematics (closed form)
# ---------------------------------------------------------------------------

def solve_2r_ik(shoulder, tip, L1, L2):
    """Closed-form planar 2R IK. Returns [(theta1_world, q2), ...] (elbow up/down),
    empty if the target is out of [|L1-L2|, L1+L2] reach of `shoulder`.

    theta1_world is the WORLD-frame direction of link1 (theta1 = thetab + q1 in
    RobotSpec's convention); q2 is the elbow (relative) joint angle.
    """
    dx = tip[0] - shoulder[0]
    dy = tip[1] - shoulder[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return []
    if d > L1 + L2 + 1e-9 or d < abs(L1 - L2) - 1e-9:
        return []
    d_clamped = min(d, L1 + L2)
    d_clamped = max(d_clamped, abs(L1 - L2))
    cos_q2 = (d_clamped * d_clamped - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_q2 = max(-1.0, min(1.0, cos_q2))
    sols = []
    base_angle = math.atan2(dy, dx)
    for sign in (+1.0, -1.0):
        q2 = sign * math.acos(cos_q2)
        theta1 = base_angle - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        sols.append((theta1, q2))
    # de-duplicate the degenerate case q2 == 0 or q2 == pi (both signs equal)
    if len(sols) == 2 and abs(wrap_angle(sols[0][1] - sols[1][1])) < 1e-12:
        sols = sols[:1]
    return sols


# ---------------------------------------------------------------------------
# Contact pose search: IK + full-body collision + friction cone
# ---------------------------------------------------------------------------

def try_contact_pose(scene, robot, base_pose, contact_point, push_normal_angle,
                      friction_half_angle, other_obbs=None):
    """Try to realize a contact at `contact_point` from base pose `base_pose`.

    Solves 2R IK so the end-link TIP is exactly coincident with contact_point,
    then checks: joint/base-angle bounds, world bounds, friction cone (the
    end-link's elbow->tip direction must be within `friction_half_angle` of
    -push_normal_angle, i.e. flush against the push face), and SAT collision
    of all 3 robot OBBs vs `other_obbs` (obstacles + all objects EXCEPT the one
    currently being contacted -- touching the contacted object at the contact
    point is the intended physical event, not a collision; see 09b §2).

    Returns a full robot state [xb,yb,thetab,q1,q2] or None.
    """
    xb, yb, thb = base_pose
    if not within_bounds(thb, robot.angle_bounds_base):
        return None
    world = scene["world"]
    shoulder = robot.shoulder_pos(xb, yb, thb)
    others = other_obbs if other_obbs is not None else []

    desired_axis = wrap_angle(push_normal_angle + math.pi)  # -n_world direction

    for theta1_world, q2 in solve_2r_ik(shoulder, contact_point, robot.L1, robot.L2):
        q1 = wrap_angle(theta1_world - thb)
        if not robot.joint_ok(thb, q1, q2):
            continue
        theta2_world = theta1_world + q2
        diff = abs(wrap_angle(theta2_world - desired_axis))
        if diff > friction_half_angle + 1e-9:
            continue
        state = [xb, yb, thb, q1, q2]
        base_obb, link1_obb, link2_obb = robot.obbs(state)
        ok = True
        for obb in (base_obb, link1_obb, link2_obb):
            if not in_world(obb, world):
                ok = False
                break
            if obbs_collide(obb, others):
                ok = False
                break
        if not ok:
            continue
        return state
    return None


# ---------------------------------------------------------------------------
# Base-pose search (reach test + per-sample follow search inside push_transfer)
# ---------------------------------------------------------------------------

def _grid_search_base(scene, robot, contact_point, push_normal_angle, friction_half_angle,
                       other_obbs, cx_range, cy_range, xy_step, n_theta,
                       prefer_near=None):
    """Exhaustive grid over (cx,cy,theta) in the given box; returns the first hit
    (or the one closest to `prefer_near`'s base xy if given), else None."""
    world = scene["world"]
    xmin = max(world["xmin"], cx_range[0])
    xmax = min(world["xmax"], cx_range[1])
    ymin = max(world["ymin"], cy_range[0])
    ymax = min(world["ymax"], cy_range[1])
    if xmax < xmin or ymax < ymin:
        return None

    best = None
    best_d2 = float("inf")
    nx = max(1, int(round((xmax - xmin) / xy_step)))
    ny = max(1, int(round((ymax - ymin) / xy_step)))
    for ix in range(nx + 1):
        cx = xmin + (xmax - xmin) * ix / nx
        for iy in range(ny + 1):
            cy = ymin + (ymax - ymin) * iy / ny
            for it in range(n_theta):
                th = -math.pi + TWO_PI * it / n_theta
                state = try_contact_pose(scene, robot, (cx, cy, th), contact_point,
                                          push_normal_angle, friction_half_angle, other_obbs)
                if state is None:
                    continue
                if prefer_near is None:
                    return state
                d2 = (state[0] - prefer_near[0]) ** 2 + (state[1] - prefer_near[1]) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best = state
    return best


def find_approach_pose(scene, robot, contact_point, push_normal_angle, friction_half_angle,
                        other_obbs, prefer_near=None):
    """Reach test: search the whole plausible contact-support region (bounded by
    arm reach) for a collision-free base+arm pose realizing the contact. Used
    (a) standalone by the sequencer to pick the approach pose transit must
    reach, and (b) as the escalation path inside push_transfer when a local
    follow-search fails (needed so an ABORT is a genuine geometric
    infeasibility, not a greedy-search artifact -- 09b's "risk 3")."""
    reach = robot.L1 + robot.L2
    margin = max(robot.base_hx, robot.base_hy) * 1.5 + 0.05
    r = reach + margin
    cx_range = (contact_point[0] - r, contact_point[0] + r)
    cy_range = (contact_point[1] - r, contact_point[1] + r)
    # Fine resolution: this is the soundness net for both the standalone reach
    # test and push_transfer's escalation path (09b chair note, "risk 3" -- an
    # ABORT must be a genuine geometric infeasibility, not a search artifact).
    return _grid_search_base(scene, robot, contact_point, push_normal_angle, friction_half_angle,
                              other_obbs, cx_range, cy_range, xy_step=0.05, n_theta=24,
                              prefer_near=prefer_near)


def find_following_pose(scene, robot, contact_point, push_normal_angle, friction_half_angle,
                         other_obbs, prev_base):
    """Per-sample base search used inside push_transfer: (1) try reusing the
    previous base pose unchanged; (2) a small local search around it (the base
    "follows" the object); (3) escalate to the full reach-bounded global grid
    search (find_approach_pose) so a failure here is a real infeasibility, not
    an artifact of a lazy local search (09b chair note, risk 3)."""
    state = try_contact_pose(scene, robot, prev_base, contact_point, push_normal_angle,
                              friction_half_angle, other_obbs)
    if state is not None:
        return state

    offsets = []
    for dx in (-0.1, 0.0, 0.1, -0.2, 0.2, -0.3, 0.3):
        for dy in (-0.1, 0.0, 0.1, -0.2, 0.2, -0.3, 0.3):
            for dth in (0.0, -0.15, 0.15, -0.3, 0.3):
                offsets.append((dx, dy, dth))
    offsets.sort(key=lambda o: o[0] ** 2 + o[1] ** 2 + o[2] ** 2)
    xb, yb, thb = prev_base
    for dx, dy, dth in offsets:
        cand = (xb + dx, yb + dy, thb + dth)
        state = try_contact_pose(scene, robot, cand, contact_point, push_normal_angle,
                                  friction_half_angle, other_obbs)
        if state is not None:
            return state

    # Escalate: full reach-bounded global search (soundness net for ABORT).
    return find_approach_pose(scene, robot, contact_point, push_normal_angle,
                               friction_half_angle, other_obbs, prefer_near=prev_base)


# ---------------------------------------------------------------------------
# Per-object-kind contact point / normal (world frame) as a function of DOF
# ---------------------------------------------------------------------------

def contact_point_and_normal(obj, dof, face):
    t = obj.get("type", "box")
    nx0, ny0 = face["normal_local"]
    ox0, oy0 = face["contact_offset_local"]
    if t == "box":
        cx, cy = dof[0], dof[1]
        theta = dof[2] if len(dof) >= 3 else obj.get("angle", 0.0)
        n_world = rot(theta, nx0, ny0)
        off_world = rot(theta, ox0, oy0)
        return (cx + off_world[0], cy + off_world[1]), n_world
    if t == "door" or t == "revolute":
        a = dof[0]
        hx, hy = obj["hinge"] if t == "door" else obj["anchor"]
        n_world = rot(a, nx0, ny0)
        off_world = rot(a, ox0, oy0)
        return (hx + off_world[0], hy + off_world[1]), n_world
    raise ValueError("unsupported pushable object type: %r" % t)


# ---------------------------------------------------------------------------
# TRANSFER primitive: explicit full-state waypoint sweep (route 1)
# ---------------------------------------------------------------------------

def push_transfer(scene, robot, obj, face, from_full_state, target_obj_dof, other_objects,
                   nsteps=200):
    """Sweep object `obj`'s DOF from from_full_state[obj.name] to target_obj_dof.

    Per sample: compute contact point/normal from the object's own DOF model
    (box: translate along R(theta)*normal_local; door/revolute: rotate about
    hinge), call try_contact_pose/find_following_pose to realize it, then
    collision-check the 3 robot OBBs + the moving object vs walls & the OTHER
    (frozen) objects. All-or-nothing: aborts the whole transfer on any sample
    failure, returning the last valid object DOF reached.

    `other_objects` = [(name, obj_spec, dof), ...] for every object except `obj`,
    frozen at the given dof for the duration of this transfer.

    Returns a dict:
      status: "SUCCESS" | "ABORT"
      waypoints: list of {"robot": [...], obj.name: dof, "t": t}
      max_coincidence_err: max |tip - contact_point| over all successful samples
      collided_samples: count of samples where a collision was found (should be
                         0 -- any collision triggers an immediate abort)
      last_obj_dof / fail_t / reason: present on ABORT
    """
    name = obj["name"]
    mode = {"contactedObject": name, "faceId": face["id"]}
    t_obj = obj.get("type", "box")
    cur_dof = list(from_full_state[name])
    base_pose = tuple(from_full_state["robot"][:3])
    world = scene["world"]
    obst_obbs = [obstacle_to_obb(o) for o in scene.get("obstacles", [])]
    other_static_obbs = [obb_from_dof(ospec, odof) for (_, ospec, odof) in other_objects]
    fixed_obbs = obst_obbs + other_static_obbs

    waypoints = []
    max_err = 0.0
    collided_samples = 0

    if t_obj == "box":
        cur_center = (cur_dof[0], cur_dof[1])
        theta_fixed = cur_dof[2] if len(cur_dof) >= 3 else obj.get("angle", 0.0)
        _, n_world0 = contact_point_and_normal(obj, cur_dof, face)
        target_center = (target_obj_dof[0], target_obj_dof[1])
        disp = (target_center[0] - cur_center[0], target_center[1] - cur_center[1])
        U = disp[0] * n_world0[0] + disp[1] * n_world0[1]

        def dof_at(t):
            u = t * U
            c = (cur_center[0] + u * n_world0[0], cur_center[1] + u * n_world0[1])
            return [c[0], c[1], theta_fixed] if len(cur_dof) >= 3 else [c[0], c[1]]
    elif t_obj in ("door", "revolute"):
        a0 = cur_dof[0]
        a1 = target_obj_dof[0]
        delta = a1 - a0

        def dof_at(t):
            return [a0 + t * delta]
    else:
        raise ValueError("unsupported pushable object type: %r" % t_obj)

    last_good_dof = list(cur_dof)

    for i in range(nsteps + 1):
        t = i / float(nsteps)
        new_dof = dof_at(t)
        contact, n_world = contact_point_and_normal(obj, new_dof, face)
        push_normal_angle = math.atan2(n_world[1], n_world[0])

        state = find_following_pose(scene, robot, contact, push_normal_angle,
                                     face["friction_half_angle"], fixed_obbs, base_pose)
        if state is None:
            return {
                "status": "ABORT",
                "waypoints": waypoints,
                "last_obj_dof": last_good_dof,
                "fail_step": i,
                "fail_t": t,
                "fail_target_dof": new_dof,
                "fail_contact_point": contact,
                "max_coincidence_err": max_err,
                "collided_samples": collided_samples,
                "reason": ("no collision-free base+arm configuration reaches contact point "
                           "%s within reach (robot cannot escort the object further)" % (contact,)),
            }

        # moving object itself vs walls + other (frozen) objects
        obj_obb = obb_from_dof(obj, new_dof)
        if not in_world(obj_obb, world) or obbs_collide(obj_obb, fixed_obbs):
            collided_samples += 1
            return {
                "status": "ABORT",
                "waypoints": waypoints,
                "last_obj_dof": last_good_dof,
                "fail_step": i,
                "fail_t": t,
                "fail_target_dof": new_dof,
                "max_coincidence_err": max_err,
                "collided_samples": collided_samples,
                "reason": "moving object collides with walls/other objects at u=%g" % t,
            }

        base_pose = tuple(state[:3])
        _, _, tip, _, _ = robot.fk(state)
        err = math.hypot(tip[0] - contact[0], tip[1] - contact[1])
        max_err = max(max_err, err)

        waypoints.append({"robot": state, name: new_dof, "t": t, "mode": mode})
        last_good_dof = new_dof

    return {
        "status": "SUCCESS",
        "waypoints": waypoints,
        "max_coincidence_err": max_err,
        "collided_samples": collided_samples,
        "last_obj_dof": last_good_dof,
    }


# ---------------------------------------------------------------------------
# TRANSIT: basic RRT in the robot's SE(2) x T^2 space, objects frozen
# ---------------------------------------------------------------------------

_ANGLE_DIMS = (2, 3, 4)


def _state_diff(a, b):
    out = []
    for i in range(len(a)):
        if i in _ANGLE_DIMS:
            out.append(wrap_angle(b[i] - a[i]))
        else:
            out.append(b[i] - a[i])
    return out


def _state_dist(a, b):
    d = _state_diff(a, b)
    return math.sqrt(sum(x * x for x in d))


def _interp(a, b, t):
    d = _state_diff(a, b)
    return [a[i] + t * d[i] for i in range(len(a))]


def _steer(a, b, max_step):
    d = _state_diff(a, b)
    dist = math.sqrt(sum(x * x for x in d))
    if dist <= max_step:
        return list(b)
    scale = max_step / dist
    return [a[i] + d[i] * scale for i in range(len(a))]


def _state_valid(state, robot, fixed_obbs, world):
    if not robot.joint_ok(state[2], state[3], state[4]):
        return False
    for obb in robot.obbs(state):
        if not in_world(obb, world):
            return False
        if obbs_collide(obb, fixed_obbs):
            return False
    return True


def _edge_valid(a, b, robot, fixed_obbs, world, samples=10):
    for i in range(samples + 1):
        st = _interp(a, b, i / float(samples))
        if not _state_valid(st, robot, fixed_obbs, world):
            return False
    return True


def transit_rrt(start, goal, robot, obst_obbs, frozen_obj_obbs, world,
                 max_iters=6000, step=0.6, seed=42):
    """Basic RRT for the robot's 5-DOF space (SE(2) base x T^2 arm), objects
    frozen (baked into fixed_obbs). Returns a waypoint list start..goal, or
    None if no path was found within max_iters."""
    fixed_obbs = list(obst_obbs) + list(frozen_obj_obbs)
    if not _state_valid(start, robot, fixed_obbs, world):
        return None
    if not _state_valid(goal, robot, fixed_obbs, world):
        return None
    if _edge_valid(start, goal, robot, fixed_obbs, world):
        return [list(start), list(goal)]

    rng = random.Random(seed)
    bounds = [
        (world["xmin"], world["xmax"]),
        (world["ymin"], world["ymax"]),
        robot.angle_bounds_base,
        robot.q1_bounds,
        robot.q2_bounds,
    ]

    tree = [list(start)]
    parent = [-1]

    for _ in range(max_iters):
        if rng.random() < 0.1:
            sample = list(goal)
        else:
            sample = [rng.uniform(lo, hi) for (lo, hi) in bounds]

        nearest_i = min(range(len(tree)), key=lambda i: _state_dist(tree[i], sample))
        new_state = _steer(tree[nearest_i], sample, step)
        if not _state_valid(new_state, robot, fixed_obbs, world):
            continue
        if not _edge_valid(tree[nearest_i], new_state, robot, fixed_obbs, world):
            continue
        tree.append(new_state)
        parent.append(nearest_i)

        if _edge_valid(new_state, goal, robot, fixed_obbs, world):
            tree.append(list(goal))
            parent.append(len(tree) - 2)
            path = []
            idx = len(tree) - 1
            while idx != -1:
                path.append(tree[idx])
                idx = parent[idx]
            path.reverse()
            return path

    return None


# ---------------------------------------------------------------------------
# Mode sequencer
# ---------------------------------------------------------------------------

def mode_sequencer(scene):
    robot_obj = next(o for o in scene["objects"] if o.get("type") == "robot")
    robot = RobotSpec(robot_obj)
    objects = [o for o in scene["objects"] if o.get("type") != "robot"]
    world = scene["world"]
    obst_obbs = [obstacle_to_obb(o) for o in scene.get("obstacles", [])]

    robot_state = list(robot_obj["start"])
    targets = [o for o in objects if o.get("target") and o.get("goal") is not None]

    overall_status = "SUCCESS"
    last_result = None

    for obj in targets:
        name = obj["name"]
        face = obj["pushable_faces"][0]
        other_objects = [(o["name"], o, list(o["start"])) for o in objects if o["name"] != name]
        other_obbs_static = [obb_from_dof(ospec, odof) for (_, ospec, odof) in other_objects]

        cur_dof = list(obj["start"])
        contact0, n0 = contact_point_and_normal(obj, cur_dof, face)
        push_normal_angle0 = math.atan2(n0[1], n0[0])

        print("== mode: object=%s face=%s ==" % (name, face["id"]))
        print("[reach test] seeking an approach pose for initial contact %s"
              % (tuple(round(c, 3) for c in contact0),))

        approach_state = find_approach_pose(
            scene, robot, contact0, push_normal_angle0, face["friction_half_angle"],
            obst_obbs + other_obbs_static, prefer_near=robot_state)

        if approach_state is None:
            print("[reach test] ABORT: no collision-free base+arm configuration can reach "
                  "the initial contact point at all.")
            overall_status = "ABORT"
            last_result = {"status": "ABORT", "stage": "reach", "object": name}
            break
        print("[reach test] OK: approach_state=%s"
              % [round(v, 4) for v in approach_state])

        print("[transit] planning robot-only RRT (objects frozen) from %s to approach pose"
              % [round(v, 3) for v in robot_state])
        path = transit_rrt(robot_state, approach_state, robot, obst_obbs, other_obbs_static, world)
        if path is None:
            print("[transit] ABORT: RRT failed to find a collision-free path to the approach pose.")
            overall_status = "ABORT"
            last_result = {"status": "ABORT", "stage": "transit", "object": name}
            break
        transit_waypoints = [{"robot": s, "mode": {"contactedObject": -1, "faceId": -1}} for s in path]
        print("[transit] SUCCESS: %d waypoints" % len(transit_waypoints))
        robot_state = approach_state

        from_full_state = {"robot": robot_state, name: cur_dof}
        for oname, ospec, odof in other_objects:
            from_full_state[oname] = odof

        print("[transfer] pushing %s: dof %s -> %s (nsteps=200)"
              % (name, [round(v, 3) for v in cur_dof], [round(v, 3) for v in obj["goal"]]))
        result = push_transfer(scene, robot, obj, face, from_full_state, obj["goal"],
                                other_objects, nsteps=200)
        last_result = result
        last_result["object"] = name

        if result["status"] != "SUCCESS":
            print("[transfer] ABORT for %s at t=%.4f (step %d/%d)"
                  % (name, result["fail_t"], result["fail_step"], 200))
            print("           reason: %s" % result["reason"])
            print("           last valid object dof reached: %s"
                  % [round(v, 4) for v in result["last_obj_dof"]])
            overall_status = "ABORT"
            break

        print("[transfer] SUCCESS for %s: %d waypoints, max_contact_coincidence_error=%.3e, "
              "collided_samples=%d" % (name, len(result["waypoints"]),
                                        result["max_coincidence_err"], result["collided_samples"]))
        robot_state = result["waypoints"][-1]["robot"]

    print("RESULT: %s" % overall_status)
    return overall_status, last_result


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv):
    if len(argv) != 2:
        print("usage: python3 embodied_prototype.py <scene.json>")
        return 2
    with open(argv[1]) as f:
        scene = json.load(f)

    print("scene: %s" % scene.get("name", argv[1]))
    status, result = mode_sequencer(scene)
    return 0 if status == "SUCCESS" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
