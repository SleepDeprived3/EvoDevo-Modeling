"""regression_tests_actuation.py

Synthetic-robot regression tests for the actuation/joint-limit machinery in
physics_engine_MULTI.py, covering the failure modes documented in
claude/joint-explosion-investigation.md (Mechanisms 1/3/5/6/7) and the
2026-09-13 architecture review (claude/pybullet-architecture-review.md:
switch from a hand-rolled TORQUE_CONTROL PD to native POSITION_CONTROL, plus
two independently-discovered pybullet gotchas fixed at the same time --
jointLimitForce being required for changeDynamics' jointLowerLimit/
jointUpperLimit to have any holding force at all, and jointLimitForce/
jointLowerLimit/jointUpperLimit needing their OWN changeDynamics call,
separate from maxJointVelocity, or the limit silently has no effect).

Earlier ad-hoc versions of these reproductions were built and run directly
against the code during past investigation sessions but were never
committed, so each new investigation had to rebuild them from scratch. This
version is meant to stay in the repo and be re-run whenever actuate_joint,
torque_cap_for_joint, or the changeDynamics calls in create_robot change.

Run directly: `python3 regression_tests_actuation.py`. Exits nonzero (and
prints which check failed) if any assertion fails.
"""

import math
import sys

import pybullet as p

from physics_engine_MULTI import (
    PyBulletWorld,
    BodyPart,
    JointPart,
    MAX_MOTOR_SPEED,
    REFERENCE_SIZE,
    REFERENCE_MUSCLE_FORCE,
    REFERENCE_MOMENT_ARM,
)


def _biological_cap(size):
    scale = size / REFERENCE_SIZE
    return REFERENCE_MUSCLE_FORCE * (scale ** 2) * REFERENCE_MOMENT_ARM * scale


def build_two_body_robot(world, parent_radius, child_radius, axis=(0, 0, 1),
                          mount_dir=(1, 0, 0), lower=-3.0, upper=3.0, motor=True):
    norm = math.sqrt(sum(c * c for c in mount_dir))
    mount_dir = [c / norm for c in mount_dir]
    base = BodyPart(id=0, x=0.0, y=0.0, z=parent_radius + 0.01, size=parent_radius)
    d = parent_radius + child_radius + 0.01
    child = BodyPart(
        id=1,
        x=base.x + mount_dir[0] * d, y=base.y + mount_dir[1] * d, z=base.z + mount_dir[2] * d,
        size=child_radius,
    )
    pivot = [base.x + mount_dir[0] * parent_radius,
             base.y + mount_dir[1] * parent_radius,
             base.z + mount_dir[2] * parent_radius]
    joint = JointPart(id=0, base_body=0, other_body=1,
                       px=pivot[0], py=pivot[1], pz=pivot[2],
                       ax=axis[0], ay=axis[1], az=axis[2],
                       lower_limit=lower, upper_limit=upper, motor=motor)
    robot_id = world.create_robot([base, child], [joint], [])
    return robot_id, joint


failures = []


def check(label, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {label}" + (f" -- {detail}" if detail else ""))
    if not condition:
        failures.append(label)


def test_matched_size_converges():
    world = PyBulletWorld(gravity=-10.0, headless=True)
    robot_id, joint = build_two_body_robot(world, 4.6, 4.6, lower=-3.0, upper=3.0)
    idx = world.joint_indices_map[joint.id]
    torque_cap = world.torque_cap_for_joint(joint)
    max_vel = 0.0
    for _ in range(300):
        world.actuate_joint(idx, 0.5, world.dt, torque_cap=torque_cap)
        p.stepSimulation(physicsClientId=world.client)
        st = p.getJointState(robot_id, idx, physicsClientId=world.client)
        max_vel = max(max_vel, abs(st[1]))
    final_pos = st[0]
    world.disconnect()
    check("matched-size converges near target",
          abs(final_pos - 0.5) < 0.05,
          f"final_pos={final_pos:.4f}")
    check("matched-size never exceeds MAX_MOTOR_SPEED",
          max_vel <= MAX_MOTOR_SPEED + 1e-6,
          f"max_vel={max_vel:.4f}")


def test_mismatched_child_reaches_target_without_slamming():
    world = PyBulletWorld(gravity=-10.0, headless=True)
    robot_id, joint = build_two_body_robot(world, 4.6, 0.05, lower=-3.0, upper=3.0)
    idx = world.joint_indices_map[joint.id]
    torque_cap = world.torque_cap_for_joint(joint)
    for _ in range(300):
        world.actuate_joint(idx, 1.2, world.dt, torque_cap=torque_cap)
        p.stepSimulation(physicsClientId=world.client)
    st = p.getJointState(robot_id, idx, physicsClientId=world.client)
    world.disconnect()
    check("mismatched-child (Mechanism 5 repro) reaches target",
          abs(st[0] - 1.2) < 0.01, f"final_pos={st[0]:.4f}")


def test_narrow_range_limit_actually_holds():
    """The core regression for the jointLimitForce / split-changeDynamics
    fix: an unreachable target must be stopped AT the joint's own limit,
    not sail through it."""
    world = PyBulletWorld(gravity=-10.0, headless=True)
    robot_id, joint = build_two_body_robot(world, 4.6, 0.05, lower=-0.05, upper=0.05)
    idx = world.joint_indices_map[joint.id]
    torque_cap = world.torque_cap_for_joint(joint)
    for _ in range(300):
        world.actuate_joint(idx, 1.2, world.dt, torque_cap=torque_cap)
        p.stepSimulation(physicsClientId=world.client)
    st = p.getJointState(robot_id, idx, physicsClientId=world.client)
    world.disconnect()
    check("narrow-range joint limit holds against an unreachable target",
          abs(st[0] - 0.05) < 0.01, f"final_pos={st[0]:.4f} (limit=0.05)")


def test_light_parent_does_not_float():
    """Mechanism 6 repro: light parent (0.5) / heavy child (6.4). Must
    converge and hold, not climb (base_z should not increase over the run)."""
    world = PyBulletWorld(gravity=-10.0, headless=True)
    robot_id, joint = build_two_body_robot(world, 0.5, 6.4, lower=-1.6, upper=1.6)
    idx = world.joint_indices_map[joint.id]
    torque_cap = world.torque_cap_for_joint(joint)
    base_z = []
    for _ in range(1000):
        world.actuate_joint(idx, 1.0, world.dt, torque_cap=torque_cap)
        p.stepSimulation(physicsClientId=world.client)
        pos, _ = p.getBasePositionAndOrientation(robot_id, physicsClientId=world.client)
        base_z.append(pos[2])
    st = p.getJointState(robot_id, idx, physicsClientId=world.client)
    world.disconnect()
    check("light-parent/heavy-child converges to held target",
          abs(st[0] - 1.0) < 0.02, f"final_pos={st[0]:.4f}")
    check("light-parent/heavy-child base does not climb (no floating)",
          base_z[-1] <= base_z[0] + 0.05,
          f"base_z[0]={base_z[0]:.3f} base_z[-1]={base_z[-1]:.3f}")


def test_geometric_clamp_holds_under_oversized_torque():
    """Mechanisms 1/7 repro, re-checked now that jointLimitForce actually
    works: a matched-size pair's evolved range gets clamped to theta_max at
    build time, and that clamp must hold as a real mechanical stop (near-zero
    self-contact penetration) even under a deliberately oversized torque."""
    world = PyBulletWorld(gravity=-10.0, headless=True)
    robot_id, joint = build_two_body_robot(world, 1.0, 1.0, lower=-3.0, upper=3.0)
    idx = world.joint_indices_map[joint.id]
    theta_max = joint.geometric_theta_max
    torque_cap = world.torque_cap_for_joint(joint)
    max_penetration = 0.0
    for _ in range(400):
        world.actuate_joint(idx, 3.0, world.dt, torque_cap=torque_cap * 50)
        p.stepSimulation(physicsClientId=world.client)
        for c in p.getContactPoints(bodyA=robot_id, bodyB=robot_id, physicsClientId=world.client):
            if c[8] < 0:
                max_penetration = max(max_penetration, -c[8])
    st = p.getJointState(robot_id, idx, physicsClientId=world.client)
    world.disconnect()
    check("geometric theta_max clamp computed",
          theta_max is not None and theta_max > 0, f"theta_max={theta_max}")
    check("oversized torque held at theta_max, not beyond",
          abs(st[0] - theta_max) < 0.02, f"final_pos={st[0]:.4f} theta_max={theta_max:.4f}")
    check("no self-collision penetration while pinned at the clamp",
          max_penetration < 1e-6, f"max_penetration={max_penetration:.6f}")


def test_multi_joint_chain_respects_each_joints_own_limit():
    world = PyBulletWorld(gravity=-10.0, headless=True)
    bodies = [BodyPart(id=0, x=0, y=0, z=3.0, size=1.0)]
    joints = []
    sizes = [1.0, 0.6, 0.3]
    x, prev_id = 0.0, 0
    for i, sz in enumerate(sizes):
        prev_size = bodies[prev_id].size
        d = prev_size + sz + 0.02
        x += d
        bodies.append(BodyPart(id=i + 1, x=x, y=0, z=3.0, size=sz))
        pivot_x = bodies[prev_id].x + prev_size
        joints.append(JointPart(id=i, base_body=prev_id, other_body=i + 1,
                                 px=pivot_x, py=0, pz=3.0, ax=0, ay=0, az=1,
                                 lower_limit=-1.0, upper_limit=1.0, motor=True))
        prev_id = i + 1
    robot_id = world.create_robot(bodies, joints, [])
    for _ in range(200):
        for j in world.joint_parts:
            idx = world.joint_indices_map[j.id]
            world.actuate_joint(idx, 0.8, world.dt, torque_cap=world.torque_cap_for_joint(j))
        p.stepSimulation(physicsClientId=world.client)
    ok = True
    for j in world.joint_parts:
        idx = world.joint_indices_map[j.id]
        st = p.getJointState(robot_id, idx, physicsClientId=world.client)
        within = abs(st[0]) <= j.geometric_theta_max + 0.02
        ok = ok and within
        print(f"    joint {j.id}: pos={st[0]:.4f} theta_max={j.geometric_theta_max:.4f} within={within}")
    world.disconnect()
    check("every joint in a multi-joint chain respects its own clamped range", ok)


def test_joint_can_lift_limb_against_gravity():
    """2026-09-14 (toppling-dominance investigation, continued): every case
    above uses axis=(0,0,1)/mount_dir=(1,0,0) -- a hinge that swings a
    horizontally-offset child about a VERTICAL axis, i.e. in the horizontal
    plane. That motion never changes the child's height, so gravity supplies
    exactly zero torque about that hinge -- these tests are, by construction,
    gravity-NEUTRAL, which is exactly why none of them ever caught the real
    bug found in this investigation: with the un-scaled REFERENCE_MUSCLE_FORCE
    (before MUSCLE_STRENGTH_SCALE), a joint had nowhere near enough torque to
    hold a same-size limb against this sim's own -10 gravity baseline in a
    hinge that actually has to fight gravity (axis perpendicular to both the
    mount direction and vertical, e.g. axis=(0,1,0) here -- the hinge a real
    "leg" joint would use to lift/lower a limb). Confirmed directly: under the
    un-scaled constant this exact test oscillated forever and never
    converged, at every size from 0.5 to 5.0. This is the direct regression
    check for that fix -- it must keep passing for any future change to
    REFERENCE_MUSCLE_FORCE / MUSCLE_STRENGTH_SCALE / torque_cap_for_joint.
    """
    for size, label in [(1.0, "reference size"), (0.5, "genome floor size")]:
        world = PyBulletWorld(gravity=-10.0, headless=True)
        robot_id, joint = build_two_body_robot(
            world, parent_radius=size, child_radius=size,
            axis=(0, 1, 0), mount_dir=(1, 0, 0),
            lower=-0.9, upper=0.9,
        )
        idx = world.joint_indices_map[joint.id]
        torque_cap = world.torque_cap_for_joint(joint)
        # Spawn pose (theta=0, child mounted horizontally from the parent) IS
        # the worst-case, fully-loaded configuration here -- commanding a
        # hold at 0.0 is the hardest case gravity produces, not an easy one.
        for _ in range(400):
            world.actuate_joint(idx, 0.0, world.dt, torque_cap=torque_cap)
            p.stepSimulation(physicsClientId=world.client)
        st = p.getJointState(robot_id, idx, physicsClientId=world.client)
        pos, vel = st[0], st[1]
        print(f"    size={size} ({label}): torque_cap={torque_cap:.2f} N*m, "
              f"held pos={pos:.4f} vel={vel:.5f}")
        world.disconnect()
        check(f"joint at {label} (size={size}) holds a level limb against gravity",
              abs(pos) < 0.05 and abs(vel) < 0.05,
              f"pos={pos:.4f} vel={vel:.5f} torque_cap={torque_cap:.2f}")


if __name__ == "__main__":
    test_matched_size_converges()
    test_mismatched_child_reaches_target_without_slamming()
    test_narrow_range_limit_actually_holds()
    test_light_parent_does_not_float()
    test_geometric_clamp_holds_under_oversized_torque()
    test_multi_joint_chain_respects_each_joints_own_limit()
    test_joint_can_lift_limb_against_gravity()

    print()
    if failures:
        print(f"{len(failures)} check(s) FAILED: {failures}")
        sys.exit(1)
    else:
        print("All checks passed.")
        sys.exit(0)
