"""physics_engine_(MULTI)
Used to run a pybullet implementation of the physics engine (instead of Bullet sim, 
which was previously used in previous iterations).

This file is currently a work in progress, and is not fully implemented yet,
but the hope is that this engine will allow for a stronger and more easily adoptable/adjustable
implementation of the physics simulation used in the previous ECE mode.

Author: James Hatch
"""

from pathlib import Path

import pybullet as p
import pybullet_data
import numpy as np
import math
import time
import csv
from collections import deque


# ---- All identified part constants ----
# world properties
UNITS_TO_RADS = math.pi
SENSOR_RADIUS = 0.1
DENSITY = 1.9098593171   
FRICTION = 0.8
ROLLING_FRICTION = 0.5

# motors
MOTOR_MAX_IMPULSE = 0.4
DT = 1.0 / 60.0
BASE_TORQUE_CONSTANT = MOTOR_MAX_IMPULSE / DT
# rad/s; passed as maxJointVelocity in create_robot's changeDynamics call so
# every joint is capped here instead of at PyBullet's own unconfigured
# default (100 rad/s). 6 rad/s =~ 340 deg/s, a fast but not absurd peak
# joint speed at this sim's scale.
MAX_MOTOR_SPEED = 6.0
JOINT_DAMPING = 0.5

# bounded PD controller gains (dimensionless relative to the joint torque cap)
# De-saturation of Mechanism 3 ("the PD controller is a disguised bang-bang
# controller", joint-explosion-investigation.md): with KP=2.0, normalized
# torque already exceeded 1 (i.e. the controller was saturated to
# +/-torque_cap, not doing proportional control at all) for any angle_error
# past ~0.5 rad (28.6 deg) -- and an untrained/early-generation ANN has no
# reason to output small errors. Halving KP roughly doubles the error range
# over which the controller is genuinely proportional (now ~1.0 rad / 57.3
# deg) before saturating.
PD_KP = 1.0
PD_KD = 0.5

# Halving PD_KP alone doesn't stop a single saturated step from being a
# shock -- a genome that *does* command a large angle_error still jumps
# straight to torque_cap on the very first step. This directly bounds that:
# the fraction of a joint's own torque_cap that the commanded torque is
# allowed to change by in one physics step (see the slew-rate limiter in
# actuate_joint). 0.2 means a full 0 -> torque_cap ramp takes ~5 steps
# (~83ms at 60Hz) instead of one instantaneous jump into the rigid
# multibody joint -- comparable to biological muscle activation timescales
# (fast-twitch fibers reach peak force in roughly 30-100ms), and still fast
# enough to feel responsive. Expressed as a fraction of torque_cap (not an
# absolute torque) so it scales correctly with body/joint size.
TORQUE_SLEW_FRACTION_PER_STEP = 0.2

# NOTE (2026-09-13, "floating" investigation): a couple of variants of
# "let the PD's damping term react faster than the slew limit allows" were
# tried here -- an additive extra damping term, then an asymmetric slew
# rule that exempted a shrinking/reversing torque -- both intended to fix
# residual small-joint oscillation. Both were tested against the existing
# mismatched-size/narrow-range regression case and found to let that joint
# overshoot its own configured range, which the plain symmetric slew below
# does not. Reverted; see actuate_joint for what actually shipped. Left as
# a candidate for future, more carefully validated tuning.

# Mechanism 6 ("floating" persists after the inverted-limit repair; see
# joint-explosion-investigation.md, 2026-09-13 continued): torque_cap_for_joint's
# size^3 muscle-force law is scaled off the CHILD body's size alone (see that
# function's docstring -- this was Mechanism 5's fix, and it is still correct
# as far as it goes). But a rigid two-body joint's *actual* dynamical
# response is governed by the REDUCED inertia of BOTH bodies about the pivot
# -- exactly like a classic two-body problem, 1/I_reduced = 1/I_parent +
# 1/I_child -- which is dominated by whichever side is lighter, not
# necessarily the child. Mechanism 5's fix handles a light CHILD correctly;
# it has no way to notice a light PARENT. Confirmed directly: a large child
# (e.g. size 6.4) jointed to a much smaller/lighter parent gets a torque_cap
# sized for the large child, but the parent -- not the child -- is what
# actually has to absorb the reaction torque, and its own tiny pivot inertia
# means even the slew-limiter's smallest first-allowed step (20% of
# torque_cap) is already far more than enough to send the joint's relative
# velocity straight to MAX_MOTOR_SPEED in a single 1/60s step (reproduced
# against real evolved-genome diagnostics: commanded_torque = 0.2*torque_cap
# at step 0, velocity_rad_s = +/-6.0 by step 1).
#
# NOTE (2026-09-13, same day, later): a first version of this fix bounded
# torque_cap against an ABSOLUTE target -- "N steps of sustained torque_cap
# to reach MAX_MOTOR_SPEED from rest," via a since-removed
# TORQUE_INERTIA_RAMP_STEPS constant, matching the slew limiter's own
# ~5-step ramp intent. That shipped, was verified against the regression
# suite and both reported "floating" genomes, and looked correct -- but a
# subsequent 500-generation run at gravity -10 came back with essentially
# all joint-driven movement gone (the fittest robots were 2 large segments
# locomoting by toppling over, not articulating). Root cause: inertia
# scales as size^5 while the biological torque law scales as size^3, so
# tying the safety cap to a FIXED absolute velocity/step target made the
# cap shrink far faster than the biological law for genuinely small (but
# perfectly well-MATCHED, non-mismatched) bodies -- e.g. two matched
# 0.1-radius spheres had torque_cap crushed by ~83% even though there was
# no torque/inertia mismatch between them at all; two matched 0.05-radius
# spheres, ~96%. That was an artifact of the absolute target, not a real
# fix requirement -- a joint with no size mismatch should be unaffected by
# Mechanism 6 regardless of how small both bodies happen to be. See
# `_inertia_safety_multiplier` below for the corrected, scale-invariant
# (ratio-based, not absolute-target-based) replacement: dimensionless,
# leaves a matched pair's torque_cap completely untouched regardless of
# absolute size, and only reduces torque_cap as the parent becomes lighter
# relative to the child.


# motor reference
REFERENCE_SIZE = 1.0
REFERENCE_MOMENT_ARM = REFERENCE_SIZE
# (meant to replicate the 0.4 from the legacy code)
REFERENCE_MUSCLE_FORCE = BASE_TORQUE_CONSTANT / REFERENCE_MOMENT_ARM

# spawn overlap handling
BODY_OVERLAP_TOLERANCE = 1e-4
SIBLING_MIN_GAP_FRACTION = 0.03   # clearance epsilon, as a fraction of the smaller radius in a pair
SIBLING_OFFSET_STEP = 0.1        # how much the offset multiplier grows per retry
MAX_SIBLING_OFFSET_MULT = 3.0    # cap; beyond this the joint/subtree is pruned instead

# diagnostics (see step()/_record_diagnostics): tolerances used only for
# flagging "at limit" / "in contact" in the log, not for any physics
JOINT_LIMIT_EPS = 1e-3          # rad; within this of lower/upper_limit counts as "at limit"
CONTACT_PENETRATION_EPS = 1e-6  # m; contact distances more negative than -this count as penetrating

# collision-shape shrink factor applied to every sphere's *collision* shape
# (visual shapes stay full-size) -- kept as one named constant so the
# geometric range-of-motion clamp below reasons about the same radii pybullet
# actually collides, not the nominal blueprint size.
COLLISION_RADIUS_SCALE = 0.95

# a joint whose geometrically-safe swing (see _max_safe_swing_angle) comes
# out smaller than this (radians) is treated as unable to articulate at all
# and is locked at its rest angle rather than left free to explode; this
# should only ever trigger in pathological blueprints, since spawn placement
# already guarantees clearance at the joint's rest angle (theta=0).
MIN_USABLE_SWING_RAD = 1e-3



class BodyPart: ###
    """
    Represents a rigid body (sphere) in the simulation
    Properties include the sphere's ID, position (x,y,z), and size (radius)
    """
    def __init__(self, id, x, y, z, size):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.size = size

class JointPart: ###
    """
    Represents a hinge constraint between two bodies
    Properties include the joint's ID, connecting bodies, 
    and constraint parameters (location, flexion limits, associated motor)
    """
    def __init__(self, id, base_body, other_body, px, py, pz, 
                 ax, ay, az, lower_limit, upper_limit, motor):
        self.id = id
        self.base_body = base_body
        self.other_body = other_body
        self.px = px
        self.py = py
        self.pz = pz
        self.ax = ax
        self.ay = ay
        self.az = az
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.motor = motor

        # Set by _resolve_body_placements_and_prune's geometric range-of-motion
        # clamp (see _max_safe_swing_angle) only when the evolved
        # lower_limit/upper_limit above had to be narrowed to keep this
        # joint's child sphere from swinging back into its parent. None
        # means "not clamped" -- lower_limit/upper_limit above are exactly
        # what was evolved.
        self.geometric_theta_max = None
        self.evolved_lower_limit = None
        self.evolved_upper_limit = None

        # Set by _resolve_body_placements_and_prune if the RAW evolved
        # lower_limit/upper_limit above arrived inverted (lower > upper) --
        # see the "validate/repair the evolved range itself" comment there.
        # False for a normal, correctly-ordered evolved range.
        self.genome_limits_were_inverted = False

        # Raw geometry breadcrumb from the range-of-motion clamp computation
        # (see _max_safe_swing_angle / the clamp block in
        # _resolve_body_placements_and_prune), recorded for EVERY joint that
        # reaches that computation -- regardless of whether the clamp
        # actually narrowed anything. Unlike geometric_theta_max (only set
        # when clamping fires), these let a logged range like the exact
        # (-0.0, -0.0) seen in joint-explosion-investigation.md's "Open
        # question" be told apart directly: a genuinely evolved zero-width
        # range (these breadcrumbs present with clamp_diag_theta_max > 0 and
        # evolved_lower_limit still None, i.e. the clamp never had to touch
        # anything) versus an actual clamp edge case (evolved_lower_limit
        # set) versus this joint never reaching the clamp computation at all
        # (breadcrumbs stay None).
        self.clamp_diag_parent_radius = None
        self.clamp_diag_child_radius = None
        self.clamp_diag_axis_dot_mount = None  # None when the hinge axis was zero-length
        self.clamp_diag_theta_max = None

        # Pivot-to-child-COM distance ("r_off" in _resolve_body_placements_
        # and_prune / _max_safe_swing_angle), recorded alongside the other
        # clamp_diag_* breadcrumbs above. Used by torque_cap_for_joint's
        # Mechanism-6 reduced-inertia safety multiplier (see
        # _inertia_safety_multiplier) to get the child's actual pivot
        # inertia via the parallel axis theorem, rather than approximating
        # it as touching-contact distance (child_radius). None for a joint
        # that never reached that computation (e.g. a hand-built test robot
        # that skips _resolve_body_placements_and_prune) -- callers fall
        # back to child_radius in that case.
        self.clamp_diag_child_pivot_offset = None

class SensorPart: ###
    """
    Represents a touch sensor attached to a body
    Properties include the sensor's ID, associated body, and relative position (x,y,z)
    """
    def __init__(self, id, body_id, x, y, z):
        self.id = id
        self.body_id = body_id
        self.x = x
        self.y = y
        self.z = z


class InvalidBodyPlanError(ValueError):
    """Raised when an evolved blueprint begins with intersecting spheres."""


def _sphere_pivot_inertia(mass, radius, pivot_offset):
    """Rotational inertia of a solid sphere about an axis through a pivot
    point offset from the sphere's own center by `pivot_offset` (parallel
    axis theorem: I = I_cm + m*d^2). Uses the same COLLISION_RADIUS_SCALE-
    shrunk radius PyBullet actually builds the collision sphere from (and
    therefore what it computes automatic mass/inertia from in createMultiBody),
    so this matches the engine's own numbers rather than the nominal
    (visual) blueprint size -- see torque_cap_for_joint / Mechanism 6 in
    joint-explosion-investigation.md.
    """
    collision_radius = COLLISION_RADIUS_SCALE * radius
    i_cm = 0.4 * mass * (collision_radius ** 2)
    return i_cm + mass * (pivot_offset ** 2)


def _clamp_range_to_safe_swing(lower, upper, theta_max):
    """Fit an evolved [lower, upper] joint range inside the geometrically
    safe swing window [-theta_max, theta_max], preserving as much of its
    width as possible instead of clamping each endpoint independently.

    Mechanism 7 (see joint-explosion-investigation.md, 2026-09-13
    continued): the original implementation here was
    `clamped_lower = min(max(lower, -theta_max), theta_max)` and the same
    for `upper`, clamping each endpoint on its own. That is correct and
    width-preserving whenever the evolved range already overlaps the safe
    window -- but when the evolved [lower, upper] lies ENTIRELY on one
    side of it (both endpoints above theta_max, or both below -theta_max --
    an entirely plausible mutation outcome, and confirmed directly against
    real evolved-genome diagnostics to actually occur), both endpoints
    clamp to the SAME boundary value, collapsing the range to a single
    point. `step()`'s own target clamp
    (`max(min(motor_command, upper), lower)`) then discards the ANN's
    output completely whenever `lower == upper`, for exactly the same
    reason an inverted range does (see the "validate/repair" comment
    above) -- the joint is driven toward one fixed, constant angle every
    step, forever, with any apparent motion coming only from PyBullet's
    compliant (not perfectly rigid) joint-limit enforcement being pushed
    around by external forces (contacts, a toppling body), not from the
    ANN. This reads exactly like "motors can't move the robot," and was
    confirmed to be the actual mechanism behind a real report of that
    symptom: two out of three joints in the reported genome had
    `lower_limit == upper_limit` to full float precision, at values
    matching plausible `theta_max` outputs for this sim's scale, not a
    single evolved zero-width range appearing on every joint by chance.

    This instead SHIFTS the interval to fit within the safe window when
    the evolved range lies entirely on one side, preserving up to its
    full original width (clipped to the window's own width, `2*theta_max`,
    when the evolved range is wider than what's geometrically safe at
    all): pushed flush against `+theta_max` if the evolved range was
    entirely above it, flush against `-theta_max` if entirely below.
    When the evolved range already overlaps the window (the common,
    already-correct case), this reduces to the original per-endpoint
    clamp exactly. Never returns a range wider than `2*theta_max` or
    outside `[-theta_max, theta_max]`, so Mechanism 1's self-collision
    safety guarantee is unaffected -- only which *sub-interval* of the
    safe window is kept when the evolved range doesn't fit changes.
    """
    if lower > upper:
        lower, upper = upper, lower  # defensive; callers already repair this

    width = min(upper - lower, 2.0 * theta_max)

    if lower >= theta_max:
        # Evolved range entirely at/above the safe window: push flush
        # against the upper safe boundary instead of collapsing to it.
        new_upper = theta_max
        new_lower = theta_max - width
    elif upper <= -theta_max:
        # Evolved range entirely at/below the safe window: push flush
        # against the lower safe boundary instead of collapsing to it.
        new_lower = -theta_max
        new_upper = -theta_max + width
    else:
        # Already overlaps the safe window -- independent per-endpoint
        # clamping is correct and width-preserving here.
        new_lower = max(lower, -theta_max)
        new_upper = min(upper, theta_max)

    return new_lower, new_upper


def _max_safe_swing_angle(parent_radius, child_pivot_offset, axis_dot_mount, safe_distance):
    """
    Largest symmetric hinge swing |theta| (radians, measured from the
    child's spawn/rest pose) that keeps a child sphere's *center* at least
    `safe_distance` away from the parent sphere's center for every angle in
    [-theta, theta] -- i.e. the largest range of motion that cannot swing
    the child back into its own parent.

    Geometry: the pivot sits on the parent's surface, `parent_radius` from
    the parent's center, along the parent->child mount direction. The child
    sits `child_pivot_offset` from that pivot, initially aligned with the
    same mount direction (theta=0). Rotating the hinge by theta about its
    axis moves the child's center along a circle (or, if the axis isn't
    perpendicular to the mount direction, a smaller circle / cone) centered
    on the pivot. Via Rodrigues' rotation formula, the squared distance from
    the parent's center works out to:

        |C(theta) - P|^2 = d_p^2 + r_off^2 + 2*d_p*r_off*[cos(theta)*(1-c^2) + c^2]

    where d_p = parent_radius, r_off = child_pivot_offset, and
    c = axis_dot_mount = cos(angle between the hinge axis and the mount
    direction). The bracketed term is symmetric in theta and monotonically
    non-increasing in |theta| over [0, pi] (its derivative is
    -sin(theta)*(1-c^2), which is <= 0 there), so solving for where the
    distance first drops to `safe_distance` gives a single, well-defined
    maximum safe |theta|.

    Special case: if the axis is (anti)parallel to the mount direction
    (c^2 == 1), rotating about it never moves the child relative to the
    parent at all -- the child just spins in place -- so any angle is safe
    and this returns pi.
    """
    d_p = parent_radius
    r_off = child_pivot_offset

    if d_p <= 0.0 or r_off <= 0.0:
        return math.pi

    c2 = axis_dot_mount * axis_dot_mount
    denom = 1.0 - c2

    if denom < 1e-9:
        return math.pi

    numerator = (safe_distance ** 2 - d_p ** 2 - r_off ** 2) / (2.0 * d_p * r_off)
    cos_theta_max = (numerator - c2) / denom
    cos_theta_max = max(-1.0, min(1.0, cos_theta_max))
    return math.acos(cos_theta_max)



# ---- A class that defines part contact ----
# originally from - NoiseWorld.cpp
class ContactCallback:
    """Manages collision detection and touch tracking"""
    def __init__(self):
        self.body_touches = {}
        self.touches_point = {}

    def check_collisions(self, physics_client):
        """Check all collisions and update body_touches"""
        contacts = (p.getContactPoints(physicsClientId = physics_client))
        
        # reset all contacts
        for body_id in self.body_touches:
            self.body_touches[body_id] = 0
            self.touches_point[body_id] = np.array([0.0, 0.0, 0.0])
        
        # process current contacts
        for contact in contacts:

            # finding the bodies and links that are contacting + the position on each body that is beng contacted
            body_id_a = contact[1]
            body_id_b = contact[2]
            link_index_a = contact[3]
            link_index_b = contact[4]
            contact_point_a = np.array(contact[5]) 
            contact_point_b = np.array(contact[6])
            
            # map tracking directly to unique (body, link) configurations <--------------------- TODO:
            self.body_touches[(body_id_a, link_index_a)] = 1
            self.touches_point[(body_id_a, link_index_a)] = contact_point_a
            
            self.body_touches[(body_id_b, link_index_b)] = 1
            self.touches_point[(body_id_b, link_index_b)] = contact_point_b




class PyBulletWorld:
    """
    A class that defines the world using the previously defined bodies and collision properties
    """
    
    def __init__(self, gravity=-9.81, dt=DT, headless=True, diagnostics_path=None):
        """
        Initialize PyBullet world using gravity, timestep rate, and mode (GUI vs. headless)

        diagnostics_path: if given, per-step/per-joint diagnostics (commanded
        torque, joint-limit state, parent-link self-contact penetration, base
        speed) are written to this CSV path -- see _record_diagnostics().
        Left as None, no diagnostics file is written and step() behaves as
        before (minus the old raw print of last_joint_states).
        """
        # Connect to PyBullet (GUI or DIRECT mode... we need headless for most cases)
        # Most information taken from: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0
        if headless:
            self.client = p.connect(p.DIRECT)
        else:
            self.client = p.connect(p.GUI)

        # setting the physics solver to more iterations to solve competing constraints moree easily
        # useSplitImpulse separates penetration-recovery velocity from real
        # contact-response velocity so a corrected-for penetration can't
        # leak into the body's actual reported velocity/momentum -- a cheap,
        # standard solver safety net (still open item from the "floating"
        # investigation, joint-explosion-investigation.md, 2026-09-13) that
        # doesn't fix any specific mechanism found so far on its own, but
        # removes one more way a contact-resolution correction could inject
        # spurious velocity into the system.
        p.setPhysicsEngineParameter(
            numSolverIterations=100, useSplitImpulse=1, physicsClientId=self.client
        )

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # world parameters (gravity, timesteps, etc.)
        p.setGravity(0, 0, gravity, physicsClientId=self.client)
        self.gravity = gravity

        p.setTimeStep(dt, physicsClientId=self.client)
        self.dt = dt
        self.time_step = 0
        
        # setting the bounds of the 3D world
        self.world_aabb_min = [-10000, -10000, -10000]
        self.world_aabb_max = [10000, 10000, 10000]
        
        # setting up dictionaries to tie part ids with their Pybullet ids # TODO: Has now been fricken made superfluous
        self.bodies = {} 
        self.bodies_by_id = {}
        self.joints = {}  
        # setting up the lists of all physical robot structures
        self.body_parts = []  
        self.joint_parts = [] 
        self.sensors = []  
        
        # setting upcollision detection using the previously defined ContactCallback class
        self.contact_callback = ContactCallback()
        
        # setting up ANN matrices
        self.weights_s2n = []  # sensor to neuron
        self.weights_n2n = []  # neuron to neuron
        self.weights_s2j = []  # sensor to joint
        self.weights_n2j = []  # neuron to joint
        
        # setting up ANN outputs
        self.sensor_touches = []
        self.output_s2n = []
        self.output_n2n = []
        self.output_s2j = []
        self.output_n2j = []

        # These are updated every step after physics has run.  Keeping the
        # measured state separate from the requested target makes it possible
        # to tell a silent ANN from a motor/constraint that is not moving.
        self.last_motor_targets = {}
        self.last_motor_commands = {}
        self.last_target_velocities = {}
        self.last_torque_caps = {}
        self.last_joint_states = {}
        # Previous step's post-slew-limit applied torque per joint, used by
        # actuate_joint's slew-rate limiter (Mechanism 3 de-saturation).
        # Deliberately a separate dict from last_motor_commands: that one
        # gets overwritten with the raw desired *angle* (not torque) by
        # step() immediately before actuate_joint runs each frame, so it
        # cannot be trusted to hold "last step's torque" at the point
        # actuate_joint needs it.
        self._prev_applied_torque = {}

        self.robot_id = None

        # link_parent_index[i] = pybullet link index of link i's parent, or
        # -1 if link i's parent is the base. Filled in by create_robot();
        # used by _record_diagnostics() to identify parent/child self-contacts.
        self.link_parent_index = []

        # ---- diagnostics (optional CSV log; see _record_diagnostics) ----
        self.diagnostics_path = diagnostics_path
        self._diag_file = None
        self._diag_writer = None
        if self.diagnostics_path is not None:
            self._diag_file = open(self.diagnostics_path, 'w', newline='')
            self._diag_writer = csv.writer(self._diag_file)
            self._diag_writer.writerow([
                "step", "joint_id", "pybullet_idx",
                "position_rad", "velocity_rad_s", "applied_torque",
                "commanded_torque", "torque_cap", "torque_saturated",
                "lower_limit", "upper_limit", "at_lower_limit", "at_upper_limit",
                "genome_limits_were_inverted",
                "parent_contact_penetration_m", "max_self_contact_penetration_m",
                "ground_contact_penetration_m", "max_ground_contact_penetration_m",
                "base_z", "base_linear_speed", "base_angular_speed",
            ])

        # creating a ground plane (defined in the following function)
        self.create_ground()



    # ---- Creating world objects ----
    def create_ground(self): ###
        """
        Create static ground plane at y = 0
        """
        # sets up the plane shape
        ground_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.client)
        
        # creates the ground body (static and massless)
        ground_body = p.createMultiBody(
            baseMass = 0,
            baseCollisionShapeIndex = ground_shape,
            basePosition = [0, 0, 0],  # trying this for now since the bodies keep spawning below it
            physicsClientId = self.client
        )

        # setting up the collision group
        p.setCollisionFilterGroupMask(ground_body, -1, collisionFilterGroup=1, collisionFilterMask=3, physicsClientId=self.client)

        # sets the ground id and contact tracking for the ground body
        self.ground_id = ground_body
        self.contact_callback.body_touches[ground_body] = 0
        self.contact_callback.touches_point[ground_body] = np.array([0.0, 0.0, 0.0])
    

    # ---------- Overlap handling ---------
    @staticmethod
    def _initial_body_overlaps(body_blueprints):
        """Return initially intersecting sphere pairs from a body blueprint."""
        overlaps = []
        for index, body_a in enumerate(body_blueprints):
            center_a = np.array([body_a.x, body_a.y, body_a.z])
            for body_b in body_blueprints[index + 1:]:
                center_b = np.array([body_b.x, body_b.y, body_b.z])
                penetration = body_a.size + body_b.size - np.linalg.norm(center_a - center_b)
                if penetration > BODY_OVERLAP_TOLERANCE:
                    overlaps.append((body_a.id, body_b.id, penetration))
        return overlaps

    @staticmethod
    def _spheres_overlap(center_a, radius_a, center_b, radius_b):
        """True if two spheres interpenetrate by more than BODY_OVERLAP_TOLERANCE."""
        distance = np.linalg.norm(np.asarray(center_a) - np.asarray(center_b))
        penetration = (radius_a + radius_b) - distance
        return penetration > BODY_OVERLAP_TOLERANCE

    @staticmethod
    def _resolve_body_placements_and_prune(body_blueprints, joint_blueprints):
        # retrive the full body map
        body_map = {b.id: b for b in body_blueprints}

        # saving the initial positions post-spawn
        original_positions = {
            b.id: np.array([b.x, b.y, b.z], dtype=float) for b in body_blueprints
        }

        # get a list of joints for every base body
        adjacency_list = {b.id: [] for b in body_blueprints}
        for joint in joint_blueprints:
            if joint.base_body in adjacency_list:
                adjacency_list[joint.base_body].append(joint)

        # find the base body and its branches
        child_ids = {joint.other_body for joint in joint_blueprints}
        base_blueprint = next(
            (b for b in body_blueprints if b.id not in child_ids),
            body_blueprints[0],
        )

        # visited holds every body with a final valid position 
        # (anything added to it gets built for simulation)
        visited = {base_blueprint.id: original_positions[base_blueprint.id]}
        pruned_joint_ids = set()
        pruned_details = []

        # make a queue starting with the base body (traversing from base to children)
        queue = deque([base_blueprint.id])
        while queue:
            # each body withdrawn is a parent, investigate every attached joint for children
            parent_id = queue.popleft()
            parent_blueprint = body_map[parent_id]
            parent_center = visited[parent_id]
            parent_radius = parent_blueprint.size

            # [(center, radius), ...] for this parent's children so far
            placed_siblings = []  

            # loop through children
            for joint in adjacency_list[parent_id]:
                child_id = joint.other_body
                if child_id in visited or child_id not in body_map:
                    continue
 
                child_blueprint = body_map[child_id]
                child_radius = child_blueprint.size
 
                mount_dir = original_positions[child_id] - original_positions[parent_id]
                norm = np.linalg.norm(mount_dir)
                if norm < 1e-9:
                    # Degenerate blueprint (child originally coincides with
                    # parent) -- fall back to the joint's own axis so we at
                    # least get *a* consistent direction instead of
                    # dividing by zero.
                    fallback = np.array([joint.ax, joint.ay, joint.az], dtype=float)
                    if np.linalg.norm(fallback) < 1e-9:
                        fallback = np.array([0.0, 0.0, 1.0])
                    mount_dir = fallback / np.linalg.norm(fallback)
                    print(
                        f"WARNING: body {child_id} originally coincided with "
                        f"parent {parent_id}; falling back to joint axis for "
                        "mount direction."
                    )
                else:
                    mount_dir = mount_dir / norm

                # determiniing needed offset for overlapping joints based on parent/child radii
                gap = SIBLING_MIN_GAP_FRACTION * min(parent_radius, child_radius)
                base_distance = parent_radius + child_radius + gap
 
                offset_mult = 1.0
                accepted_center = None
                while offset_mult <= MAX_SIBLING_OFFSET_MULT + 1e-9:
                    candidate_center = parent_center + mount_dir * base_distance * offset_mult
                    if all(
                        not PyBulletWorld._spheres_overlap(
                            candidate_center, child_radius, sib_center, sib_radius
                        )
                        for sib_center, sib_radius in placed_siblings
                    ):
                        accepted_center = candidate_center
                        break
                    offset_mult += SIBLING_OFFSET_STEP
 
                if accepted_center is None:
                    # Could not clear its siblings within the cap, so prune just this joint
                    # (since we loop over children for each parent, children are pruned to...
                    # I am realizing this sounds terrible out of context lol)
                    pruned_joint_ids.add(joint.id)
                    pruned_details.append({
                        "joint_id": joint.id,
                        "parent_id": parent_id,
                        "child_id": child_id,
                        "reason": "sibling overlap exceeded max offset",
                    })
                    continue
 
                # ccepted: commit the new position, move the joint's pivot
                # to sit on the parent's surface along that same (unchanged)
                # mount direction, and keep walking.
                child_blueprint.x, child_blueprint.y, child_blueprint.z = accepted_center.tolist()
                pivot = parent_center + mount_dir * parent_radius
                joint.px, joint.py, joint.pz = pivot.tolist()

                # ---- geometric range-of-motion clamp ----
                # Nothing above checks whether *rotating* this hinge through
                # its evolved [lower_limit, upper_limit] would swing the
                # child sphere back into its own parent -- only that the
                # spawn pose (theta=0) is clear. Clamp the usable range to
                # the largest swing that provably keeps the child's collision
                # sphere clear of the parent's at every angle in range (see
                # _max_safe_swing_angle for the derivation). This is what
                # prevents PyBullet's rigid multibody joint from ever having
                # to resolve a deep self-penetration with a single-step
                # correction -- the mechanism identified in
                # joint-explosion-investigation.md as the most likely direct
                # cause of low-segment robots "exploding."
                axis_vec = np.array([joint.ax, joint.ay, joint.az], dtype=float)
                axis_norm = np.linalg.norm(axis_vec)
                r_off = float(np.linalg.norm(accepted_center - pivot))
                safe_distance = COLLISION_RADIUS_SCALE * (parent_radius + child_radius) + gap

                if axis_norm < 1e-9:
                    # Zero-length hinge axis; create_robot() rejects this
                    # blueprint outright later with a clear error, so don't
                    # guess a direction here -- just leave the range alone.
                    theta_max = math.pi
                    cos_axis_mount = None
                else:
                    axis_unit = axis_vec / axis_norm
                    cos_axis_mount = float(np.dot(axis_unit, mount_dir))
                    theta_max = _max_safe_swing_angle(
                        parent_radius, r_off, cos_axis_mount, safe_distance
                    )

                # Diagnostic breadcrumb (see JointPart.__init__): recorded
                # unconditionally, for every joint that reaches this point,
                # so a future "why is this range (0,0)" question is directly
                # readable from the log instead of requiring reverse
                # engineering (see joint-explosion-investigation.md's "Open
                # question", 2026-09-13).
                joint.clamp_diag_parent_radius = parent_radius
                joint.clamp_diag_child_radius = child_radius
                joint.clamp_diag_axis_dot_mount = cos_axis_mount
                joint.clamp_diag_theta_max = theta_max
                joint.clamp_diag_child_pivot_offset = r_off

                # ---- validate/repair the evolved range itself ----
                # Nothing in the blueprint loader (main.py) or anywhere else
                # checks that lower_limit <= upper_limit. Mutation perturbs
                # the two bounds independently, so an evolved genome can
                # (and, per a real 80-generation "floating" run, does)
                # produce lower_limit > upper_limit. That inversion is not a
                # harmless no-op: PyBullet's own changeDynamics convention
                # treats an inverted (lower > upper) pair as "unbounded" (it
                # is literally how PyBullet marks a joint as having no
                # limit -- see getJointInfo on a freshly created joint), and
                # separately, step()'s own per-frame target clamp
                # `max(min(motor_command, upper), lower)` *degenerates* when
                # lower > upper: min(x, upper) is always <= upper < lower,
                # so max(..., lower) always returns exactly `lower` --
                # meaning the ANN's actual output is silently thrown away
                # and this joint is driven toward the SAME fixed angle every
                # single step, forever, regardless of what the network
                # computes. Reproduced directly against this codebase (see
                # joint-explosion-investigation.md, "floating" investigation,
                # 2026-09-13): with an inverted range straight from a real
                # evolved genome, the joint never receives an ANN-driven
                # command again and instead chases a permanently-wrong,
                # constant target -- unable to ever settle the way a
                # correctly-bounded joint does, which reads exactly like the
                # non-stop "shaking"/floating that was reported. Repair it
                # here, at the single choke point every joint blueprint
                # passes through, by treating the two evolved numbers as an
                # *interval* regardless of which arrived first: sort them
                # rather than discarding either, so the genome's own
                # (otherwise valid) range keeps mutating and recombining
                # normally instead of this joint being silently bricked.
                joint.genome_limits_were_inverted = joint.lower_limit > joint.upper_limit
                if joint.genome_limits_were_inverted:
                    joint.lower_limit, joint.upper_limit = joint.upper_limit, joint.lower_limit
                    print(
                        f"WARNING: joint {joint.id} (parent {parent_id} -> "
                        f"child {child_id}) had an inverted evolved range "
                        f"(lower={joint.upper_limit:.4f} > upper="
                        f"{joint.lower_limit:.4f} before repair) -- sorted "
                        f"into [{joint.lower_limit:.4f}, {joint.upper_limit:.4f}]. "
                        f"Left unrepaired, this joint would ignore its ANN "
                        f"input entirely (see the comment above this print)."
                    )

                original_lower, original_upper = joint.lower_limit, joint.upper_limit
                clamped_lower, clamped_upper = _clamp_range_to_safe_swing(
                    original_lower, original_upper, theta_max
                )

                if (clamped_lower, clamped_upper) != (original_lower, original_upper):
                    # Keep the evolved values around for debugging/analysis
                    # (see the joint-mapping debug print at the end of
                    # create_robot) without losing them outright.
                    joint.geometric_theta_max = theta_max
                    joint.evolved_lower_limit = original_lower
                    joint.evolved_upper_limit = original_upper
                    joint.lower_limit = clamped_lower
                    joint.upper_limit = clamped_upper
                    if theta_max < MIN_USABLE_SWING_RAD:
                        print(
                            f"WARNING: joint {joint.id} (parent {parent_id} -> "
                            f"child {child_id}) has essentially no geometrically "
                            f"safe range of motion (theta_max={theta_max:.4f} rad "
                            f"given parent_radius={parent_radius:.3f}, "
                            f"child_radius={child_radius:.3f}); it will sit "
                            f"effectively locked near its rest angle instead of "
                            f"its evolved range [{original_lower:.3f}, "
                            f"{original_upper:.3f}]."
                        )

                visited[child_id] = accepted_center
                placed_siblings.append((accepted_center, child_radius))
                queue.append(child_id)

        # return the pruned list
        final_body_blueprints = [b for b in body_blueprints if b.id in visited]
        final_joint_blueprints = [
            j for j in joint_blueprints
            if j.base_body in visited and j.other_body in visited and j.id not in pruned_joint_ids
        ]
        report = {
            "pruned_joint_count": len(pruned_joint_ids),
            "pruned_body_count": len(body_blueprints) - len(final_body_blueprints),
            "pruned_details": pruned_details,
        }
        return final_body_blueprints, final_joint_blueprints, report

    

    def create_robot(self, body_blueprints, joint_blueprints, sensor_blueprints=[]):
        """
        Assembles a single robot structure out of individual 
        body and joint blueprints using reduced-coordinate multibodies.
        """

        # ----------- The base body ------------

        # end if there is no base
        if not body_blueprints:
            return None
        
        # trying something new here... graph tree
        # making a dict of body id's which have values of each body + a similar dict which uses empty lists as the value
        body_map = {b.id: b for b in body_blueprints}
        if len(body_map) != len(body_blueprints):
            raise ValueError("Body blueprint IDs must be unique")


        # ----------- Overlaps -----------

        # overlap handling (initial use of _initial_body_overlaps)
        body_blueprints, joint_blueprints, sibling_report = (
            self._resolve_body_placements_and_prune(body_blueprints, joint_blueprints)
        )
        # >>> debugging check
        '''
        if sibling_report["pruned_joint_count"]:
            print(
                f"INFO: pruned {sibling_report['pruned_joint_count']} joint(s) / "
                f"subtree(s) ({sibling_report['pruned_body_count']} body part(s)) "
                "due to unresolved sibling overlap:"
            )
            for detail in sibling_report["pruned_details"]:
                print(
                    f"       joint {detail['joint_id']}: "
                    f"parent {detail['parent_id']} -> child {detail['child_id']} "
                    f"({detail['reason']})"
                )
        '''
 
        # deal with parts (sensors) that were eliminated
        kept_body_ids = {b.id for b in body_blueprints}
        sensor_blueprints = [s for s in sensor_blueprints if s.body_id in kept_body_ids]
 
        if not body_blueprints:
            return None
 
        body_map = {b.id: b for b in body_blueprints}
        self.bodies_by_id = body_map


        # overlap handling (elimination of residual overlaps)
        overlaps = self._initial_body_overlaps(body_blueprints)
        if overlaps:
            preview = ", ".join(
                f"{body_a}/{body_b} ({penetration:.4f})"
                for body_a, body_b, penetration in overlaps[:5]
            )
            remaining = "" if len(overlaps) <= 5 else f"; +{len(overlaps) - 5} more"
            raise InvalidBodyPlanError(
                "Rejected body blueprint with intersecting spheres: "
                f"{preview}{remaining}"
            )


        blueprint_joint_index_by_id = {joint.id: index for index, joint in enumerate(joint_blueprints)}
        if len(blueprint_joint_index_by_id) != len(joint_blueprints):
            raise ValueError("Joint blueprint IDs must be unique")
        adjacency_list = {b.id: [] for b in body_blueprints}

        # if a base body is in the adjacency list, add the attached joint to the dict value list
        for joint in joint_blueprints:
            if joint.base_body in adjacency_list:
                adjacency_list[joint.base_body].append(joint)


        # map out all children to find the root
        child_ids = {joint.other_body for joint in joint_blueprints}
        # find at least one body that is never a child
        base_blueprint = None
        for b in body_blueprints:
            if b.id not in child_ids:
                base_blueprint = b
                break    
        # fallback to the first body if no child is found
        if base_blueprint is None:
            base_blueprint = body_blueprints[0]

        
        # calculate base body mass/volume (sphere = 4/3 * size^3 * pi)
        base_size = base_blueprint.size
        base_volume = (4.0 / 3.0) * base_size * base_size * base_size * math.pi
        base_mass = base_volume * DENSITY

        # >>> debug
        '''
        print(f"DEBUG: Selected Base ID: {base_blueprint.id} (Type: {type(base_blueprint.id)})")
        print(f"DEBUG: Adjacency list keys: {list(adjacency_list.keys())}")
        print(f"DEBUG: Children of Base: {adjacency_list[base_blueprint.id]}")
        '''

        # create a collision shape and visual shape for the base body
        base_col_id = p.createCollisionShape(
            p.GEOM_SPHERE, 
            radius = base_size * COLLISION_RADIUS_SCALE, # adding these now to prevent weird parent-body collisions
            physicsClientId = self.client
        )
        base_vis_id = p.createVisualShape(
            p.GEOM_SPHERE, 
            radius = base_size,
            physicsClientId = self.client
        )
        
        # set the base body to position 0,0,0
        base_pos = [base_blueprint.x, base_blueprint.y, base_blueprint.z]
        base_orn = [0, 0, 0, 1]

        # shift the entire robot up so the lowest sphere clears the ground (z=0)
        # (only adjusting the base since all other positions are relative to the base)
        min_z = min(b.z - b.size for b in body_blueprints)
        if min_z < 0.01:
            base_pos[2] += -min_z + 0.01


        # ----------- The non-bass bodies ------------

        # preparing body lists
        link_masses = []
        link_collision_shapes = []
        link_visual_shapes = []
        link_positions = []        # (local offset relative to parent frame)
        link_orientations = []     # (local orientation relative to parent frame)
        link_parent_indices = [] 
        link_joint_types = []    # all are just p.JOINT_REVOLUTE, but we need a fricken list man
        link_joint_axes = []     # Hinge axis vector
        link_inertial_positions = []      # inertial tracking and orientations
        link_inertial_orientations = []

        # defining the self variables
        self.bodies = {base_blueprint.id: -1} 
        self.joint_indices_map = {}
        self.joint_parts = []
        self.body_parts = body_blueprints
        self.sensors = sensor_blueprints 

        # used to track world coordinates
        body_frame_world = {
            base_blueprint.id: np.array([base_blueprint.x, base_blueprint.y, base_blueprint.z])
        }

        # using a queue to iterate through the link tree
        queue = deque([base_blueprint.id])

        while queue:
            # removes the first item from the queue (the parent)
            parent_id = queue.popleft()
            
            # find the adjancy list of the parent to find the nearby bodies
            for joint in adjacency_list[parent_id]:
                child_id = joint.other_body
                if child_id in self.bodies:
                    continue 
                
                # find the blueprint of the unencountered child body
                child_blueprint = body_map[child_id]
                parent_blueprint = body_map[parent_id]
                
                # Set the body id to the last value in the self.bodies / self.joints lists
                self.bodies[child_id] = len(link_masses)
                self.joint_indices_map[joint.id] = len(link_masses)
                joint.blueprint_idx = blueprint_joint_index_by_id[joint.id]
                self.joint_parts.append(joint)
                
                # finding the pivot locations
                child_com = np.array([child_blueprint.x, child_blueprint.y, child_blueprint.z])

                # use blueprint joint pivot (world coords) and convert to local offsets
                pivot_world = np.array([joint.px, joint.py, joint.pz])
                parent_local_pivot = pivot_world - body_frame_world[parent_id] # AAA - parent_com
                child_local_pivot = child_com - pivot_world
                # storing world position
                body_frame_world[child_id] = pivot_world

                # add physics values
                child_volume = (4.0 / 3.0) * (child_blueprint.size ** 3) * math.pi
                link_masses.append(child_volume * DENSITY)

                # offset from joint pivot to child body center
                # (to determine where to place collision and visual shapes relative to the joint)
                child_offset = (child_com - pivot_world).tolist()

                # adding collision shapes and graphics shapes
                col_id = p.createCollisionShape(
                    p.GEOM_SPHERE, 
                    radius=child_blueprint.size * COLLISION_RADIUS_SCALE,
                    collisionFramePosition = child_offset,
                    physicsClientId = self.client
                )
                vis_id = p.createVisualShape(
                    p.GEOM_SPHERE, 
                    radius = child_blueprint.size,
                    visualFramePosition = child_offset,
                    physicsClientId = self.client
                )

                # adding the physics values to the body lists
                link_collision_shapes.append(col_id)
                link_visual_shapes.append(vis_id)

                # defining the link orientation and center of mass
                # Use the parent-local pivot as the link position so the joint aligns at the blueprint pivot
                link_positions.append(parent_local_pivot.tolist())
                link_orientations.append([0, 0, 0, 1])
                link_inertial_positions.append(child_local_pivot.tolist())
                link_inertial_orientations.append([0.0, 0.0, 0.0, 1.0])
                
                # shifting body indexes since we start at index -1 for the basee
                parent_link_idx = self.bodies[parent_id]
                if parent_link_idx == -1:
                    link_parent_indices.append(0)  
                else:
                    link_parent_indices.append(parent_link_idx + 1)

                # setting up everything as a Revolute joint
                link_joint_types.append(p.JOINT_REVOLUTE)
                # Axis specified in blueprint is in world coords; since we initialize with identity orientations, use it directly
                joint_axis = [joint.ax, joint.ay, joint.az]
                if np.linalg.norm(joint_axis) == 0:
                    raise ValueError(f"Joint {joint.id} has a zero-length hinge axis")
                link_joint_axes.append(joint_axis)

                queue.append(child_id)


        # create the entire robot multibody
        self.robot_id = p.createMultiBody(
            # base properties
            baseMass = base_mass,
            baseCollisionShapeIndex = base_col_id,
            baseVisualShapeIndex = base_vis_id,
            basePosition = base_pos,
            baseOrientation = base_orn,

            # link properties (with child objects attached)
            linkMasses = link_masses,
            linkCollisionShapeIndices = link_collision_shapes,
            linkVisualShapeIndices = link_visual_shapes,
            linkPositions = link_positions,
            linkOrientations = link_orientations,
            linkInertialFramePositions = link_inertial_positions, 
            linkInertialFrameOrientations = link_inertial_orientations,
            linkParentIndices = link_parent_indices,
            linkJointTypes = link_joint_types,
            linkJointAxis = link_joint_axes,

            # enable self-collision between all links (base + every link), matching
            # legacy Bullet's addConstraint(..., false) behavior of colliding everything.
            # createMultiBody disables self-collision entirely by default otherwise.
            flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT,

            # client again
            physicsClientId = self.client
        )

        # record parent-link mapping for diagnostics (see _record_diagnostics):
        # link_parent_indices uses createMultiBody's convention (0 = base is
        # parent, k = link k-1 is parent); convert to plain pybullet link
        # indices (-1 = base) indexed by this link's own pybullet index.
        self.link_parent_index = [idx - 1 for idx in link_parent_indices]

        mapped_joint_ids = set(self.joint_indices_map)
        expected_joint_ids = set(blueprint_joint_index_by_id)
        if mapped_joint_ids != expected_joint_ids:
            missing = sorted(expected_joint_ids - mapped_joint_ids)
            raise ValueError(
                "Every joint blueprint must be reachable from the selected root; "
                f"unmapped joint IDs: {missing}"
            )
        if p.getNumJoints(self.robot_id, physicsClientId=self.client) != len(joint_blueprints):
            raise RuntimeError("PyBullet joint count does not match the joint blueprint count")


        # setting up joint limits on the created body (iterating over robot's list of joints)
        #
        # maxJointVelocity is set explicitly here rather than left at
        # PyBullet's own default (100 rad/s -- an unconfigured global that
        # has nothing to do with this sim's scale or biology). Without this,
        # a joint whose torque badly outpaces its own rotational inertia
        # (see torque_cap_for_joint below, and Mechanism 5 in
        # joint-explosion-investigation.md) can reach 100 rad/s in a single
        # 1/60s step, overshoot any realistic target by 100*dt =~ 1.667 rad,
        # and settle into a stable wall-to-wall bang-bang limit cycle at
        # that speed every step. Capping it at MAX_MOTOR_SPEED instead bounds
        # the worst-case single-step overshoot to MAX_MOTOR_SPEED*dt, and
        # keeps peak joint speed in a biologically plausible range regardless
        # of how any individual torque_cap/inertia pairing works out.
        for joint in self.joint_parts:
            pybullet_idx = self.joint_indices_map[joint.id]
            p.changeDynamics(
                self.robot_id,
                pybullet_idx,
                jointLowerLimit=float(joint.lower_limit),
                jointUpperLimit=float(joint.upper_limit),
                maxJointVelocity=MAX_MOTOR_SPEED,
                physicsClientId=self.client
            )

        # disabling default motors to allow full motor control YIPEE (this was to resolve an error where joints 
        # could not be actuated and would just swing freely)
        for i in range(p.getNumJoints(self.robot_id, physicsClientId=self.client)):
            p.setJointMotorControl2(
                self.robot_id, 
                i,
                controlMode = p.VELOCITY_CONTROL,
                targetVelocity = 0, # initially stationary
                force = 0,  # non-free-swinging
                physicsClientId = self.client
            )

        
        # surface properties for every sub-link
        for link_idx in range(-1, len(link_masses)):
            p.changeDynamics(
                self.robot_id, link_idx,
                lateralFriction = FRICTION,
                rollingFriction = ROLLING_FRICTION,
                physicsClientId = self.client
            )
        # for debugging (print mapping from blueprint joint id to pybullet joint index,
        # plus whether the geometric range-of-motion clamp had to narrow this
        # joint's evolved [lower_limit, upper_limit] -- see
        # _max_safe_swing_angle / the clamp in _resolve_body_placements_and_prune)
        try:
            for joint in self.joint_parts:
                py_idx = self.joint_indices_map.get(joint.id, None)
                line = (
                    f"DEBUG: joint blueprint id={joint.id} -> pybullet_index={py_idx}, "
                    f"blueprint_idx={getattr(joint, 'blueprint_idx', None)}, "
                    f"motor={getattr(joint, 'motor', False)}, "
                    f"range=[{joint.lower_limit:.3f}, {joint.upper_limit:.3f}]"
                )
                evolved_lower = getattr(joint, 'evolved_lower_limit', None)
                evolved_upper = getattr(joint, 'evolved_upper_limit', None)
                if evolved_lower is not None:
                    line += (
                        f" (geometry-clamped from evolved "
                        f"[{evolved_lower:.3f}, {evolved_upper:.3f}]; "
                        f"theta_max={getattr(joint, 'geometric_theta_max', float('nan')):.3f})"
                    )
                if getattr(joint, 'genome_limits_were_inverted', False):
                    line += " [REPAIRED: evolved range arrived inverted (lower>upper)]"

                # Raw clamp-geometry breadcrumb (see JointPart.__init__ and
                # the clamp block in _resolve_body_placements_and_prune):
                # printed for every joint that reached that computation,
                # clamped or not, so an unexpectedly narrow evolved range
                # (e.g. the (-0.0, -0.0) case in
                # joint-explosion-investigation.md) can be told apart from a
                # clamp artifact just by reading this line, instead of
                # reverse-engineering it after the fact.
                pr = getattr(joint, 'clamp_diag_parent_radius', None)
                cr = getattr(joint, 'clamp_diag_child_radius', None)
                dot = getattr(joint, 'clamp_diag_axis_dot_mount', None)
                cdtm = getattr(joint, 'clamp_diag_theta_max', None)
                if pr is not None:
                    dot_str = f"{dot:.3f}" if dot is not None else "n/a (zero-length axis)"
                    line += (
                        f" [clamp geometry: parent_radius={pr:.3f}, "
                        f"child_radius={cr:.3f}, axis_dot_mount={dot_str}, "
                        f"computed_theta_max={cdtm:.3f}]"
                    )
                print(line)
        except Exception as e:
            print("DEBUG: failed to print joint mappings:", e)
            
        return self.robot_id
    
    
    def add_sensor(self, sensor_part): #<----------------------------------------------------------DO I NEED
        """
        Add a touch sensor to a body
        """
        self.sensors.append(sensor_part)
    

    def PointWorldToLocal(self, bodyIndex, point): ###
        """
        Convert a point from world coordinates to local coordinates for a given body
        """
        if bodyIndex in self.bodies:
            # find the body index
            link_idx = self.bodies[bodyIndex]

            # if the body is the base body
            if link_idx == -1:
                position, orientation = p.getBasePositionAndOrientation(
                    self.robot_id, 
                    physicsClientId=self.client
                )
            # if the body is a child body
            else:
                state = p.getLinkState(
                    self.robot_id, 
                    link_idx, 
                    physicsClientId=self.client
                )
                position, orientation = state[4], state[5] # worldLinkFramePosition, worldLinkFrameOrientation
            
            # invert the transform (gets inverse position and orientation)
            inv_pos, inv_orn = p.invertTransform(position, orientation)

            # apply inverse transform to the point (includes both rotation and translation)
            point_local = np.array(p.multiplyTransforms(inv_pos, inv_orn, point, [0, 0, 0, 1])[0])
            return point_local
        
        return np.array([0.0, 0.0, 0.0]) # surely returning zeros like this in null cases will have no unintended consequences


    

    def AxisWorldToLocal(self, bodyIndex, axis): ###
        """
        Convert an axis from world coordinates to local coordinates for a given body
        """
        if bodyIndex in self.bodies:
            # find the link index
            link_idx = self.bodies[bodyIndex]

            # for the base body
            if link_idx == -1:
                _, orientation = p.getBasePositionAndOrientation(
                    self.robot_id, 
                    physicsClientId = self.client
                )
            
            # for all child bodies
            else:
                state = p.getLinkState(
                    self.robot_id, 
                    link_idx, 
                    physicsClientId = self.client
                )
                orientation = state[5]

            # invert the vector to get the inverse rotation
            inv_orientation = p.invertTransform([0, 0, 0], orientation)[1]

            # apply inverse rotation to convert from world to local coordinates
            axis_local = p.rotateVector(inv_orientation, axis)
            return axis_local
        return axis


    def torque_cap_for_joint(self, joint):
        """Return a biologically scaled maximum joint torque.

        Muscle force scales with cross-sectional area (size^2), while the
        tangential moment arm scales linearly with body size.  Therefore,
        maximum joint torque scales with size^3.

        This is scaled off the CHILD body's size (joint.other_body) -- the
        link this joint actually rotates -- not off whichever of the pair
        happens to be bigger. A joint's usable muscle force should reflect
        the limb it has to move, the same way a real joint's torque is
        limited by the muscles anchored to and moving that limb, not by the
        size of whatever it's attached to on the other side.

        Using max(parent_size, child_size) here previously let a joint
        between a large and a much smaller body get a torque cap sized for
        the LARGE body while that torque was actually applied to accelerate
        the SMALL body's own (much smaller) rotational inertia -- inertia
        scales as size^5, so this was not a small mismatch. In the worst
        cases this was enough to reach PyBullet's global default
        maxJointVelocity in a single step and settle into a stable
        wall-to-wall bang-bang limit cycle every step regardless of the
        joint's own configured range (see Mechanism 5 in
        joint-explosion-investigation.md, reproduced directly against this
        codebase). Scaling off the child alone fixes that at the source;
        MAX_MOTOR_SPEED (see create_robot's changeDynamics call) remains as
        a second, independent backstop against any other torque/inertia
        mismatch this doesn't anticipate.

        Mechanism 6, v2 (see joint-explosion-investigation.md, 2026-09-13
        continued -- this replaces a first version of Mechanism 6 that was
        shipped, then found to over-penalize ordinary small-but-matched
        joints and reverted/corrected in the same investigation): scaling
        off the child alone still leaves a gap when the *parent* is the
        lighter side of the joint -- a large child gets a large biological
        cap regardless of how little inertia its (tiny) parent has to
        resist the reaction torque with. `_inertia_safety_multiplier` below
        catches that case with a dimensionless, scale-invariant multiplier
        in (0, 1]: 1.0 (no change at all) whenever the parent is at least
        as heavy as the child -- exactly the case this biological law was
        already designed around -- falling toward 0 only as the parent
        becomes much lighter than the child. Multiplying (not taking an
        independent min() against an absolute, size-blind velocity/step
        target, which is what the reverted first version did) is what keeps
        this scale-invariant: a matched pair gets no penalty regardless of
        whether it's a matched *small* pair or a matched *large* pair.
        """
        child_size = self.bodies_by_id[joint.other_body].size
        joint_size = child_size

        scale = joint_size / REFERENCE_SIZE
        muscle_force = REFERENCE_MUSCLE_FORCE * (scale ** 2)
        moment_arm = REFERENCE_MOMENT_ARM * scale

        biological_cap = muscle_force * moment_arm

        return biological_cap * self._inertia_safety_multiplier(joint)

    def _inertia_safety_multiplier(self, joint):
        """Mechanism 6 safety multiplier on torque_cap_for_joint's
        biological cap (see that function's docstring).

        Estimates the joint's actual two-body reduced rotational inertia
        about its own pivot (parent and child each treated as an isolated
        sphere, via the parallel axis theorem -- an approximation for a
        multi-link chain, but it captures the dominant effect: the
        immediate neighboring body's mass, confirmed empirically to match
        PyBullet's own reported single-step angular response to within
        numerical precision for an isolated two-body joint), and combines
        them the way a classic two-body problem combines reduced mass
        (1/I_reduced = 1/I_parent + 1/I_child, so the lighter side
        dominates).

        `torque_cap_for_joint`'s biological law already scales off the
        CHILD alone, implicitly assuming the child rotates against an
        effectively infinite/heavy anchor (I_parent >> I_child) -- true
        whenever the parent is the same size or bigger, per Mechanism 5.
        This returns `min(1, 2*I_reduced/I_child)`: exactly 1.0 whenever
        I_parent >= I_child (I_reduced/I_child >= 0.5, so the "min" clamp is
        what actually binds -- no change from the biological law at all,
        regardless of absolute size -- verified against a sweep of matched
        parent/child pairs from radius 0.05 to 20.0: every single one comes
        back with a multiplier of exactly 1.0, unlike the reverted first
        version of this mechanism, which crushed small matched pairs by
        over 90% purely as a side effect of comparing against an absolute
        velocity target instead of this pair's own child-alone baseline),
        and falls smoothly toward 0 as the parent becomes much lighter than
        the child (I_reduced -> I_parent as I_parent -> 0, so the
        multiplier -> 2*I_parent/I_child -> 0) -- exactly the "floating"
        case Mechanism 6 exists to catch. Because this is a ratio against
        the child's own inertia rather than a fixed step/velocity target,
        it stays scale-invariant: worked through algebraically, a joint
        driven at this multiplier-reduced cap produces a single-step
        angular velocity change of at most `2 * biological_cap * dt /
        I_child` regardless of how light the parent is (the parent-mass
        dependence cancels out of the algebra once the multiplier is
        applied) -- and since `biological_cap` and `I_child` are both
        derived from the same child body, this bound inherits the same
        size scaling the biological law already relies on elsewhere in
        this file (see Mechanism 5's own aside on `size^-2` divergence for
        very small children -- a pre-existing, already-accepted residual
        risk this multiplier does not change, since it multiplies to
        exactly 1.0 whenever the child is the lighter side, same as before
        Mechanism 6 existed).

        Returns 1.0 (no adjustment) if the joint's bodies aren't resolvable
        (should not happen for any joint that went through create_robot).
        """
        parent_body = self.bodies_by_id.get(joint.base_body)
        child_body = self.bodies_by_id.get(joint.other_body)
        if parent_body is None or child_body is None:
            return 1.0

        parent_radius = parent_body.size
        child_radius = child_body.size
        if parent_radius <= 0.0 or child_radius <= 0.0:
            return 1.0

        # Pivot-to-COM offset: exact for the parent (_resolve_body_placements_
        # and_prune places the pivot at `parent_center + mount_dir*parent_radius`,
        # i.e. exactly parent_radius from the parent's own center); recorded
        # as a breadcrumb for the child (falls back to the child's own
        # radius -- the typical touching-contact placement -- for a joint
        # built outside that pipeline, e.g. a hand-built test robot).
        parent_pivot_offset = parent_radius
        child_pivot_offset = getattr(joint, 'clamp_diag_child_pivot_offset', None)
        if child_pivot_offset is None:
            child_pivot_offset = child_radius

        parent_mass = (4.0 / 3.0) * math.pi * (parent_radius ** 3) * DENSITY
        child_mass = (4.0 / 3.0) * math.pi * (child_radius ** 3) * DENSITY

        parent_inertia = _sphere_pivot_inertia(parent_mass, parent_radius, parent_pivot_offset)
        child_inertia = _sphere_pivot_inertia(child_mass, child_radius, child_pivot_offset)
        if parent_inertia <= 0.0 or child_inertia <= 0.0:
            return 1.0

        reduced_inertia = (parent_inertia * child_inertia) / (parent_inertia + child_inertia)

        return min(1.0, 2.0 * reduced_inertia / child_inertia)


    def actuate_joint(self, joint_id, desired_angle, dt, torque_cap=BASE_TORQUE_CONSTANT): ###
        """
        Apply a bounded PD torque controller to a joint.

        The neural network supplies a desired joint angle.  The PD controller
        converts position and velocity error into a torque, which is then
        bounded by the biologically scaled maximum joint torque.
        """
        # Get the current joint angle and angular velocity.
        joint_state = p.getJointState(
            self.robot_id, joint_id, physicsClientId=self.client
        )
        current_angle, current_velocity = joint_state[0], joint_state[1]

        # Compute the shortest angular error in (-pi, pi).
        angle_error = desired_angle - current_angle
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Bounded PD torque.  Gains are expressed relative to the joint's
        # maximum torque so that changing body size changes actuator capacity
        # without changing the controller's dimensionless behavior.
        normalized_torque = (PD_KP * angle_error) - (PD_KD * current_velocity)
        torque = torque_cap * normalized_torque
        torque = max(-torque_cap, min(torque_cap, torque))

        # Slew-rate limit (Mechanism 3 de-saturation, see
        # joint-explosion-investigation.md): bound how far the *applied*
        # torque is allowed to move from what was actually applied last
        # step, regardless of what the raw PD law above computes. This is
        # what stops a full 0 -> torque_cap jump from landing on the rigid
        # multibody joint in a single 1/60s step -- lowering PD_KP alone
        # only makes saturation less frequent, it doesn't bound the size of
        # a saturated step once one occurs.
        #
        # An asymmetric version of this (exempting a shrinking/reversing
        # torque from slewing, so the controller's damping could react
        # faster once a joint was already oscillating) was tried during the
        # 2026-09-13 "floating" investigation and reverted: it let a
        # severely size-mismatched joint with a tightly clamped range
        # overshoot its own configured limit, where this symmetric version
        # (verified in that same investigation) does not. Left as a
        # candidate for future, more careful tuning rather than shipped.
        prev_torque = self._prev_applied_torque.get(joint_id, 0.0)
        max_step_change = TORQUE_SLEW_FRACTION_PER_STEP * torque_cap
        torque = max(prev_torque - max_step_change, min(prev_torque + max_step_change, torque))
        self._prev_applied_torque[joint_id] = torque

        # Save requested target/control values for post-step diagnostics.
        self.last_motor_targets[joint_id] = desired_angle
        self.last_target_velocities[joint_id] = None
        self.last_torque_caps[joint_id] = torque_cap
        self.last_motor_commands[joint_id] = torque

        # Apply the bounded torque directly.
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=joint_id,
            controlMode=p.TORQUE_CONTROL,
            force=torque,
            physicsClientId=self.client
        )


    def _zero_vector_for_matrix(self, weights, fallback_length=0):
        """Return a zero-filled output vector matching the matrix width, or the fallback length if provided."""
        if not weights:
            return []

        width = max((len(row) for row in weights), default=0)
        if width == 0 and fallback_length > 0:
            return [0.0] * fallback_length
        if width == 0:
            return []
        if fallback_length > width:
            width = fallback_length
        return [0.0] * width


    @staticmethod
    def _output_at(output, index):
        """Return one ANN contribution, treating an absent pathway as zero."""
        return float(output[index]) if 0 <= index < len(output) else 0.0


    def calculate_layer(self, weights, data_in): ###
        """
        Neural network layer calculation 
        output[j] = sum_i(data_in[i] * weights[i][j])
        """
        if not weights:
            return []

        num_outputs = max((len(row) for row in weights), default=0)
        if num_outputs == 0:
            return []

        if not data_in:
            return [0.0] * num_outputs

        output = [0.0] * num_outputs
        num_inputs = min(len(data_in), len(weights))

        for i in range(num_inputs):
            row = weights[i]
            for j in range(min(len(row), num_outputs)):
                output[j] += float(data_in[i]) * float(row[j])

        return output


    def tanh(self, x):
        """
        Used in steps to adjust the neural network
        Tanh activation: 2/(1+exp(-x)) - 1
        """
        try:
            return 2.0 / (1.0 + math.exp(-x)) - 1.0
        except OverflowError:
            return 1.0 if x > 0 else -1.0
    


    def detect_sensor_touches(self):
        """
        Detect which sensors are in contact
        """
        self.sensor_touches = [0] * len(self.sensors)
        
        # for all sensors...
        for sensor_idx, sensor in enumerate(self.sensors):
            sensor_body_id = self.bodies[sensor.body_id]

            # body/sensor key
            tracking_key = (self.robot_id, sensor_body_id)

            # check if body is touching
            if (tracking_key in self.contact_callback.body_touches) and (self.contact_callback.body_touches[tracking_key] == 1):
                
                # if it is and the part is a sensor, use it for the following functions
                body_part = next((part for part in self.body_parts if part.id == sensor.body_id), None)
                
                if body_part:
                    # global contact points
                    contact_point_world = self.contact_callback.touches_point[tracking_key]
    
                    # Get contact point (local coordinates)
                    contact_point = self.PointWorldToLocal(body_part.id, contact_point_world)
                    
                    body_size = body_part.size
                    sensor_x = sensor.x * body_size
                    sensor_y = sensor.y * body_size
                    sensor_z = sensor.z * body_size
                    
                    # Axis-aligned box check (matching C++)
                    if (((contact_point[0] <= (sensor_x + SENSOR_RADIUS)) and
                         (contact_point[1] <= (sensor_y + SENSOR_RADIUS)) and
                         (contact_point[2] <= (sensor_z + SENSOR_RADIUS))) and
                        ((contact_point[0] >= (sensor_x - SENSOR_RADIUS)) and
                         (contact_point[1] >= (sensor_y - SENSOR_RADIUS)) and
                         (contact_point[2] >= (sensor_z - SENSOR_RADIUS)))):
                        self.sensor_touches[sensor_idx] = 1
    



    def step(self, headless=True):
        """Advance simulation by one timestep and process control"""
        # move camera if in GUI mode
        if (not headless) and (len(self.bodies) > 0):
            
            # Get first body position (same as what I did for save position)
            pos, _ = p.getBasePositionAndOrientation(
                self.robot_id,
                physicsClientId=self.client
            )
            
            p.resetDebugVisualizerCamera(
                cameraDistance = 30,      
                cameraYaw = 50,            
                cameraPitch = -35,         
                cameraTargetPosition = pos 
            )
        
        # Step physics
        p.stepSimulation(physicsClientId=self.client)

        # Query contacts after the step.  This matches the legacy callback:
        # contacts and their local points belong to the pose used for sensing.
        self.contact_callback.check_collisions(self.client)
        
        # Detect sensor touches
        self.detect_sensor_touches()
        
        # Calculate ANN layers.
        self.output_s2n = self.calculate_layer(self.weights_s2n, self.sensor_touches)
        self.output_n2n = self.calculate_layer(self.weights_n2n, self.output_s2n)
        self.output_s2j = self.calculate_layer(self.weights_s2j, self.sensor_touches)
        self.output_n2j = self.calculate_layer(self.weights_n2j, self.output_n2n)

        expected_outputs = len(self.joint_parts)
        if self.weights_s2j and len(self.output_s2j) < expected_outputs:
            print(
                "WARNING: s2j output length does not match the number of joints: "
                f"got {len(self.output_s2j)}, expected at least {len(self.joint_parts)}"
            )
        if self.weights_n2j and len(self.output_n2j) < expected_outputs:
            print(
                "WARNING: n2j output length does not match the number of joints: "
                f"got {len(self.output_n2j)}, expected at least {len(self.joint_parts)}"
            )

        # Actuate joints 
        for joint_idx, joint in enumerate(self.joint_parts):
            if (joint.id in self.joint_indices_map) and getattr(joint, 'motor', False):
                # 1. Fetch the correct PyBullet link/joint index
                pybullet_joint_idx = self.joint_indices_map[joint.id]

                # 2. Find neural net index from blueprint ordering
                nn_index = getattr(joint, 'blueprint_idx', None)
                if nn_index is None:
                    raise RuntimeError(f"Joint {joint.id} has no ANN blueprint index")

                # The legacy controller sums the pathways.  A pathway that
                # does not exist (for example, no neurons) contributes zero;
                # it must not erase a valid signal from the other pathway.
                motor_command = (
                    self._output_at(self.output_s2j, nn_index)
                    + self._output_at(self.output_n2j, nn_index)
                )
                motor_command = self.tanh(motor_command) * UNITS_TO_RADS

                # 3. Clamp to joint blueprint limits if present
                try:
                    low = float(joint.lower_limit)
                    high = float(joint.upper_limit)
                    motor_command = max(min(motor_command, high), low)
                except Exception:
                    pass

                # Apply to the corresponding PyBullet joint.
                torque_cap = self.torque_cap_for_joint(joint)
                self.last_motor_commands[pybullet_joint_idx] = motor_command
                self.actuate_joint(pybullet_joint_idx, motor_command, self.dt, torque_cap)
                '''
                # find the neural net index
                nn_index = joint.blueprint_idx

                if nn_index < len(self.output_s2j) and nn_index < len(self.output_n2j):
                    # Combine outputs
                    motor_command = self.output_s2j[nn_index] + self.output_n2j[nn_index]
                    # Apply tanh activation
                    motor_command = self.tanh(motor_command)
                    # Convert to radians
                    motor_command = motor_command * UNITS_TO_RADS
                else:
                    motor_command = 0.0
                
                # map joint actuation to the link index
                pybullet_joint_index = self.joint_indices_map[joint.id]
                self.actuate_joint(self.robot_id, pybullet_joint_index, motor_command, self.dt)
                '''
        # Record states after physics, so callers can verify that targets are
        # producing changing joint angles over successive calls to step().
        self.last_joint_states = {}
        for joint in self.joint_parts:
            pybullet_joint_idx = self.joint_indices_map[joint.id]
            position, velocity, _, applied_torque = p.getJointState(
                self.robot_id, pybullet_joint_idx, physicsClientId=self.client
            )
            target = self.last_motor_targets.get(pybullet_joint_idx)
            target_velocity = self.last_target_velocities.get(pybullet_joint_idx)
            torque_cap = self.last_torque_caps.get(pybullet_joint_idx)
            motor_command = self.last_motor_commands.get(pybullet_joint_idx)

            self.last_joint_states[joint.id] = {
                "position": position,
                "velocity": velocity,
                "applied_torque": applied_torque,
                "target": target,
                "angle_error": (
                    target - position
                    if target is not None
                    else None
                ),
                "target_velocity": target_velocity,
                "actual_velocity": velocity,
                "torque_cap": torque_cap,
                "motor_command": motor_command,
            }

        # Optional structured diagnostics (see _record_diagnostics): written
        # to diagnostics_path if the world was constructed with one, to help
        # pin down whether a given step's instability is coming from a
        # saturated motor, a joint-limit hit, or a parent/child self-contact.
        if self._diag_writer is not None:
            self._record_diagnostics()

        self.time_step += 1


    def _record_diagnostics(self):
        """
        Write one CSV row per joint for the step that just ran, to
        self.diagnostics_path. Meant to answer, for a known "explosive"
        genome, which failure mode is actually firing at the moment things
        blow up:

          - torque_saturated: the PD controller is at its bounded output
            (see actuate_joint / torque_cap_for_joint) -- a saturated
            command straight into a rigid joint is one hypothesized cause.
          - at_lower_limit / at_upper_limit: the joint is sitting on its
            changeDynamics jointLowerLimit/jointUpperLimit stop.
          - parent_contact_penetration_m: how deep (in meters) this link is
            currently interpenetrating its own parent link, from
            p.getContactPoints self-collision data -- nonzero here is the
            other hypothesized cause (the child sphere swinging back through
            its parent because nothing bounds the joint's range of motion
            to the geometry -- see the θ_max clamp this file is moving
            towards).
          - max_self_contact_penetration_m: worst self-contact penetration
            anywhere on the robot this step (adjacent or not), so a chain
            folding on itself further down shows up too even before that
            link's own parent pair is checked.
          - base_linear_speed / base_angular_speed: lets you scan the log
            for the step index where the base suddenly launches, then look
            at what every joint's columns say on that same step (and the
            few steps before it, since the triggering contact/saturation
            event usually precedes the visible launch by a step or two).
          - base_z: the base link's world height. Added 2026-09-13 (the
            "floating" investigation) -- none of the columns above can
            distinguish "settled on the ground" from "hovering", only
            base_z can. Read alongside ground_contact_penetration_m /
            max_ground_contact_penetration_m (also added then) to tell
            "still touching the ground while shaking" apart from "airborne".
          - genome_limits_were_inverted: True if this joint's raw evolved
            lower_limit/upper_limit arrived with lower > upper and had to be
            sorted back into a valid interval (see the "validate/repair the
            evolved range itself" comment in
            _resolve_body_placements_and_prune). An inverted range, left
            unrepaired, would silently freeze this joint's ANN input to a
            constant, permanently-wrong target every step -- logged here so
            a run that still behaves strangely can be checked against this
            column directly instead of reverse-engineered from lower_limit/
            upper_limit alone.

        Does nothing (and step() does not call this) unless the world was
        constructed with diagnostics_path set.
        """
        # Per-link (parent-adjacent) and global self-contact penetration
        # depths for this step. contact[8] is contactDistance; negative
        # means the two shapes are currently interpenetrating by that many
        # meters, which is what we want to log, not just "touching".
        parent_contact_penetration = {
            idx: 0.0 for idx in range(len(self.link_parent_index))
        }
        max_self_contact_penetration = 0.0

        self_contacts = p.getContactPoints(
            bodyA=self.robot_id, bodyB=self.robot_id, physicsClientId=self.client
        )
        for contact in self_contacts:
            link_a, link_b = contact[3], contact[4]
            contact_distance = contact[8]
            if contact_distance >= -CONTACT_PENETRATION_EPS:
                continue  # not actually penetrating, just touching/near

            penetration = -contact_distance
            max_self_contact_penetration = max(max_self_contact_penetration, penetration)

            # Is this specific contact between a link and its own immediate
            # parent (the case the θ_max range-of-motion clamp targets)?
            parent_of_a = self.link_parent_index[link_a] if link_a >= 0 else -1
            parent_of_b = self.link_parent_index[link_b] if link_b >= 0 else -1
            if link_a >= 0 and parent_of_a == link_b:
                parent_contact_penetration[link_a] = max(
                    parent_contact_penetration[link_a], penetration
                )
            if link_b >= 0 and parent_of_b == link_a:
                parent_contact_penetration[link_b] = max(
                    parent_contact_penetration[link_b], penetration
                )

        # Ground contact penetration, per link (keyed the same way as
        # parent_contact_penetration above; -1 is the base) and globally.
        # Added for the "floating" investigation -- lets a future run show
        # directly whether a shaking segment is still touching the ground
        # (penetrating/re-penetrating it every step) or has actually left it.
        ground_contact_penetration = {
            idx: 0.0 for idx in range(len(self.link_parent_index))
        }
        base_ground_contact_penetration = 0.0
        max_ground_contact_penetration = 0.0
        ground_contacts = p.getContactPoints(
            bodyA=self.robot_id, bodyB=self.ground_id, physicsClientId=self.client
        )
        for contact in ground_contacts:
            link_a = contact[3]  # link index on self.robot_id (bodyA); -1 = base
            contact_distance = contact[8]
            if contact_distance >= -CONTACT_PENETRATION_EPS:
                continue
            penetration = -contact_distance
            max_ground_contact_penetration = max(max_ground_contact_penetration, penetration)
            if link_a >= 0:
                ground_contact_penetration[link_a] = max(
                    ground_contact_penetration[link_a], penetration
                )
            else:
                base_ground_contact_penetration = max(base_ground_contact_penetration, penetration)

        base_position, _ = p.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.client
        )
        base_z = float(base_position[2])

        base_linear_velocity, base_angular_velocity = p.getBaseVelocity(
            self.robot_id, physicsClientId=self.client
        )
        base_linear_speed = float(np.linalg.norm(base_linear_velocity))
        base_angular_speed = float(np.linalg.norm(base_angular_velocity))

        for joint in self.joint_parts:
            pybullet_joint_idx = self.joint_indices_map[joint.id]
            state = self.last_joint_states.get(joint.id, {})

            position = state.get("position")
            velocity = state.get("velocity")
            applied_torque = state.get("applied_torque")
            commanded_torque = state.get("motor_command")
            torque_cap = state.get("torque_cap")

            torque_saturated = (
                commanded_torque is not None
                and torque_cap is not None
                and abs(abs(commanded_torque) - abs(torque_cap)) < 1e-9
            )

            try:
                lower_limit = float(joint.lower_limit)
                upper_limit = float(joint.upper_limit)
            except (TypeError, ValueError):
                lower_limit = upper_limit = None

            at_lower_limit = (
                lower_limit is not None
                and position is not None
                and position <= lower_limit + JOINT_LIMIT_EPS
            )
            at_upper_limit = (
                upper_limit is not None
                and position is not None
                and position >= upper_limit - JOINT_LIMIT_EPS
            )

            self._diag_writer.writerow([
                self.time_step, joint.id, pybullet_joint_idx,
                position, velocity, applied_torque,
                commanded_torque, torque_cap, torque_saturated,
                lower_limit, upper_limit, at_lower_limit, at_upper_limit,
                getattr(joint, 'genome_limits_were_inverted', False),
                parent_contact_penetration.get(pybullet_joint_idx, 0.0),
                max_self_contact_penetration,
                ground_contact_penetration.get(pybullet_joint_idx, 0.0),
                max_ground_contact_penetration,
                base_z, base_linear_speed, base_angular_speed,
            ])

        self._diag_file.flush()


    def get_body_position(self, body_id):
        """Get position of a body inside the robot multibody"""
        if body_id in self.bodies:
            # find the body index
            index = self.bodies[body_id]

            # if the body is the base body
            if index == -1:
                position, _ = p.getBasePositionAndOrientation(
                    self.robot_id, 
                    physicsClientId=self.client
                )

            # if the body is a child body
            else:
                link_state = p.getLinkState(
                    self.robot_id,
                    index,
                    physicsClientId=self.client
                )
                position = link_state[0]

            # return the position
            return position
        return None
    


    def save_position(self, output_file, completed): ###
        """
        Save final position of first body 
        Only calculates distance if completed flag is True
        """
        distance = 0.0
        if self.robot_id is not None and completed:
            # Get first body position
            pos, _ = p.getBasePositionAndOrientation(
                self.robot_id,
                physicsClientId=self.client
            )
            
            # Calculate distance from origin (A^2 + B^2 = C^2)
            distance = math.sqrt(pos[0]**2 + pos[1]**2)
        
        output_path = Path(output_file)

        # create the file if it doesn't exist, otherwise overwrite
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to file
        with open(output_path, 'w') as f:
            f.write(f"{distance}\n")
        
        return distance
    



    def disconnect(self):
        """
        Clean up PyBullet connection
        """
        if self._diag_file is not None:
            self._diag_file.close()
            self._diag_file = None
            self._diag_writer = None
        p.disconnect(physicsClientId=self.client)