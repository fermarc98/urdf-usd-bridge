# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The scene every backend is given, and the two guards that keep it honest.

``docs/PHASE4_DESIGN.md`` section 2.3 records how close this phase came to
publishing a confident null result: fixture (a) at its home pose has a vertical
shoulder axis, so gravity exerts no torque about it and the *unrepaired* arm
holds its pose exactly as well as the repaired one. A second probe drifted
1.06 rad for an unrelated reason -- the default ground plane was intersecting
the arm, so contact, not gravity, was moving it.

Both failure modes produce numbers that look like measurements. So the scene is
declared once, here, and two guards must pass before any suite is allowed to
record a metric:

:func:`check_gravity_loaded`
    every actuated joint sees a non-trivial gravity torque at the test pose, so
    "no drift" means the drive held something up.
:func:`check_no_interpenetration`
    no collision geometry starts below the ground plane, so the first steps
    measure the robot rather than a depenetration impulse.

Neither guard is advisory. A suite whose guard fails is recorded as
``guard_failed`` with the reason, and its metrics are withheld rather than
reported with a caveat nobody reads.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

#: Gravity, on the -Z axis both converters declare as up.
GRAVITY = (0.0, 0.0, -9.81)

#: Minimum |gravity torque| about a joint axis for the pose to be "loaded".
#: 1 mN*m is small enough to admit a light gripper finger and large enough to
#: exclude an axis that is simply aligned with gravity.
MIN_GRAVITY_TORQUE = 1e-3

#: Collision geometry may start this far below z=0 before it counts as
#: interpenetrating. One tenth of a millimetre absorbs float32 round-trips in
#: the mesh bounds without admitting a real overlap.
PENETRATION_TOLERANCE = 1e-4


@dataclass
class SceneSpec:
    """Everything a backend needs to build the same scene as the others.

    What is *comparable* across PhysX, Featherstone and MuJoCo is fixed here.
    What is not -- solver iteration counts, contact models, integrator choice --
    is deliberately absent and gets recorded per backend instead, because
    pretending three different solvers were equalised would be the bigger lie.
    """

    gravity: tuple[float, float, float] = GRAVITY
    dt: float = 1.0 / 240.0
    duration_s: float = 5.0
    settle_s: float = 0.5
    ground_plane: bool = True
    ground_friction: float = 1.0
    ground_restitution: float = 0.0
    fixed_base: bool = True
    #: Height above the plane to release a floating base from, in metres.
    drop_height: float = 0.10
    #: Joint targets, per DOF, in SI. ``None`` means "hold the initial pose".
    targets: list[float] | None = None
    notes: dict[str, Any] = field(default_factory=dict)

    @property
    def steps(self) -> int:
        return round(self.duration_s / self.dt)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class GuardResult:
    name: str
    passed: bool
    reason: str
    evidence: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "guard": self.name,
            "passed": self.passed,
            "reason": self.reason,
            "evidence": self.evidence,
        }


def gravity_torque_about(joint, articulation, gravity=GRAVITY, angle: float = 0.0) -> float:
    """Gravity torque about a joint's axis, with that joint turned by ``angle``.

    ``tau(theta) = sum_i axis . (R(axis, theta) (com_i - anchor) x m_i g)`` over
    the subtree the joint carries, with ``R`` the Rodrigues rotation about the
    joint's own axis. For a prismatic DOF the equivalent quantity is the force
    along the axis, ``sum_i m_i (axis . g)``, which does not depend on position.

    The angle matters because a joint can be unloaded at one pose and loaded at
    another -- fixture (a)'s shoulder carries its arm directly above the axis at
    the home pose, where the lever arm is parallel to gravity and the torque is
    exactly zero, and loads up as soon as it turns.
    """
    from ..model.units import LINEAR

    members = articulation.subtree(joint.body1)
    if not members:
        return 0.0
    g = np.asarray(gravity, dtype=float)
    axis = np.asarray([float(v) for v in joint.axis_world], dtype=float)
    anchor = np.asarray([float(v) for v in joint.anchor_world], dtype=float)

    if joint.dof == LINEAR:
        mass = sum(float(articulation.bodies[p].mass or 0.0) for p in members)
        return abs(float(mass * np.dot(axis, g)))

    cos, sin = math.cos(angle), math.sin(angle)
    total = 0.0
    for path in members:
        body = articulation.bodies[path]
        mass = float(body.mass or 0.0)
        r = np.asarray([float(v) for v in body.center_of_mass_world], dtype=float) - anchor
        # Rodrigues: rotate the lever arm about the joint's own axis.
        rotated = r * cos + np.cross(axis, r) * sin + axis * float(np.dot(axis, r)) * (1.0 - cos)
        total += float(np.dot(axis, np.cross(rotated, mass * g)))
    return abs(total)


#: How many angles to try when looking for a pose that loads a joint.
_ANGLE_SAMPLES = 72

#: Keep the chosen pose this far inside the joint's range, as a fraction of it.
#: A joint resting against its own hard limit is held by the *stop*, not by the
#: drive, so a pose on the limit measures the constraint solver and reports it
#: as if the drive had done the work. Observed: fixture (b) in PhysX held to
#: 1e-7 rad with zero stiffness, purely because the target sat on the stop.
LIMIT_INSET_FRACTION = 0.1

#: And never closer than this in absolute terms, for a joint with a huge range.
LIMIT_INSET_MIN_RAD = 0.05


def best_loading_angle(joint, articulation, *, lower=None, upper=None, gravity=GRAVITY):
    """``(angle, torque)`` maximising the gravity torque within the joint range.

    A joint whose lever arms are all parallel to its own axis -- a turntable --
    has zero torque at every angle, and no pose can load it. Returning the
    maximum makes that distinguishable from "unloaded at the pose we happened
    to pick", which is the distinction the guard needs.
    """
    from ..model.units import ANGULAR

    if joint.dof != ANGULAR:
        return 0.0, gravity_torque_about(joint, articulation, gravity)

    low = lower if lower is not None and math.isfinite(lower) else -math.pi
    high = upper if upper is not None and math.isfinite(upper) else math.pi
    if high - low < 1e-9:
        return 0.0, gravity_torque_about(joint, articulation, gravity)

    # Stay off the stops: a joint parked on its limit is held by the constraint,
    # not by the drive, and would report a drive that does nothing as perfect.
    inset = min(LIMIT_INSET_FRACTION * (high - low), max(LIMIT_INSET_MIN_RAD, 0.0))
    if (high - low) > 2 * inset:
        low, high = low + inset, high - inset

    samples = []
    for index in range(_ANGLE_SAMPLES + 1):
        angle = low + (high - low) * index / _ANGLE_SAMPLES
        samples.append((angle, gravity_torque_about(joint, articulation, gravity, angle=angle)))

    best_torque = max(torque for _, torque in samples)
    # Several angles often load a joint equally (an arm swung left or right
    # loads it the same). Prefer the one nearest the asset's authored pose, so
    # the test configuration stays close to the robot as shipped.
    near_best = [angle for angle, torque in samples if torque >= best_torque * 0.99]
    best_angle = min(near_best, key=abs) if near_best else low
    return float(best_angle), float(best_torque)


def check_gravity_loaded(stage, *, pose: dict[str, float] | None = None, gravity=GRAVITY) -> GuardResult:
    """Fail unless every actuated joint is actually loaded at the test pose.

    Without this, a hold-pose result of "zero drift, repaired and unrepaired
    alike" reads as "the repair does not help" when it really means "nothing was
    asked of it".

    The torque is evaluated at the pose that will be commanded, not at the
    asset's home pose, because the two can differ by everything: fixture (a)'s
    shoulder is exactly unloaded at home and well loaded 30 degrees away.
    """
    from ..model.articulation import build_articulation

    articulation = build_articulation(stage)
    actuated = [j for j in articulation.joints if j.dof is not None]
    if not actuated:
        return GuardResult("gravity_loaded", False, "the asset has no actuated joints to load", {"joints": 0})

    pose = pose if pose is not None else loaded_pose(stage)
    torques = {
        j.path: gravity_torque_about(j, articulation, gravity, angle=float(pose.get(j.path, 0.0)))
        for j in actuated
    }
    reachable = {j.path: best_loading_angle(j, articulation, gravity=gravity)[1] for j in actuated}

    loaded = sorted(path for path, value in torques.items() if value >= MIN_GRAVITY_TORQUE)
    unloaded = sorted(path for path, value in torques.items() if value < MIN_GRAVITY_TORQUE)
    unloadable = sorted(p for p in unloaded if reachable.get(p, 0.0) < MIN_GRAVITY_TORQUE)
    missed = [p for p in unloaded if p not in unloadable]

    evidence = {
        "gravity_torque_per_joint": {k: round(v, 9) for k, v in sorted(torques.items())},
        "max_reachable_torque": {k: round(v, 9) for k, v in sorted(reachable.items())},
        "loaded_joints": loaded,
        "unloadable_joints": unloadable,
        "unloaded_at_this_pose": missed,
        "threshold": MIN_GRAVITY_TORQUE,
        "pose": {k: round(float(v), 9) for k, v in sorted((pose or {}).items())},
    }

    # A joint that *could* have been loaded is the more specific diagnosis, so
    # it is reported first even when nothing ended up loaded.
    if missed:
        return GuardResult(
            "gravity_loaded",
            False,
            (
                f"{len(missed)} joint(s) see no gravity torque at the commanded pose though a "
                "loading angle exists: " + ", ".join(p.rsplit("/", 1)[-1] for p in missed)
            ),
            evidence,
        )

    if not loaded:
        return GuardResult(
            "gravity_loaded",
            False,
            (
                "no actuated joint is gravity-loaded at this pose, so holding it would prove "
                "nothing about the drives: " + ", ".join(p.rsplit("/", 1)[-1] for p in unloaded)
            ),
            evidence,
        )

    note = ""
    if unloadable:
        note = (
            f"; {len(unloadable)} joint(s) excluded from the measurement because their mass sits "
            "on the joint axis and no pose can load them: "
            + ", ".join(p.rsplit("/", 1)[-1] for p in unloadable)
        )
    return GuardResult(
        "gravity_loaded",
        True,
        f"{len(loaded)} of {len(actuated)} actuated joints are gravity-loaded at this pose{note}",
        evidence,
    )


def collision_bounds_min_z(stage) -> tuple[float, str | None]:
    """Lowest point of any collision geometry, in stage units.

    Collision geometry is what the solver contacts, and the converter gives it
    ``purpose = guide``, so the bound is computed over the guide purpose rather
    than over the visual meshes -- which can hang lower without ever touching
    anything.
    """
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.guide, UsdGeom.Tokens.default_])
    lowest = math.inf
    culprit = None
    for prim in stage.TraverseAll():
        if "PhysicsCollisionAPI" not in set(prim.GetAppliedSchemas()):
            continue
        try:
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        except Exception:  # pragma: no cover - defensive across USD versions
            continue
        if box.IsEmpty():
            continue
        z = float(box.GetMin()[2])
        if z < lowest:
            lowest, culprit = z, prim.GetPath().pathString
    return (lowest if math.isfinite(lowest) else math.nan), culprit


def check_no_interpenetration(stage, *, ground_z: float = 0.0, lift: float = 0.0) -> GuardResult:
    """Fail if any collider starts below the ground plane.

    A robot spawned intersecting the floor gets a depenetration impulse on step
    zero, and every metric afterwards measures that impulse rather than the
    asset. This is the artefact that made a repaired arm look like it was
    failing to hold its pose.
    """
    lowest, culprit = collision_bounds_min_z(stage)
    if not math.isfinite(lowest):
        return GuardResult("no_interpenetration", False, "no collision geometry found to check", {})
    clearance = float(lowest + lift - ground_z)
    evidence = {
        "lowest_collider_z": lowest,
        "lift_applied": lift,
        "ground_z": ground_z,
        "clearance": clearance,
        "lowest_collider": culprit,
    }
    if clearance < -PENETRATION_TOLERANCE:
        return GuardResult(
            "no_interpenetration",
            False,
            (
                f"collision geometry starts {-clearance:.6g} below the ground plane "
                f"({culprit}); contact forces would dominate every metric"
            ),
            evidence,
        )
    return GuardResult(
        "no_interpenetration", True, f"clearance {clearance:.6g} above the ground plane", evidence
    )


def required_lift(stage, *, ground_z: float = 0.0, margin: float = 0.0) -> float:
    """How far to raise the asset so nothing starts below the plane."""
    lowest, _ = collision_bounds_min_z(stage)
    if not math.isfinite(lowest):
        return 0.0
    return max(0.0, ground_z - lowest + margin)


def loaded_pose(stage, *, offset_rad: float = math.pi / 6.0) -> dict[str, float]:
    """A joint pose that gravity actually acts on.

    For each revolute joint, the angle within its range that maximises the
    gravity torque about its own axis. That is a real choice rather than a
    convention: the midpoint of the range is exactly the unloaded pose for a
    joint whose arm hangs along the axis, which is how fixture (a) is built.

    Joints that cannot be loaded at any angle keep ``offset_rad`` so the pose is
    still well defined; :func:`check_gravity_loaded` is what refuses to report
    metrics for them.
    """
    from ..model.articulation import build_articulation
    from ..model.units import ANGULAR

    articulation = build_articulation(stage)
    pose: dict[str, float] = {}
    for joint in articulation.joints:
        if joint.dof is None:
            continue
        prim = stage.GetPrimAtPath(joint.path)
        lower = _authored_float(prim, "physics:lowerLimit")
        upper = _authored_float(prim, "physics:upperLimit")
        if joint.dof != ANGULAR:
            pose[joint.path] = 0.0
            continue

        low = math.radians(lower) if lower is not None and math.isfinite(lower) else None
        high = math.radians(upper) if upper is not None and math.isfinite(upper) else None
        if low is not None and high is not None and abs(high - low) < 1e-9:
            pose[joint.path] = float(low)  # locked; the limit rules report it separately
            continue

        angle, torque = best_loading_angle(joint, articulation, lower=low, upper=high)
        if torque < MIN_GRAVITY_TORQUE:
            angle = offset_rad
            if low is not None and high is not None:
                angle = min(max(offset_rad, low), high)
        pose[joint.path] = float(angle)
    return pose


def _authored_float(prim, name: str) -> float | None:
    if not prim or not prim.IsValid():
        return None
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid() or not attr.HasAuthoredValue():
        return None
    try:
        return float(attr.Get())
    except (TypeError, ValueError):  # pragma: no cover - defensive
        return None
