# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Attribute-name tables for every namespace a converted robot may use.

The point of this module is that a single physical quantity -- joint damping,
say -- is spelled differently by each producer, and the spelling is the whole
story: Isaac Sim 6.1.0 reads ``urdf:dynamics:damping`` while the converter it
pins writes ``newton:damping``, so the value is silently dropped
(``docs/history/ANALYSIS.md`` G1). Inspection therefore reports *every* spelling
separately and never collapses them into one number.

Attribute names are read from the prim directly rather than through typed
schema APIs, so an asset inspects correctly even when the ``physx``, ``mjc``
or ``newton`` schema plugins are not registered in the running process.
"""

from __future__ import annotations

#: Version of the JSON report schema emitted by ``inspect --json``.
SCHEMA_VERSION = 1

# --- producers -------------------------------------------------------------

#: Namespace prefix -> the layer it is routed to by Isaac's asset transformer.
NAMESPACE_ROUTING = {
    "physics:": "physics.usda",
    "newton:": "physics.usda",
    "physxJoint:": "physx.usda",
    "physxArticulation:": "physx.usda",
    "physxRigidBody:": "physx.usda",
    "physxCollision:": "physx.usda",
    "physxScene:": "physx.usda",
    "mjc:": "mujoco.usda",
    "urdf:": "(custom, unrouted)",
}

# --- joint dynamics --------------------------------------------------------

#: Every spelling of joint damping, by producer.
JOINT_DAMPING_ATTRS = {
    "newton": "newton:damping",
    "mjc": "mjc:damping",
    "urdf_custom": "urdf:dynamics:damping",
}

#: Every spelling of joint (Coulomb) friction, by producer.
JOINT_FRICTION_ATTRS = {
    "newton": "newton:friction",
    "physx": "physxJoint:jointFriction",
    "mjc": "mjc:frictionloss",
    "urdf_custom": "urdf:dynamics:friction",
}

#: Every spelling of joint armature / rotor inertia, by producer.
JOINT_ARMATURE_ATTRS = {
    "physx": "physxJoint:armature",
    "newton": "newton:armature",
    "mjc": "mjc:armature",
}

#: Velocity limits and limit compliance.
JOINT_LIMIT_EXTRA_ATTRS = {
    "newton_velocity_limit": "newton:velocityLimit",
    "newton_limit_stiffness": "newton:limitStiffness",
    "newton_limit_damping": "newton:limitDamping",
    "mjc_solreflimit": "mjc:solreflimit",
    "mjc_ref": "mjc:ref",
    "urdf_effort": "urdf:limit:effort",
}

#: URDF data the converter parks as inert custom attributes.
URDF_CUSTOM_ATTRS = (
    "urdf:limit:effort",
    "urdf:dynamics:damping",
    "urdf:dynamics:friction",
    "urdf:calibration:rising",
    "urdf:calibration:falling",
    "urdf:calibration:reference_position",
    "urdf:safety_controller:k_velocity",
    "urdf:safety_controller:k_position",
    "urdf:safety_controller:soft_lower_limit",
    "urdf:safety_controller:soft_upper_limit",
)

#: Mimic-joint attributes.
MIMIC_ATTRS = {
    "newton_enabled": "newton:mimicEnabled",
    "newton_coef0": "newton:mimicCoef0",
    "newton_coef1": "newton:mimicCoef1",
}
MIMIC_RELS = {"newton_joint": "newton:mimicJoint"}

# --- drives ----------------------------------------------------------------

#: Suffixes of the multi-apply ``UsdPhysics.DriveAPI``, after ``drive:<inst>:``.
DRIVE_SUFFIXES = (
    "physics:type",
    "physics:maxForce",
    "physics:targetPosition",
    "physics:targetVelocity",
    "physics:damping",
    "physics:stiffness",
)

DRIVE_PREFIX = "drive:"
LIMIT_PREFIX = "limit:"

# --- mass and inertia ------------------------------------------------------

MASS_ATTRS = {
    "mass": "physics:mass",
    "density": "physics:density",
    "center_of_mass": "physics:centerOfMass",
    "diagonal_inertia": "physics:diagonalInertia",
    "principal_axes": "physics:principalAxes",
}

#: Newton's exact body-frame inertia tensor: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
NEWTON_INERTIA_ATTR = "newton:inertia"

# --- collision -------------------------------------------------------------

COLLISION_ATTRS = {
    "collision_enabled": "physics:collisionEnabled",
    "approximation": "physics:approximation",
    "contact_offset": "physxCollision:contactOffset",
    "rest_offset": "physxCollision:restOffset",
}

SELF_COLLISION_ATTRS = {
    "newton": "newton:selfCollisionEnabled",
    "physx": "physxArticulation:enabledSelfCollisions",
}

FILTERED_PAIRS_REL = "physics:filteredPairs"

# --- physics materials -----------------------------------------------------

MATERIAL_ATTRS = {
    "static_friction": "physics:staticFriction",
    "dynamic_friction": "physics:dynamicFriction",
    "restitution": "physics:restitution",
    "density": "physics:density",
}
PHYSICS_MATERIAL_BINDING_REL = "material:binding:physics"

# --- MuJoCo actuators ------------------------------------------------------

MJC_ACTUATOR_TYPE = "MjcActuator"
MJC_ACTUATOR_ATTRS = (
    "mjc:gainType",
    "mjc:biasType",
    "mjc:gainPrm",
    "mjc:biasPrm",
    "mjc:forceRange:min",
    "mjc:forceRange:max",
    "mjc:ctrlRange:min",
    "mjc:ctrlRange:max",
    "mjc:ctrlLimited",
    "mjc:forceLimited",
)
MJC_ACTUATOR_TARGET_REL = "mjc:target"

# --- scene -----------------------------------------------------------------

SCENE_ATTRS = {
    "gravity_direction": "physics:gravityDirection",
    "gravity_magnitude": "physics:gravityMagnitude",
}

# --- prim type names we recognise without schema registration --------------

JOINT_TYPE_NAMES = {
    "PhysicsJoint",
    "PhysicsFixedJoint",
    "PhysicsRevoluteJoint",
    "PhysicsPrismaticJoint",
    "PhysicsSphericalJoint",
    "PhysicsDistanceJoint",
}
ANGULAR_JOINT_TYPE_NAMES = {"PhysicsRevoluteJoint"}
LINEAR_JOINT_TYPE_NAMES = {"PhysicsPrismaticJoint"}
