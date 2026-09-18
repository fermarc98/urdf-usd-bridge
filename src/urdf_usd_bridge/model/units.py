# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Unit conventions, in one place, asserted rather than assumed.

Degrees-vs-radians drift across the URDF -> USD -> backend boundary is a
demonstrated, recurring bug class (see ``docs/history/ANALYSIS.md`` G6). Every
conversion in this project goes through this module so the convention is
stated once and testable.

Conventions
-----------
URDF
    Lengths in metres, mass in kilograms, **angles in radians**, angular rates
    in rad/s, revolute stiffness in N*m/rad, revolute damping in N*m*s/rad.
OpenUSD
    Lengths in ``metersPerUnit``, mass in ``kilogramsPerUnit``, **angles in
    degrees**. Angular drive gains are therefore per *degree*: stiffness in
    N*m/deg and damping in N*m*s/deg.
Prismatic joints
    Linear throughout; no angular conversion applies.
"""

from __future__ import annotations

import math

#: Multiply a per-radian coefficient by this to get a per-degree coefficient.
#: This is a coefficient rescale, not an angle conversion, which is why
#: ``math.radians`` is deliberately not used.
PER_RADIAN_TO_PER_DEGREE = math.pi / 180.0

#: Multiply a per-degree coefficient by this to get a per-radian coefficient.
PER_DEGREE_TO_PER_RADIAN = 180.0 / math.pi

ANGULAR = "angular"
LINEAR = "linear"


def angle_urdf_to_usd(radians: float) -> float:
    """Convert a URDF angle (radians) to a USD angle (degrees)."""
    return math.degrees(radians)


def angle_usd_to_urdf(degrees: float) -> float:
    """Convert a USD angle (degrees) to a URDF angle (radians)."""
    return math.radians(degrees)


def gain_urdf_to_usd(value: float, *, angular: bool) -> float:
    """Convert a URDF stiffness/damping coefficient to the USD convention.

    Args:
        value: Coefficient in per-radian units for angular DOFs, or linear
            units for prismatic DOFs.
        angular: True for revolute/continuous DOFs.

    Returns:
        The coefficient in USD's per-degree convention (unchanged when linear).
    """
    return value * PER_RADIAN_TO_PER_DEGREE if angular else value


def gain_usd_to_urdf(value: float, *, angular: bool) -> float:
    """Inverse of :func:`gain_urdf_to_usd`."""
    return value * PER_DEGREE_TO_PER_RADIAN if angular else value
