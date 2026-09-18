# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Trajectory metrics. Pure functions on arrays -- no backend, no GPU, no USD.

Everything in this module takes recorded arrays and returns numbers, which is
what makes the majority of Phase 4 testable in CI on any machine. A metric that
needed a simulator to test would be a metric nobody could check.

Conventions
-----------
``q`` and ``v`` are ``(steps, dofs)`` arrays in SI (radians or metres). ``t`` is
``(steps,)`` in seconds. Metrics are computed **after the settle window**, so an
initial contact or drive transient never lands in a steady-state number; callers
slice first via :func:`after_settle`.

Definitions are the ones in ``docs/PHASE4_DESIGN.md`` section 6, and the tests
in ``tests/unit/test_sim_metrics.py`` pin them against hand-built trajectories.
"""

from __future__ import annotations

from typing import Any

import numpy as np

#: A joint is "at rest" below this speed, in rad/s or m/s.
REST_SPEED = 1e-3

#: How long the speed must stay below :data:`REST_SPEED` to count as settled.
REST_WINDOW_S = 0.5

#: Velocity above which we call the run diverged rather than merely bad.
DIVERGENCE_SPEED = 1e3

#: Position error above which we call the run diverged (radians; a robot that
#: has folded past half a turn is not "drifting").
DIVERGENCE_POSITION = float(np.pi)

#: Sustained overshoot past a joint limit that counts as escaping it.
LIMIT_ESCAPE_RAD = 0.05
LIMIT_ESCAPE_S = 0.1


def after_settle(t: np.ndarray, settle_s: float) -> np.ndarray:
    """Boolean mask selecting the steady-state part of a trajectory."""
    return np.asarray(t) >= settle_s


def _finite(*arrays: np.ndarray) -> bool:
    return all(np.all(np.isfinite(np.asarray(a))) for a in arrays)


def pose_drift(q: np.ndarray, target: np.ndarray) -> dict[str, float]:
    """How far the joints wander from what they were commanded to hold."""
    q = np.asarray(q, dtype=float)
    target = np.asarray(target, dtype=float).reshape(1, -1)
    if q.size == 0:
        return {"pose_drift_max": float("nan"), "pose_drift_final": float("nan")}
    error = np.abs(q - target)
    return {
        "pose_drift_max": float(np.nanmax(error)),
        "pose_drift_final": float(np.nanmax(error[-1])),
    }


def velocity_stats(v: np.ndarray) -> dict[str, float]:
    v = np.asarray(v, dtype=float)
    if v.size == 0:
        return {"velocity_rms": float("nan"), "velocity_peak": float("nan")}
    return {
        "velocity_rms": float(np.sqrt(np.nanmean(np.square(v)))),
        "velocity_peak": float(np.nanmax(np.abs(v))),
    }


def jitter(v: np.ndarray, dt: float) -> float:
    """RMS joint acceleration -- the quantity that reads as buzz.

    Differentiating velocity rather than position keeps this independent of the
    pose the joint is holding, so a robot parked at a large angle and one parked
    at zero are comparable.
    """
    v = np.asarray(v, dtype=float)
    if v.shape[0] < 2 or dt <= 0:
        return float("nan")
    return float(np.sqrt(np.nanmean(np.square(np.diff(v, axis=0) / dt))))


def diverged(q: np.ndarray, v: np.ndarray, *, target: np.ndarray | None = None) -> bool:
    """Three triggers, because divergence has three faces.

    NaN is the obvious one. A solver gaining energy shows up as a velocity
    blow-up first, and a robot quietly folding in half shows up as position
    error -- neither of which is a NaN.

    An ``energy_drift`` metric was written in Phase 4 and **removed in Phase 5**:
    no backend adapter produced an energy series, so it was never populated and
    only looked like something that had been measured. The velocity trigger
    below covers the case it was meant to catch.
    """
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    if q.size == 0 or v.size == 0:
        return True
    if not _finite(q, v):
        return True
    if float(np.nanmax(np.abs(v))) > DIVERGENCE_SPEED:
        return True
    if target is not None:
        error = np.abs(q - np.asarray(target, dtype=float).reshape(1, -1))
        if float(np.nanmax(error)) > DIVERGENCE_POSITION:
            return True
    return False


def settle_time(
    t: np.ndarray,
    v: np.ndarray,
    *,
    tolerance: float = REST_SPEED,
    window_s: float = REST_WINDOW_S,
) -> float:
    """First time after which every joint stays below ``tolerance``.

    Requires the condition to *hold* for ``window_s``, so a trajectory that
    happens to pass through zero velocity on its way somewhere else does not
    count as settled.
    """
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    if t.size == 0 or v.size == 0 or not _finite(v):
        return float("inf")
    slow = np.max(np.abs(v), axis=1) < tolerance
    for index in range(len(t)):
        if not slow[index]:
            continue
        if t[-1] - t[index] < window_s:
            # Not enough trajectory left to prove it stays settled.
            return float("inf")
        horizon = t <= t[index] + window_s
        if np.all(slow[index:][horizon[index:]]):
            return float(t[index])
    return float("inf")


def limit_metrics(t: np.ndarray, q: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> dict[str, Any]:
    """Overshoot past a joint limit, and whether it escaped and stayed out."""
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    lower = np.asarray(lower, dtype=float).reshape(1, -1)
    upper = np.asarray(upper, dtype=float).reshape(1, -1)
    if q.size == 0:
        return {"limit_overshoot": float("nan"), "limit_escape": True}

    over = np.maximum(np.maximum(q - upper, lower - q), 0.0)
    overshoot = float(np.nanmax(over))

    escaped = False
    beyond = np.max(over, axis=1) > LIMIT_ESCAPE_RAD
    if np.any(beyond) and t.size > 1:
        dt = float(np.median(np.diff(t)))
        run = 0
        needed = max(1, round(LIMIT_ESCAPE_S / max(dt, 1e-12)))
        for flag in beyond:
            run = run + 1 if flag else 0
            if run >= needed:
                escaped = True
                break
    return {"limit_overshoot": overshoot, "limit_escape": bool(escaped)}


def chatter_frequency(v: np.ndarray, dt: float) -> float:
    """Dominant oscillation frequency of the joint velocities, in Hz.

    Used at a joint stop, where a solver that cannot settle the constraint
    oscillates at a characteristic rate rather than converging.
    """
    v = np.asarray(v, dtype=float)
    if v.shape[0] < 8 or dt <= 0 or not _finite(v):
        return float("nan")
    signal = v - np.mean(v, axis=0, keepdims=True)
    spectrum = np.abs(np.fft.rfft(signal, axis=0))
    if spectrum.shape[0] < 2:
        return float("nan")
    power = np.sum(np.square(spectrum), axis=1)
    power[0] = 0.0  # ignore the DC component we just removed
    frequencies = np.fft.rfftfreq(signal.shape[0], d=dt)
    return float(frequencies[int(np.argmax(power))])


def step_response(
    t: np.ndarray, q: np.ndarray, *, start: float, target: float, tolerance: float = 0.02
) -> dict[str, float]:
    """Overshoot and settling time for a single-joint step command."""
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float).ravel()
    span = float(target - start)
    if q.size == 0 or abs(span) < 1e-12 or not _finite(q):
        return {"step_overshoot": float("nan"), "step_settle_time": float("inf")}

    peak = float(np.nanmax(q) if span > 0 else np.nanmin(q))
    overshoot = (peak - target) / span
    band = abs(tolerance * span)
    inside = np.abs(q - target) <= band
    settle = float("inf")
    for index in range(len(t)):
        if inside[index] and np.all(inside[index:]):
            settle = float(t[index])
            break
    return {"step_overshoot": float(max(overshoot, 0.0)), "step_settle_time": settle}


def largest_stable_dt(rows: list[dict[str, Any]], *, drift_limit: float) -> float:
    """Largest swept ``dt`` that neither diverged nor drifted past the limit.

    The asset's stability margin. ``rows`` are ``{"dt": float, "diverged": bool,
    "pose_drift_max": float}`` in any order.
    """
    stable = [
        float(row["dt"])
        for row in rows
        if not row.get("diverged", True)
        and np.isfinite(row.get("pose_drift_max", np.inf))
        and row.get("pose_drift_max", np.inf) <= drift_limit
    ]
    return max(stable) if stable else float("nan")


# --- cross-backend agreement ----------------------------------------------


def _pairs(values: dict[str, Any]) -> list[tuple[str, str]]:
    names = sorted(values)
    return [(a, b) for i, a in enumerate(names) for b in names[i + 1 :]]


def agreement(per_backend: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """How far apart the backends end up, given the same asset and commands.

    This is the project's actual claim, so it gets its own metric rather than
    being inferred from each backend improving separately.
    """
    out: dict[str, Any] = {"backends": sorted(per_backend), "backends_compared": len(per_backend)}
    if len(per_backend) < 2:
        out["agreement_incomparable"] = True
        return out

    finals = {
        name: np.asarray(result["final_q"], dtype=float)
        for name, result in per_backend.items()
        if result.get("final_q") is not None
    }
    comparable = {
        name: value
        for name, value in finals.items()
        if value.size and value.size == next(iter(finals.values())).size
    }
    if len(comparable) >= 2:
        out["agreement_pose"] = max(
            float(np.nanmax(np.abs(comparable[a] - comparable[b]))) for a, b in _pairs(comparable)
        )

    for key, metric in (
        ("settle_time", "agreement_settle"),
        ("base_height_final", "agreement_base_height"),
    ):
        values = {
            name: float(result[key])
            for name, result in per_backend.items()
            if result.get(key) is not None and np.isfinite(result.get(key, np.nan))
        }
        if len(values) >= 2:
            out[metric] = max(abs(values[a] - values[b]) for a, b in _pairs(values))

    margins = {
        name: float(result["dt_max_stable"])
        for name, result in per_backend.items()
        if result.get("dt_max_stable") and np.isfinite(result.get("dt_max_stable", np.nan))
    }
    if len(margins) >= 2:
        out["agreement_dt_margin_octaves"] = max(
            abs(np.log2(margins[a]) - np.log2(margins[b])) for a, b in _pairs(margins)
        )

    out["backends_diverged"] = sum(1 for r in per_backend.values() if r.get("diverged"))
    return out


# --- comparison ------------------------------------------------------------

#: Relative change below which we call a difference "no measurable effect".
NEGLIGIBLE = 0.05

#: Absolute floors, below which a metric is physically uninteresting whatever
#: the relative change. Without these, 2.3e-7 rad improving to 1.1e-7 rad reads
#: as a 52% win, and the report fills up with noise dressed as results.
ABSOLUTE_FLOOR = {
    "pose_drift_max": 1e-4,
    "pose_drift_final": 1e-4,
    "velocity_rms": 1e-4,
    "velocity_peak": 1e-3,
    "jitter": 1e-3,
    "limit_overshoot": 1e-3,
    "penetration_max": 1e-4,
    "base_height_error": 1e-3,
    "agreement_pose": 1e-4,
}

#: Metrics where a smaller number is better.
LOWER_IS_BETTER = {
    "pose_drift_max",
    "pose_drift_final",
    "velocity_rms",
    "velocity_peak",
    "jitter",
    "settle_time",
    "penetration_max",
    "base_height_error",
    "limit_overshoot",
    "step_overshoot",
    "step_settle_time",
    "agreement_pose",
    "agreement_settle",
    "agreement_base_height",
    "agreement_dt_margin_octaves",
}

#: Metrics where a larger number is better.
HIGHER_IS_BETTER = {"dt_max_stable", "range_traversed"}


def compare(baseline: float | None, repaired: float | None, metric: str) -> dict[str, Any]:
    """Verdict for one metric: improved, no measurable difference, or worse.

    Returning "no measurable difference" as a first-class verdict is
    deliberate. A repair that changes nothing has to be reportable, or the
    report only ever contains good news.
    """
    result: dict[str, Any] = {"metric": metric, "baseline": baseline, "repaired": repaired}
    if baseline is None or repaired is None:
        result["verdict"] = "incomparable"
        return result
    baseline, repaired = float(baseline), float(repaired)

    if not np.isfinite(baseline) or not np.isfinite(repaired):
        if np.isfinite(repaired) and not np.isfinite(baseline):
            # A baseline that diverged and a repaired run that did not is the
            # strongest result this comparison can produce.
            result["verdict"] = "improved"
            result["note"] = "baseline had no finite value"
        elif np.isfinite(baseline) and not np.isfinite(repaired):
            result["verdict"] = "worse"
            result["note"] = "repaired had no finite value"
        else:
            result["verdict"] = "incomparable"
            result["note"] = "neither value is finite"
        return result

    denominator = max(abs(baseline), 1e-12)
    relative = (repaired - baseline) / denominator
    result["relative_change"] = float(relative)

    floor = ABSOLUTE_FLOOR.get(metric)
    if floor is not None and abs(baseline) < floor and abs(repaired) < floor:
        result["verdict"] = "no measurable difference"
        result["note"] = f"both values are below the {floor:g} floor for this metric"
        return result

    if abs(relative) < NEGLIGIBLE:
        result["verdict"] = "no measurable difference"
        return result
    if metric in HIGHER_IS_BETTER:
        result["verdict"] = "improved" if relative > 0 else "worse"
    else:
        result["verdict"] = "improved" if relative < 0 else "worse"
    return result
