# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Metric definitions, pinned against hand-built trajectories.

No simulator, no GPU, no USD. Every metric that decides whether a repair
"worked" is checked here against a trajectory whose answer is known by
construction, so a number in ``docs/PHASE4_REPORT.md`` can be traced back to a
definition somebody can read and test.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from urdf_usd_bridge.sim import metrics as sim_metrics


def _traj(values: list[list[float]], dt: float = 0.01):
    q = np.asarray(values, dtype=float)
    t = np.arange(q.shape[0]) * dt
    return t, q


def test_pose_drift_is_the_worst_error_over_time_and_at_the_end():
    _, q = _traj([[0.0, 0.0], [0.5, 0.0], [0.1, 0.0]])
    out = sim_metrics.pose_drift(q, np.array([0.0, 0.0]))
    assert out["pose_drift_max"] == pytest.approx(0.5)
    assert out["pose_drift_final"] == pytest.approx(0.1)


def test_velocity_stats():
    v = np.array([[3.0, 0.0], [-4.0, 0.0]])
    out = sim_metrics.velocity_stats(v)
    assert out["velocity_peak"] == pytest.approx(4.0)
    assert out["velocity_rms"] == pytest.approx(math.sqrt((9 + 16) / 4))


def test_jitter_is_rms_acceleration():
    dt = 0.1
    # Velocity stepping by 1.0 each sample -> acceleration 10 rad/s^2 throughout.
    v = np.array([[0.0], [1.0], [2.0], [3.0]])
    assert sim_metrics.jitter(v, dt) == pytest.approx(10.0)
    # A constant velocity has no jitter at all.
    assert sim_metrics.jitter(np.array([[2.0], [2.0], [2.0]]), dt) == pytest.approx(0.0)


def test_jitter_is_independent_of_the_pose_being_held():
    """Differentiating velocity, not position, is what makes this true."""
    dt = 0.01
    rng = np.random.default_rng(0)
    noise = rng.normal(size=(200, 1))
    assert sim_metrics.jitter(noise, dt) == pytest.approx(sim_metrics.jitter(noise + 5.0, dt))


def test_energy_drift_is_signed_and_relative():
    assert sim_metrics.energy_drift(np.array([10.0, 11.0])) == pytest.approx(0.1)
    assert sim_metrics.energy_drift(np.array([10.0, 9.0])) == pytest.approx(-0.1)


def test_divergence_has_three_triggers():
    good_q = np.zeros((10, 2))
    good_v = np.zeros((10, 2))
    assert not sim_metrics.diverged(good_q, good_v, target=np.zeros(2))

    nan_q = good_q.copy()
    nan_q[5, 0] = np.nan
    assert sim_metrics.diverged(nan_q, good_v)

    fast_v = good_v.copy()
    fast_v[3, 1] = 1e6
    assert sim_metrics.diverged(good_q, fast_v)

    folded = good_q.copy()
    folded[7, 0] = 4.0  # more than pi from the target
    assert sim_metrics.diverged(folded, good_v, target=np.zeros(2))


def test_settle_time_requires_the_condition_to_hold():
    dt = 0.01
    steps = 300
    t = np.arange(steps) * dt
    v = np.ones((steps, 1)) * 1.0
    v[100:] = 0.0
    assert sim_metrics.settle_time(t, v, window_s=0.5) == pytest.approx(1.0)

    # A single quiet sample on the way past does not count as settled.
    twitchy = np.ones((steps, 1))
    twitchy[100] = 0.0
    assert not np.isfinite(sim_metrics.settle_time(t, twitchy, window_s=0.5))


def test_settle_time_is_infinite_when_it_never_settles():
    t = np.arange(100) * 0.01
    assert not np.isfinite(sim_metrics.settle_time(t, np.ones((100, 1))))


def test_limit_metrics_measure_overshoot_and_sustained_escape():
    dt = 0.01
    lower, upper = np.array([-1.0]), np.array([1.0])

    t, q = _traj([[0.0], [1.02], [0.9]], dt)
    out = sim_metrics.limit_metrics(t, q, lower, upper)
    assert out["limit_overshoot"] == pytest.approx(0.02)
    assert out["limit_escape"] is False  # brief and small

    steps = 100
    t = np.arange(steps) * dt
    q = np.full((steps, 1), 1.30)  # 0.3 past the stop, for a full second
    out = sim_metrics.limit_metrics(t, q, lower, upper)
    assert out["limit_overshoot"] == pytest.approx(0.30)
    assert out["limit_escape"] is True


def test_chatter_frequency_finds_the_oscillation():
    dt = 1.0 / 1000.0
    t = np.arange(1000) * dt
    v = np.sin(2 * math.pi * 37.0 * t).reshape(-1, 1)
    assert sim_metrics.chatter_frequency(v, dt) == pytest.approx(37.0, abs=1.5)


def test_step_response_overshoot_and_settle():
    dt = 0.01
    steps = 400
    t = np.arange(steps) * dt
    q = np.zeros(steps)
    q[:50] = np.linspace(0.0, 1.2, 50)  # rise, overshooting 1.0 by 20%
    q[50:] = 1.0
    out = sim_metrics.step_response(t, q, start=0.0, target=1.0)
    assert out["step_overshoot"] == pytest.approx(0.2, abs=1e-6)
    assert out["step_settle_time"] < 0.6

    # A response that never arrives never settles.
    never = np.linspace(0.0, 0.3, steps)
    assert not np.isfinite(sim_metrics.step_response(t, never, start=0.0, target=1.0)["step_settle_time"])


def test_largest_stable_dt_picks_the_biggest_survivor():
    rows = [
        {"dt": 1 / 30, "diverged": True, "pose_drift_max": float("nan")},
        {"dt": 1 / 60, "diverged": False, "pose_drift_max": 0.30},
        {"dt": 1 / 120, "diverged": False, "pose_drift_max": 0.02},
        {"dt": 1 / 240, "diverged": False, "pose_drift_max": 0.01},
    ]
    # 1/60 drifts past the threshold, so the margin is 1/120.
    assert sim_metrics.largest_stable_dt(rows, drift_limit=0.05) == pytest.approx(1 / 120)
    assert math.isnan(sim_metrics.largest_stable_dt(rows[:1], drift_limit=0.05))


# --- comparison ------------------------------------------------------------


def test_compare_reports_no_measurable_difference():
    """The verdict that makes an honest report possible."""
    out = sim_metrics.compare(1.0, 1.02, "pose_drift_max")
    assert out["verdict"] == "no measurable difference"
    assert out["relative_change"] == pytest.approx(0.02)


def test_compare_knows_which_direction_is_better():
    assert sim_metrics.compare(1.0, 0.5, "pose_drift_max")["verdict"] == "improved"
    assert sim_metrics.compare(1.0, 2.0, "pose_drift_max")["verdict"] == "worse"
    # Bigger is better for a stability margin.
    assert sim_metrics.compare(0.004, 0.008, "dt_max_stable")["verdict"] == "improved"
    assert sim_metrics.compare(0.008, 0.004, "dt_max_stable")["verdict"] == "worse"


def test_compare_handles_a_baseline_that_diverged():
    out = sim_metrics.compare(float("inf"), 1.2, "settle_time")
    assert out["verdict"] == "improved"
    assert "no finite value" in out["note"]

    out = sim_metrics.compare(1.2, float("inf"), "settle_time")
    assert out["verdict"] == "worse"

    assert sim_metrics.compare(float("nan"), float("nan"), "settle_time")["verdict"] == "incomparable"
    assert sim_metrics.compare(None, 1.0, "settle_time")["verdict"] == "incomparable"


# --- agreement -------------------------------------------------------------


def test_agreement_is_the_worst_pairwise_gap():
    per_backend = {
        "physx": {"final_q": [0.10, 0.0], "settle_time": 1.0},
        "newton": {"final_q": [0.12, 0.0], "settle_time": 1.4},
        "mujoco": {"final_q": [0.20, 0.0], "settle_time": 1.1},
    }
    out = sim_metrics.agreement(per_backend)
    assert out["agreement_pose"] == pytest.approx(0.10)  # physx vs mujoco
    assert out["agreement_settle"] == pytest.approx(0.4)  # physx vs newton
    assert out["backends_compared"] == 3


def test_agreement_in_octaves_for_stability_margin():
    out = sim_metrics.agreement(
        {
            "a": {"dt_max_stable": 1 / 60, "final_q": [0.0]},
            "b": {"dt_max_stable": 1 / 240, "final_q": [0.0]},
        }
    )
    assert out["agreement_dt_margin_octaves"] == pytest.approx(2.0)


def test_agreement_needs_two_backends():
    out = sim_metrics.agreement({"physx": {"final_q": [0.0]}})
    assert out["agreement_incomparable"] is True
    assert "agreement_pose" not in out


def test_agreement_counts_diverged_backends():
    out = sim_metrics.agreement(
        {
            "a": {"final_q": [0.0], "diverged": True},
            "b": {"final_q": [0.0], "diverged": False},
        }
    )
    assert out["backends_diverged"] == 1


def test_a_relative_win_on_a_negligible_number_is_not_a_result():
    """Measured: PhysX held a joint to 2.3e-7 rad before and 1.1e-7 rad after.

    That is a 52% relative improvement on a quantity nobody can perceive, and
    reporting it as a win would fill the report with noise.
    """
    out = sim_metrics.compare(2.267e-7, 1.075e-7, "pose_drift_max")
    assert out["verdict"] == "no measurable difference"
    assert "floor" in out["note"]

    # The same relative change on a number that matters is still a win.
    assert sim_metrics.compare(0.13, 0.010, "pose_drift_max")["verdict"] == "improved"
