# SPDX-FileCopyrightText: Copyright (c) 2026 urdf-usd-bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""The corpus definition. No network: these test the spec, not the download."""

from __future__ import annotations

from urdf_usd_bridge.sim.corpus import (
    DROPPED_ROBOTS,
    FIXTURE_ROBOTS,
    PUBLIC_ROBOTS,
    RobotSpec,
    corpus,
    fetch,
)


def test_the_corpus_has_at_least_ten_real_robots():
    """Phase 5 target: a corpus big enough that one odd robot cannot carry it."""
    assert len(PUBLIC_ROBOTS) >= 10


def test_the_corpus_spans_more_than_one_morphology():
    """Arms, quadrupeds and humanoids fail differently -- Newton proved that."""
    notes = " ".join(spec.notes.lower() for spec in PUBLIC_ROBOTS)
    assert "arm" in notes
    assert "quadruped" in notes
    assert "humanoid" in notes


def test_every_public_robot_names_a_source_and_a_licence():
    for spec in PUBLIC_ROBOTS:
        assert spec.git_url, spec.label
        assert spec.git_ref, spec.label
        assert spec.urdf_in_repo, spec.label
        assert spec.licence, f"{spec.label} has no licence recorded"


def test_labels_are_unique():
    labels = [spec.label for spec in FIXTURE_ROBOTS + PUBLIC_ROBOTS]
    assert len(labels) == len(set(labels))


def test_robots_from_one_repository_share_a_clone():
    """Twelve robots should not mean twelve downloads."""
    unitree = [s for s in PUBLIC_ROBOTS if s.git_url and "unitree" in s.git_url]
    assert len(unitree) >= 8
    assert {s.share_clone for s in unitree} == {"unitree_ros"}


def test_exploratory_robots_are_excluded_by_default():
    """Decision N2: the quadruped is reported separately from headline numbers."""
    default = {s.label for s in corpus()}
    everything = {s.label for s in corpus(include_exploratory=True)}
    exploratory = {s.label for s in PUBLIC_ROBOTS if s.exploratory}
    assert exploratory
    assert not (exploratory & default)
    assert exploratory <= everything


def test_dropped_robots_record_why():
    assert "franka_panda" in DROPPED_ROBOTS
    assert "ur5" in DROPPED_ROBOTS
    for label, reason in DROPPED_ROBOTS.items():
        assert len(reason) > 30, f"{label} has no real explanation"
        assert label not in {s.label for s in PUBLIC_ROBOTS}


def test_a_missing_urdf_skips_rather_than_substituting_another(tmp_path):
    """Picking "some other URDF in the repo" would measure a different robot."""
    clone = tmp_path / "myrepo"
    (clone / ".git").mkdir(parents=True)
    (clone / "other.urdf").write_text("<robot name='other'/>")

    spec = RobotSpec(
        label="myrepo",
        git_url="https://example.invalid/myrepo",
        git_ref="main",
        urdf_in_repo="robots/wanted.urdf",
    )
    resolved = fetch(spec, cache=tmp_path)
    assert "skipped" in resolved.resolved
    assert "wanted.urdf" in resolved.resolved["skipped"]


def test_a_local_fixture_resolves_without_touching_the_network():
    spec = FIXTURE_ROBOTS[0]
    resolved = fetch(spec)
    assert resolved.resolved["urdf"].endswith(".urdf")
    assert resolved.resolved["commit"] is None
