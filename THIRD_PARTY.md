# Third-party code

This file records every file in this repository that was copied or adapted from
another project, as required by `CLAUDE.md`.

Policy:

- Only Apache-2.0 material is copied or adapted. Files marked proprietary, or
  covered by NVIDIA's additional Isaac Sim licensing terms, are never copied.
- Adapted files keep their original `SPDX-FileCopyrightText` header and carry a
  `Modified by urdf-usd-bridge contributors` line.
- Each entry below records the upstream repository, path, tag, commit, license,
  and what was changed.

## Current entries

### `LICENSE`

| Field | Value |
|---|---|
| Upstream | canonical Apache License 2.0 text |
| Source used | `references/urdf-usd-converter/LICENSE.md` @ `v0.3.3` (`b636469`) |
| License | Apache-2.0 |
| Change | Appendix boilerplate completed with `Copyright 2026 urdf-usd-bridge contributors` |

The Apache-2.0 text is the license itself, not licensed work; it is reproduced
verbatim as the license requires.

## No source code has been copied or adapted

As of v0.1.0 the `src/urdf_usd_bridge/` tree is entirely original work. No
upstream file has been copied, and no upstream implementation has been
reproduced. The attribute-name tables in
`src/urdf_usd_bridge/inspection/schemas.py` and the repair modules record
*names* observed in upstream output (`newton:damping`, `mjc:gainPrm`,
`physxJoint:armature`, …). Schema and attribute identifiers are interface facts
needed for interoperability, not expressive code.

## Derivation references

Phase 3 implements two things whose *definition* was taken from an Apache-2.0
NVIDIA file, without copying its code. Both are recorded here because the
project's source cites them, and a reader should be able to check the citation.
Neither carries an upstream copyright header, because neither reproduces
upstream expression — adding one would misattribute original work.

### Second-order gain relations

| Field | Value |
|---|---|
| Our code | `src/urdf_usd_bridge/repair/drives.py`, `gains_from_frequency` |
| Reference | `source/extensions/isaacsim.robot_setup.gain_tuner/isaacsim/robot_setup/gain_tuner/gain_tuner_drive_math.py`, `stiffness_and_damping_from_natural_frequency_position_drive` |
| Upstream | Isaac Sim `v6.1.0` / `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` |
| License | Apache-2.0 |
| What was taken | The relations `K = I (2 pi f_n)^2` and `D = 2 zeta sqrt(I K)`, and the fact that the stored-gain scale is `pi/180` for angular DOFs and `1.0` for linear ones. These are textbook second-order relations; the upstream file is cited as the authority for the *stored-gain convention*, which is the part that is not textbook. |
| What differs | Four lines written from the formula. The stored-gain conversion is not performed here at all -- it lives in `src/urdf_usd_bridge/model/units.py`, so one table governs every backend rather than each call site deciding. |

### MJCF position-servo gain encoding

| Field | Value |
|---|---|
| Our code | `src/urdf_usd_bridge/repair/drives.py`, `_author_mujoco` |
| Reference | `source/libraries/isaacsim/asset/importer/utils/python/impl/urdf_to_mjc_physx_conversion_utils.py`, `create_mjc_actuator_from_physics` (lines 279-301) |
| Upstream | Isaac Sim `v6.1.0` / `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` |
| License | Apache-2.0 |
| What was taken | The slot layout that expresses a PD position servo as an MJCF actuator: `gainPrm = [kp, 0, ...]`, `biasPrm = [0, -kp, -kd, ...]`, `gainType = "fixed"`, `biasType = "affine"`. This is an interface fact about MJCF, documented in comments in that function. |
| What differs | **The units.** Upstream writes the per-degree `UsdPhysics` value into the per-radian MJCF slot, a 57.3x error (`docs/UPSTREAM_ISSUES.md` issue 2a, confirmed at runtime). We author the value in radians, and `tests/unit/test_repair_units.py` fails if that ever regresses. We also author the attributes with the `uniform double` types the MJC schema declares, rather than `Float`. |

`source/extensions/isaacsim.asset.importer.urdf/python/impl/urdf_utils.py`
(parallel-axis composition for merged fixed joints) was listed as a planned
adaptation in Phase 2. It was **not** used: the parallel-axis sums in
`src/urdf_usd_bridge/model/articulation.py` and
`src/urdf_usd_bridge/repair/geometry_inertia.py` are written from the theorem
directly, and serve a different purpose (equivalent inertia about a joint axis,
not inertia composition across a merged joint).

## Distribution

Neither the wheel nor the sdist bundles, vendors or redistributes any
third-party source. Both contain only `src/urdf_usd_bridge/` (plus, in the
sdist, this project's own tests, docs, examples and scripts). `references/` is
`.gitignore`d, is excluded from the sdist, and is never read at run time --
it exists only so a reader can check the citations above.

Upstream projects are consumed as **declared dependencies**, resolved by the
installer from PyPI under their own licenses, never copied into this tree. The
optional extras in `pyproject.toml` name them and the versions they were tested
against.

## Audit log

### v0.1.0 -- 2026-09-18

Scope: every file in `src/`, `tests/`, `scripts/` and `examples/`, plus the
packaging inputs. Method and result:

| Check | Result |
|---|---|
| SPDX header on every Python file (64 files) | **pass** -- all 64 carry `Apache-2.0`; three empty `tests/**/__init__.py` files were missing one and were given it during this audit |
| Any `SPDX-FileCopyrightText` naming a party other than this project | **none found** -- no upstream copyright line exists in the tree, which is consistent with nothing having been copied |
| Any `Modified by urdf-usd-bridge contributors` line | **none found** -- correct, since that line marks adapted files and there are none |
| `LICENSE` entry above still resolves | **pass** -- `references/urdf-usd-converter` is at `v0.3.3` / `b6364698371346a8e01bdbbb9bae312b78c67975` |
| Derivation reference 1 still resolves | **pass** -- `gain_tuner_drive_math.py` exists at the cited path in `references/IsaacSim` @ `v6.1.0` / `7c206f75`, header reads `SPDX-License-Identifier: Apache-2.0`, and `stiffness_and_damping_from_natural_frequency_position_drive` is at line 122 |
| Derivation reference 2 still resolves | **pass** -- `urdf_to_mjc_physx_conversion_utils.py` exists at the cited path, same tag and license, `create_mjc_actuator_from_physics` at line 234, and the cited slot layout is at lines 279-301 as recorded |
| Wheel and sdist contents | **pass** -- no third-party file in either; see **Distribution** above |
| `references/` tracked by git | **no** -- ignored, and `git status` reports nothing under it |

Nothing was found that required a new entry. The two derivation references and
the `LICENSE` entry remain the complete set.

## Explicitly excluded

Never copied into this repository:

| Path | Reason |
|---|---|
| `IsaacSim/source/extensions/isaacsim.asset.importer.urdf/config/extension.toml` | `LicenseRef-NvidiaProprietary` |
| `IsaacSim/source/extensions/isaacsim.asset.importer.urdf.ui/config/extension.toml` | `LicenseRef-NvidiaProprietary` |
| `IsaacSim/**/data/**` (Git-LFS USD assets, meshes, textures, MDL) | Covered by NVIDIA's additional software and materials license |
| Omniverse Kit SDK and any of its binaries | Not Apache-2.0 |
