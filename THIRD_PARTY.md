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

## No source code has been copied or adapted yet

As of Phase 2 the `src/urdf_usd_bridge/` tree is entirely original work. The
attribute-name tables in `src/urdf_usd_bridge/inspection/schemas.py` record
*names* observed in upstream output (`newton:damping`, `mjc:gainPrm`,
`physxJoint:armature`, …). Schema and attribute identifiers are interface facts
needed for interoperability, not expressive code, and no upstream implementation
was reproduced.

## Planned adaptations

These are anticipated for Phase 3 and will be added here **when the code
actually lands**, not before:

| Planned use | Upstream file | Tag / commit | License |
|---|---|---|---|
| MJC actuator gain/bias derivation from PD gains | `source/libraries/isaacsim/asset/importer/utils/python/impl/urdf_to_mjc_physx_conversion_utils.py` | Isaac Sim `v6.1.0` / `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` | Apache-2.0 |
| ω_n / ζ ↔ (stiffness, damping) conversions | `source/extensions/isaacsim.robot_setup.gain_tuner/isaacsim/robot_setup/gain_tuner/gain_tuner_drive_math.py` | Isaac Sim `v6.1.0` / `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` | Apache-2.0 |
| Parallel-axis inertia composition for merged fixed joints | `source/extensions/isaacsim.asset.importer.urdf/python/impl/urdf_utils.py` | Isaac Sim `v6.1.0` / `7c206f75bdadd9e05fc457f19863ca4c3f0cb693` | Apache-2.0 |

## Explicitly excluded

Never copied into this repository:

| Path | Reason |
|---|---|
| `IsaacSim/source/extensions/isaacsim.asset.importer.urdf/config/extension.toml` | `LicenseRef-NvidiaProprietary` |
| `IsaacSim/source/extensions/isaacsim.asset.importer.urdf.ui/config/extension.toml` | `LicenseRef-NvidiaProprietary` |
| `IsaacSim/**/data/**` (Git-LFS USD assets, meshes, textures, MDL) | Covered by NVIDIA's additional software and materials license |
| Omniverse Kit SDK and any of its binaries | Not Apache-2.0 |
