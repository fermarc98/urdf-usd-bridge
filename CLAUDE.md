# urdf-usd-bridge — project rules

Python library + CLI that converts URDF to USD which simulates **stably** across
PhysX, Newton, and MuJoCo (MJC schemas / MuJoCo Warp).

## Licensing

- **Our license: Apache 2.0.**
- Before reusing any file from `references/`, **check its license header.**
  Only copy or adapt Apache-2.0 code. Never copy files marked proprietary or
  covered by NVIDIA's additional licensing terms (e.g. the NVIDIA Omniverse
  License Agreement, or Isaac Sim's supplemental terms).
- Copied or adapted code **keeps its original copyright header**, gains a
  `Modified by urdf-usd-bridge contributors` line, and is listed in
  `THIRD_PARTY.md` with: source repo, file path, upstream commit/tag, license,
  and a one-line description of the modification.
- `README.md` acknowledges `newton-physics/urdf-usd-converter` and Isaac Sim.

## Reference repos

- `references/` is **read-only**. Never modify, never commit, never `git add`
  anything under it. It is in `.gitignore`; keep it that way.
- Reference repos and pinned states (see `docs/ANALYSIS.md` for detail):
  - `references/urdf-usd-converter` — newton-physics, Apache 2.0
  - `references/mujoco-usd-converter` — newton-physics, Apache 2.0
  - `references/IsaacSim` — tags v6.1.0 (HEAD), v6.0.1, v5.1.0 (shallow)

## Positioning and claims

- **Never imply endorsement** by Newton, NVIDIA, Google DeepMind, or Disney.
  No use of their logos or trademarks beyond nominative "works with X" phrasing.
- Do not describe the project as "official", "certified", or "supported by".

## Versioning

- **Pin and document tested versions**: Isaac Sim, Newton, `mujoco`,
  `usd-exchange`, `usd-core`, and the converter packages. Keep the tested
  matrix in `README.md` and the lockfile/constraints in the repo.

## Working agreements

- Phase 1 is **analysis only** — no library code until the analysis is reviewed.
- Verify claims against the reference sources; do not assume version behavior.
