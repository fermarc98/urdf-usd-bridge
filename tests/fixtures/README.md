# URDF fixtures

Hand-written for `urdf-usd-bridge`. **Nothing here is copied or derived from
anything in `references/`.** Geometry is boxes only, so conversion needs no
external mesh files and the fixtures stay readable.

Each fixture isolates one finding from `docs/ANALYSIS.md` and contains at least
one negative control, so a test that passes for the wrong reason is visible.

| File | Targets | Negative control |
|---|---|---|
| `a_dynamics_damping.urdf` | **G1** — which attribute carries `<dynamics damping/friction>` | prismatic joint (linear units, no angular rescale) |
| `b_inertial_origin_mass_no_inertia.urdf` | **G2** — mass without an inertia tensor; 0.3.2's zero `physics:principalAxes` | `control_link`, fully specified |
| `c_revolute_no_limit.urdf` | **G7** — revolute joint authored as `[0, 0]`, i.e. welded shut | `continuous_joint`, legitimately unbounded; `limited_joint`, ordinary |

## Expected conversion results

Filled in from real runs by `scripts/run_converter_matrix.py`; see
`docs/PHASE2_REPORT.md` for which of these have actually been executed and on
what platform.

| Fixture | urdf-usd-converter 0.3.2 | urdf-usd-converter 0.3.3 |
|---|---|---|
| (a) | `newton:damping`, `newton:friction`; **no** `urdf:dynamics:*`; no `DriveAPI` | same |
| (b) | `physics:mass` set, `physics:diagonalInertia` absent/zero, `physics:principalAxes` authored as a **zero quaternion** | `principalAxes` and `diagonalInertia` left unauthored |
| (c) | `physics:lowerLimit == physics:upperLimit == 0` on `no_limit_joint` and `partial_limit_joint` | same |
