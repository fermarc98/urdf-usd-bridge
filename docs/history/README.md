# Development records

These are not user documentation. They are the working documents the project
was built from — the prior-art analysis, the design proposals that were
reviewed before any code was written, and the report written at the end of each
phase. They are kept because the claims in the current documentation were
*produced* here, and a reader who wants to check one should be able to see the
method, the raw numbers, and the things that were tried and did not work.

They are **not maintained**. Each describes the state of the project on the day
it was written. Where one disagrees with the current documentation, the current
documentation is right.

**If you are trying to use the project**, none of this is the place to start:

| You want | Read |
|---|---|
| What it does, and how to install it | [`../../README.md`](../../README.md) |
| What each repair rule does and why | [`../HOW_IT_WORKS.md`](../HOW_IT_WORKS.md) |
| The CLI and the Python API | [`../API.md`](../API.md) |
| The measured results | [`../BENCHMARK.md`](../BENCHMARK.md) |
| What is not done yet | [`../ROADMAP.md`](../ROADMAP.md) |

## What is here

| Document | Date | What it is |
|---|---|---|
| [`ANALYSIS.md`](ANALYSIS.md) | 2026-09-17 | Phase 1. The prior-art read: what `urdf-usd-converter`, `mujoco-usd-converter` and Isaac Sim's importer actually do, cited to file:line in the pinned checkouts, and the catalogue of gaps **G1–G10** that the whole project is organised around. Still the reference for what a gap number means |
| [`PHASE2_REPORT.md`](PHASE2_REPORT.md) | 2026-09-17 | The scaffold, and the first pass at turning Phase 1's source-level claims into executed evidence. Written on macOS, which is why it stops at tier T0 |
| [`PHASE3_DESIGN.md`](PHASE3_DESIGN.md) | 2026-09-18 | The repair rules proposed as formulas, before implementation. The derivations behind the gain and armature relations live here |
| [`PHASE3_REPORT.md`](PHASE3_REPORT.md) | 2026-09-18 | The stability layer as built: how each rule authors its opinion, and why it is a separate sublayer |
| [`PHASE4_DESIGN.md`](PHASE4_DESIGN.md) | 2026-09-18 | The simulation harness proposed: the scene contract, the metric definitions, and the guards that refuse to report a meaningless number |
| [`PHASE4_REPORT.md`](PHASE4_REPORT.md) | 2026-09-18 | **The measurements.** Every tuning default the project ships is justified here, including the sweep that found `armature_fraction` does nothing |
| [`PHASE5_REPORT.md`](PHASE5_REPORT.md) | 2026-09-18 | The 12-robot corpus, and the elimination table for the Newton divergence — five hypotheses ruled out by measurement, including two of our own that were wrong |

## Why the failures are still in here

The phase reports record hypotheses that turned out to be false, a chain-depth
correlation that was withdrawn because the script computing it was broken, and
two separate cases where a metric was found to be measuring nothing. None of
that was edited out.

That is deliberate. A project whose entire claim is "these numbers were
measured" has to show the measurements that went the other way, or the ones
that went its way are worth less.
