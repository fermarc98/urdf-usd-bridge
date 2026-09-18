# Releasing

The maintainer runbook. Two things have to be true before a tag can publish:
the artifacts have to be verified (`docs/VERIFY.md` T4), and PyPI has to have
been told to trust this repository.

## One-time: trusted publishing

No API token is stored in this repository, and none should be. PyPI issues a
short-lived credential to the workflow through OIDC instead.

On [pypi.org](https://pypi.org/manage/account/publishing/), add a **pending
publisher** (the project does not exist yet, so it has to be the pending form):

| Field | Value |
|---|---|
| PyPI project name | `urdf-usd-bridge` |
| Owner | `fermarc98` |
| Repository name | `urdf-usd-bridge` |
| Workflow name | `release.yml` |
| Environment name | `pypi` |

Repeat on [test.pypi.org](https://test.pypi.org/manage/account/publishing/)
with environment `testpypi` if you want a rehearsal target.

Then, in the repository's **Settings → Environments**, create `pypi` and add
yourself as a **required reviewer**.

That reviewer is the whole safety mechanism. With it, pushing a tag builds the
artifacts and then stops, waiting for a human to approve the upload; without
it, a pushed tag publishes to PyPI unattended and a version number on PyPI can
never be reused. Do not remove it.

## Every release

1. **Land the work.** `main` green: `ruff`, `black`, the T0 matrix, T1, and the
   `package` job.

2. **Bump the version** in `src/urdf_usd_bridge/_version.py`. Nothing else
   carries it — `pyproject.toml` reads it, and the output layer's
   `customLayerData` records whatever the installed package reports.

3. **Write the `CHANGELOG.md` section.** The heading must be
   `## [<version>] - <date>`, **with the version in square brackets**, and the
   section goes *above* the previous release, not inside it. The workflow reads
   the body and uses it verbatim as the GitHub release notes, so it has to
   stand on its own. Keep the house rule: say which numbers were measured, on
   what, and name anything that was measured and found to do nothing.

   Both requirements are enforced by `tests/unit/test_changelog.py`, which runs
   on every PR, so an ordinary test run catches this long before a tag does.

4. **Run the pre-tag checks**, which are exactly what the workflow will run:

   ```bash
   # what will be published as the release notes -- read it
   python scripts/changelog_section.py

   # the two gates, spelled the way release.yml spells them
   v=<version>
   test "$v" = "$(sed -n 's/^__version__ = "\(.*\)"$/\1/p' src/urdf_usd_bridge/_version.py)" \
     && echo "version matches" || echo "VERSION MISMATCH"
   python scripts/changelog_section.py --check "$v" && echo "changelog ok"
   ```

5. **Run T4 locally** (`docs/VERIFY.md`) before tagging. On a host with ROS or
   conda in the login profile, every command needs
   `env -u PYTHONPATH -u PYTHONHOME` or the "clean" venv is not clean.

6. **Tag and push.**

   ```bash
   git tag -a v<version> -m "urdf-usd-bridge v<version>"
   git push origin v<version>
   ```

   If a tag has to move — the release failed on something in the repository
   rather than in the upload — delete it in both places first, or the push is
   rejected and `--verify-tag` later disagrees with the remote:

   ```bash
   git tag -d v<version>
   git push origin :refs/tags/v<version>
   ```

7. **Approve the deployment** when the `pypi` environment asks. Check the build
   job's own output first: it fails the tag if `_version.py` disagrees with the
   tag, if `CHANGELOG.md` has no section for it, if `twine check` fails, if the
   wheel will not install and run in a clean venv, or if anything from
   `references/` or an artifact directory reached the sdist.

8. **After the upload**, the workflow creates the GitHub release with the
   CHANGELOG section and attaches both artifacts.

## Rehearsing without publishing

`workflow_dispatch` with `target: none` builds and runs every check and
publishes nothing. `target: testpypi` uploads to TestPyPI. Use the first
freely; the second consumes a version number on TestPyPI, which is also
non-reusable.

## If a release is wrong

It depends on whether anything was published.

**Nothing uploaded yet** — the build failed, or you have not approved the
`pypi` environment. The tag is just a pointer; fix the repository, then move it
with the delete-both-places recipe in step 6. Nothing downstream has seen it.

**Already on PyPI** — the version is spent. A released version cannot be
replaced or re-uploaded. Yank it on PyPI, which keeps existing pins working
while stopping new resolutions from picking it, and release a fixed patch
version. Do not move or delete the tag at that point; add the next one.
