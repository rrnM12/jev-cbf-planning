# Repository reorganization plan

Status: implemented on 22 October 2026 after Nihar approved the full plan.
The sections below preserve the reviewed design and rationale; commands/paths in
README and current docs describe the final interface. This cleanup adds no paid
calls, research datasets, hardware work or controller/contact changes.

## Implementation outcome

- Replaced 32 root modules with `src/jev_cbf`, one lazy argparse CLI, and shared
  infrastructure extracted from old runners. Active libraries do not import
  historical workflows. Root tracked files fall from 39 to seven.
- Moved current/historical configs and all 131 original test identities into named
  folders; added package/provenance and historical-parser regression checks.
  Shared fabricated test input now lives in `tests/fixtures`.
- Preserved the four original root docs byte-for-byte under `docs/history` with
  SHA256 records; replaced current docs with concise orientation and references.
- Chose the bundled uv build backend and an editable checkout-required package.
  Runtime lock entries retain their existing versions; only this project's
  virtual/editable source marker changes. Source distributions include root
  configs/docs/tests; wheels contain the library and require the checkout.
- New manifests use format 2 and nested package-relative snapshots. The legacy
  adapter checks original source bytes for saved-data reads and refuses to resume
  unfinished old physics with renamed code. No root import shims remain.
- Cataloged 53 existing artifact manifests without moving frozen folders. Verified
  every one of 8,630 ZIP members externally before deleting the 2,054,892,329-byte
  duplicate ZIP. Fourteen unique older versions existed before migration; recovery
  now also retains retired source/config paths (62 preserved old paths total).
  Generated root junk was removed; the environment and uv cache remain useful.
- Preserved four original derived HTML pages before fixing the deleted ZIP link
  and three pre-existing missing-response links. No response was fabricated.
  Current/historical display links resolve; archived original pages retain their
  original context and bytes.
- Migration checks found and fixed extraction cycles/self-imports, a missing test
  import, a moved mock target and an omitted historical parser constant. Symbol
  analysis and historical `--help` checks caught the latter before completion.
- All 142 tests pass (131 retained + 11 added). Offline native audits cover both corrected galleries (18 records each) and both
  retained earlier galleries (19 and 18). All four pass offline playback checks.
  The original ledger and 53 manifests are byte-identical; upstream revisions and
  clean worktrees remain unchanged. Build contents and exact credential exclusion
  are verified. Detailed logs/recovery are local under
  `artifacts/reorganization_20261022` and `artifacts/archive`.

Implementation completed locally on 22 October. Nihar requested source publication
on 23 October; the current publication checkpoint is recorded in PROGRESS.md.
The remaining paragraphs are the original implementation plan, now carried out.

## What is wrong today

- 79 tracked files, including 32 Python modules and 39 tracked files directly in
  the repository root. Current workflows, shared libraries and historical demos
  occupy the same namespace.
- The static import closure of the current RGBD runner, repeated runner, gallery
  and verifier reaches 26 of those 32 modules. This includes conditional/local
  imports; it is a dependency inventory, not proof that every module always loads.
- Shared functions live in executable scripts: JSON/hash helpers in
  `run_suite.py`, the project root and SDK validation in `inspect_selection.py`,
  the budget path/runtime versions in `run_pickup.py`, and simulator integration
  in `run_static.py`.
- Current gallery code imports rendering from `pickup_gallery.py`, CSS from
  `showcase_gallery.py`, PNG writing from `static_gallery.py`, and depth coloring
  from `inspect_camera.py`. Some two-way dependencies work only because imports
  are delayed inside functions.
- Documentation mixes current instructions and superseded checkpoints:
  README 875 lines, PROGRESS 1,945, ARCHITECTURE 1,154, AGENTS 346.
- The local checkout is about 6.7 GB, but tracked file contents total only about
  1.23 MB. Approximately 5.7 GB is ignored experiment output; the environment and
  uv cache each use about 0.5 GB. GitHub clutter and local storage need different
  cleanup actions.

## Proposed layout

```text
jev-cbf-planning/
  README.md                     # current setup, commands, results and limitations
  AGENTS.md                     # current repository rules only
  PROGRESS.md                   # current checkpoint and links to history
  ARCHITECTURE.md               # short link to docs/architecture.md for old links
  pyproject.toml
  uv.lock
  .gitignore
  src/jev_cbf/
    cli.py                      # thin argparse entry point
    paths.py                    # project/config/upstream paths and the one ledger
    io.py                       # read, save and digest
    provenance.py               # manifests, versions and source snapshots
    upstream.py                 # read-only FRAX/OSCBF/CBFpy adapters
    evaluation.py               # independent native collision/pickup scoring
    geometry/
    sensing/
    control/
    selection/
    experiments/
    reporting/
    legacy/                     # historical workflows; no imports from active code
  configs/
    scenes/rgbd_pickup.json
    prompts/rgbd_pickup.json
    cameras/realsense.example.json
    legacy/                     # old scene/prompt fixtures still needed by history
  tests/
    unit/
    integration/
    workflows/
    playback/
    fixtures/
  docs/
    architecture.md             # actual current integration and data boundaries
    experiments.md              # methods, 1/2/3 scenes, results and limitations
    camera.md                   # software boundary and hardware-unverified setup
    legacy-workflows.md
    history/                    # earlier progress, plans and authorization records
    reorganization-plan.md
  artifacts/                    # ignored, with one catalog of current/history runs
```

Do not create a generic plugin framework, configuration framework or dozens of
one-function modules. Use the existing implementations and split where an actual
dependency boundary requires it.

## Code moves and extraction

| Existing code | Destination / treatment |
| --- | --- |
| `adaptive_geometry.py`, `geometry.py` | `geometry/`: adaptive covers and fixed-cover primitives; keep their algorithms unchanged. |
| `camera_io.py`, `rgbd_scene.py`, `realsense_camera.py` | `sensing/`: frame/replay boundary, RGBD estimator and optional camera adapter. Keep RealSense explicitly hardware-unverified. |
| `perception.py` | Extract shared camera/calibration and frame-geometry helpers into `sensing/`; isolate the older oracle-segmentation runtime/studies in `legacy/`. |
| `pickup.py`, `rgbd_pickup.py`, `runtime_cbf.py`, `sim_model.py` | `control/`: preserve controller reuse, task phases, sphere pairing and contact policy. Move native scoring to `evaluation.py`; separate old request construction where necessary. |
| `online.py`, `robot_selection.py`, `inspect_selection.py`, `api_budget.py` | `selection/`: client, validation, measured robot context, recorded queries, update admission/scheduling and budget. Move proxy-only request/view functionality to `legacy/`. |
| `run_static.py` | Extract `load_upstream`, adapters, scene construction and `run_case` into library modules. Move its historical static-comparison CLI to `legacy/`. Break the `runtime_cbf`/`run_static` dependency cycle. |
| `run_suite.py` | Extract `read`, `save`, `digest` and manifest helpers; move the old static-suite workflow to `legacy/`. |
| `run_pickup.py` | Extract `BUDGET_PATH` and `runtime_versions`; move the oracle-input experiment runner to `legacy/`. |
| `run_rgbd_pickup.py`, `rgbd_repeated.py` | `experiments/`: retain startup/repeated procedures and five methods. CLI dispatch lives in `cli.py`. Move plant construction out of the runner so reporting/evaluation do not import a command module. |
| `verify_rgbd_pickup.py` | `evaluation.py` plus a thin CLI route; keep native truth confined to evaluation. |
| `rgbd_gallery.py` | `reporting/`: current gallery and observation/coverage views. |
| `pickup_gallery.py`, `static_gallery.py`, `showcase_gallery.py`, `inspect_geometry.py`, `inspect_camera.py` | Extract shared frame rendering, labels, PNG/depth display and styles into `reporting/`; move historical galleries to `legacy/`; retain useful current inspection commands. |
| `run_demo.py` | Keep the unchanged-upstream demo capability behind the new CLI; reuse its platform/environment validation in `upstream.py`. |
| `analyze_pickup.py`, `pickup_layouts.py`, `run_showcase.py`, `study_pickup.py`, `view_selection.py` | Historical analysis, layout generation, dynamic showcase, prior study and proxy viewer: isolate in `legacy/`, then retire superseded entry points after checking their remaining consumers. |

The six files outside the current static import closure are `analyze_pickup.py`,
`pickup_layouts.py`, `run_demo.py`, `run_showcase.py`, `study_pickup.py` and
`view_selection.py`. They are not all dead: three have direct test imports,
`run_pickup.SOURCES` names study/layout files, and historical workflows reference
the others. Do not delete them solely on import-graph evidence.

The desired dependency direction is CLI -> workflows -> library modules.
Control, sensing, selection and reporting must not import old runners or legacy
modules just to obtain constants or utility functions.

## One command surface

Make the project an installable, uv-managed `src` package with one console entry
point, keeping argparse and the existing dependency stack. Illustrative commands:

```bash
uv run jev-cbf rgbd prepare --output artifacts/NEW_RUN
uv run jev-cbf rgbd qualify --output artifacts/NEW_RUN
uv run --extra jev --env-file .env.local jev-cbf rgbd run --output artifacts/NEW_RUN --live
uv run jev-cbf repeated prepare --reference artifacts/STARTUP_RUN --output artifacts/REPEAT_RUN
uv run --extra jev --env-file .env.local jev-cbf repeated run --output artifacts/REPEAT_RUN --live
uv run jev-cbf rgbd gallery --output artifacts/NEW_RUN
uv run jev-cbf rgbd verify --output artifacts/NEW_RUN
uv run jev-cbf inspect camera --help
uv run jev-cbf demo --check
```

New live operations still require `--live`, existing credentials and the existing
cumulative budget. These are proposed command names, not commands to execute now.
Once callers/docs/tests migrate, delete the replaced root scripts rather than
leaving 32 compatibility launchers. Keep historical commands out of the normal
quickstart and primary CLI.

## What to delete, archive or preserve

| Item | Proposed action |
| --- | --- |
| Root `__pycache__/`, `*.pyc`, `MUJOCO_LOG.TXT`, `.DS_Store` | Delete generated junk; retain ignore rules. |
| Root command files replaced by the package/CLI | Delete their old paths after moving the needed implementations and updating all consumers. Git records the history; avoid duplicate implementations and permanent forwarding wrappers. |
| Superseded proxy demos and old command-only code | Retire after shared functionality and meaningful tests have new homes; keep any historical capability still required under `legacy/`. |
| Old examples/prompts | Move used historical fixtures into `configs/legacy/` or `tests/fixtures/`. No exact duplicate JSON files were found, so similar names alone do not justify deletion. |
| Superseded instructions/permission troubleshooting | Move unique historical material to `docs/history/`; remove repetition from current README/AGENTS/PROGRESS/architecture. |
| `artifacts/pickup/review_bundle.zip` | Primary disk-recovery candidate: 2,054,892,329 bytes (about 1.9 GiB). All 8,630 entry paths still exist outside the ZIP, but byte equivalence has not been established. Verify contents against manifests/current files/Git or frozen sources, retain any unique old versions, then remove the duplicate bundle. |
| Old thumbnails, atlases and other derived displays | Delete redundant copies only after proving they can be regenerated offline and checking links/manifests. These are not automatically expendable if they preserve a historical visualization. |
| Prototype/failure run directories | Catalog as development/history first. Preserve unique requests, responses, attempts, trajectories, calibration, source snapshots and failure evidence. Delete only verified disposable scratch/derived output. |
| `.uv-cache/` | Optional final space cleanup (about 496 MB), after the package/setup migration works. Keep it during migration so offline checks remain available. |
| `.venv/`, `uv.lock` | Keep the working environment and lock. Update only necessary package/build metadata; no opportunistic dependency upgrades. |
| `.env.local`, `artifacts/usage/pickup_research_v1.json` | Preserve ignored credentials and the single authoritative cumulative ledger. Do not move/reset the ledger or create a second budget. |
| Current corrected runs and earlier raw scientific evidence | Preserve; the old 2/2/3 and corrected 1/2/3 datasets must remain distinguishable. |

Do not physically shuffle frozen artifact directories in the first pass. Relative
references connect startup/repeated runs, galleries and manifests. Add a small
catalog identifying current, historical, development and derived outputs; move
data later only with an explicit path/reference migration. Current source cleanup
does not require retaining every historical CLI as a public command.

## Implementation sequence

1. **Record the starting point.** Inventory tracked files, imports, subprocess
   commands, source lists and config paths. Record current artifact/ledger hashes,
   1/2/3 scene definitions and the 131-test baseline. Classify deletion candidates.
2. **Extract shared infrastructure first.** Centralize paths, I/O, provenance,
   upstream loading, native evaluation and rendering primitives. Update imports
   without changing equations, gains, geometry algorithms, task thresholds or
   policy authority. Run targeted tests after each extraction.
3. **Move the active implementation into the package.** Add package/build metadata
   and the single CLI, update imports/config resolution/tests, then remove the
   replaced root files. Discover the project root independently of module depth
   and working directory; retain sibling FRAX/OSCBF/CBFpy locations read-only.
4. **Isolate history and simplify documentation.** Move historical runners and
   fixtures out of the current path. Retain useful old data readers/auditors.
   Reduce README to current setup/workflow/results, AGENTS to current rules and
   PROGRESS to the latest checkpoint; link preserved history from each. Keep a
   short root architecture pointer so existing gallery links keep resolving
   without rewriting frozen historical HTML.
5. **Validate historical compatibility and remove obsolete material.** Only after
   the package/CLI and saved-run readers work, delete superseded entry points,
   generated junk and proven duplicate data. Do not remove tests merely because
   a moved module makes them fail; migrate them or retire only feature-specific
   tests along with a deliberately retired feature.
6. **Verify and finish.** Check the final tree, imports, test discovery, offline
   setup/commands, saved-run verification, playback, secrets exclusion, unchanged
   ledger and clean upstreams. Review the deletion list and diff. No paid rerun is
   needed to reorganize the source or validate recorded results.

Keep the changes in small reversible commits: infrastructure extraction, package
move/CLI, legacy/config/docs migration, then proven deletions. Avoid mixing bulk
formatting, controller tuning and experiment changes into the structural move.

## Compatibility and acceptance checks

- Existing 131 tests remain represented and pass; split unit/integration/workflow
  suites without breaking unittest discovery (including package markers).
- Add focused tests for project-root/config resolution from a different working
  directory, CLI import/help without launching JAX/GUI/provider calls, and moved
  package data. Verify setup through uv with frozen runtime dependencies.
- Exact 1/2/3 obstacle counts, contact masks, intrinsic protections, controller
  parity, sphere coverage and completion-driven scheduler checks still pass.
- Both corrected galleries and the retained historical datasets remain readable,
  independently auditable and playable without another API request or rollout.
- Treat renamed source snapshots as a new manifest format. Preserve old manifest
  paths and source bytes; use a read-only historical-format adapter. Do not rewrite
  raw records, normalize hashes until they appear to match, or bypass source
  checks to resume old unfinished physics. Old interrupted provider attempts stay
  non-repeatable; completed-result reads/resumes make zero new calls.
- The ledger bytes and all upstream Git worktrees are unchanged. Credentials,
  environments, caches and generated evidence remain excluded from publication.
- Final root contains the short orientation/rule/status documents and project
  metadata, with implementation/config/tests/history in their named folders.

The first implementation action should be extracting shared paths/I/O/provenance
and renderer helpers, not bulk-moving or deleting the old scripts.
