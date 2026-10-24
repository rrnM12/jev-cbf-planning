# Working in this repository

Read [PROGRESS.md](PROGRESS.md), [docs/architecture.md](docs/architecture.md) and
[README.md](README.md). The user's latest directions supersede historical gates.
Earlier authorizations and checkpoints are preserved verbatim in
[docs/history](docs/history/README.md); they are historical context.

## Current scope

On 23 October, Nihar requested pushing the completed reorganization. Commit and
push its source, configs, tests and documentation to the existing `origin/main`
after offline verification and staged credential/artifact exclusion checks.
Generated evidence, the ledger, credentials, environments and caches stay local.
This publication includes no new experiment, provider call or hardware work.

The user approved complete implementation of the repository reorganization.
Work autonomously through extraction, package/CLI migration, configs/docs/tests,
proven duplicate/junk deletion and offline verification. Make documented,
reversible choices without routine approval requests. This cleanup does not
change experiment scope, safety/contact assumptions, controllers, paid services
or spending limits, and does not require a new experiment or provider call.

The corrected easy/medium/hard startup and repeated-Jev studies are complete at
`artifacts/rgbd/count123_startup_v1` and `count123_repeated_v1`.
Easy/medium/hard mean 1/2/3 clutter obstacles, excluding the target.
Preserve the earlier 2/2/3 datasets and all failures. Do not initiate another
research dataset or live call merely because an earlier checkpoint allowed one.
Commit/push only when requested; keep local completion distinct from publication.

## Integration and research boundaries

- All work belongs inside this repository. Sibling `../frax`, `../oscbf`,
  `../cbfpy` are read-only, including source, config, dependencies and Git state.
- Reuse existing controller, robot, barrier and optimizer implementations.
  Scoped in-process adapters are allowed; upstream disk edits/copies are not.
  Disable upstream bytecode/build metadata writes.
- Preserve FRAX nominal torque control, intrinsic robot protections and actuator
  limits in every mode. Off removes only scene-object constraints.
- Static clutter pickup is the task: maneuver, grasp, verify lift, stop. Intended
  finger-target contact is distinct from unwanted arm/palm-target or clutter
  contact. No placing, transport, pushing, moving clutter or physical robot work.
- Current control geometry must come from RGBD observations and known calibration,
  not native object metadata or oracle masks. Native truth is for plant/evaluation.
  Legacy oracle segmentation is explicitly historical scaffolding. Visible/cell
  coverage is not proof of unseen physical-object coverage.
- Keep one completion-driven request in flight and the 2 Hz wall-clock ceiling.
  Preserve raw attempts/replies, including failures/late answers. No re-querying
  for preferred results, fabricated fallback decisions or unvalidated safe stops.
- Use the existing local environment key securely and the sole cumulative ledger
  `artifacts/usage/pickup_research_v1.json` before any authorized live call. Its
  original $7.50 cap is cumulative, not per run. Never print credentials or search
  unrelated credential stores. No new paid service or budget reset.
- Ask only for genuine blockers or consequential scope/contact/safety/spend
  changes. Routine helper names, organization and reversible fixes need no gate.
- Report evidence limits: designed scenes, empirical sampled-control guard,
  approximate dynamics, no hardware/formal safety/generalization claim. Do not
  claim Jev superiority when deterministic covers also work.

## Structure and verification

- Use uv and the local environment/cache. Install with
  `uv sync --frozen --extra jev --cache-dir .uv-cache --python 3.12`.
- Public commands use `uv run jev-cbf ...`; implementation is in `src/jev_cbf`.
  Active library modules must not import `legacy`. CLI historical inspection may
  dispatch lazily. Keep help free of JAX, GUI, device and SDK initialization.
- Config paths and the ledger resolve from `jev_cbf.paths`, never caller cwd.
  Keep runtime dependency pins; avoid unrelated upgrades and new frameworks.
- Tests: `uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B
  -m unittest discover -s tests -v`. Native tests require the sibling sources.
  Playback: `node tests/playback/check_rgbd_playback.cjs RUN_DIRECTORY`.
- New manifests use format 2 and package-relative source snapshots. Read old
  snapshots under their original names/hashes. Never weaken checks or rewrite raw
  data to resume old unfinished physics; require its original revision/fresh run.
- Preserve frozen requests/responses, trajectories, calibration, source snapshots,
  failed attempts and the original ledger. Keep frozen artifact folders in place;
  relative references connect startup/repeated runs. Derived displays can be
  regenerated offline after retaining unique versions and checking links.
- Ignore credentials, environments, caches, simulator logs and artifacts. Verify
  exclusions before publication. Do not silently change machine/upstream/solver.
- Update PROGRESS after meaningful changes/checks, record provisional decisions
  and failures, and keep current documentation concise with links to history.
