# Jev CBF planning

RGBD-based static-clutter pickup experiments using adaptive sphere covers and
Jev geometry/role selection, with the unchanged FRAX nominal controller and
read-only FRAX, OSCBF and CBFpy integrations.

Easy, medium and hard contain **one, two and three clutter obstacles**, respectively,
plus a separate pickup target. Both startup and repeated-Jev comparisons are
complete. Adaptive conditions achieved safe pickup in all three designed scenes;
single-sphere conditions stalled, and scene-constraint-off runs collided.
See [methods, results and limitations](docs/experiments.md).

## Setup

Requires Python 3.12+, uv, and sibling checkouts `../frax`, `../oscbf`, `../cbfpy`
at the revisions recorded by each experiment. Run these commands in this checkout:

```bash
uv sync --frozen --extra jev --cache-dir .uv-cache --python 3.12
uv run --offline --frozen --extra jev --cache-dir .uv-cache jev-cbf --help
uv run --offline --frozen --extra jev --cache-dir .uv-cache jev-cbf demo --check
uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B -m unittest discover -s tests -v
```

The package is installed editable from `src/jev_cbf`. It requires this research
checkout; the console command resolves configs and the one usage ledger from the
repository, independently of the working directory. Wheels contain the Python
library; source distributions also include configs, tests and docs. A wheel alone
outside the checkout is intentionally insufficient to run research workflows.
No upstream package is installed editable or modified.

On macOS, `jev-cbf demo` launches the original demo through `mjpython`.
`demo --check` only checks setup presence. Headless tests and saved-data audits
use ordinary Python. Tests exercise native physics; they do not validate hardware.

## Current saved results

Generated evidence is local and ignored by Git. Existing checkout links:

- [Startup gallery](artifacts/rgbd/count123_startup_v1/index.html): all five methods, three scenes.
- [Repeated-Jev gallery](artifacts/rgbd/count123_repeated_v1/index.html): nine new Jev runs, reused non-Jev baselines.
- [Local artifact catalog](artifacts/index.html): current, historical and development records.

```bash
uv run --offline --frozen --extra jev --cache-dir .uv-cache jev-cbf rgbd status --output artifacts/rgbd/count123_startup_v1
uv run --offline --frozen --extra jev --cache-dir .uv-cache jev-cbf rgbd verify --output artifacts/rgbd/count123_startup_v1 --report artifacts/rgbd/startup_audit.json
uv run --offline --frozen --extra jev --cache-dir .uv-cache jev-cbf rgbd verify --output artifacts/rgbd/count123_repeated_v1 --report artifacts/rgbd/repeated_audit.json
node tests/playback/check_rgbd_playback.cjs artifacts/rgbd/count123_startup_v1
```

Verification independently evaluates saved trajectories without another rollout
or API request. `--report` keeps the run's existing derived report untouched.
`rgbd gallery --output ...` rebuilds the display offline; source changes may
invalidate frame caches and make rendering take several minutes. Earlier
`gallery_v2` / `repeated_v1` use 2/2/3 clutter counts and remain historical.

## Workflow commands

```bash
uv run jev-cbf rgbd prepare --output artifacts/rgbd/NEW_STARTUP
uv run jev-cbf rgbd qualify --output artifacts/rgbd/NEW_STARTUP
uv run --extra jev --env-file .env.local jev-cbf rgbd run --output artifacts/rgbd/NEW_STARTUP --live
uv run jev-cbf rgbd gallery --output artifacts/rgbd/NEW_STARTUP
uv run jev-cbf repeated prepare --reference artifacts/rgbd/NEW_STARTUP --output artifacts/rgbd/NEW_REPEATED
uv run --extra jev --env-file .env.local jev-cbf repeated run --output artifacts/rgbd/NEW_REPEATED --live
uv run jev-cbf repeated report --output artifacts/rgbd/NEW_REPEATED
uv run jev-cbf repeated gallery --output artifacts/rgbd/NEW_REPEATED
```

These are commands for a future deliberately authorized experiment, not a request
to rerun the completed study. New calls require `--live`, the local
`TYPESAFE_API_KEY`, and the existing cumulative ledger at
`artifacts/usage/pickup_research_v1.json`. Do not create/reset a replacement budget.
Current cumulative cap: $7.50. No cleanup validation needs a live call.
New outputs require fresh directories; interrupted attempts and raw failures stay.
Historical manifests remain readable but cannot resume unfinished physics under
moved producer code. Use the original revision or prepare a fresh run.

## Where to work

| Directory | Contents |
| --- | --- |
| `src/jev_cbf/geometry`, `sensing`, `control`, `selection` | Current library and integration boundaries |
| `src/jev_cbf/experiments`, `reporting` | Current workflows and displays |
| `configs/scenes`, `prompts`, `cameras` | Editable current scenes, questions and camera example |
| `src/jev_cbf/legacy`, `configs/legacy` | Isolated historical workflows and fixtures |
| `tests/unit`, `integration`, `workflows`, `playback` | Migrated regression tests and gallery contract checks |
| `docs/history` | Exact pre-reorganization documentation |
| `artifacts` | Ignored requests, responses, trajectories, failures, snapshots and reports |

Read [AGENTS.md](AGENTS.md) and [PROGRESS.md](PROGRESS.md) before continuing.
[Architecture](docs/architecture.md), [camera boundary](docs/camera.md),
[historical workflows](docs/legacy-workflows.md) and the
[reorganization record](docs/reorganization-plan.md) provide more context.
