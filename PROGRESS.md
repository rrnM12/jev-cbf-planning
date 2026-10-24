# Progress and decisions

Updated: 23 October 2026.

## Current checkpoint: reorganization source publication (D32)

Nihar requested committing and pushing the completed reorganization to the
existing `origin/main` at `rrnM12/jev-cbf-planning`. Publication includes source,
configs, tests and docs. Credentials, generated evidence, the authoritative
ledger, environments and caches remain ignored/local. Publication verification:
all 142 tests passed again in 61.001 s; staged credential/private-path scans,
CLI/setup checks and diff checks passed. The ledger, 53 original manifests and
all three clean upstream revisions are unchanged. No provider call or research
run was added. The final commit/remote identity is recorded in the local receipt
`artifacts/publication/reorganization_20261023_final.json`.

## Completed checkpoint: repository reorganization (D31)

Nihar approved complete implementation of the [reviewed plan](docs/reorganization-plan.md).
The 32 root Python files are now an editable `src/jev_cbf` package with one
`jev-cbf` CLI. Shared paths/I/O/provenance/upstream/evaluation/rendering code was
extracted before removing old entry paths. Current library imports are independent
of historical workflows. Configs and tests have named subdirectories.

Earlier root documentation is preserved exactly in [docs/history](docs/history/README.md).
Current integration, results and camera limitations live in `docs/`; root
ARCHITECTURE remains a pointer so saved gallery links survive. Frozen artifact
folders and the one ledger retain their paths. Unique old ZIP members were
preserved before duplicate storage cleanup. No new paid call, hardware connection,
controller/contact change or runtime dependency upgrade is part of this work.

Provisional implementation choices: uv's bundled build backend (no new runtime
dependency); repository-required package/config discovery; format-2 snapshots
with package-relative names; read-only format-1 adapter; unfinished historical
physics requires its original revision/fresh run. Active CLI help stays lightweight.

Migration failures found and corrected: extraction-induced self/cyclic imports,
a missing relocated test import, a stale mock target and an omitted historical
parser constant. A global-name audit and historical CLI help checks caught the
last issue before completion.

Verification: **142 tests passed in 59.119 s**, preserving all 131 original test
identities and adding 11 package/provenance/parser checks. Both corrected galleries
pass independent saved native audit (18 records each), as do the retained earlier
2/2/3 galleries (19 and 18). All four pass offline playback. 3,715 local references
across current/historical displays and current docs resolve after four preserved
derived-page repairs. Camera replay, frozen uv setup, CLI use from another cwd,
lightweight help, package builds and exact configured-key exclusion pass.

The 53 original manifests and ledger bytes are unchanged; all three upstreams
remain clean at the same revisions. About 1.9 GiB was recovered; artifacts now
use about 3.8 GiB. The working environment/cache remain. Detailed evidence is at
`artifacts/reorganization_20261022/acceptance.json`; the local browsing catalog
is `artifacts/index.html`. The reorganization completed locally on 22 October;
source publication is authorized under D32 above. No experiment/provider call is
pending for this task.

## Completed research checkpoint: 1/2/3 obstacle comparison (D29)

Current galleries: `artifacts/rgbd/count123_startup_v1/index.html` and
`artifacts/rgbd/count123_repeated_v1/index.html`. Easy/medium/hard have 1/2/3 clutter
obstacles plus the target. All 15 startup and nine repeated Jev runs finished;
non-Jev repeated panels are explicitly reused. Adaptive conditions achieved safe
pickup in 3/3 scenes; single-sphere conditions stalled; off collided. Deterministic
adaptive witnesses also work, so these results do not establish Jev superiority.
See [methods and results](docs/experiments.md).

The original cumulative ledger has 595 calls, $4.913233920 reservations and
$0.534521820 estimated usage against the same $7.50 cap. Earlier 2/2/3 results,
requests, late/failing answers, trajectories and snapshots remain historical.
No further research experiment/provider call follows this completed correction.

The complete revision plan and decisions through D30 are preserved in
[the progress archive](docs/history/progress-2026-10-22.md).
