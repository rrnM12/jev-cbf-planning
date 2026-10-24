# Historical workflows

`src/jev_cbf/legacy` isolates earlier proxy selection, static reaching suites,
dynamic showcase, oracle-input pickup, depth/oracle-label studies, layout
creation and their galleries/analyses. Current libraries never import these
modules. They remain because tests or unique historical evidence still use them.
No historical module is a root forwarding script.

| Earlier root command | Historical module |
| --- | --- |
| `inspect_selection.py`, `view_selection.py` | `jev_cbf.legacy.selection`, `view_selection` |
| `run_static.py`, `run_suite.py` | `jev_cbf.legacy.static`, `suite` |
| `run_showcase.py` | `jev_cbf.legacy.run_showcase` |
| `run_pickup.py`, `study_pickup.py`, `pickup_layouts.py` | `jev_cbf.legacy.pickup`, `study_pickup`, `pickup_layouts` |
| `analyze_pickup.py` | `jev_cbf.legacy.analyze_pickup` |
| `pickup_gallery.py`, `showcase_gallery.py`, `static_gallery.py` | Corresponding `jev_cbf.legacy` modules |

Run historical modules with `uv run python -B -m MODULE --help` when necessary.
Used historical scenes/questions live in `configs/legacy/{scenes,prompts}`.
Current inspection is `jev-cbf inspect geometry|selection|camera`; selection is
an offline proxy preview and is labeled accordingly. The normal quickstart uses
only the RGBD experiments. Historical live/reproduction commands are retained
research tools, not permission for new calls or experiments.

Read-only reports/audits consume original snapshots without rewriting raw data.
Old unfinished producer execution, including archived task-source replay, requires
its original checkout/revision. The old standalone script imports are deliberately
not recreated with shims. Existing completed RGBD data can be audited and played
with the new package. Retained historical studies remain separately identifiable.

The old 2 GB portable ZIP duplicated 8,616 current files and contained 14 unique
old versions. Those versions and all member SHA256 fingerprints are retained at
`artifacts/archive/review_bundle_unique/manifest.json`. The ZIP is removed only
after every member has a verified external copy. Its old review link points to
the recovery catalog. Current raw experiments/trajectories/failures are retained.
Exact pre-cleanup source/config copies and validation logs are also saved locally
in `artifacts/reorganization_20261022`; Git retains earlier tracked versions.
