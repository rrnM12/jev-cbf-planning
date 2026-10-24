# Current integration architecture

The source package separates current observation/control code from historical
experiments. `cli.py` parses arguments before lazy dispatch; help initializes no
JAX, GL, device or provider. `paths.py` locates the checkout from project metadata,
resolves configs independently of cwd, and preserves the authoritative ledger
path. The checkout and read-only sibling sources remain runtime requirements.

## Observation to control

1. `sensing/camera_io.py` defines calibrated frames, recording/replay, transforms,
   separate masks and bounded background acquisition/processing.
2. `sensing/rgbd.py` captures simulator RGB/metric depth with known calibration.
   RGB components and metric depth edges establish observed identities, including
   the designated color target. The estimator does not consume native IDs, poses,
   dimensions or segmentation. Missing observations remain explicit.
3. `geometry/adaptive.py` covers observed/table-supported occupied cells with
   recursively refined enclosing spheres under computation and count limits.
   Each completed cover encloses its represented cells. This differs from
   full physical-object coverage, which may fail under occlusion/model mismatch.
   `geometry/covers.py` retains fixed and analytic shape primitives.
4. `control/rgbd.py` builds observation-derived requests, uses measured robot
   geometry, runs five comparison methods and applies validated updates through
   existing admission checks. `selection/validation.py`, `runtime.py`, `client.py`
   and `budget.py` separate decoding, scheduling/recording, SDK transport and spend.
5. `control/pickup.py` owns task phase targets and native gripper actuation.
   It reuses FRAX nominal control and existing CBF machinery through
   `control/cbf.py`, `model.py`, `simulation.py` and `upstream.py`.

`sensing/calibration.py` and `geometry.py` hold shared calibration/depth fitting;
legacy oracle segmentation/runtime lives only in `legacy/perception.py`.
The current import graph has no active library imports into `legacy`.

## Truth and safety boundary

`control/scene.py` constructs the plant from native scene definitions; this is
also used for rendering and independent evaluation, not request/candidate fitting.
`evaluation.py` measures native collision/contact geometry. `experiments/verify.py`
re-evaluates saved trajectories, hashes, lift/contact duration, enabled counts,
1 ms times and nominal-controller parity independently of policy selections.

The arm keeps upstream joint, singularity, self-collision, floor and actuator
protections. Off removes only scene-object barriers, matching the agreed baseline.
Other conditions retain mandatory arm-target protection; only intended terminal
finger-target contact is exempt. Clutter is static. Pickup needs verified lift
and bilateral finger contact over the configured confirmation window.
No gains, numerical equations, controller authority or contact masks changed in
this reorganization. These are simulation assumptions, not hardware guarantees.

## Experiment and reporting layers

`experiments/rgbd.py` freezes inputs/source bytes, computes observation-derived
kinematic references and qualifies each scene with an actual off collision,
nonpenetrating initial geometry and deterministic all-adaptive feasibility witness.
The shared startup response drives the three Jev ablations. Five methods remain
off, all/single, all/Jev cover, Jev roles/single, Jev roles/adaptive cover.

`experiments/repeated.py` copies frozen startup inputs, answers, qualification and
non-Jev baselines with provenance; only the three Jev conditions run again.
A completion-driven scheduler dispatches at most every 500 ms wall clock, with
one request in flight and fresh measured snapshots. Completion/late logs survive
shutdown. `selection/budget.py` reserves against the original cumulative ledger
before calls; failed calls do not silently refund reservations.

`reporting/rgbd.py` produces shared-timeline five-panel playback, camera views,
coverage and cost reports. Shared rendering, PNG/depth conversion and styles now
live in `reporting/rendering.py`, `images.py`, `styles.py`. It no longer imports
historical gallery scripts. `reporting/camera.py` inspects saved frames;
`reporting/inspection.py` retains geometric inspection.

## Frozen data compatibility

`io.py` centralizes JSON and hashes. `provenance.py` checks frozen inputs and source
snapshots. New format-2 manifests record paths such as
`src/jev_cbf/control/rgbd.py`; snapshots preserve that directory structure.
Preparation verifies current source bytes again after collecting inputs.

Historical format-1 manifests retain root-module names and original bytes.
Saved-result inspection/audit checks these snapshots without trying to equate
renamed current code to an old hash. Completed results are returned without new
calls. An unfinished historical rollout requires the original revision or a fresh
run. No import shims, hash rewriting or unchecked old-source execution is used.
Unsupported formats, missing snapshots and changed inputs fail closed.

Frozen run directories and the usage ledger have not moved. Current and historical
runs are cataloged in the ignored `artifacts/index.json` and `index.html`.
The pre-cleanup documentation is preserved byte-for-byte in `docs/history`;
root `ARCHITECTURE.md` remains a pointer for existing gallery links.

## Research limits

Known simulated calibration, table support, color detection, static scenes,
designed layouts and observed-cell completion are provisional research choices.
The simulation/robot contact dynamics remain approximate. Constraint counts are
reported separately from fixed allocated rows and measured loop cost. Saved
native audits are empirical checks; neither those audits nor passing tests imply
formal safety, hard real-time operation, learned perception or hardware readiness.
See [experiment details](experiments.md) and [camera limitations](camera.md).
