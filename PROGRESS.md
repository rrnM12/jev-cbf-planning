# Progress and decisions

Updated: 20 October 2026.

## Current checkpoint

Inspect the first standalone Jev result in its full synthetic scene. Nihar
approved a saved-result viewer after the successful live request. Keep all three
upstream repositories unchanged. No robot integration, real contact exemptions,
online update loop, or additional service calls in this viewer checkpoint.

**Viewer checkpoint complete: implementation, automated/launch checks, and
Nihar's visual review.** Nihar said "looks good" after viewing the scene.
`view_selection.py` reads the saved request/answers,
validates them, and displays exact recorded world poses and selected sphere
covers, object/decision labels, start/goal proxies, and the unfiltered nominal
sweep. The omitted object stays visible without a cover. Table is display-only;
all contacts are disabled and no physics step or controller runs. Both oblique
and top-down bounded previews exited 0. All 39 offline tests pass (eight new
viewer tests). Shared viewer/label helpers remain in `inspect_geometry.py`; its
original geometry viewer also passed a bounded regression launch. Visual review
is based on Nihar's observation, not an agent screenshot or a safety evaluation.
Opened the unbounded viewer for joint review (agent exec session 89710); it stays
open until its window is closed. Do not assume that process remains live on resume.

**First live Jev request succeeded.** The stable alias returned `jev-1.13.0` in
0.628746 s for one SDK invocation: 6,021 input tokens and 277 output tokens. Jev
proposed `avoid / box-split-4` for the bottle and nearby cube, and `omit` for the
far cube. All six answers passed contract validation. The cube geometry answer
was ambiguous (P(4 spheres)=0.46, P(2 spheres)=0.37, reported confidence=0.19).
This is one synthetic-scene observation, not a benchmark, optimality result,
whole-arm safety check, or real-time claim. No proposal controls the robot.

The complete request and SDK response are saved locally in
`artifacts/selection/live_initial.json` (ignored by Git). The 39 offline tests
cover request/geometry/launcher/adapter/viewer behavior. Nihar filled the ignored
`.env.local`; verified that its key loads and the SDK uses the official HTTPS
endpoint without printing the key. Owner-only permissions and Git exclusion
remain in place; the key is absent from the saved report. No unrelated credential
stores or global security settings were accessed/changed.

The starting input is a synthetic free-space task with one swept end-effector
sphere, three static axis-aligned objects, and no authorized contact. It is not
the FRAX arm or a simulator rollout. Role choices and conditional geometry choices
share one request but are independent, as required by the API. All outputs remain
inspectable proposals. Templates, counts, task text, and scene facts are easy to
change; these starting choices do not settle the eventual task/contact policy.

### Completed isolated geometry checkpoint

The isolated sphere-cover implementation, 19 geometry/launcher tests, and initial
review of both attached screenshots are complete. Added
`geometry.py`, `inspect_geometry.py`, and
`tests/test_geometry.py`; all 19 tests pass (12 geometry/inspection, 7 launcher).
The 1/2/4-sphere construction covers the known box by enclosing every partition
section. Reference cylinder/cube runs estimate excess union volume using shared,
seeded samples. Primitive count is a cost proxy, not QP timing. No new dependencies.
The real MuJoCo inspection viewer launched and closed normally. Direct screenshot
access through computer-use tools was denied, but Nihar supplied cylinder and cube
comparison screenshots. Both were reviewed: candidates are clearly separated and
reference objects visible through the covers. Cylinder chains visibly reduce
lateral excess; the cube's chain spreads farther along the split axis without
making each sphere much smaller, consistent with the measured non-monotonic union
volume. These views are not 3D containment checks, controller validation, or Nihar's
approval of a later feature. Direct Computer Use approval remains unresolved but
did not prevent review of the attached images.
Read-only diagnosis now confirms Computer Use is installed/enabled and its native
service is running. The installed policy code emits this error when the app-access
approval response is not accepted, before the app operation. The current saved
turn has `danger-full-access` with `approval_policy = "never"`; prompt suppression
is the leading explanation, pending an interactive approval retry. The earlier
generic plugin-registry lookup is not evidence that this bundled plugin is absent.
No missing macOS Screen Recording/Accessibility grant has been established.

### Completed baseline checkpoint

The agent ran the original Panda demo for 1,000 control steps (2 simulated
seconds), recorded numerical diagnostics, and closed the viewer through the
upstream cleanup path. Position error fell from 0.154486 m to 0.003087 m. All
recorded states, torques, and 64 model barriers were finite; sampled barriers were
positive and measured torque-limit excess was zero. This is a bounded
default-target smoke check, not independent safety or obstacle-stress evaluation.

Nihar's manual-Terminal bottleneck is also resolved. The current runtime permits
direct network and GUI execution; a fresh PyPI check returned HTTP 200 without an
approval request, and the bounded viewer run completed normally (exit 0). No
additional research feature, upstream edit, solver change, or security-config
change was made during that baseline checkpoint. Nihar has since approved the
isolated sphere-cover checkpoint described above.

## Completed work and verification

- Inspected the upstream demo, its environment/helpers, and runtime manifests.
- Added `pyproject.toml`, `.gitignore`, and the minimal `run_demo.py` launcher.
  It executes the original demo, selects `mjpython` on macOS, and imports the
  existing FRAX/CBFpy source without building/installing into their checkouts.
- Added README setup/run instructions and offline launcher tests.
- Added AGENTS.md and this progress/decision record, as requested by Nihar.
- Linked the tracking files from README/ARCHITECTURE and updated the architecture
  status to reflect that first-checkpoint implementation has started.
- Verified the user-created `.venv` and `uv.lock`, then ran the checks below using
  `uv run --offline --frozen --cache-dir .uv-cache` without new downloads.
- Added `tests/smoke_demo.py` to make the bounded original-demo check rerunnable.
  Its wrappers only record values and bound execution; the real viewer, physics,
  controller, CBF configuration, and safety-filter implementation remain upstream.
- Inspected FRAX collision utilities/documentation: they consume existing robot
  sphere models rather than implement this scene-object box-partition generator.
  Added only the new geometry-specific logic; did not copy robotics/CBF code.
- Added conservative box-section covers with stable candidate IDs and explicit
  object-local metres, plus independent analytic box/cylinder membership and
  shared-point Monte Carlo union/excess-volume estimates.
- Added a CLI for numeric reports and a static MuJoCo comparison viewer, with
  adjustable shape/dimensions/counts/sampling settings and optional JSON output.
  Outputs in `artifacts/geometry/` are local/ignored and reproducible with README
  commands. No simulator scene or robot controller was changed.
- Added the approved tiny selection experiment. `inspect_selection.py` imports
  the existing geometry routines and uses a thin official-SDK adapter; it contains
  no controller, CBF implementation, SDK reimplementation, or asynchronous worker.
- Added `examples/selection_scene.json` and `prompts/selection.json` for easy scene
  and question changes. Generated requests include full candidates, measured
  excess volumes, and nominal swept-proxy clearances, without oracle role labels.
- Added preview-by-default and explicit `--live` modes, optional JSON recording,
  response-contract validation, and preservation of probabilities/confidence.
  Missing keys, HTTP failures, and invalid selected IDs do not yield fallback
  model decisions. No confidence threshold was introduced.
- Added `view_selection.py` and eight offline display tests. The viewer uses the
  saved candidate coordinates directly, not regenerated defaults; no additional
  model calls, dependency changes, or upstream edits. Native labels/captions use
  shared static-viewer helpers rather than a separate visualization framework.

| Check | Result |
| --- | --- |
| `uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B -m unittest discover -s tests -v` | 39 tests passed: 7 launcher, 12 geometry/inspection, 12 selection/SDK, and 8 saved-scene viewer tests. SDK/display test answers are fabricated, not live model evidence. No robot safety claim. |
| Saved-result scene check | `uv run --offline --frozen --cache-dir .uv-cache python -B view_selection.py --check` passed against the actual saved result: 16 finite display geoms. No physics or service call. |
| Saved-result GUI launches | `mjpython -B -u view_selection.py --seconds 3`, with and without `--top`, both exited 0 under uv offline/frozen. No screenshot-based visual QA yet. |
| Geometry viewer regression | `mjpython -B -u inspect_geometry.py --samples 1000 --seconds 3` exited 0 under uv offline/frozen after shared-viewer refactor. Sampling here is only a quick launch check, not replacement volume measurements. |
| `python3 -B run_demo.py --help` | Passed; CLI available without simulation dependencies. |
| Parse `pyproject.toml` with Python's `tomllib` | Valid TOML; 9 external runtime dependencies declared. Not a uv resolution test. |
| `uv lock --check --offline --cache-dir .uv-cache` | Passed after adding the optional SDK; generated lockfile matches the requirements (35 packages resolved). |
| `uv sync --locked --offline --cache-dir .uv-cache` | Passed when run by the agent: 22 packages resolved and 21 installed packages checked. uv itself is operational. |
| `uv run --offline --frozen --cache-dir .uv-cache python -B run_demo.py --check` | Passed: upstream files, runtime modules, and mjpython present. |
| Runtime imports / `jax.devices()` | JAX initialized a CPU device; MuJoCo, elastiqp, and qpax imports passed. |
| One-off upstream model/CBF check | Imported the original demo; loaded Panda; constructed its CBF config/filter; evaluated at upstream default joints with zero velocity and gravity nominal torque. All 64 barrier values and seven output torques were finite. No QP residual, trajectory, or safety claim. |
| Earlier sandboxed demo / minimal mjpython | Both aborted with exit 134 before scoped execution approval was available. Retained as diagnostic history, not the current approved-launch result. |
| Full demo in normal Mac Terminal (user report) | Nihar pasted the upstream "Initializing OSCBF demo" and "Demo ready" messages. Initialization/initial JIT reached readiness outside the agent session. No visual motion or obstacle-avoidance result reported yet. |
| Approved agent PyPI connectivity check | `curl -I --connect-timeout 10 --max-time 15 https://pypi.org/simple/uv/` returned HTTP 200 outside the sandbox after approval. In-sandbox DNS remains restricted, as expected. |
| Approved agent full demo | `uv run --offline --frozen --cache-dir .uv-cache python -B -u run_demo.py` reached "Demo ready" and remained running. Deliberate Ctrl-C exited with 130 from `env.step()`, exercising the existing loop/cleanup path. No interactive safety evaluation. |
| Bounded agent full demo | `uv run --offline --frozen --cache-dir .uv-cache mjpython -B -u tests/smoke_demo.py --steps 1000` completed 1,000 original-controller steps with the real viewer, printed diagnostics, closed automatically, and exited 0. Metrics below. A preceding one-off version produced identical values. |
| Smoke-check CLI | `python3 -B tests/smoke_demo.py --help` passed; `--steps 0` rejected with argparse exit 2 before loading robotics libraries. |
| Current direct networking | PyPI HEAD request returned HTTP 200 under the updated unrestricted runtime, with no escalation parameter or approval request. |
| Upstream `git status --short` | Still clean in FRAX, OSCBF, and CBFpy after the new checks. |
| Geometry numeric experiments | Cylinder (7 × 7 × 30 cm bounds) and cube (8 cm side), 100,000 shared points per case, seed 0; no sampled object misses. Measurements below. |
| Additional boundary diagnostic | 136,000 section corners across 1,000 random boxes and 1/2/3/4/7-sphere covers: zero union-membership misses. Not a floating-point proof. |
| Geometry viewer | Bounded cylinder/cube previews exited 0; scene compilation tested. Direct computer-use screenshot access denied, but initial layout/opacity review completed from Nihar's attached cylinder/cube screenshots. Not a 3D containment proof. |
| Optional SDK setup | `uv sync --extra jev --cache-dir .uv-cache` succeeded; resolved 35 packages and installed 12 additional packages, including typesafe-sdk 0.7.5. A metadata check confirmed JAX/jaxlib, MuJoCo, elastiqp, qpax, and NumPy versions are unchanged. |
| Selection request preview | `uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B inspect_selection.py --output artifacts/selection/request_preview.json` passed; three objects, six Choice questions, no answers fabricated and no network call. |
| Missing-key live path | Same command with `--live --output artifacts/selection/live_missing_key.json` exited 2 before sending a request. Report contains `mode: live_requested` and an explicit missing-key error, with no response or proposals. |
| Real SDK, fake transport | SDK serialized the actual generated request to the expected endpoint and parsed a fabricated response locally. A fabricated HTTP 400 surfaced as failure without fallback or copying private error bodies. No live API access tested. |
| First live Jev call | `uv run --offline --frozen --extra jev --cache-dir .uv-cache --env-file .env.local python -B -u inspect_selection.py --live --output artifacts/selection/live_initial.json` exited 0. Actual model `jev-1.13.0`, 0.628746 s SDK invocation, 6,021 input / 277 output tokens; all six answers and candidate IDs validated. Results below, not a safety/performance benchmark. |
| Local credential handling | Loaded key presence only; verified official default HTTPS destination, `.env.local` permissions `600`, Git exclusion, and key absence from the saved report. No key value printed or committed. |

The installed environment is Python 3.12.7 on Darwin arm64, uv 0.13.4, JAX/jaxlib
0.12.0, MuJoCo 3.16.0, elastiqp 0.1.0, qpax 0.1.4, NumPy 2.5.4, and optional
typesafe-sdk 0.7.5. The generated
`uv.lock` is present as a local, uncommitted file. FRAX emits nonfatal CPU
performance warnings; no solver, JAX version, machine, or upstream revision was
changed in response. QP residuals/slack, manual target interaction, obstacle stress
cases, and independent collision evaluation remain unverified. No claim is made
that the safety filter actively intervened in this default-target run.

### Bounded baseline observations

The smoke check captures the actual `DemoOSCBFConfig` constructed by upstream
`main()` and evaluates its `h_2` on the initial state plus every post-step state.
Groups follow that inspected config's concatenation order. These are robot-model
sphere/barrier measurements, not independent MuJoCo mesh clearances or a
continuous-time safety certificate. The target, gains, and constraints are not
changed; leave the target untouched when rerunning the check.

| Observation | Value |
| --- | --- |
| Steps / simulated time | 1,000 / 2.0 s |
| Initial / final EE position error | 0.1544862062 / 0.00308708965 m |
| Maximum torque-limit excess | 0.0 N m |
| Minimum joint-limit barrier | 0.7156055098 rad |
| Minimum singularity barrier | 0.07936774483 (upstream manipulability-product criterion) |
| Minimum external-obstacle sphere clearance | 0.09032637602 m |
| Minimum self-collision barrier | 0.1485205842 m |
| Minimum floor barrier (includes upstream floor offset) | 0.1066044273 m |

### Isolated geometry observations

The generator encloses sections of a known bounding box, not a perceived mesh.
The cylinder is a bottle proxy, not a detailed bottle model. All raw cover lengths
are object-local metres; clearance margin and robot-body expansion are absent.
The analytic coverage argument assumes a correct full-object bound and exact
arithmetic. Dimensions must be finite and strictly positive; reject degeneracy
rather than introduce an arbitrary minimum size. One-ULP outward radius rounding
is numerical padding, not a physical margin or floating-point certificate.

Volumes below are estimates of the **union** outside the exact reference shape;
overlapping sphere volumes are not summed. Each ± is one estimated Monte Carlo
standard error, not calibrated perception uncertainty. Every candidate in a case
uses the same points and a domain enclosing all candidate spheres. Zero sampled
misses are only a diagnostic, separate from the construction argument.

| Reference / exact volume | 1-sphere excess | 2-sphere excess | 4-sphere excess |
| --- | --- | --- | --- |
| Cylinder, 7 × 7 × 30 cm bounds / 1.1545 L | 15.4225 ± 0.0547 L | 4.7699 ± 0.0379 L | 2.2098 ± 0.0269 L |
| Cube, 8 cm side / 0.5120 L | 0.8784 ± 0.0047 L | 0.8346 ± 0.0046 L | 0.9045 ± 0.0047 L |

More spheres substantially reduce excess for the elongated case but are not
monotonically better for the cube. These simple covers can still be very loose.
No candidate is automatically selected, and this does not establish Jev's value.
No timing claims, obstacle/robot-pair choices, margins, ellipsoids, world-pose
integration, dynamic updates, perception, or CBF enforcement have been tested by
this geometry experiment.

### Selection request observations (not Jev answers)

The preview uses the same 1/2/4-sphere construction. Its proxy has radius 0.025 m
and nominal segment from [0.10, 0, 0.06] to [0.60, 0, 0.06] m. Objects are factual
inputs only: no expected avoidance decision is supplied. Per-candidate minima are
descriptive features, not min-aggregated CBFs or full-arm safety checks.

| Object centre (m) | 1-sphere nominal clearance | 2-sphere nominal clearance | 4-sphere nominal clearance |
| --- | --- | --- | --- |
| Bottle [0.30, 0.09, 0.15] | -55.7 mm | -23.6 mm | +5.7 mm |
| Cube [0.44, -0.06, 0.04] | -31.0 mm | -21.8 mm | -19.2 mm |
| Far cube [0.25, 0.45, 0.04] | +356.2 mm | +365.4 mm | +368.0 mm |

These numbers distinguish a conservative-cover obstruction from a clear
representation for the bottle, and an obstructed nominal motion versus a distant
object. They do not determine or validate Jev's response. The default model is the
documented stable alias `jev-latest`; the first call returned `jev-1.13.0`. No
preview model was selected silently.

### First live model observations

One successful SDK invocation, using the unchanged starting scene/questions,
returned these proposals. Full probabilities are retained in the local report.
Reported model confidence is not probability of robotic safety.

| Object | Proposed role (probability) | Selected cover | Conditional geometry probabilities: 1 / 2 / 4 spheres | Geometry confidence |
| --- | --- | --- | --- | --- |
| Bottle | avoid (0.89) | 4 spheres | 0.06 / 0.19 / 0.75 | 0.62 |
| Cube | avoid (0.96) | 4 spheres | 0.17 / 0.37 / 0.46 | 0.19 |
| Far cube | omit (0.96) | none | 0.33 / 0.47 / 0.20 | 0.20 |

The far cube's conditional geometry answer is retained but not selected because
its role is `omit`. Two objects and eight spheres were selected for the modeled
proxy; no QP was built. Omitting one object does not establish fewer constraints
than an all-on/single-sphere baseline (which would have three spheres here).

The bottle answer is consistent with the supplied local-clearance/volume features.
The cube's four-sphere choice has the largest estimated global excess volume of
its three candidates and more primitives, but slightly less intrusion along the
particular nominal segment. It is not dominated on every supplied feature. The
model provides classifications, not explanations, so favoring local clearance is
only a possible interpretation, not a reported reason. Its weak preference is a
useful question for prompt/task comparisons, not evidence that one representation
is optimal or that the answer is definitively wrong.

The 0.628746 s measurement includes SDK client/request/response handling, not just
model inference; one observation is not a latency benchmark. No controller,
physics, model thresholds, prompt wording, or scene was changed to improve the
result, and no follow-up service queries have been made in this checkpoint run.

## Execution workflow and next action

Follow the active runtime policy. Current permissions allow direct network,
uv, and GUI operations; do not request escalation under a policy that forbids
approval requests. In a future restricted session that permits scoped approvals,
use its supported approval flow rather than hand commands to Nihar. Earlier
narrow approvals were saved for the PyPI HEAD check and this demo launch:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_demo.py
```

Keep using the Jev repo as the working directory. Runtime permission breadth does
not expand the project's authorized scope. No global security settings, runtime
databases, upstream source, solver, or controller were modified. Both bounded
test runs closed their own viewers; user-run processes were not targeted.

Historical cause: the active chat had workspace-only permissions, restricted
networking, and sandbox-approval requests disabled even though a fresh diagnostic
loaded broader user-config defaults. Reading config.toml did not establish active
permissions, and restarting/resuming did not resolve them. An earlier doctor run
also reported a database-open error inside the sandbox; no database corruption
was established and no database repair or deletion was attempted.

Nihar approved the small standalone selection experiment, configured the key,
then approved viewing its saved result in the scene. The viewer is ready:

```bash
uv run --offline --frozen --cache-dir .uv-cache mjpython -B -u view_selection.py
```

Nihar reviewed the scene and said "looks good"; this checkpoint is complete.
The uncertain cube tradeoff remains a research question, and the static nominal
path has not been made safe. Proposed next step, not yet approved: connect saved
selections to the FRAX/CBFpy controller for a static, no-contact reaching scene,
preserving existing robot protections, before attempting online updates.
Agree on the next integration step with Nihar; do not silently change prompts,
introduce a cost weighting, or
launch repeated service calls to obtain a preferred result. Credentials load via
uv's `--env-file .env.local`; do not print the key or search unrelated stores.
Do not automatically add ellipsoids, controller integration, real contact
exemptions, repeated querying, or a benchmark suite. A model's type-correct answer
does not establish its obstacle-selection correctness or safety.

## Approved decision log

These decisions came from Nihar's explicit directions/approvals in this discussion.

| ID | Decision | Rationale / scope |
| --- | --- | --- |
| D01 | The Jev repository owns new work; the original three repos stay read-only. | Reuse/import upstream implementations rather than edit or duplicate them. |
| D02 | Develop one small step at a time, review the result, then choose the next. | Avoid a brittle, overprescribed architecture or premature future decisions. |
| D03 | Jev templates/orchestration are evolving research components. | Do not treat a fixed question decomposition or update policy as settled. |
| D04 | First checkpoint: existing FRAX OSCBF simulation demo, no new Jev functionality. | Establish the baseline before building the new framework. This is not final hardware selection. |
| D05 | Use uv, an isolated project environment, and existing local read-only checkouts. | Keep setup separate from global environments and upstream source trees. |
| D06 | Try the current Mac first; report compatibility problems before changing course. | Learn from the approved starting environment rather than silently switching. |
| D07 | Keep AGENTS.md plus progress and decision tracking in this repository. | Make continuation, assumptions, and approval history explicit. |
| D08 | Next checkpoint: isolated sphere-cover candidates, coverage/excess-space checks, and inspectable visualizations. | Nihar approved "ok start working" after the proposed 1-sphere versus chain-of-spheres experiment. No Jev calls or controller integration included. |
| D09 | Tiny standalone task-conditioned Jev experiment using known scene objects and candidate IDs. | Nihar answered "yes" after screenshot review. Build/inspect the request and model proposals without robot integration; live access needs local credentials. This does not approve broad loops, contact-policy changes, or hardware work. |
| D10 | View the saved Jev selection in its full synthetic scene. | Nihar approved the next-step viewer. Display recorded objects, selected covers, and nominal proxy motion without additional API calls or controller integration. |

Nihar owns consequential ambiguities; routine local implementation choices do not
require an approval ceremony. Approval of D02/D03 did not approve all mentor ideas.

## Provisional implementation notes

- The launcher uses a child-process-only PYTHONPATH and executes the upstream
  script directly. This preserves its imports/assets and avoids upstream build
  metadata/bytecode writes. It is a minimal demo adapter, not a final packaging
  decision for the full framework.
- External requirements follow the inspected FRAX/CBFpy manifests and this demo;
  unrelated extras and the old OSCBF package's pinned stack are not installed.
- Launcher checks intentionally do not import/initialize JAX or start a viewer.
- The seven-module layout in ARCHITECTURE.md is illustrative, not a scaffold to
  build before completing this checkpoint.
- Geometry currently uses ordinary functions and one immutable sphere-cover data
  record, not the full future scene/constraint schema. Box/cylinder truth and
  1/2/4 sections are inspectable starting cases, not a frozen candidate family.
- Rejecting zero extents instead of silently adding a minimum size is a local
  input-validation choice for positive-volume test objects. Revisit if real
  degenerate measurements need handling; do not silently inflate future inputs.
- The text-only scene is a provisional free-space fixture, not a new FRAX robot
  task. No interaction exemption is granted. `avoid`/`omit`/`review` and a separate
  conditional geometry Choice are starting questions; their wording is in JSON.
  Changing role keys also requires updating the small response decoder.
- The official TypeSafe SDK is optional (`--extra jev`) and supplies HTTP/auth,
  retries/timeouts, and response types. Local fake-transport checks are labeled
  fabricated; default preview does not import the SDK or call the service.

## Pending research proposals, not current blockers

Selection priorities/contact exemptions; Jev-selected calibrated margins;
additional primitives and broader mesh/perception coverage evaluation; new reaching/pushing or
manipulation tasks; once/repeated Jev and thinking-model comparisons; gallery and
expanded replay/viewing workflow; adaptive gains; optional ponytail tooling/style.
Ground-truth inputs initially and later perception replacement are already part
of the project direction, not a requirement to implement perception now.

Decide relevant questions with Nihar when the corresponding step is chosen.

## Activity and publication status

- 20 October 2026: created the planning repository; recorded the architecture,
  mentor feedback review, iterative approach, and approved first-checkpoint choices.
- 20 October 2026: Nihar said to start; prepared the launcher/configuration and
  checked it offline. Requested help with uv/dependency setup after confirming
  terminal download failures.
- 20 October 2026: Nihar requested AGENTS.md and progress/decision tracking; added
  them without extending the research scope.
- 20 October 2026: retried terminal network access at Nihar's request; installer
  and package-index endpoints still failed DNS resolution. Setup remains blocked.
- 20 October 2026: investigated the config/session mismatch; confirmed the
  subprocess network-disabled marker and checked supported CLI resume flags.
- 20 October 2026: Nihar confirmed the restart. Inspected current-session
  metadata and found the effective `:workspace` profile with networking off;
  requested the CLI's configuration-layer diagnostic to locate its source.
- 20 October 2026: Nihar completed manual setup. Verified the installed stack,
  reran all seven launcher tests, and passed a headless upstream model/CBF smoke
  check. Full demo and minimal mjpython both aborted during Cocoa/GLFW startup;
  normal-Terminal viewer verification is the next action. Upstreams stay clean.
- 20 October 2026: Nihar supplied normal-Terminal output reaching "Demo ready".
  Recorded successful user-run startup; target tracking, obstacle response, and
  shutdown observations are still pending. No new research feature was started.
- 20 October 2026: Nihar prioritized eliminating the manual-Terminal bottleneck.
  Confirmed agent-run uv sync works. Compared fresh Codex diagnostic settings
  with this conversation's saved permissions: defaults allow network/approval,
  but this thread remains restricted with escalation disabled. Next step is the
  current CLI's supported permission-mode control; no security settings changed.
- 20 October 2026: Nihar changed the current CLI's permission mode. Scoped
  execution-approval requests became available. Agent verified PyPI HTTP 200,
  launched the demo to readiness, observed it remain running, and stopped its
  own test via Ctrl-C. The manual-Terminal bottleneck is resolved for this workflow.
- 20 October 2026: Nihar enabled the unrestricted runtime and said to continue.
  Verified direct networking, completed two identical bounded 1,000-step baseline
  runs, and added the rerunnable smoke check. All seven launcher tests still pass.
  Baseline execution checkpoint complete; next research choice has been requested.
- 20 October 2026: Nihar approved the isolated sphere-cover step. Implemented
  deterministic covers, independent reference-shape volume checks, a rerunnable
  comparison CLI/viewer, and 12 new tests. All 19 tests pass. Cylinder/cube results
  show the intended primitive-count/conservatism tradeoff; bounded viewer launches
  succeed, but computer-use permission denied screenshots, leaving visual review
  with Nihar pending. All upstreams remain read-only; no model calls/dependency
  changes/controller edits. Paused for checkpoint review, not a technical blocker.
- 20 October 2026: Nihar requested joint inspection. Opened the live cylinder
  comparison with `mjpython -B -u inspect_geometry.py`, leaving it open for review
  rather than using a timed preview. Computer-use access to mjpython remains
  denied; awaiting Nihar's observations or a shared screenshot before marking
  visual review complete. No geometry/controller code or research choices changed.
- 20 October 2026: Nihar requested a concrete permission diagnosis after generic
  settings advice was unhelpful. Verified the installed/enabled bundled plugins,
  running native Computer Use service, current chat policy, and exact installed
  code path for "Computer Use was not approved to use mjpython". The failure is
  a non-accepted app approval; `never` mode likely suppresses its prompt. Proposed
  a user-controlled interactive approval retry, not another installation or uv
  change. No global configuration, permission database, or app policy was edited;
  no alternative screenshot channel was used to bypass the denied app access.
- 20 October 2026: Nihar supplied the cylinder comparison screenshot directly.
  Reviewed the 1/2/4-sphere layout and transparency; chains visibly tighten the
  representation but retain substantial excess volume. Their overlap is expected
  for a union cover. This image does not prove containment or resolve direct
  Computer Use approval. Opened the live cube comparison (8 cm side, 1/2/4
  spheres) for the next inspection. Cylinder visual review recorded; cube review
  remains pending Nihar's observations or image.
- 20 October 2026: Nihar supplied the rotated cube comparison screenshot. Reviewed
  all three covers: splitting only one axis leaves two section dimensions fixed,
  so the spheres shrink only modestly while their centres spread out. The visible
  extension is consistent with the measured larger excess for four spheres than
  one. Initial screenshot review is complete for both examples; it is not a 3D
  coverage proof. No generation changes, Jev calls, or controller integration
  were made. Await Nihar's next checkpoint choice.
- 20 October 2026: Nihar approved a tiny standalone Jev experiment. Verified the
  current official API/SDK shape, installed optional SDK 0.7.5 through uv, and
  implemented editable synthetic scene/questions, generated request features,
  preview/live modes, response checks, and 12 new tests. All 31 tests pass,
  including real-SDK/local-fake-transport serialization and failure checks. Preview
  succeeds; live mode exits before network because TYPESAFE_API_KEY is absent.
  Asked Nihar about access; no service calls, real responses, credentials stored,
  controller changes, or upstream edits. Work remains local/unpushed.

- 20 October 2026: Nihar requested a local credential template. Created
  `.env.local` with an empty TYPESAFE_API_KEY assignment, restricted it to owner
  read/write, and verified Git ignores it and does not track it. Asked Nihar to
  fill/save the local file, not upload it into chat. No live request was made.
- 20 October 2026: Nihar confirmed the key was ready. Verified presence, official
  default HTTPS endpoint, owner-only file permissions, Git exclusion, and no key
  in the saved result. One live SDK invocation succeeded and exited 0, returning
  `jev-1.13.0`, six valid answers, and 6,021/277 input/output tokens in 0.628746 s.
  Bottle/cube: avoid with four spheres; far cube: omit. Cube geometry confidence
  was low (0.19), so recorded the ambiguity without changing the prompt or making
  additional calls. Saved the complete local result and paused for review. No
  robot integration, upstream edits, security changes, commit, or push.
- 20 October 2026: Nihar approved the saved-result scene viewer. Implemented it
  with shared static-viewer helpers and eight fabricated-response display tests.
  All 39 offline tests pass; actual-report scene compilation, bounded oblique and
  top-down windows, and the original geometry viewer regression exited 0. Saved
  live report unchanged; no key loading, service calls, physics, upstream changes,
  commit, or push. Ready for visual review; closed-loop simulation remains future
  work, not a claim of this checkpoint.
- 20 October 2026: Nihar inspected the open viewer and said "looks good." Marked
  the viewer checkpoint complete based on user visual review. No controller
  integration, new API request, or next-step scope was authorized by that review.

- 20 October 2026: Nihar requested committing and pushing the completed work to
  the existing GitHub repository before beginning integration. Re-ran all 39
  offline tests successfully. Publication scope is the launcher, geometry and
  selection tools, viewer, tests, editable fixtures/templates, uv requirements
  and lockfile, and documentation. Credentials, environments/caches, and generated
  reports remain local/ignored. This request does not approve controller work.

Publication requested; commit/push verification is recorded below once complete.
The deleted stale parent architecture copy remains recoverable from the
repository's original commit `b347afe`.
