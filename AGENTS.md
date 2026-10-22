# Working in the Jev repository

Read [PROGRESS.md](PROGRESS.md) before continuing work. It records the current
checkpoint, completed checks, blockers, approved decisions, and the next action.
[ARCHITECTURE.md](ARCHITECTURE.md) is an evolving research reference, not a frozen
implementation checklist. Follow the user's latest directions if they differ.

## Scope and upstream reuse

- All new code and documentation belong in this repository. Do not create another
  architecture/progress file in the parent workspace.
- The sibling FRAX, OSCBF, and CBFpy repositories are read-only: inspect/import
  them, but do not edit their source, configuration, dependencies, or Git state.
- Reuse existing implementations. Do not copy controller, robot-model, barrier,
  or optimizer code into this project to bypass an integration issue.
- Avoid generating build metadata or bytecode in upstream directories. The
  current launcher uses child-process-only import paths and disables bytecode
  writes. Report interface mismatches rather than silently patching dependencies.

## Work one checkpoint at a time

- Active correction (22 October; D29): Nihar requires exactly one/two/three
  clutter obstacles in easy/medium/hard, excluding the pickup target. Correct
  the fixture, validate/display counts and rerun qualifications plus startup
  and repeated comparisons in fresh folders. Preserve the old 2/2/3 datasets
  and label them historical. This supersedes the D27 experiment stop for this
  correction only. Keep existing contact assumptions, provider/model, original
  cumulative $7.50 cap, read-only upstreams and all other experiment settings.
  No hardware, new paid service or dependency change. Publish the verified source
  correction under the existing D28 push request, excluding generated evidence
  and credentials as before.
  D29 is complete: `artifacts/rgbd/count123_startup_v1/index.html` and
  `artifacts/rgbd/count123_repeated_v1/index.html`. All 15 startup and nine
  repeated runs finished; 131 tests pass. The original ledger is at 595 calls
  and $4.913233920 reservations against $7.50. Prior 2/2/3 results remain intact.
  No further experiment or provider call follows this completed correction.

- Publication authorization (22 October; D28): Nihar requested pushing the
  completed D26/D27 work to the existing GitHub repository. Commit and push the
  RGBD/adaptive-cover and repeated-Jev source, tests, scene/prompt fixtures,
  renderer changes and documentation after verifying offline tests, the staged
  credential exclusion, upstream cleanliness and the remote identity. Keep
  generated evidence, the cumulative usage ledger, credentials, environments
  and caches excluded. No new experiment, API call, hardware work or dependency
  change is part of this publication request. Earlier no-push checkpoint stops
  are superseded for this source publication only.

- Active authorization (21 October; D27): Nihar requested repeated Jev tests as
  well. Run all three revised Jev ablations across easy/medium/hard, paired to
  `artifacts/rgbd/gallery_v2`, using exact prior inputs/startup answers and reused
  non-Jev baselines. This supersedes the D26 no-new-experiment stop for these nine
  rollouts. Keep the existing completion-driven 2 Hz ceiling, one request in
  flight, read-only upstreams, contact assumptions and cumulative $7.50 ledger.
  Preserve all original outcomes; no scene/prompt/controller retuning, new paid
  service, hardware, commit or push. Verify and document results and failures.
  D27 is now complete at `artifacts/rgbd/repeated_v1/index.html`: nine new runs,
  unchanged safe-pickup outcomes (3/3 adaptive conditions, 0/3 single-sphere),
  115 new requests, one invalid late answer preserved without application or
  re-query. All nine new trajectories pass independent native audit. The ledger
  has 499 calls and $4.120510464 reservations against $7.50. No further automatic
  experiment/call is authorized by this completed comparison; see PROGRESS.md.

- Active authorization (21 October; D26): implement the RGBD/adaptive-cover
  revision autonomously through the easy/medium/hard five-method gallery.
  This supersedes historical planning-only/completed-checkpoint stops below.
  Make documented reversible choices; retain failures, verify with tests, obtain
  existing credentials and cumulative ledger securely before live calls. Preserve
  scope, contact policy, upstream reuse and the existing $7.50 cumulative cap.
  Ask only for genuine blockers or consequential changes to those boundaries.
  No hardware, new paid services, commit or push is included.
  This revision is now complete locally: `artifacts/rgbd/gallery_v2/index.html`,
  15 primary runs, 19 independent audits and 125 passing tests. Ten new logical
  Jev calls cost an estimated $0.007258944; the cumulative original ledger remains
  authoritative. Preserve the archived producer sources and retained failures.
  Nihar reaffirmed autonomous completion without routine permission prompts.
  See current PROGRESS.md before any further experiment or provider call.

- Publication authorization (21 October; D25): Nihar requested publishing the
  latest revision plan and engineer handoff to the existing GitHub repository.
  Commit/push AGENTS.md, ARCHITECTURE.md and PROGRESS.md only after checking the
  diff, staged credential exclusion and remote identity. Keep generated evidence,
  credentials and environments excluded, and upstreams read-only. This publishes
  D22-D24 planning, not implementation or validation of the new experiment.
  No refactoring, new run/provider call, hardware access or installation is in scope.
  Plan/handoff commit `a55d3dd` has been pushed to `origin/main` and the remote
  hash verified. No experimental implementation was started during publication.
- Current direction (21 October; D22): planning only for the next experiment
  revision. Record the five agreed comparisons, unchanged current-off semantics,
  proposed 1 ms simulation/control step, independent completion-driven Jev loop
  capped at 2 Hz wall-clock dispatch, and hardware-available observation inputs.
  PROGRESS.md's revision plan is authoritative over older oracle-input proposals.
  Do not implement these revisions, rerun studies, call a provider, connect a
  device or push until Nihar explicitly requests the next implementation step.
  Scene and target estimates must eventually come from simulated RGBD through
  the shared camera/perception boundary, not simulator object metadata or oracle
  masks; simulator-only truth remains separate for plant construction/evaluation.
  Perception/grasp methods and remaining experimental details are not yet chosen.
  D23 adds a geometry requirement: replace fixed count menus/axial box chains in
  the new method with observation-driven, adaptive sphere covers bounded by
  sphere-count and computation budgets. PROGRESS.md records primary research
  and a proposed adaptive enclosing-sphere method; exact algorithm, numeric caps
  and latency are not yet approved/measured. Keep single-sphere baselines and
  historical results. No implementation or new run is authorized by this planning.
  D24 adds four primary metrics (unwanted collision, pickup success, CBF count,
  run cost) and a first revised demo targeting easy/medium/hard static scenes
  qualified by real off-mode collisions, not Jev wins. See PROGRESS.md's handoff
  subsection; D25 separately authorizes publishing these planning documents.
- Publication authorization (21 October; D21): Nihar requested pushing the
  completed D20 camera plumbing to the existing Jev repository. Commit/push
  source, tests, the unverified example configuration and documentation only.
  Keep credentials, environments, caches and generated camera/experiment data
  local/ignored. Verify tests, staged credential exclusion, upstream cleanliness
  and remote commit identity. No device connection, installation, new experiment
  or provider call is included in this publication request.
  Implementation commit `6be13ca` has been pushed to `origin/main` and its remote
  hash verified. Generated evidence remains local. No further automatic work.
- Active authorization (21 October; D20): build the approved hardware-unverified
  camera plumbing: shared frames/replay, explicit calibration, optional RealSense
  adapter, separate masks, background acquisition/processing/recording, inspection
  and offline tests. Keep old simulation numerics available. No device connection,
  robot deployment, learned detector, Jev calls, upstream edits or new push.
  SDK mocks and synthetic/simulator data are software evidence, not hardware QA.
  This checkpoint is now complete locally: 117 passing tests, replay-to-CBF
  simulation integration and an offline scene/wrist inspection at
  `artifacts/camera/review_v1/index.html`. See PROGRESS.md for evidence and limits.
  No SDK/device was used; upstreams remain clean. Source is published under D21.
  Do not connect hardware, install drivers, start new experiments or push without
  new direction. The earlier simulation datasets/gallery remain unchanged.
- Publication authorization (21 October; D19): Nihar requested pushing the
  completed D18 checkpoint to the existing Jev repository. Commit/push source,
  tests, fixtures and documentation only. Keep credentials, environments, caches,
  generated trajectories/galleries and the 2 GB review bundle local/ignored.
  Verify tests, staged credential exclusion, upstream cleanliness and remote
  commit identity. This request does not authorize new experiments/API calls.
  Source checkpoint `b1c5209` is now published on `origin/main` and its remote
  hash verified. Generated evidence/bundle stay local; no further automatic work.
- Active authorization (21 October; D18): Nihar approved autonomous work through
  selection diagnosis/improvement, comparison with a geometric rule, stronger
  unseen static-clutter pickup evaluation, simulated wrist/scene RGB/depth
  geometry estimation, and imperfect-observation/model-failure tests (points 1–5).
  Also investigate richer sphere counts than 1/2/4. This supersedes the completed
  package's no-new-experiment stop below. Work evidence-first, keep settings and
  prompt changes editable, retain failed attempts, and do not manufacture Jev
  superiority. Keep the existing persistent $7.50 total Jev budget, not a fresh
  cap. No frontier-provider calls, hardware, moving clutter, upstream edits or
  new push are included. Source checkpoint `cae821a` is the starting publication.
  D18 is now complete locally: 354 frozen study trajectories independently
  audited, four additional startup-correction pickups and two refusals, and
  101 passing tests. Review `artifacts/pickup/review/index.html` and current
  PROGRESS.md. Rich covers help; Jev does not beat the geometric rule here.
  Final packaging checks are offline only. Do not start another automatic
  experiment/provider call or push without new direction. Preserve all failures.
- For perception, distinguish simulator segmentation/ground-truth labels used
  as scaffolding from actual learned detection. Keep physical evaluation truth
  separate from observation-derived controller geometry; do not silently fill
  occluded/unseen surfaces with oracle dimensions or claim full-object coverage
  from visible points. Robot/camera calibration can remain known in simulation.
- Current task direction (21 October; D14): use one task family only. The arm
  maneuvers through static clutter, picks up the designated target, and stops
  once pickup is verified. Layouts/target positions are variations of that task,
  not separate task families. No placing, transport, pushing, or moving-obstacle
  experiments. The target may move during pickup; surrounding obstacles do not.
  Intended gripper-target contact must be distinguished from collisions with
  clutter or other robot parts. Physical pickup is now implemented locally via
  native articulated fingers/free target in the `frax_gripper` profile; the older
  `frax` reaching profile keeps rigid fingers. See current evidence in PROGRESS.
  Preserve the completed moving-object gallery
  as historical infrastructure evidence, not the intended manipulation task.
  Nihar approved implementation through the pickup gallery and then the seven
  evidence-driven follow-on items (D15). Start with functional gripper/pickup
  physics, then integrate selection/contact handling and comparisons. Continue
  offline diagnosis, broader static layouts, geometry/selection checks, profiling,
  and workflow improvements as evidence warrants. Nihar authorized up to $7.50
  total new Jev API spend for this work (D16), using the existing key. Verify
  current direct-API pricing and track/reserve usage before live requests; the
  dollar limit replaces the proposed request-count caps. Publication/push is not
  included in this approval. Keep failures and record consequential choices.
  The four-item pickup gallery is now complete locally at
  `artifacts/pickup/showcase_v1/index.html` (12 rollouts/14 new calls/79 tests).
  The follow-on package is now implemented: review hub at
  `artifacts/pickup/review/index.html`; corrected 12-layout/six-method replication
  and original failures are separate. 89 tests pass. Repeated 12/12 safe pickups
  versus once 6/12; all-on/rule also 12/12, with the rule stronger on geometric
  costs. Do not claim Jev superiority. 114 new logical requests cost an estimated
  $0.067044516; conservative reservations $0.941359104. Keep the persistent ledger,
  not a reset budget for each dataset. Do not start another experiment/API call
  automatically after this completed package. Preserve frozen inputs, raw answers,
  trajectories and source snapshots; no push unless requested.
  Nihar separately requested publication on 21 October (D17): commit and push this
  completed source/test/fixture/documentation checkpoint to the existing Jev repo.
  Keep credentials, environments, caches and generated experiment/gallery artifacts
  local/ignored. This publication request does not authorize another experiment.
  Source checkpoint `dc566c7` was pushed successfully to `origin/main`; see
  PROGRESS.md for publication verification. Generated review galleries remain local.
- Completed authorization (superseded the older checkpoint stop points below):
  Nihar asked to continue autonomously through diagnosis, fixes, continual
  constraint updates, verification, and the final simulation showcase, without
  intermediate human input. Make evidence-based local implementation choices and
  document them. Necessary bounded Jev calls for the dynamic comparison are in
  scope; retain every attempt, never re-query for preferred answers, and use the
  existing SDK/environment credentials. Keep upstream checkouts read-only, the
  FRAX nominal policy, no-contact simulation scope, and all robot protections.
  No hardware, perception, publication/push, or unrelated research expansion.
  Do not stop at the old static-review gate or require routine approvals.
  This task is now complete (21 October): final synchronized gallery at
  `artifacts/showcase/dynamic_v1/index.html`, eight dynamic rollouts, 12 new
  logical SDK invocations, and 71 passing tests. Do not start another dataset,
  prompt variant, or live call automatically. Keep reported limitations visible:
  idealized matched plant, empirical sampled-control guard, fixed allocated QP
  rows, and no observed late dropping of previously selected obstacles.
- The original FRAX OSCBF baseline and isolated sphere-cover checkpoint are
  complete. Nihar approved a tiny, standalone text-only Jev selection experiment
  using known scene objects and generated candidate IDs, followed by a viewer of
  the saved result in its scene. Both are implemented; Nihar reviewed the new
  scene viewer and said "looks good." Nihar subsequently approved continuing
  through a static, no-contact Panda simulation comparison: selected-cover
  integration, tests, scene-obstacle off/all-on/selection-replay runs, diagnostics,
  and a rerunnable visual gallery, then human review. See PROGRESS.md.
- Static integration and the first three-mode comparison are implemented. Nihar
  reviewed the gallery (`artifacts/static/review/index.html`) and said "all the
  animations look good and normal." This checkpoint is complete. Agree on the
  next experiment before expanding scope or model authority. All modes worked in this easy scene;
  do not claim a demonstrated safety advantage or a whole-arm Jev evaluation.
- Nihar approved the next extended checkpoint: a robot-aware static suite of
  about six fixed scene variations, actual Panda/motion context, off/all-on/Jev
  comparisons, tests, diagnostics, and a combined review gallery. Six new one-shot
  SDK invocations (one per scene) are in scope; ordinary SDK retries may apply.
  Freeze scenes/questions/requests before inspecting model answers, retain all
  outcomes, and never re-query to obtain preferred answers. See D12 in PROGRESS.
  This checkpoint is now implemented: six calls and 18 rollouts are complete;
  Nihar accepted the gallery's visual-review checkpoint, saying "yes it looks
  good" after the guided comparison. All six authorized calls have been spent.
  Agree on the next task before further experiments or model calls. This visual
  acceptance does not resolve the four negative modeled scene-barrier cases or observed
  model/simulator acceleration mismatch; do not silently retune or fix upstreams.
- Reuse the unchanged FRAX nominal controller, demo CBF config, and environment.
  Preserve joint/singularity/floor/self-collision and actuator protections in
  every mode. "Off" means only scene-object constraints off. All three upstream
  repos remain read-only; a scoped in-process adapter is allowed, not disk edits.
- The older standalone saved selection concerns an end-effector proxy, not Panda. Label reuse as
  proxy-decision replay, record the actual robot start/goal and context mismatch,
  and evaluate all objects independently of which are constrained. Do not present
  this as a fresh whole-arm Jev decision or a model-performance experiment.
  The newer robot-aware suite uses its own frozen actual-Panda requests and fresh
  answers; keep these two sources distinct.
- No online updates, interaction/contact exemptions, pushing/grasping,
  perception, other primitive families, hardware, or controller replacement in
  this checkpoint. Further model calls beyond the six approved invocations need
  agreement. Ordinary test/scene implementation choices are recorded, not a new
  approval gate at every helper or numeric pose.
- Show the result and relevant failures to Nihar before expanding scope. The
  next step follows evidence, not a predetermined milestone sequence.
- Nihar owns consequential research/safety ambiguities. Ask before changing
  scope, contact assumptions, model authority, or the upstream reuse strategy.
  Ordinary helper names, local organization, and implementation mechanisms can
  follow what is simple and natural without individual approval.
- Jev templates and orchestration are research components. Keep them inspectable
  and easy to revise when introduced; do not freeze a question taxonomy, timing
  policy, confidence cutoff, or worker design without a demonstrated need.

## Environment and verification

- Use uv and the local `.venv`, not a shared pip/conda environment. Keep the uv
  cache local with `--cache-dir .uv-cache` when executing in a restricted session.
- Do not commit `.venv`, caches, credentials, or simulator logs. Commit a real
  `uv.lock` when dependency resolution succeeds; do not invent one.
- Offline tests in the installed project environment:
  `uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B -m unittest discover -s tests -v`.
  Launcher-only tests need no external dependencies:
  `python3 -B -m unittest discover -s tests -p test_run_demo.py -v`.
- Once setup is available: `uv sync --cache-dir .uv-cache --python 3.12`, then
  `uv run --cache-dir .uv-cache python run_demo.py --check` and
  `uv run --cache-dir .uv-cache python run_demo.py`.
- uv setup is now verified. Follow the active runtime permissions, not historical
  troubleshooting notes: execute directly when allowed. If a restricted session
  permits scoped approvals, request those for blocked GUI/network operations;
  never request escalation when the active policy forbids it. Do not make Nihar
  type commands in a separate Terminal as the normal workflow.
  The verified repeatable GUI command is
  `uv run --offline --frozen --cache-dir .uv-cache python -B -u run_demo.py`.
  New capabilities may still require approval; existing approvals are not a
  blanket grant. Do not change security configuration or bypass restrictions.
- Bounded baseline check on this Mac:
  `uv run --offline --frozen --cache-dir .uv-cache mjpython -B -u tests/smoke_demo.py --steps 1000`.
  This opens the real viewer and calls the original upstream main/controller.
  Leave its default target unchanged; it closes automatically at the step count.
  Report model-barrier values as sampled diagnostics, not independent safety proof.
- Isolated geometry checks:
  `uv run --offline --frozen --cache-dir .uv-cache python -B inspect_geometry.py --no-view`.
  Use `mjpython` instead of `python` and omit `--no-view` for the Mac viewer.
  Coverage follows the section-enclosure construction; sampled points are only
  diagnostics. Union/excess-volume Monte Carlo errors are sampling errors, not
  calibrated perception uncertainty. Primitive count is not measured QP runtime.
- Selection request preview (no network/key/SDK required):
  `uv run --offline --frozen --cache-dir .uv-cache python -B inspect_selection.py`.
  The scene and questions are editable JSON under `examples/` and `prompts/`.
  The synthetic end-effector sphere is not the FRAX arm or a whole-arm risk model.
- Optional Jev SDK: `uv sync --extra jev --cache-dir .uv-cache`.
  Add `--extra jev` to uv run for the SDK transport tests or `--live` experiment.
  Read TYPESAFE_API_KEY from the environment; never print it, search unrelated
  credential stores, or commit credentials. `.env` and `.env.*` are ignored.
  Missing keys/model failures must not become fabricated or fallback Jev answers.
  Keep request previews and offline fabricated test responses clearly distinct
  from live service results. Model probabilities are not robot-safety guarantees.
- Saved selection viewer (no key, SDK, or API call):
  `uv run --offline --frozen --cache-dir .uv-cache mjpython -B -u view_selection.py`.
  Reads `artifacts/selection/live_initial.json` by default; `--report` selects
  another successful saved result. `--check` with regular python validates and
  compiles without a window; `--top` changes the initial camera. The nominal sweep
  is unfiltered, not a safe trajectory. Table geometry is display-only. Do not
  imply this static inspection is a closed-loop simulation or visual QA complete
  merely because launch checks pass.
- Static simulation: `uv run --offline --frozen --cache-dir .uv-cache python -B -u
  run_static.py --output artifacts/static/NEW_RUN`. Existing runs are not overwritten.
  Add `--view` and use mjpython on this Mac for bounded live physics. Render the
  saved data with `static_gallery.py --run-dir artifacts/static/NEW_RUN`, then open
  its index.html. No new dependencies, service calls, or screenshot access needed.
  Headless simulation and offscreen rendering are separate; GL failures must not
  be described as successful gallery generation. The real-upstream regression
  test needs the sibling sources and checks protection parity/frame agreement.
- Passing launcher tests or package-presence checks is not a successful simulation,
  controller-parity result, or safety guarantee. State exactly what was run.
- Dynamic showcase: `run_showcase.py status` and `run_showcase.py gallery` under
  uv offline/frozen need no key, API call, or new physics. `run --live` is only
  for a deliberately authorized experiment; the completed output skips recorded
  calls/runs. Scenes/templates are editable JSON, not a frozen research taxonomy.
  Original dynamic baseline summaries retain an early mocap-as-robot diagnostic
  error; `collision_recheck.json` is hash-bound to trajectories, and the gallery's
  `review_results.json` applies it explicitly without replacing original data.
- Preserve explicit units/frames/action meaning and report numerical failures.
  Never claim an unvalidated replacement torque or zero torque is a safe stop.
- Do not silently change machines, solvers, or upstream revisions when blocked.
  Report the cause and the options. Do not bypass access restrictions.

## Keep progress and decisions current

- Update PROGRESS.md after meaningful implementation, verification, a new blocker,
  or an approved decision, and before handing work back.
- Keep current status concise. Record the commands/outcomes needed to resume,
  what is still unverified, and the next small action.
- Distinguish user-approved decisions, provisional implementation choices, and
  pending mentor proposals. Do not mark a suggestion approved without confirmation.
- Preserve a short dated activity/decision history; avoid a second competing log.
- Update architecture assumptions when they materially change, not for each helper
  or local refactor. Use descriptive task names rather than unexplained M1/M2 labels.
- Commit or push only when requested, and distinguish local work from published work.
