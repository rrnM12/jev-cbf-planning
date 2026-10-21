# Progress and decisions

Updated: 21 October 2026.

## Current checkpoint

### Complete: pickup gallery and approved follow-on study

Start with `artifacts/pickup/review/index.html`. The original four-item gallery,
geometry-component study, two development templates, independent physical checks,
deterministic-rule baseline, geometry-only ablation, CPU profiling, replay tooling,
and 12-layout evaluation/corrected replication are implemented.

| Corrected replication (same 12 layouts) | Safe pickups | Unwanted-contact cases | Mean enabled pairs |
| --- | --- | --- | --- |
| Scene CBFs off | 11/12 | 1 | 0 |
| All-on | 12/12 | 0 | 296.00 |
| Geometric rule | 12/12 | 0 | 151.71 |
| All objects / Jev geometry | 6/12 | 0 | 172.75 |
| Jev once | 6/12 | 0 | 146.17 |
| Jev repeated | 12/12 | 0 | 156.94 |

All 72 corrected trajectories pass independent native re-evaluation, identical-
start/static-clutter/free-target checks, hashes, finite state/control checks and
successful pickup confirmation windows. All 36 model-query snapshots match the
recorded repeated-run q/v states; no sampled modeled h2 is negative, transformed
h1 stays positive, and requested torque-limit excess is zero. Original held-out
data (7/12 once, 11/12 repeated with one failed grasp) remains at `heldout_v1`;
the corrected replication is not new unseen evaluation. Raw model answers were
not changed. 89 tests pass. Final handoff checks passed: 39 HTML pages / 908 local
links, all 12 layouts' six-panel playback controls and atlas mappings, automatic
source-archive hashes, versioned layout-fixture identity, completed-run resume
without new calls/physics, and credential exclusion across 2,282 source/result
text files. Generated comparison stills were inspected directly; playback checks
used a mocked DOM, not full browser visual certification. The key remains ignored,
untracked and owner-only. All three upstream Git worktrees remain clean.
Machine-readable handoff: `artifacts/pickup/handoff_checks_v1.json`.

114 new logical SDK invocations in this approved package: estimated public-price
token cost $0.067044516; conservative reservations $0.941359104, below $7.50.
Ledger: `artifacts/usage/pickup_research_v1.json`. No further automatic queries
or experiments are needed. Nihar requested publication on 21 October (D17);
the source/test/fixture/documentation checkpoint is being verified for commit
and push. Credentials, environments, caches and generated evidence stay local;
upstreams remain read-only. This completes the software package, not the full
research system or hardware/perception deployment.

Main finding: continual selection fixes coarse initial representations, but the
simple rule is stronger than Jev here on enabled count and scene-union excess
(1.23 L rule vs 2.48 L repeated). Fewer enabled pairs do not shrink the fixed
348-row scene allocation; warm filter timings are about 0.22 ms with all masks.
The current prototype does not establish semantic-model value under oracle
geometry. A useful next research input is per-candidate grasp/lift-reference
clearance; geometry can block a future goal despite looking cheap in the current
phase. Do not silently add a planner or relax contact authority to hide stalls.

Reproducibility additions: future `prepare` saves source snapshots automatically
and rejects mid-preparation source changes. Baseline reuse now requires exact
source/state/settings matches; the earlier explicitly labeled development
replays crossed only an inspected generator-helper edit and remain historical.
`analyze_pickup.py replay --task-protocol archived` loads hash-checked archived
task code with the corrected decoder. It reproduced the decoder-only failure
with an exactly matching trajectory prefix and no API call. The current-task
replay succeeds with the same proposals, but has changed controls/context.

### Implementation history: static-clutter pickup

**The first four pickup work items are complete.** Open
`artifacts/pickup/showcase_v1/index.html`: 12 physical rollouts, three fixed
layouts, off/all-on/Jev-once/Jev-repeated, with synchronized playback. All methods
complete pickup in open_pickup and offset_target. In approach_clutter, off contacts
clutter and fails; all three constrained methods avoid unwanted contact but stall
during descent. Independent checks verified every saved trajectory's hash, finite
states/torques, target free-body trace, length, and successful 0.25 s bilateral
contact/lowest-point lift windows (`verification.json`). All 79 tests pass.

14 new logical Jev invocations returned successfully. Reported input-token cost
estimate is $0.00611037, with $0.115605504 conservatively reserved for possible SDK
retries/full contexts, under the $7.50 cap. The persistent ledger is
`artifacts/usage/pickup_research_v1.json`. Initial answers are shared by once and
repeated; no requery-for-preferred-answer or post-answer prompt/scene edits.
Repeated selection drops the now-irrelevant bottle in open_pickup and reduces
other covers from four to two spheres. Mean enabled pairs: all-on 296; once
122/180/180 and repeated 96.52/133.80/154.02 in open/approach/offset respectively.
Allocated rows remain 348; no solver-speedup or completion improvement is shown.

The difficult nominal grasp posture has a 31.5 mm native link5/bottle penetration
and a -52.4 mm sphere barrier. The stall is not merely excess cover volume; the
original local pose controller does not find another posture/route. Retained
diagnostics are under `artifacts/pickup/development_v3/approach_clutter`. Do not
claim that tighter geometry alone solves it, or silently add/replace a planner.

Now continuing the approved follow-on work. A measured CPU bottleneck was eager
per-step FK: median 3.697 ms versus 0.00946 ms for a warmed JIT position wrapper.
Added that wrapper without changing controller equations/task logic; checking
regressions before new runs. First-showcase physics sources were preserved in
its `source_snapshot/` before edits. Completed-run resume now skips calls/runs
even after later source edits; incomplete runs still require matching frozen
sources. New optional shape-aware sphere candidates (exact cylinder slabs and a
four-cell box grid) are being checked independently before selection studies.

Follow-on update: 83 tests pass after the compiled FK wrapper, shared policy
admission gate, and optional shape-aware candidates. The geometry-only study at
`artifacts/pickup/geometry_study_v1` uses 300,000 shared Monte Carlo points per
case. Four-sphere cylinder excess drops from 2.2144 L to 0.9358 L (about 58%);
cube excess drops from 0.8947 L to 0.6958 L with a four-cell grid. Analytic cell/
slab enclosure arguments and independent boundary tests establish coverage in
exact arithmetic; zero sampled misses are a separate diagnostic, not a proof.

Added an explicit non-model `rule` baseline: constrain objects if any cover
intersects the sampled phase forecast; choose least estimated global excess
volume, then count/ID. Uses the same admission gate, no fabricated Jev answer.
It completes the two easy development layouts and safely stalls in the original
hard layout. Its rule is inspectable, not a certified fallback or tuned optimum.

Preparing two frozen development comparisons on the original three layouts with
the expanded geometry family: `development_shape_prompt1_v3` and
`development_shape_prompt2_v2`. The new JSON template is
`prompts/pickup_selection_v2.json`; no new calls beyond the first 14 yet. Identical
non-model baselines may be reused across prompt variants only after exact state,
settings, physics/feature-source and trajectory-hash checks; copies are explicitly
labeled as replay, not fresh independent trials.

Separately generating 12 new static layouts offline at `layout_generation_v3`
for later held-out evaluation, without consulting model answers. The generator
looks for nominal-path obstruction/near misses while preserving clear initial
and grasp/lift configurations, retaining its search metadata. Drafts v1/v2 had
an invalid static-world probe pose/cache assumption; both are retained with
`evaluation_correction.json` and had no model calls. Version 3 uses fixed-size
evaluation-only mocap probes (excluded from robot geometry) and asserts actual
probe world poses; these are offline placements, not moving benchmark obstacles.

The 12-layout original held-out batch is now complete at `heldout_v1`: 72 runs
across off/all/rule/all-with-Jev-geometry/once/repeated. Safe completions are
11/12, 12/12, 12/12, 7/12, 7/12, 11/12 respectively; all sampled modeled barriers
remain nonnegative. Repeated querying resolves four one-shot stalls, but all-on
and the simple rule are stronger completion baselines here. No superiority claim.
The first prompt was selected before held-out model answers: the revised prompt
did not improve development outcomes and used more enabled pairs. Protocol is
`heldout_protocol_v1.json`; raw data and independent audits are retained.

Diagnosis of layout_02 repeated failure: it is NOT a clutter collision. The
open gripper was deflected laterally while descending, disturbed the target,
then closed/lifted without a real grasp. The fallen cylinder eventually rolled
into fixed link0 at 17.376 s. Two earlier updates were rejected solely because
rounded probabilities summed to 0.99 (including an unused box-geometry answer).
The installed provider SDK's Choice schema explicitly allows approximately-one
mass. Corrected our decoder to allow K*0.005 rounding error for K hundredth-
rounded entries, retaining raw values/IDs and all range/key/argmax checks.
`decoder_correction_replay_v1` uses original answers/times without new calls;
its trajectory prefix matches exactly, but it still fails: decoder correction
alone does not fix the grasp skill.

Added a small high-level task correction, not new FRAX arm-control math: align
above the target before descent; require settled pose/rotation before closing;
back off above it if the descent is deflected sideways; do not lift without
bilateral contact. Provisional editable defaults: 4 cm pregrasp clearance,
5 mm alignment, 0.08 rad rotation tolerance, 0.02 m/s speed. These are simulated
skill settings, not hardware safety limits or architecture requirements.
`alignment_correction_replay_v1` succeeds safely using the same recorded
proposals (5.146 s); controls now differ before old query times, so this is a
diagnostic replay, not fresh Jev performance. Also clarified request wording:
shape-aware cylinder covers enclose the full cylinder, not unused box corners.

Preparing `corrected_replication_v2` on the SAME 12 frozen layouts and selected
original template, with the corrections above and all six methods rerun. This
is a code/skill-corrected replication, NOT a new unseen benchmark or retry-until-
success. Original outcomes remain untouched. Freeze/reason record is
`corrected_replication_protocol_v2.json`. At this point the session has 78 new
logical requests, estimated token cost $0.044869188 and conservative reservations
$0.644087808 under the $7.50 cap. No new replication model calls yet.

Implementation in progress: `pickup.py`, `run_pickup.py`, `pickup_gallery.py`, and
`api_budget.py`. New `frax_gripper` profile retains native finger actuation and
splits their inertias out of the FRAX aggregate at 2 cm reference opening: native
arm-block mass/bias match to about 1e-11 there. Finger motion, grasp contact and
payload are not modeled by the seven-DOF CBF dynamics; no contact-safety proof.
The original FRAX nominal torque controller is still executed, not copied.

Offline native free-body grasp/lift succeeds and stops at the configured
verification window; no weld/attachment/teleport. Starting parameters in
`pickup.DEFAULTS` are editable provisional choices: 2 ms step, 25 cm approach,
9.5 cm grasp height for the 12 cm target, 8 cm commanded verification lift,
5 cm lowest-point success clearance held with bilateral contact for 0.25 s,
20 s timeout. They are not settled research/hardware requirements.

Found the original closed-tool spheres miss open-finger vertices by up to 24.9 mm.
Added eight sphere envelopes derived from native finger collision bounds across
their full joint travel; independent aperture/pose regression checks pass. All
original protections remain, with added envelope-floor barriers. Clutter pairs
with 29 spheres; the designated target protects the 16 upstream arm spheres.
Only intended tool-target CBF pairs are exempt, and the independent native audit
still rejects palm/arm-target and all robot/target-clutter contact.

`artifacts/pickup/development_v3` has six offline baseline runs: off/all-on both
pick up successfully in open_pickup and offset_target. In approach_clutter, off
contacts clutter and fails pickup; all-on avoids contact and keeps modeled
barriers positive but stalls during descent. This failure is retained, not tuned
away. Four pickup tests and three budget tests pass, in addition to the previous
71-test regression pass before the envelope extension. This was the pre-live
development checkpoint; the completed frozen showcase and calls are above.

Nihar clarified the direction on 21 October (D14): one task family only. Maneuver
the arm/gripper through static clutter, pick up the designated target, and end
when pickup is verified. Different clutter layouts and target positions are
variations, not different tasks. No placement, transport, pushing, or independently
moving obstacles. Target motion during pickup is expected, not obstacle motion.

The immediate implementation gap is genuine simulated grasping/pickup: the
current matched plant has rigid fingers and the showcase demonstrates reaching,
not object acquisition. Reuse the controller, geometry selection, CBF integration,
and viewing infrastructure; distinguish intended gripper-target contact from
unwanted collisions. Do not preselect arbitrary grasp-success thresholds or a
full future research protocol here. Nihar subsequently approved the four pickup
work items and then the seven proposed follow-on items (D15): genuine-success
checks, additional static layouts, failure diagnosis, geometry-selection study,
development/held-out prompting comparisons, measured profiling, and rerunnable
viewing/reporting. Their order follows evidence, not a rigid milestone sequence.
Implementation has started with inspection of the existing gripper/model/controller
interfaces. Nihar authorized a hard $7.50 cap on new Jev API usage for this work,
using the existing key (D16). It replaces proposed request-count caps. Current
official pricing at https://docs.typesafe.ai/models (checked 21 October) is
$0.042 / million input tokens, with output free; default context is 64k/request.
Reserve before requests and record actual usage, including uncertain failed
attempts, rather than treating a successful logical call as the only possible
charge. No new model calls have been made. Push is not approved.

### Completed showcase

**Complete: autonomous diagnosis through the initial dynamic simulation showcase.**
Open `artifacts/showcase/dynamic_v1/index.html`; start with the handoff fixture and
scrub 4–8 seconds. Four synchronized panels show off/all/once/repeated with physical
objects, enabled covers, current clearances, goal error, and constraint counts.

| Dynamic fixture | Off | All-on | Jev once | Jev repeated |
| --- | --- | --- | --- | --- |
| Distant object approaches/leaves | Contact | Pass | Pass | Pass |
| Obstacle handoff | Contact | Pass | Contact | Pass |

Eight rollouts completed 6,000 steps / 12 simulated seconds each, with no numerical
warnings. All-on and repeated have no sampled native scene contact and reach both
final goals. Repeated adds object_b at 4.170 s in the handoff, before its approach
causes contact in the one-shot run. Both modes share the exact initial answer;
once still tracks current poses/velocities, freezing only role/geometry choices.
Incoming once/repeated both protect object_a from the start, so that fixture
does not demonstrate a benefit from repolling. All constrained modeled barriers
remain positive (minimum about 0.478 mm), including the one-shot run that contacts
an omitted object: this shows why native evaluation of every object is necessary.

Jev's mean enabled pairs are 84 versus 168 all-on in incoming and 138.81 versus
252 in handoff. The fixed-capacity QP keeps all allocated rows: this is not a
demonstrated QP dimension/runtime reduction. Jev never removes an already selected
object later in these runs; runtime removal is supported/tested, but effective
dropping remains an open research limitation. These two frozen fixtures are not
a statistical benchmark, general safety result, or real-time/hardware guarantee.

All 12 planned new logical SDK invocations succeeded, returning jev-1.13.0:
100,310 input / 2,736 output tokens, invocation times 0.128–0.357 s. The ten later
responses were applied approximately 0.138–0.170 simulated seconds after dispatch
in wall-paced rollouts. Initial answers are shared; ordinary SDK retries may
apply, not twelve necessarily unique HTTP attempts. No model/prompt/scene tuning
after answers and no additional calls to obtain preferred decisions. Raw queries,
responses/probabilities, events, trajectories, and source/input hashes are saved.

Verification: 71 tests pass; native mass/bias/acceleration parity, original
protection parity, obstacle-velocity derivatives, runtime-array updates, admissible
installation/failure retention, collision positive controls, one-shot ledgers,
and audit-bound reports are covered. Synchronized scrub/pause/restart/play controls
passed a Node mocked-DOM check; generated baseline/final/7-second comparison
frames were inspected directly. No browser screenshot/safety certification claim.
Upstreams stay read-only; all new code remains local/uncommitted/unpushed.
Final audit also verified identical mode starts/object paths, every query's actual
recorded q/v snapshot, unchanged frozen inputs/core physics sources, all gallery
links, credential exclusion/owner-only permissions, and no new calls on a completed
workflow resume. Evidence is in `verification.json` and `test_output.txt` alongside
the showcase. All three upstream Git worktrees remain clean; `git diff --check`
passes locally.
Further research follows the next user direction; the showcase task needs no
additional model calls or background experiment.

### Implementation decisions and diagnosis

Nihar explicitly asked to diagnose and solve problems independently and continue
through the final showcase without intermediate human input. The scope included
local simulation/model integration corrections, continual constraint updates,
bounded necessary live Jev calls, tests, comparisons, and a final gallery. Preserve
the original nominal policy and robot protections, keep upstream repos read-only,
retain failures, and document consequential choices. No hardware/perception or
commit/push. The older six-call/static stop points below are historical. The
diagnosis/correction preceded construction of the online loop.

Diagnosis/correction evidence (21 October): the original XML has armature 0.1,
damping 1, and a different fixed-tool inertia from the FRAX URDF. Added an explicit
`frax` simulation profile in `sim_model.py`: transfer FRAX inertial parameters
into an in-memory native spec, use a rigid closed gripper, and remove passive
damping/rotor inertia absent from that model. Keep original collision/visual
geometry and the unchanged `upstream` profile. This is an idealized matched plant,
not calibration or a fix in upstream source. Independent native mass/bias/
acceleration checks agree to <1e-8 across sampled states (acceleration ~2e-10).

Matching dynamics reduces the hand-path crossing from 2.759 mm to 0.0214 mm.
Halving the hold period reduces it to 0.00854 mm, with near-zero QP inequality
residuals. A local CBFpy subclass tightens only scene acceleration inequalities
by 0.05 m/s²; geometry and both alphas remain unchanged. This empirical
sampled-control guard is not calibrated perception uncertainty or a formal
robustness bound. All 12 constrained rollouts of the saved six-scene selections
now have nonnegative sampled original h2; four still miss the goal. Their selected
covers overlap the nominal goal configuration, so better enforcement cannot by
itself fix completion. Original data are retained. New correction audit:
`artifacts/diagnosis/static_corrected_v1`; paired diagnostics under
`artifacts/diagnosis/matched_v1`. Reused answers are a correction replay, not fresh
model performance for the changed plant.

Implemented `runtime_cbf.py` and `online.py`: reuse FRAX barrier/objective/dynamics
methods and CBFpy via runtime array arguments, explicit time state/obstacle
velocities, fixed-capacity masked slots, background requests, saved attempts,
current-pose rebasing, and h2/h1 admissibility checks before installing answers.
The original nested FRAX nominal torque computation remains unchanged; its
filter moves outside the nominal JIT to avoid stale captured Python attributes.
Errors/review/rejected updates retain the previous policy, initially all-on;
this is explicit behavior, not a certified fallback. Fewer enabled rows do not
reduce the allocated QP shape in this first implementation.

Prepared two frozen 12-second dynamic fixtures in
`artifacts/showcase/dynamic_v1`, with editable source scenes/questions under
`examples/dynamic_scenes.json` and `prompts/dynamic_selection.json`. Future motion
scripts/titles are not passed to Jev. Current poses/velocities and a clearly
labeled heuristic whole-arm joint-interpolation forecast are supplied. Four
offline off/all baselines completed: off contacts in both, all-on no sampled
contact and reaches both goals. A new mocap-body diagnostic bug initially counted
scene bodies as robot geometry; fixed exclusion and added a positive-control
regression. Original summaries/NPZs are retained; hash-bound collision re-audits
provide corrected data independently. Six online-loop tests and three dynamics/
guard tests passed before the live calls. The planned 12-call execution and
synchronized playback are now complete, with outcomes above. No upstream changes.

### Completed robot-aware static checkpoint

**Complete, including user visual review: robot-aware static suite and reusable workflow.** Nihar
approved the extended checkpoint: approximately six scene variations, actual Panda
state/intended-motion context, one-shot Jev selection, off/all/Jev comparisons,
independent diagnostics, and a combined gallery. Six new SDK invocations are in
scope, one per scene, without repeated prompting for preferred answers. Freeze
scene/question/request snapshots before looking at model answers. No perception,
online updates, contact tasks, hardware, controller replacement, or upstream edits.

Infrastructure first: extend the reusable scene/request/runner interfaces; the
suite is their first evaluation consumer, not six throwaway demo scripts. Keep
the original controller/protections. A free-space nominal rollout (scene contact
forces disabled only for that prediction) supplies measured whole-arm motion
features; evaluated off/all/Jev runs keep physical object collisions enabled.
The all-on reference will use explicit fixed four-sphere covers, independent of
Jev's answers. This tests the combined role/geometry choice, not an isolated causal
effect of role selection. Record failures and coverage/conservatism costs; do not
silently repair model decisions. All of these are inspectable starting choices,
not a frozen future architecture.

Implemented the shared motion/validation interface, `robot_selection.py` feature
builder, `run_suite.py` phased workflow, editable `examples/robot_scenes.json`, and
`prompts/robot_selection.json`. Baselines use no fabricated model responses;
`jev` mode requires a real robot-context report, while old proxy data stays in
explicit replay mode. 58 offline tests pass, including frozen-input/one-shot
guards and no oracle label leakage. No additional dependencies.

Pre-model qualification: draft `artifacts/suite/panda_static_v1` failed native
endpoint-clearance checks in two layouts. Retained it unqueried, including a
pre-model geometric layout search. Moved the tall object and two-sided passage
before any model answers; final `artifacts/suite/panda_static_v2` has valid initial
geometry, a native-collision-free nominal endpoint, and initially admissible
fixed-four covers in all six scenes. Snapshots/requests/questions are hashed and
frozen. These checks do not prove every path/representation is feasible.

Completed all 12 physical baseline rollouts (3,000 steps each) before live calls:
off has contacts in hand_path, arm_clutter, and distractors; fixed-four all-on has
no sampled scene collisions, but misses the 2 cm goal criterion in those three
and tight_passage. Those failures are retained, not retuned away. This baseline
phase finished before any of the six new model calls.

All six new SDK invocations have now succeeded, returning `jev-1.13.0`. Each
response and all role/geometry probabilities are saved under the frozen v2 scene
directory. All six attempts are recorded successful in `attempts.json`; no further
model calls remain in this checkpoint's budget. No prompt, scene, gain, or model
changes after seeing answers. All 18 physical rollouts and the combined gallery
are complete. Jev omitted distant clutter, chose four spheres
for the collision-relevant objects, and chose one sphere for the clear-path cube.
Shape-tradeoff object's compact-neighbor omission had only P=0.51; record/show that
ambiguity without replacing its answer or querying again.

Open `artifacts/suite/panda_static_v2/index.html` for the six-scene review. This is
six hand-designed variations of one no-contact reaching task, not six independent
tasks or a statistical benchmark. Each mode ran 3,000 steps / six simulated seconds.

| Scene | Off contact flag | All-on pairs / final error | Jev pairs / final error |
| --- | --- | --- | --- |
| Clear path | No | 252 / 1.4 mm | 105 / 1.4 mm |
| Hand path | Yes | 168 / 32.0 mm | 84 / 32.0 mm |
| Tall arm clutter | Yes | 168 / 36.2 mm | 84 / 36.2 mm |
| Distractors | Yes | 336 / 32.0 mm | 84 / 32.0 mm |
| Tight passage | No | 252 / 22.9 mm | 168 / 22.9 mm |
| Shape tradeoff | No | 252 / 1.4 mm | 84 / 1.4 mm |

All-on and Jev have no sampled native scene collision/contact flags. Jev uses
33–75% fewer scene pairs per case than fixed-four all-on, but **does not improve
goal completion**. Both have negative modeled scene `h_2` in the hand-path,
tall-clutter, distractor, and tight-passage cases: approximately -2.759, -1.079,
-2.759, and -0.427 mm respectively. These cases also miss the unchanged 20 mm
final-error criterion. Complete + within 20 mm + no sampled scene violation + no
negative `h_2` gives off 3/6, all 2/6, Jev 2/6. No contact is not proof that the
modeled CBF safe set held. QP inequality excess is about 1e-14 and requested
torque-limit excess is zero; do not attribute the crossings to solver relaxation
as an established cause.

A read-only same-state acceleration probe found FRAX `f + g*u` and MuJoCo
`mj_forward` disagree for the hand-path Jev trajectory. Maximum joint acceleration
differences were 40.73 rad/s² initially and 2.76 rad/s² at the worst scene barrier
sample. The original simulator has joint damping 1 and armature 0.1. This observes
a dynamics mismatch; it does not isolate a unique cause of barrier crossings.
Saved `model_diagnostics.json` records the probe. No controller, physics, gain,
solver, or upstream change was made to hide the failure. Discuss the appropriate
next diagnostic/correction with Nihar after visual review.

Final verification: 58 offline tests pass, including real-upstream protection/frame
parity, physical collision positive controls, frozen-request/one-shot guards,
and display-label preservation. All frozen hashes and request/runtime contexts
match; every gallery image/data link exists. All 18 rollouts completed without
numerical warnings. The six calls returned 100,474 input / 1,548 output tokens in
total. SDK invocation times ranged 0.186–0.582 s; these single ordered observations
are not an inference-latency or real-time benchmark. Generated frame images were
inspected directly, including object-ID labels. Nihar accepted the visual-review
checkpoint, saying "yes it looks good" after the guided OFF/ALL/JEV comparison.
This is visual acceptance, not a safety certification or evidence that the known
barrier/dynamics issues are resolved. Agree on the next diagnostic task before
changing code/controller behavior or starting further experiments.
The three upstream worktrees remain clean. Code is local/uncommitted; last
published checkpoint remains `f432eef`. No further model calls are authorized.

### Completed first static replay checkpoint

Nihar approved continuing through the static, no-contact simulation comparison:
connect saved covers to the FRAX/CBFpy controller, test the integration, compare
scene-object constraints off/all-on/selection-replay, and produce diagnostics and
a rerunnable gallery before the next human review. No additional model calls,
online updates, contact exemptions, perception, or hardware. Upstreams read-only.

**Static comparison checkpoint complete, including Nihar's animation review.**
Nihar reported "all the animations look good and normal." This confirms the user
visual check, not a safety advantage or approval for additional experiments.
Open `artifacts/static/review/index.html`. All three modes completed 3,000 steps
(6 simulated seconds), reached approximately 1.4 mm final EE error, and had no
sampled object penetration or negative modeled barriers. All-on and replay
trajectories matched to floating-point precision; off also succeeded. This is an
integration result, **not a demonstrated safety/completion advantage for Jev**.
No new model calls, prompts/gains/scene tuning, dependencies, or upstream edits.
**Verification: all 51 tests pass**, including real-upstream protection/frame
parity and independent collision positive controls. A bounded 100-step live
`--view` run also completed and closed normally; its short duration is not a
goal-completion test. The gallery has 61 replay frames per mode plus final stills.
Checked that every gallery image/data link exists, regenerated it with the
corrected collision audit, and opened `artifacts/static/review/index.html` using
the normal desktop launcher for Nihar. Nihar has now reviewed all animations;
the agent inspected the generated final-frame images directly. No extra
experiment is running. Agree on the next research step before implementation.

The original demo's nominal controller is nested inside `main()`; it is reused
through scoped module bindings rather than copying its equations. Its existing config already
accepts arbitrary obstacle sphere arrays, including an empty array for scene-off.
Original joint/singularity/floor/self-collision and torque protections stay active.
`run_static.py` supplies arrays and in-memory scene additions, records physics and
diagnostics, and restores module bindings. `static_gallery.py` renders actual
recorded states into looping APNGs and a local HTML report. Renderer frame images
were inspected directly; this uses generated simulation data, not denied app
screenshots. No screen-access bypass was needed.

| Review run | Scene spheres / pair constraints | Final EE error | Minimum object distance | Median / p95 controller host time |
| --- | --- | --- | --- | --- |
| Scene off | 0 / 0 | 1.403 mm | 57.0 mm | 0.141 / 0.163 ms |
| All conditional covers | 10 / 210 | 1.402 mm | 63.1 mm | 0.193 / 0.242 ms |
| Saved selection replay | 8 / 168 | 1.402 mm | 63.1 mm | 0.169 / 0.195 ms |

Total barriers are 43/253/211 respectively; 43 existing robot protections remain
in every mode. All-on adds the omitted far cube's saved two-sphere conditional
cover, keeping selected-object geometry equal across all/replay. The 20% reduction
is in included scene pairs, not necessarily active constraints: near-binding pair
counts had median 0 and maximum 1 in both constrained runs (absolute residual
within solver_tol=1e-5; descriptive, not the solver's active set). This is one easy
scene, not a statistical safety-rate or speedup benchmark; timing includes desired
state handling and synchronized torque retrieval, excludes the first control
step, and is not pure solver time or full-loop real-time certification.

All 23 MuJoCo robot collision geoms, including base/gripper, were checked against
all three exact scene primitives at 3,001 recorded states per mode using separate
MjData. No contact samples were found. Distances use MuJoCo collision shapes/hulls,
not visual-mesh truth, and do not establish continuous-time safety. Maximum CBF
inequality excess was approximately 1.1e-14 (all) / 8.9e-15 (replay), and requested
torque-limit excess was zero; the inherited elastic solver still has no blanket
hard-safety guarantee. Minimum modeled floor and self-collision barriers were
positive. The far cube's minimum measured robot clearance was 376.7 mm. Off/replay
EE paths differed by at most 8.23 mm; all/replay joint trajectories differed by
at most 4.44e-16 rad in the first full comparison.

The final positive-control test initially failed: a deliberate box/robot-mesh
overlap returned exactly zero from `mj_geomDistance`, which the first evaluator
had not flagged as penetration. Corrected the evaluator to flag all nonpositive
distances and contacts, supplementing distance queries with negative contact
depths. Both fixed-base overlap/zero-distance and recompiled moving-link overlap
controls now pass. Reanalyzed all 3,001 saved states per mode: original distances
were already strictly positive and are exactly unchanged, with zero contact or
nonpositive-distance flags. Reanalysis is saved separately in
`artifacts/static/review/collision_recheck.json`; original trajectories/timing
reports remain intact. The gallery incorporates this recheck. This avoids
silently claiming the initial strict-negative test alone certified clearance.

Initial states match exactly across modes. An additional probe at four recorded
states found protected barriers exactly equal to the original upstream config and
joint-origin frame discrepancies below 4e-12 m. This is protection/frame parity,
not a full dynamics equivalence proof. A regression checks parity/frame
consistency and deliberately places an omitted object into robot geometry to
verify the independent checker detects it. Earlier full runs under `initial/`
and `verified/` are retained; the final `review/` repeat has exactly equal qpos/qvel
to `verified/` after a logging-only initialization-timer correction. Timing-only
repeats are not independent experimental trials or selected best runs.

New code and documentation are local/uncommitted. The last published checkpoint
is still `f432eef`; do not push this new work without a request.

The saved proxy starts at [0.10, 0, 0.06] m; default Panda EE is approximately
[0.3749033, 0, 0.4093534] m. A local fixed-orientation IK attempt for the proxy start
stopped 7.4 mm away and had a -91.5 mm self-collision barrier. This is not a proof
that no valid IK solution exists, but does not justify initializing there. Keep
the valid upstream default joints, recorded objects, and recorded goal; explicitly
label reused answers as proxy-decision replay with changed start/body context.
Do not claim new whole-arm Jev decisions. Independent collision checks include
all objects, not just selected ones. The recorded goal has a local IK solution
with 2.3 micrometre position error and positive sampled floor/self barriers; this
does not certify a collision-free path to it.

### Completed saved-result viewer checkpoint

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
whole-arm safety check, or real-time claim. No robot control ran in that standalone
request; the later static replay is a separate experiment described above.

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
share one request but are independent, as required by the API. Standalone outputs
remain inspectable proposals; their later use is explicitly labeled simulation
replay. Templates, counts, task text, and scene facts are easy to
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

## Historical execution workflow and permission notes

The current completed handoff is the static-clutter pickup review at the top of
this document. This section retains the earlier setup/reaching workflow, not a
request to restart it or an active gate on already approved work.

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
Nihar then approved the static integration/comparison bundle and has now reviewed
all animations at `artifacts/static/review/index.html`, reporting that they look
good and normal. The subsequently approved robot-aware extended checkpoint is
also implemented and its visual-review checkpoint has been accepted at
`artifacts/suite/panda_static_v2/index.html`. Nihar subsequently authorized
autonomous diagnosis/fixes/continual updates through the final showcase. That work
is complete at `artifacts/showcase/dynamic_v1/index.html`; open the handoff fixture
first and scrub 4–8 seconds. All 12 planned new calls are spent; no extra experiment
is running. Further work follows the next user direction, not an automatic
extension into perception/hardware or retuned prompts. Credentials load via uv's
`--env-file .env.local`; never print the key or search unrelated stores. A model's
type-correct answer does not establish its obstacle-selection correctness/safety.

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
| D11 | Continue through a tested static, no-contact Panda integration and three-mode comparison with diagnostics/gallery, then human review. | Nihar said "ok start working and go until then" and "keep going." Preserve robot protections; no extra Jev requests, online updates, contact tasks, perception, or hardware. Surface consequential mismatches, including proxy-to-arm context changes. |
| D12 | Robot-aware static mini-suite through combined review gallery, with about six scenes and six one-shot Jev invocations. | Nihar said "ok start working and get to that checkpoint." Fixed scenes before model answers, actual robot context, off/all/Jev runs, reusable infrastructure and inspectable failures. No online querying, contact tasks, hardware, or controller replacement. |
| D13 | Continue autonomously through diagnosis, fixes, continual updates, and the final simulation showcase, without intermediate human input. | Latest explicit user direction superseded static-review stops; local changes and necessary bounded live calls were in scope, not hardware/upstream edits/publication. Implemented with two dynamic fixtures and 12 planned calls, all retained. |
| D14 | One task family: maneuver through static clutter, pick up a designated target, then stop. | User clarified that surrounding obstacles remain static and there should be only this task type. Layouts/target positions may vary; no placing, transport, pushing, or moving-obstacle experiments. Genuine simulated pickup remains to be implemented. |
| D15 | Implement the four pickup work items, then continue with the seven evidence-driven follow-on items. | User said "yes good do these things after you get the first 4 work items that you previously mentioned done." Build functional pickup, integrate contact-aware Jev/CBFs, compare methods, and render the gallery, then check reliability/layouts/failures/geometry/prompts/cost/workflow. Additional live-call cap is pending; no push/hardware/upstream edits. |
| D16 | Up to $7.50 total new Jev API spend during this work, using the supplied key. | User explicitly authorized spending and as many calls as needed within the dollar cap. Supersedes request-count proposals and D15's pending-budget note. Track usage and do not exceed the cap. No other paid provider is authorized. |
| D17 | Commit and push the completed Jev source checkpoint to the existing GitHub repository. | User said "push this to repo" on 21 October. Publish source, tests, editable fixtures/templates and docs; keep credentials, environments, caches and generated experiment/gallery artifacts local/ignored. No new experiment or upstream edits. |

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

Learned body/contact selection beyond the current task-defined masks; Jev-selected
calibrated margins; additional primitives and broader mesh/perception coverage
evaluation; candidate grasp/lift-reference clearance features; broader
once/repeated Jev and thinking-model comparisons; reliable conditional constraint
dropping beyond the current demonstrations; compact QP dimensions rather than
masked slots; adaptive gains; optional ponytail tooling/style. Static pickup
selection/contact handling, six-method comparisons and synchronized viewing are
already implemented, not pending infrastructure. The simple geometric rule is an
important baseline for deciding which future semantic inputs add real value.
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

- 20 October 2026: published implementation snapshot `3d6fd4e` to
  `https://github.com/rrnM12/jev-cbf-planning` on `main`. Push succeeded;
  `HEAD` and `origin/main` matched, with a clean working tree. Staged-content
  checks found no configured API key, private-key headers, or GitHub-token
  patterns. `.env.local`, caches/environments, and generated reports were not
  committed. The three upstream repositories were not changed.

- 20 October 2026: Nihar approved continuing through the static comparison and
  review gallery. Inspected upstream APIs and found proxy/robot context mismatch;
  reported it and retained the valid default Panda state rather than initialize
  from an unvalidated IK fit. Added simulation-only adapters, independent
  collision/QP diagnostics, tests, and APNG/HTML rendering with no new packages.
  A 10-step smoke run and three full six-second comparisons completed. Fixed an
  initialization-timing logging issue; repeat trajectories match exactly. The
  scene is easy: all modes succeed, so no safety benefit is established. No
  additional service calls, prompt/gain/scene tuning, upstream changes, commits,
  or push. Ready for the agreed human review, not the full online experiment.
- 20 October 2026: final real-mesh positive control exposed the distance-query
  zero-overlap issue. Added contact-depth/nonpositive-distance handling and
  rechecked existing states without changing rollout results. All 51 tests now
  pass, including fixed/moving-link overlap controls and exact protection parity;
  100-step native viewer smoke exited 0. Review images inspected from generated
  simulation data. Human review remains pending; no further features started.
- 20 October 2026: Nihar reviewed all three animations and said they look good and
  normal. Marked the static comparison checkpoint complete, including user visual
  review. Similar behavior is expected in this easy scene; this does not establish
  a safety benefit or approve new scene/model experiments, API calls, or a push.
- 20 October 2026: Nihar approved the robot-aware extended checkpoint. Built shared
  actual-Panda request features and a phased reusable suite runner. Qualified six
  layouts before answers, retained the rejected unqueried draft, froze inputs,
  completed 12 physical baselines, made exactly six new SDK invocations, and ran
  six saved-selection rollouts. Added a combined labeled APNG/HTML gallery and
  seven tests (58 total). Jev reduces pair counts and has no sampled native
  contacts, but matches all-on's two successful cases and four negative scene
  barriers. Recorded a same-state dynamics mismatch without changing the original
  controller/simulator. Ready for human review; no additional calls or features.
- 20 October 2026: Nihar accepted the robot-aware gallery's visual-review
  checkpoint with "yes it looks good" after a guided comparison. Recorded visual
  acceptance only; the negative modeled barriers, stopped-short runs, and dynamics
  mismatch remain unresolved. No further implementation, experiment, API call,
  commit, or push authorized by this review.
- 21 October 2026: Nihar explicitly authorized autonomous work through the final
  showcase, including diagnosis/fixes and continual selection. Isolated the XML/
  URDF dynamics mismatch and sampled-hold error; added an explicit idealized plant
  and a scene inequality guard without upstream edits. Replayed all six static
  scenes (18 runs), retaining blocked-goal failures; all 12 constrained replays
  have nonnegative modeled barriers. Implemented runtime parameterization,
  obstacle-velocity RD2 terms, background selection, admissible updates, and two
  frozen dynamic fixtures. Four baselines plus four live-selected rollouts
  completed; all 12 planned SDK invocations succeeded. Repeated selection avoids
  the approaching-object contact seen in one-shot handoff, but does not drop
  previous obstacles later. Added hash-bound collision re-audits for the mocap
  exclusion fix, synchronized playback, and tests (71 passing). Retained original
  data and every answer/failure. Final showcase ready; no commit/push or hardware.

- 21 October 2026: implemented the approved static-clutter pickup package and
  follow-on study. Native articulated fingers grasp/lift a free target; original
  FRAX nominal torque math and all upstream worktrees remain unchanged. Added
  contact-aware masks, independent success/contact audits, shape-aware covers,
  deterministic and geometry-only baselines, editable prompt comparisons,
  profiling, replay, source snapshots and synchronized six-method playback.
  Retained the original difficult posture, invalid offline generator drafts,
  original held-out outcomes and failed decoder-only replay. Corrected rounded-
  probability decoding and the high-level grasp sequence, then reran all six
  methods on the same 12 layouts, explicitly not a new unseen benchmark.
  Repeated/all/rule achieve 12/12 safe pickups; once/geometry 6/12; off 11/12.
  The geometric rule uses tighter covers/fewer enabled pairs than repeated Jev;
  fixed allocated QP dimensions do not shrink. All 72 corrected native audits,
  89 offline tests, 908 local links, 12-layout playback checks, credential checks,
  source archives and no-new-call completed resumes pass. Total new logical
  requests 114, estimated token cost $0.067044516, conservative reservations
  $0.941359104 against $7.50. No additional experiment is running or needed for
  the completed package. No hardware/perception, commit or push.

- 21 October 2026: Nihar requested committing/pushing the completed work to the
  existing Jev repository (D17). Re-ran all 89 offline tests successfully in
  28.474 s, with no live API calls. Fetched origin; local/remote main initially
  match at `f432eef`. Preparing the source/test/fixture/documentation publication;
  credentials, environments, caches and generated evidence remain local/ignored.

The completed viewer checkpoint is published. Subsequent static/dynamic reaching
infrastructure and the current pickup/follow-on package are complete, and Nihar
has requested their publication. The last verified published checkpoint is
`f432eef` until the new push is confirmed. The current local review entry is
`artifacts/pickup/review/index.html`; generated gallery files are not in Git.
The deleted stale parent architecture copy remains recoverable from the
repository's original commit `b347afe`.
