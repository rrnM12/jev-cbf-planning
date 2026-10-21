# Jev-selected CBFs: evolving architecture and working plan

Status: a living research plan, not a frozen implementation specification.
Updated 21 October 2026. The four-item pickup gallery and original follow-on
software study are implemented. The further approved extension is complete locally:
richer sphere families, selection diagnosis, harder unseen layouts, simulated
scene/wrist depth-derived clutter geometry and controlled observation/model
failures. Start at `artifacts/pickup/review/index.html`; PROGRESS.md has live status.
The original FRAX nominal torque equations/gains remain in use, CBFpy performs
filtering, and all three upstream repositories remain read-only.

New camera boundary (D20, software only): `camera_io.py` supplies typed metric
frames, optical intrinsics/projection, explicit capture-time calibration and
hash-checked recording/replay. `realsense_camera.py` is an optional lazy SDK
adapter, mock-tested but never connected to a device or installed on this host.
`FrameGeometry` accepts a separate mask/identity provider and reuses the existing
fitter/candidate generator. Missing masks/calibration do not authorize omission.
`CameraWorker` prepares observations off the consumer thread; the optional
`PerceivedPickupRuntime` external-source path consumes finished geometry through
the existing admission gate. Native GL/live MjData is not shared across threads.
The original synchronous simulation path stays available for old behavior.
Robot pose/clock mapping, physical hand–eye calibration, SDK host compatibility,
hardware synchronization and learned segmentation still need later validation.
See README's camera section for commands/contracts and PROGRESS for evidence.

The prior corrected replication has 72 runs on the same 12 static layouts used
in the original held-out study. Safe pickups: off 11/12; all-on 12/12; geometric
rule 12/12; all-objects/Jev-geometry 6/12; Jev once 6/12; Jev repeated 12/12.
Repeated querying resolves coarse-cover stalls, but does not outperform the
simple rule on this oracle-geometry set. Mean enabled pairs are 156.94 repeated
versus 296 all-on and 151.71 rule; allocated scene pairs remain 348. Native
re-evaluation checks every object and all 23 robot collision geoms. The original
results/failures are preserved separately; this replication is not new unseen
evidence. 89 offline tests pass. Source/documentation checkpoint `dc566c7` was
published on 21 October at Nihar's request; generated evidence stays local.
No hardware deployment.

Current adapters include native articulated fingers, a free-body pickup target,
explicit task-contact masks, runtime sphere parameters, asynchronous recorded
Jev proposals, admission checks, independent collision/lift diagnostics, and
synchronized playback. A pregrasp alignment/settling waypoint, lateral backoff,
and bilateral-contact gate improve the task sequence without replacing the arm
controller. Numeric settings in `pickup.DEFAULTS` are provisional, not research
or hardware requirements. Shape-aware cylinder slabs and a box grid supplement
the original box-split sphere candidates; all use conservative cell enclosures.

`frax_gripper` splits native finger inertia out of the FRAX aggregate at a 2 cm
reference opening. The seven-DOF CBF dynamics omit finger/contact/payload forces;
the 0.05 m/s² scene-inequality guard is empirical, not calibrated uncertainty.
Target masks protect 16 arm spheres while permitting tool overlap; native
evaluation still rejects non-finger contact. Finger envelopes assume nominal
0–4 cm travel, and native soft-limit overshoot is reported. No learned detection,
learned policy, global planner, learned body pairing, frontier-model comparison,
formal contact/continuous-time safety, or real-time certification is claimed.

The prior completed package used 114 new logical Jev requests, estimated token cost
$0.067044516, with $0.941359104 conservatively reserved against the $7.50 cap.
PROGRESS.md records decisions, attempts, failed drafts, corrections and checks.
The larger research system/dream desiderata are not finished by this software
proof of concept; future choices below remain replaceable proposals.

Current extension boundaries: small/rich families use 1/2/4 versus
1/2/3/4/5/6/8 primitives, including shape-aware cylinder slabs and useful box
grids. More primitives need not improve pointwise clearance or runtime. Optional
terminal-reference features describe cover obstruction, not a collision-free plan.
The common prompt and orchestration remain experiment inputs, not a settled
research taxonomy. The geometric rule and all-object/least-excess baseline use
the same candidate family; findings must not assume semantic-model superiority.

`perception.py` backprojects calibrated scene/wrist depth and fits table-supported
clutter boxes. Instance identities remain simulator-oracle scaffolding; target
geometry/pose and robot/camera kinematics remain known. The native physical scene
is separate evaluation truth and never replaced by fitted boxes. Jev receives
structured measurements, not raw RGBD. Visible-point fits and experimental 5 mm
padding do not establish full-object containment or calibrated uncertainty.

Startup handling is an evidence-driven correction: some
coarse/noisy covers overlap the initial robot. The old failures stay retained.
New runs check h and the RD2 transformed barrier before allowing actuation, choose
an admissible all-object representation when one exists, otherwise reject the
unsupported setup rather than silently shrink margins or omit clutter. This is
not a hardware safe-stop design or a guarantee about unobserved surfaces.

The extension has 96 development, 168 frozen unseen and 90 camera/fault physical
runs, all independently re-evaluated. On the 12 new designed layouts, both Jev
once/repeated and the non-model least-excess/rule baselines improve from 6/12 safe
pickups with small covers to 12/12 with rich covers. The rule is still cheaper
geometrically; repeated polling adds no completion benefit here. Initial repeated
classifications disagree only on conditional geometry for omitted objects, not
the proposed clutter policy. These findings do not establish Jev superiority.
Six separate correction checks yield four audited pickups and two startup
refusals; one predicted refusal was wrong because a two-sphere candidate was
admissible where four/eight-sphere candidates were not. Raw predictions remain.
101 tests pass. Cumulative estimated token cost is $0.366746268 (374 logical
provider calls), conservative reservations $3.088318464 under the original $7.50.
No push or additional research experiment is included in completion.

Historical dynamic infrastructure: `artifacts/showcase/dynamic_v1/index.html`
contains the earlier moving-object reaching check, not the current task family.
Its `frax` profile has rigid fingers, unlike the articulated pickup profile.

### Current task direction: static-clutter pickup

User clarification, 21 October: use one task family only. The arm/gripper maneuvers
through static clutter, picks up the designated object, and stops once pickup is
verified. Vary clutter layouts and target positions within that same task. Do not
add placement, transport, pushing, or moving-obstacle tasks. The target itself may
move during pickup; surrounding obstacles remain static. Older task alternatives
below are historical possibilities, not the current scope.

The older reaching showcase does not implement this task; the new pickup workflow
does, using a functional simulated gripper, a graspable target, and explicit
handling of intended gripper-target contact while retaining other protections. Reuse the
existing upstream controller and local selection/CBF/viewing infrastructure where
applicable. Repeated selection responds to arm configuration and task progress
even though clutter is stationary; its benefit beyond a geometric rule remains
an open research question despite the observed one-shot/repeated difference.
The moving-object fixtures remain historical infrastructure checks, not the next
benchmark. Future contact/success changes should follow evidence rather than an
arbitrary predetermined protocol; preserve the old data when making corrections.

Earlier checkpoints (historical evidence): the launcher/environment are in place;
the original Panda demo passes a bounded 1,000-step run with finite states/torques
and positive sampled model barriers. The baseline execution checkpoint and agent
network/GUI workflow are verified. Nihar approved the next isolated sphere-cover
experiment: the generator, numeric checks, comparison viewer, and initial review
of Nihar's screenshots are complete. Nihar then approved a tiny standalone Jev
experiment. Editable scene/questions, request preview, and a thin SDK adapter are
implemented. The first live request returned `jev-1.13.0` in 0.629 s
with valid proposals; uncertain cube geometry is a check-in topic, not an optimality
or safety result. Nihar approved a scene viewer of that saved result: implemented,
39 offline tests pass, and bounded oblique/top-down launches exit normally. Nihar
visually reviewed the new viewer and said "looks good"; this checkpoint is complete.
Nihar then approved a static Panda integration/comparison through a review gallery.
Saved proxy answers now drive simulation-only obstacle constraints via the unchanged
FRAX controller/config and CBFpy. Off/all/replay runs each completed six simulated
seconds with about 1.4 mm final error, no sampled object penetration, and 0/210/168
scene constraint pairs. This is proxy-decision replay with a changed start/body
context, not a fresh whole-arm Jev decision. Off also worked, so this scene does not
demonstrate a safety advantage. Independent sampled MuJoCo collision checks and a
rendered replay gallery are available. Nihar reviewed all animations and said they
look good and normal; this static checkpoint is complete.
The next approved checkpoint is now implemented: reusable actual-Panda request
features, a phased static-suite runner, and a combined review gallery. Six frozen
scene variations produced six fresh Jev responses and 18 physical rollouts.
Jev reduces scene constraint pairs by 33–75% relative to fixed-four all-on and
has no sampled native contacts, but does not improve completion: both succeed
in two of six cases. Four constrained cases have negative modeled scene barriers
and stop short; a read-only probe also observes model/simulator acceleration
mismatch. These limitations require review, not silent controller retuning.
The review gallery is `artifacts/suite/panda_static_v2/index.html`; 58 tests pass.
Nihar accepted the visual-review checkpoint ("yes it looks good"). This does not
resolve the barrier/dynamics issues or authorize a new experiment automatically.
That earlier stop was superseded by Nihar's explicit autonomous-showcase request.
Online simulation is now implemented as described above. See [PROGRESS.md](PROGRESS.md)
for current results, blockers, and the decision log, and [AGENTS.md](AGENTS.md) for
repository working rules. Future directions below are not automatic implementation
instructions or approval for hardware deployment.

The agreed direction is a **new standalone repository** that imports and calls
existing FRAX, OSCBF, and CBFpy implementations. It owns the task-conditioned
constraint-selection framework and small integration adapters, not replacement
robotics libraries. Leave the three original repositories unchanged.

The research idea added to the brief is to generate several geometrically valid,
conservative representations deterministically, then let Jev choose among their
IDs using the task and measured geometric/computational costs. Jev does not need
to generate geometry, Python code, or arbitrary CBF equations. How we present
candidates, ask questions, combine answers, and revise decisions is itself part
of the research, not a prompting strategy already settled by this document.

### Working approach: decide the next step, not the whole project

Agree on one small step, implement or inspect it, review what happened with Nihar,
and use that evidence to choose the next step. A later step may change, split,
disappear, or become unnecessary. Do not build an entire imagined final architecture
before learning whether the first pieces work.

Keep the immediate objective and necessary assumptions explicit. Leave future
questions open until they matter. Normal implementation details can follow what
is simple and natural; exact file counts, worker counts, API retry settings,
query schedules, thresholds, and optimization techniques are not requirements
merely because an earlier draft suggested them.

The stable boundaries are the project purpose, reuse of upstream implementations,
read-only FRAX/OSCBF/CBFpy repositories, consistent physical/data conventions, and
honest validation and reporting. Nihar owns consequential research and safety
ambiguities; the coding agent should surface them rather than quietly resolve them.
Do not turn this into an approval ceremony for every helper function or local refactor.

Jev templates and orchestration are research components. Start with a simple
experiment, inspect decisions and failure cases, then revise the questions,
decomposition, context, and update policy. They should be easy to change without
rewriting the controller. A typed result contract helps integration; it should not
freeze the research design.

### How to use this document

- Sections 2, 3, and 10 are a provisional map of responsibilities and reuse options,
  not a demand to scaffold every module or finalize every data field now.
- Sections 4-8 preserve useful mathematical and integration considerations. Apply
  them when the corresponding feature is introduced; do not mistake examples for
  a complete first-deliverable checklist.
- Sections 1 and 9 describe possible later demonstrations and evaluation questions,
  not a predetermined sequence or benchmark protocol.
- Section 11 lists correctness checks to use as relevant capabilities are added.
- Section 13 separates approved choices from open decisions. Section 14 gives
  the narrow, step-by-step engineer handoff and the approved first checkpoint.

Any change to a research assumption should update this document and, once code
exists, its corresponding configuration. No implementation code is added by this
planning work.

## 1. Scope and research question

Use a supplied task and current scene to select object/robot-part avoidance pairs
and choose a conservative geometric representation for each selected object. Reuse
the FRAX operational-space torque controller and CBF objective, and CBFpy for
constraint construction and QP execution.

Test whether task-conditioned selection among valid representations improves the
task-completion / clearance / computation tradeoff compared with fixed geometry
and a deterministic geometry selector. Interface compatibility with Jev is verified
from its documentation; useful robotics performance and publication novelty are
not established.

The desired outcome is fewer unnecessary constraints and less geometric
conservatism while preserving useful task interaction, not merely generating more
barriers. CBFpy enforces the selected mathematical model; it cannot certify omitted
objects, inaccurate geometry/dynamics, or arbitrary contact transitions. Smooth
barriers alone do not guarantee whole-task safety, especially with relaxed QPs,
sampled control, and online changes to the constraint set.

Ground-truth object identities/poses/bounds and known robot state are permitted
initial simplifications. The existing Panda/MuJoCo demo is now the approved first
checkpoint, not a simulation requirement from the original brief or a choice
of final hardware. A static scene is a manageable baseline; adding motion requires
the dynamics considerations below, not just changing an object position.

### Approved first checkpoint and later possibilities

The approved first checkpoint is to run the existing FRAX OSCBF simulation demo
from the Jev repository without editing the upstream sources or adding new Jev
functionality. Use `uv` with an isolated project environment, reuse the existing
read-only upstream checkouts, and try Nihar's current Mac first. Report compatibility
problems before changing machines or approach. Learn what imports, dependencies,
and minimal adapter are actually needed; do not require the complete framework first.

After reviewing that result, choose the next useful investigation. Possibilities
include inspecting geometric covers in isolation, trying a small scene-selection
example, or adding a simple reaching/pushing demonstration. Full manipulation,
estimated geometry, cameras, and active perception remain project directions,
not a sequence whose implementation details must be fixed now.

Choose task-success definitions, timing settings, and experiment sizes when the
corresponding task is selected. Tune and record them with the experiment rather
than presenting unexplained numbers in the architecture as requirements.

## 2. Components and responsibility

```text
scene provider --> SceneSnapshot --> geometric candidate generator
                         |                       |
TaskContext -------------+------------------> Jev selector
                                                  |
                                         ConstraintProposal
                                                  |
                                      validator / update manager
                                                  |
                                         ConstraintSnapshot
                                                  |
nominal policy --> PolicyCommand --> OSC --> nominal torque --> CBFpy filter
                                                                  |
                                                             ControlResult
                                                                  |
                                                        simulator arm actuators
task phase -------------------------------------------> gripper adapter
simulator --------------------------------------------> independent evaluator
```

The accepted snapshot feeds the filter; perception and Jev never run inside the
torque-control calculation. Robot state is supplied to both control and selection
as needed; the diagram separates their update paths, not their physical state.
In the current simulated-depth prototype, camera capture/fitting and evidence
logging run synchronously in a periodic before-control hook. Only Jev requests
have a background worker. This is logical separation, not a claim that sensor
I/O cannot delay a control step. Moving acquisition to a separately owned native
rendering/sensor worker is a future measured-performance change, not implemented
hard-real-time isolation or active viewpoint planning.
That paragraph describes the retained legacy simulation path. D20 adds a separate
source-owned camera worker for replay/SDK input and a simulation consumer that
does not perform acquisition, fitting, candidate generation or raw camera logging
inside control. Fake stalled sources test the nonblocking boundary; this is not
real-time certification or validation of a live physical-camera/robot loop.

Prefer ordinary functions and small data records. Add abstractions only when an
actual substitution or maintenance need justifies them. The layout below is one
possible later organization, not files to create before the first experiment.
The repository exists; the Python package name and packaging details can be chosen
when needed.

```text
jev-cbf-planning/
  pyproject.toml
  src/<package>/
    types.py       shared records and configuration
    upstream.py    pinned-source imports and compatibility adapters
    scene.py       scene/task providers and simulator adapter
    geometry.py    candidate generation, coverage, and numeric features
    selector.py    Jev questions, response interpretation, replay/rule substitutes
    control.py     OSC integration, CBF configuration, snapshot admission/packing
    run.py         assembly, worker ownership, configuration, and recording
  third_party/
    frax/          pinned git submodule, not a copied implementation
    oscbf/         pinned git submodule
    cbfpy/         pinned git submodule
```

Keep new logic in the new repository. Use normal public imports where possible;
centralize the few source-file imports in `upstream.py`. Do not spread path hacks,
duplicate upstream controller math, or modify upstream code to make imports work.
Modules may start combined and split as needed. Lightweight recording can initially
live next to the entry point; a separate evaluator or worker is not required until
the experiment needs one. Pinned submodules are one dependency-management option,
not a prerequisite for trying imports from existing read-only checkouts.

Upstream ownership:

| Repository | Reused responsibility | Integration boundary |
| --- | --- | --- |
| FRAX | robot loading, kinematics/dynamics, Jacobians, robot collision spheres, FRAX-adapted OSCBF configuration | public robot API plus an adapter for `examples/cbf_utils.py` |
| OSCBF | existing `PoseTaskTorqueController` | standalone controller receives FRAX-computed arrays; no second robot model |
| CBFpy | CBF configuration API, differentiation, QP construction, safety-filter/solver integration | public `CBFConfig`/`CBF` APIs with a thin diagnostics adapter if needed |

The new project owns scene/task contracts, candidate generation and validation,
Jev selection, accepted-constraint lifecycle, and orchestration. These are the
missing framework pieces, not features to reimplement inside the original repos.

Substitution boundaries:

| Callable | Input | Output / authority |
| --- | --- | --- |
| scene provider | simulator or sensors | factual `SceneSnapshot`; no oracle obstacle labels |
| task provider | instruction, task state, phase | explicit `TaskContext` and allowed-contact rules |
| nominal policy | robot state, scene, task | `PolicyCommand`; cannot bypass the safety filter |
| OSC adapter | robot state, `PolicyCommand`, FRAX matrices | nominal arm torque from existing OSCBF controller |
| candidate generator | object-local bounds/geometry | valid candidates and numeric features |
| selector | scene, task/phase, candidates | proposed roles, candidate IDs, avoidance groups |
| validator / manager | proposal and latest robot/scene state | accepted immutable constraint snapshot or rejection |
| control filter | robot state, nominal torque, accepted constraints | `ControlResult` with torque and diagnostics |
| evaluator | simulator geometry, applied action, task permissions | safety/completion/coverage/timing metrics |

The evaluator may inspect simulator truth. Its answers and success labels are not
passed to Jev or used to rescue its decisions silently.

Keep relevant settings easy to find and change, without introducing a configuration
framework ahead of need. Add robot/task, template/model, geometry, solver, or update
settings as the corresponding capability is introduced. Record the choices used
for an experiment. Do not store credentials in source or experiment logs.

## 3. Data and action contracts

These names are a working vocabulary for integration, not an obligation to build
all records and fields at once. Implement the data needed for the current step and
revise its structure as the interface becomes clearer. Preserve explicit units,
frames, action meaning, and provenance as the representation evolves.

Use metres, radians, seconds, and explicitly named right-handed frames. The
reference formulas below use a world frame; matching it to the initial fixed
robot-base frame is a convenient initial option. `T_A_B` maps coordinates in B
into A. 4x4 transforms are one natural representation, not a mandatory wire format.
If quaternions are used, specify and convert their ordering explicitly.

`RobotState`: arm joint names/order, `q`, `v`, simulation time, monotonic acquisition
time, and gripper width/state separately. Read simulator joints/actuators by name,
not by assuming every robot forever occupies the first seven entries. For this
Panda arm the CBF state is `[q_arm, v_arm]` and the control is seven joint torques.
Do not generalize `nq == nv` or `q_dot == v` to floating-base robots.

`SceneSnapshot`: snapshot ID, scene epoch, both timestamps, frame, and objects.
Each object has a stable ID, descriptive attributes, `T_world_object`, object-local
bounding-box centre/half-extents, geometry revision, and observed motion status.
Reserve optional velocity, uncertainty, and image fields; initial ground truth
does not require artificial noise or confidence values. Bounds must contain the
full simulator collision geometry, not just visible points. Names/poses alone
are insufficient inputs for geometric generation.

`TaskContext`: ID/epoch, instruction text, named target or externally resolved target
reference, goal region/pose, current phase, and allowed-contact rules. The framework
is a safety layer, not an additional task planner. For initial comparisons the
nominal task goals/waypoints and contact rules are the same for every method.
Supplying the task's target reference is allowed; supplying oracle avoidance masks
or the expected candidate answer is not.

`PolicyCommand`: desired world-frame EE position/rotation and linear/angular
velocity, optional desired accelerations, and gripper opening in metres. The
initial policy is a scripted waypoint/phase routine. The existing OSCBF controller,
fed by FRAX kinematics/dynamics, maps arm commands to nominal torque; the CBF sits
after this mapping. A future learned policy must implement this command contract
or supply an explicit action adapter. A policy
producing joint velocities cannot be silently treated as a torque policy.

`GeometryCandidate`: object/revision/candidate IDs, primitive type and local
parameters, coverage certificate, estimated excess volume, primitive count, and
estimated pair count. Candidate IDs refer to numeric geometry stored in code;
Jev does not invent coordinates or radii.

`ConstraintProposal`: request/snapshot IDs, scene/task epochs, per-object role,
candidate ID, per-object robot-group avoidance decisions, probabilities/confidence,
model ID, and prompt/version hashes. Keep uncertainty in the record rather than
immediately converting everything to booleans.

`ConstraintSnapshot`: accepted revision, task/scene epochs, immutable numeric
parameters, named pair IDs/masks, fixed gains, validation record, source ages,
and any array-shape metadata the implementation needs. Do not silently mutate
registered-static configuration to represent a runtime constraint update.

`ControlResult`: nominal torque, filtered torque, actuator-limited requested torque,
accepted snapshot revision, success/failure reason, and numerical diagnostics.
Actual simulator actuator forces are recorded separately after application; a
returned command is not evidence that the simulator applied it unchanged.

Illustrative function boundaries, not a promise of exact implemented signatures:

```text
observe(...) -> (RobotState, SceneSnapshot)
task_context(...) -> TaskContext
policy(robot, scene, task) -> PolicyCommand
make_candidates(scene, geometry_settings) -> candidates
select(robot, scene, task, candidates) -> ConstraintProposal
validate_and_pack(proposal, current_state, previous_snapshot) -> snapshot or rejection
nominal_torque(robot, command) -> torque
filter_control(robot, torque, snapshot) -> ControlResult
```

`select` may be asynchronous; the other boundaries need not adopt an asynchronous
API. A ground-truth provider can later be replaced with a perception provider, and
a Jev selector with a rule/replay selector, without changing CBF mathematics. A
new torque-producing policy can bypass OSC through an explicit action adapter,
but cannot bypass the filter except in the explicitly configured no-CBF baseline.

## 4. Geometry generation and containment

Obstacle volume is a **union** of primitives. Enforcing all relevant clearance
conditions gives an intersection of allowed robot configurations. Avoiding an
intersection of obstacle primitives would require disjunctive logic; that is not
the proposed implementation.

The approved isolated geometry checkpoint now implements box-section sphere
covers with 1/2/4 sections and numeric/visual inspection tools; see README and
PROGRESS for commands, observed excess volumes, and validation limits. Saved sphere
covers are now connected in the approved static Panda replay experiment only.
The broader family below remains provisional:

1. One sphere enclosing the object-local bounding box.
2. Two spheres enclosing two equal box sections along its longest axis.
3. Four spheres enclosing four such sections.
4. One ellipsoid enclosing the box.

For box centre b and positive half-extents a, each section's sphere is at its
section centre and has radius equal to the Euclidean norm of that section's
half-extents. Every section is contained in its sphere, so their union covers
the box and hence the object. For the ellipsoid, centre b and semi-axes
`sqrt(3) * a` enclose the box: every box point has normalized squared radius <= 1.
Transform primitives from object to world using the same reported object pose.

This is a deterministic, conservative generator, not a learned shape generator.
Bounding boxes may themselves be loose; these candidates are not claimed to be
optimal approximations of a mesh. The current positive-volume box/cylinder
experiment rejects zero/nonfinite/negative extents rather than silently inflating
them. Revisit degenerate measurements explicitly when perception is introduced.

Coverage is a hard candidate-admissibility condition in the ground-truth stage.
Estimate excess **union** volume against actual reference geometry using shared,
seeded samples; summing overlapping primitive volumes would double-count overlap.
The isolated experiment uses exact analytic boxes/cylinders, not simulator meshes;
simulator-geometry coverage remains a later check when that integration is chosen.
Report that estimate separately from the analytic coverage certificate. Compute
candidate features outside Jev, including primitive/pair counts and geometric
costs. Use the same samples across methods and record sampling uncertainty.

### Smooth barriers and robot-body size

For robot sphere centre p_i(q), radius r_i, obstacle sphere centre c_j and radius
R_j, use the dimensionless quadratic barrier:

`h_ij = ||p_i(q) - c_j||^2 / (r_i + R_j + delta)^2 - 1`.

Use one barrier per enabled pair, not hard min/max aggregation. Initial geometric
clearance margin and who selects it remain decisions for the geometry/selection
experiment; this revision does not set a numerical margin. Separate any desired
clearance from perception-error inflation. A model-confidence statistic should
not be assumed to equal metric geometric uncertainty without calibration. The
mentor's proposal for Jev-selected margins remains pending a separate decision.

For an obstacle ellipsoid with semi-axes a and robot sphere plus margin rho,
simply replacing a with `a + rho` is not generally a conservative Minkowski sum.
Use a valid outer ellipsoid. One explicit construction, for rho > 0, is:

`beta = ||a|| / (sqrt(3) * rho)`

`a_outer^2 = (1 + 1/beta) * a^2 + (1 + beta) * rho^2`.

Its orientation and centre are unchanged. Its support contains the true
ellipsoid-plus-ball support by `2*sqrt(A*B) <= A/beta + beta*B` for every direction.
For rho = 0 use the original ellipsoid. The barrier is
`h_ij = sum((R_j.T @ (p_i(q)-c_j) / a_outer)^2) - 1`.
This outer approximation has its own excess volume and is not an exact distance.
Measure effective robot-expanded geometry as well as raw obstacle geometry.

The demo's alpha/alpha_2 gains provide an inherited baseline, not an instruction
to tune or redesign gain selection before trying it. Primitive quadratics have
continuous second derivatives for positive sizes, but switching candidates is a
separate hybrid-system event, not a smooth
operation. Warm up with a valid nonsingular robot configuration and EE rotation,
not an all-zero orientation or an unchecked singular pose.

## 5. Jev questions, selection, and semantic authority

### Templates and orchestration are research, not settled plumbing

Start with the simplest useful question/scene experiment when selection becomes
the chosen step. Inspect concrete answers, mistakes, uncertainty, and behavior
before adding more stages or rules. Do not commit the project to one universal
prompt, fixed role taxonomy, robot-group partition, or multi-call workflow.

Examples worth exploring include separate role/candidate/pair questions, combined
decisions, different descriptions of the task and geometry, and different ways of
handling ambiguity. Upper-arm/forearm/wrist groups and labels such as target,
avoid, or support are possible starting points, not mandatory classifications.
Even the right unit of a question is something to learn from examples.

Keep question wording, candidate descriptions, state/context construction, and
answer-combination logic easy to inspect and revise without editing controller
mathematics. A simple editable template or small configuration can be enough;
there is no requirement for a prompt framework or a particular file format.
Record the template/configuration and actual model version for comparisons.

Query timing, question batching, call dependencies, and how previous decisions
are used are also experimental choices. Choose the initial version for the small
case at hand; review its behavior with Nihar before deciding how to expand it.
Do not invent confidence cutoffs, debounce counts, or cost weights in advance.

### Keep the integration understandable while allowing experiments

The selector can propose roles, candidate IDs, and robot-body pairs through a
small structured output that code can validate. Different templates/orchestrations
can feed that same control boundary; revise the boundary deliberately if evidence
shows it is inadequate. Candidate generation and numerical barrier construction
remain separate from language-model output in the current project idea.

Respect actual API behavior while experimenting: questions evaluated independently
in one request cannot consume each other's returned answers. Dependent decisions
need an explicit later stage or questions phrased conditionally. This is an API
consideration, not a reason to prescribe one orchestration for every experiment.

Model confidence, geometric uncertainty, and collision risk are different signals.
How to use them is a research question, not a universal threshold protocol. A
semantic role alone is not a geometric coverage certificate or an unrestricted
contact permission. Changes to contact authorization, margin/gain authority, or
other consequential safety assumptions need an explicit decision with Nihar.

**Important limit:** validating coverage of selected objects does not prove that
the selector included every object/pair that needed protection. Evaluate omissions
honestly; do not silently use oracle labels to repair decisions or claim full-scene
safety for an incomplete selected set. Any guard or fallback that changes the
model's proposal should be visible in the result.

Mocks, replay, or a simple numeric selector can help investigate one component.
Use them when useful, label them accurately, and do not require all baselines or
a predetermined numerical cost function before trying the first Jev example.
Nihar subsequently approved a tiny standalone text-only Jev experiment after
geometry review. Its optional SDK/preview are implemented and a first live request
has succeeded using local credentials. This narrow approval does not authorize a continuous querying
loop or deployment of model proposals into control.

## 6. Interactions, grasping, and object motion

These are considerations for experiments that introduce contact or object motion,
not features to implement before the initial checkpoint. The examples below do
not choose the final manipulation task or settle new mentor proposals.

Permanent filtered-method protections: torque bounds, joint limits, self-collision,
floor/table clearance with explicit mounting/support exclusions, and the demo's
singularity avoidance. The latter also requires numerical checks near rank loss;
sphere/ellipsoid smoothness does not cure an unstable task-inertia inverse or SVD.
Do not silently remove fixed protections because Jev labels an object harmless.

For a free-space reaching experiment, no target contact is needed. If that task
is selected, use comparable commands across methods. An OSC safety filter is local
and can stop at a barrier; it is not expected to discover a route around every
obstruction. A stalled episode is not automatically a safety failure.

For a future pick-and-place experiment, possible phases are approach, descend,
close, lift, transport, lower, release, and retreat. This is a reference example,
not a task script to implement before choosing the demonstration. Its explicit
contact rules would permit fingertip-pad/target contact only during
the designated interaction phases, and target/table or target/placement-surface
contact where required. Palm, wrist, arm, unrelated objects, and payload/environment
pairs remain subject to the declared avoidance policy. Gripper motion/force limits
are controlled by the task/simulator adapter; the seven-torque arm QP does not
automatically constrain finger motors or guarantee grasp quality.

The current FRAX model merges hand/finger geometry into an arm-link collision
model. Fingertip-specific interaction therefore requires a validated palm/finger
model or conservative gripper swept envelope; disabling the whole hand's target constraints
and calling that fingertip-only contact is not acceptable. Map named simulator
geoms and FRAX sphere owners explicitly. The simulator already has a gripper
actuator, but the demo does not operate it as a grasping routine.

During verified rigid transport, represent the payload as collision primitives
attached to the EE through a measured frozen relative transform; their centres
are functions of q and can be differentiated by CBFpy. Check actual payload pose
for slip. Account for payload mass/inertia before claiming a matched loaded
dynamics model; choose the mechanism when this feature is introduced, rather than
prescribing prebuilt model variants now. Contact acquisition/release and
any unmodeled payload dynamics are empirically evaluated hybrid phases, not a
formal whole-task safety certificate. Address these issues when manipulation is
chosen; do not require payload infrastructure for a reaching or geometry experiment.

For a static-scene baseline, unexpected obstacle movement violates that assumption
and must be reported. Whether to pause, investigate, or abort the simulation should
be appropriate to the current experiment, not a universal preselected response.
For moving scenes, extend the state/drift to include relevant obstacle motion or
an explicit time-dependent formulation, then validate velocity/acceleration and
latency assumptions. Replacing a centre passed as an extra argument each tick does
not by itself add the missing time derivatives to current CBFpy.

## 7. Timing, updates, and JAX shape management

This section identifies issues to handle when live updates are introduced, not
a scheduler to implement before the first example. Start simply, observe behavior,
and add concurrency or optimization only when the current step needs it.

### Correctness to preserve

Keep the identity, geometry version, frame, and relevant task context of a proposal
understandable. A result must not silently refer to different objects or coordinate
conventions. Robot motion alone need not invalidate a query; check its proposal
against the state that exists when it is used, rather than trusting old positions.

A constraint change can exclude the robot's current state even if both old and
new geometries cover the obstacle. For relative-degree-two barriers, inspect both
h and the intermediate `psi = L_f h + alpha_2(h)` when evaluating such a change.
Do not assume arbitrary switching preserves the original safety conditions.
Choose and document the admission/failure response when implementing updates.

Control must consume a complete, consistent set of parameters, not a half-updated
list. How that is achieved can be a simple synchronous update initially or an
immutable snapshot/message boundary when concurrency is actually needed.

For real-time use, model/network delays must not silently stop the arm-control
calculation. An intentionally paused simulation is a different execution mode:
describe what actually ran rather than calling it real-time operation.

Runtime changes should be represented explicitly in the arguments/data used by
JAX, not hidden mutations of registered-static configuration. Inspect compilation
behavior when adding changing constraints. Do not silently omit pairs because
the chosen array representation cannot hold them.

### Choices to make from evidence

There is no prescribed number of workers or requests in flight, control/physics
frequency, timeout, retry count, semantic expiry period, or debounce rule. Use
ordinary mechanisms appropriate to the current experiment and document choices
that affect its meaning or observed behavior.

Whether to query once, periodically, or in response to an event belongs to the
research design. Freshness and fallback behavior must fit that design; a universal
expiry timer must not silently turn one query policy into another.

A capacity bank, padding, prewarming, batching, or alternative array layout may
be useful if changing shapes causes measured problems. None is a first-step
requirement. If padding is used, it needs finite, well-defined satisfied barriers,
not NaN or zero-axis geometry. Fewer enabled rows in one fixed-size QP need not
reduce its compute time; measure rather than assume.

Keep network I/O and Python orchestration outside jitted numerical functions.
Record enough timing/compile information to explain observed problems, without
requiring a full performance instrumentation system before a proof of concept.
Do not place credentials in logs.

## 8. Solver behavior and numerical failure

Reuse the existing CBFpy/FRAX solver path first and inspect its actual behavior.
The reviewed FRAX port selects ElastiQP with solver_tol 1e-5; its inherited
relaxation penalties were 1e3 for CBFs and 1e5 for control bounds. These describe
the inspected source, not a new project-wide tuning specification.

ElastiQP relaxes constraints, including control bounds. Apply physical motor
limits and evaluate what happens to the command actually applied: clipping can
invalidate an inequality satisfied by the returned torque. Verify actuator mapping
when it becomes relevant. Do not interpret a returned control vector alone as
proof of a successful, physically safe solve.

CBFpy's public wrapper returns the control vector rather than full diagnostics.
Add a thin instrumented adapter only when the experiment needs that visibility;
reuse the existing QP data/solver rather than building another optimizer. Inspect
residuals and numerical scales before choosing acceptance thresholds. This plan
does not impose an unexplained residual cutoff.

Report nonfinite data, failed solves, invalid states, and control-limit problems
explicitly. Depending on the simulation and failure, pausing for inspection,
rejecting an update, retaining still-valid constraints, or aborting may be
appropriate. Decide the behavior when it matters; never silently continue with
an arbitrary replacement torque or count a failed/aborted run as a safe success.
A zero torque or unvalidated hold is not automatically a safe stop.

Hardware stopping behavior is outside this document's current implementation
scope and needs a separate, hardware-specific plan before deployment.

## 9. Experiments and independent evaluation

This is a menu for later experiments, not a first-deliverable checklist. Decide
which question to investigate after reviewing the preceding step. Start with a
small inspectable case; broaden scenes and methods when the result motivates it.
No object counts, episode counts, success thresholds, or full metric suite are
prescribed here.

Compact/elongated shapes, rotated objects, narrow passages, or task changes may
be useful cases. For a meaningful comparison, make differing inputs, commands,
contact rules, timing conditions, or other confounds visible. Record the relevant
settings; a seed alone does not make a hosted API reproducible.

Possible later comparisons include:

| Method | Scene selection | Geometry | CBF treatment |
| --- | --- | --- | --- |
| raw nominal | none | none | all CBFs off; physical actuator limits remain |
| all on | all relevant object/body pairs, same task contact permissions | fixed single sphere | fixed protections plus all scene constraints |
| Jev / fixed | Jev roles/pairs | fixed single sphere | fixed protections plus proposed scene pairs |
| Jev / adaptive | Jev roles/pairs | Jev candidate Choice | same fixed protections |
| numeric geometry | same selection logic / replayed proposals for controlled ablation | deterministic candidate cost | same fixed protections |

Add a fixed-protections-only / scene-CBF-off diagnostic if necessary to separate
general robot safety from scene-safety effects. “All on” does not mean enable all
candidate representations simultaneously. Both all-on and adaptive methods receive
the same explicit task contact permissions; an impossible grasp is not a fair baseline.

Evaluate collision/clearance claims against actual simulator collision geometry,
not only the fitted CBF geometry or visual meshes. Choose appropriate contact
logging, tolerance, and sampling resolution for the claim and record them. Discrete
checks are not continuous-time collision proofs.

Choose measurements that answer the experiment's question: collision/completion
outcomes, coverage/conservatism, constraint count, or decision/control latency may
be enough initially. Residuals, binding/allocated rows, age, recompilation, API
cost, or detailed timing distributions can be added when they explain a problem
or support a claim. Do not build a large telemetry system before seeing behavior.
Show the result in an understandable form at the user check-in; a viewer or a
small recorded example can be sufficient for an early investigation.

If every non-target object must be avoided, obstacle-role selection can be nearly
trivial. Include meaningful instruction/contact distinctions and a rule-based
selection baseline where needed. Do not infer that Jev adds value simply because
CBFs or deterministic fitted geometry help. Negative findings remain informative.

Keep enough configuration, inputs/outputs, versions, and decisions to understand
and repeat the current experiment. Add richer recording or replay when useful,
not because every future field was listed in advance. Never record credentials.
Label replay/mock output accurately; it is not evidence of live Jev performance.

## 10. Reuse and dependency plan

The independent `jev-cbf-planning` repository has been created. Keep all new code
and documentation there; the original FRAX, OSCBF, and CBFpy repositories remain
read-only. Do not copy their implementations into the new package.

Existing read-only checkouts are enough to investigate the first import/demo
step. Pinned submodules under `third_party/` are a later reproducibility option,
not a mandatory setup step. Choose dependency packaging when its actual needs
are known, and record the revisions used for results.

All three supply runtime code, but they need not all be installed as complete
Python distributions. FRAX and CBFpy use normal package imports. Reuse OSCBF's
standalone `oscbf/core/controllers.py` from its pinned checkout through one explicit
source-module adapter. That module consumes JAX arrays and does not depend on the
older OSCBF robot model. This avoids installing OSCBF's legacy pinned JAX/CBFpy
stack, which conflicts with the newer FRAX examples. Source-only reuse is a
deliberate compatibility boundary, not a claim that the whole old stack is compatible.

Approved setup direction: use `uv` with an isolated environment in the Jev project,
reuse the current read-only upstream checkouts, and try Nihar's Mac first. The
reviewed runtime candidate was Python 3.12 with JAX >= 0.9.1, MuJoCo >= 3.3.1,
ElastiQP, and the checked-out FRAX/CBFpy. A project environment and generated
`uv.lock` now exist; imports and a headless model/CBF smoke check pass. This does
not establish general safety: the bounded 1,000-step original-controller run
passes its default-target smoke checks, but visual stress cases and independent
constraint evaluation remain unverified. GUI/network operations follow current
runtime permissions rather than manual Terminal handoffs. Current versions and evidence are
recorded in PROGRESS.md. The official TypeSafe SDK 0.7.5 is now installed as an
optional project extra for the approved standalone selection experiment; its
adapter passes local fake-transport tests. It is not needed to reproduce the
controller demo. Live API access is verified by one successful request; general
model performance, latency distribution, and robotics safety remain unverified.

Observed source revisions (20 Oct 2026):

| Repository | Local HEAD |
| --- | --- |
| FRAX | `cc0a066b5b539c729838308496ae1b8a1d6aeada` |
| CBFpy | `0ab24ecd431ca151a80cd7e4f2a5d3967e1c4b5a` |
| OSCBF | `6082329f8e1d61ee8e875a63a5cd80a14d513e12` |

Promising reuse points to try as needed, with source imports kept in a small
project-local adapter rather than requiring a particular module layout first:

1. Import FRAX's `examples/cbf_utils.py::OSCBFTorqueConfig` from its pinned source
   path. It already uses FRAX's `Manipulator`; a project-local subclass can supply
   scene barriers when that feature is introduced. Its dynamics and task-consistent QP
   objective should remain upstream implementations.
2. Import OSCBF's `PoseTaskTorqueController` from its standalone source module.
   Supply `robot.torque_control_matrices(q, v)` and the same gains, posture target,
   and desired task signals used in the FRAX demo. The source equations match the
   demo's nested nominal controller; numerical parity remains a verification gate.
   Set controller `tau_min`/`tau_max` to `None` to avoid clipping nominal torque
   before the CBF QP. Physical actuator limits are still applied and checked later.
3. Use CBFpy's public construction/filter APIs. Only add instrumentation around
   the existing QP-data/solver path where the public return value omits diagnostics;
   do not implement a second optimizer or differentiation engine.

Do not use the older OSCBF `OSCBFTorqueConfig` with a FRAX robot: it checks against
OSCBF's own `Manipulator` type. Also do not copy the nested FRAX demo controller
when the standalone OSCBF controller supplies the same computation.

Use a small explicit `importlib` loader for pinned non-public source files;
validate expected paths/symbols and fail clearly if upstream changes invalidate
the adapter. Avoid process-wide `sys.path` modifications, ambiguous `examples`
imports, duplicate imports of the same registered-static class, and loading a
different installed version accidentally. Keep precision initialization explicit
in the new entry point rather than relying on incidental upstream package imports.
Preserve upstream licenses and attribution.

## 11. Verification gates and current evidence

Apply these checks as the relevant capability is introduced and before making
claims about it. They preserve correctness; they do not require building every
future capability before the first small demo.

1. Match FRAX/MuJoCo EE transforms, joint order, torque limits, mass/gravity terms,
   and robot collision sphere ownership at valid configurations.
2. Check analytic candidate coverage and finite first/second barrier derivatives;
   compare smooth sphere/ellipsoid derivatives against numerical directional checks.
3. Verify obstacle/pair updates change the intended QP rows and preserve identity;
   inspect recompilation and latency. If capacity/padding variants are introduced,
   verify their behavior rather than assuming they work.
4. Verify the implemented state/admission and failure handling, including stale
   input, ambiguous results, API/numerical failures, or saturation as relevant.
   Check timeout/capacity mechanisms if they are introduced, not as prerequisites
   that force those mechanisms into the first implementation.
5. Demonstrate the chosen task and an actual selection/update change before
   claiming online adaptation; this does not select reaching as the required task.
6. Gate manipulation claims on gripper/palm geometry, contact permissions, grasp
   verification, payload collision/dynamics handling, and an independent contact check.
7. Gate camera-based and moving-obstacle stages on their own geometry/calibration
   and dynamics checks; do not silently claim these from the ground-truth study.

Executed during this review, using available NumPy only: 100,800 interior/corner
points across 100 boxes passed coverage probes for the 1/2/4-sphere family and
enclosing ellipsoid; 100,000 support-direction probes passed outer-ellipsoid
inflation checks. An explicit counterexample showed naive axis-plus-radius
inflation under-covering. Analytic containment arguments are above; sampling is
an implementation sanity check, not a replacement for those arguments.

At the initial review, the inspected interpreter was Python 3.12.7 on arm64 macOS
with NumPy 1.26.4. JAX, MuJoCo, qpax, ElastiQP, and typesafe-sdk were not installed
there, and no project virtualenv was found in this workspace. No controller,
simulator, Jev API, runtime, solver, or end-to-end benchmark was executed in this
review. Machine preference and Jev access remain to be confirmed, and runtime
performance, threshold calibration, and paper-level novelty are empirical/open
checks rather than architectural blanks to fill with invented results.

## 12. Source references

- Local FRAX: `frax/examples/oscbf_demo.py`, `frax/examples/cbf_utils.py`,
  `frax/examples/example_utils.py`, `frax/examples/xml/panda.xml`,
  `frax/frax/core/robot.py`, `frax/frax/robots/franka_panda.py`.
- Local CBFpy: `cbfpy/cbfpy/config/cbf_config.py`,
  `cbfpy/cbfpy/cbfs/cbf.py`; dependency manifests in each source repository.
- Local OSCBF: `oscbf/oscbf/core/controllers.py` for the reusable nominal controller;
  its original OSCBF configuration uses a different robot model from FRAX's port.
- [Jev announcement](https://typesafe.ai/blog/introducing-system-one-models-and-jev).
- [Choice](https://docs.typesafe.ai/primitives/choice),
  [Noul](https://docs.typesafe.ai/primitives/noul),
  [batching](https://docs.typesafe.ai/patterns/fan-out),
  [confidence](https://docs.typesafe.ai/confidence),
  [model versions](https://docs.typesafe.ai/models),
  [async client](https://docs.typesafe.ai/sdk/python/api/clients/async),
  [retries](https://docs.typesafe.ai/sdk/python/api/retries).
- [Experimental Jev image input](https://docs.typesafe.ai/images): standard images
  are documented behind an opt-in header. This is not evidence of metric RGB-D
  understanding. Initially keep RGB/optional depth visualization for context and
  calculate metric geometry numerically from calibrated depth outside Jev.
- [JAX JIT](https://docs.jax.dev/en/latest/jit-compilation.html).
- [MuJoCo distances and contact forces](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-geomdistance).
- [ElastiQP](https://github.com/StanfordASL/elastiqp).
- [OSCBF paper](https://arxiv.org/pdf/2503.06736),
  [FRAX paper](https://arxiv.org/pdf/2604.04310).

## 13. Decision ownership and open questions

### Settled boundaries

- The Jev repository owns all new project code and documentation. FRAX, OSCBF,
  and CBFpy are read-only upstream dependencies.
- Reuse the existing controller, robot computations, and CBFpy backend rather
  than replacing their implementations in the new project.
- Ground-truth scene geometry and robot state are permitted initially. The
  initial brief allows spheres/ellipsoids and multiple primitives per object.
- The candidate-generation/selection idea remains an experiment to investigate,
  not an already validated method or a frozen Jev template.
- Consequential ambiguities belong to Nihar. The coding agent surfaces them
  with evidence and alternatives; ordinary implementation details can evolve
  naturally without individual approval.

### Approved planning approach

Proceed one small step at a time, review the outcome with Nihar, then decide
what comes next. Future steps and their detailed choices are not automatically
approved because they appear in this document. Templates and orchestration are
research questions, not a fixed integration recipe.

Keep a lightweight record of decisions and why they were made. When a question
becomes relevant, distinguish a true correctness/interface constraint from a
preference, an inherited upstream setting, or an untested hypothesis. Do not
invent an answer solely to make the plan look complete.

### Approved immediate choices

Nihar approved these choices for the first checkpoint:

1. Run the existing FRAX OSCBF simulation demo from the Jev repository, without
   new Jev functionality or edits to any of the three upstream repositories.
2. Use `uv` with an isolated project environment and the existing read-only
   upstream checkouts.
3. Try the current Mac first; report compatibility problems before changing course.

These choices were subsequently implemented and the bounded baseline run verified;
see PROGRESS.md for evidence and limits. Nihar then approved isolated sphere-cover
generation, coverage/excess-space checks, and inspectable visualizations. After
reviewing both screenshots, Nihar approved the tiny standalone text-only Jev
experiment, followed by visual review and a static simulation comparison. Saved
proxy-selected sphere covers are now integrated through the original controller
without new model calls. This does not establish the selections' validity for the
whole arm. Nihar's visual review of the comparison gallery is complete; the next
research experiment still needs agreement.

### Questions to revisit only when relevant

The first new manipulation task, final hardware, geometry family, margin selection,
robot groups, template structure, query/update policy, failure handling, dependency
packaging beyond the initial local setup, and evaluation protocol can be decided
at the step that needs them. This is not a questionnaire that must be completed
before starting any work.

Other mentor proposals remain pending separate approval, including the precise
collision-risk/interaction selection criteria, Jev-selected uncertainty margins,
additional primitives such as half-spaces, model/query-frequency comparisons,
and the expanded viewing/evaluation workflow. The approved planning approach and
three immediate choices above do not silently settle those other research choices.

### Extension considerations, not infrastructure to build now

Ground-truth observations may later be replaced with calibrated RGB-D estimates.
Incomplete observations do not inherit the ground-truth coverage certificate.
A visualized depth image is not a substitute for metric geometry processing.

Active perception would require explicit viewpoint/observation actions, not
merely refreshing constraints. Learned policies would need an action adapter;
moving obstacles need the dynamics/time-derivative treatment in Section 6.
Keep those distinctions in mind without designing every extension in advance.

Nihar authorized the baseline checkpoint and, after its completion, the isolated
sphere-cover and standalone selection checkpoints. Actual implementation/execution status lives in
PROGRESS.md; these authorizations do not extend to unselected research proposals,
continuous model querying, controller deployment, or hardware work.

## 14. Engineer handoff: one checkpoint at a time

This handoff replaces the earlier full-framework-first implementation contract.
The engineer is not expected to prebuild every module, record, selector mode,
worker, geometry option, or benchmark before showing a useful result.

### Approved first step: reuse the existing demo

The agreed objective is to run the existing FRAX OSCBF simulation demo from the
Jev repository, without new Jev functionality and with all three upstream
repositories unchanged. Use `uv` and an isolated project environment with the
existing read-only upstream checkouts. Try Nihar's current Mac first, and report
compatibility problems before changing course.

These choices define the first implementation checkpoint, which Nihar has now
completed and reviewed sufficiently to approve the isolated geometry step. Track
what has actually been implemented/run in PROGRESS.md;
an approved setup choice is not evidence that setup or simulation succeeded.

Use the smallest natural entry point/adapter needed. Determine the actual
dependency/import requirements and report mismatches rather than quietly copying
code or changing upstream files. Do not add a Jev pipeline, a new task suite,
capacity management, or an orchestration framework merely because the eventual
system may need them.

An observable outcome could be the baseline demo running through that entry point,
with a brief explanation of what was reused and what remains unverified. If the
environment blocks execution, report the exact blocker and the useful work already
done; do not label an unexecuted integration as validated.

### Review the result, then choose the next step

At the check-in, show what worked, what failed or was surprising, and the few
decisions now needed. Agree with Nihar on the next small objective based on that
evidence. It might be inspecting primitive coverage, examining Jev decisions on
a tiny scene, changing an interface, or resolving a compatibility issue. This is
not a mandatory sequence, and later tasks may change after each result.

Prior handoff: the static-clutter pickup gallery and original follow-on software
study are complete locally. Open `artifacts/pickup/review/index.html`. Six methods
are compared on 12 layouts, with original failures and the corrected replication
kept separate. Repeated Jev achieves 12/12 safe pickups versus 6/12 once; all-on
and the geometric rule also achieve 12/12. The rule has lower geometric costs, so
this does not establish Jev superiority. All 72 corrected trajectories pass the
independent native audit; 89 offline tests pass. The completed package used 114
logical requests, with estimated token cost $0.067044516 under the $7.50 cap.
Final link, playback, credential, source-archive and no-new-call resume checks
passed. This is preserved historical evidence, not the running extension's final
result. Hardware and learned detection remain unimplemented; the newly approved
depth-derived-clutter extension is described at the top and in PROGRESS.md.

The current reusable implementation is deliberately small:

| Module | Responsibility |
| --- | --- |
| `inspect_selection.py` | Shared candidate requests, official SDK adapter, and response validation. |
| `robot_selection.py` | Ground-truth Panda state and measured nominal-motion features; replaceable later by other input providers. |
| `run_static.py` | Scoped adapter around the original FRAX demo/controller/CBFpy, recorded states, and independent diagnostics. |
| `run_suite.py` | Prepare inputs, run baselines, record one-shot selections, evaluate, and aggregate saved runs. |
| `static_gallery.py` | Render recorded trajectories and decisions without another model call or physics rollout. |
| `sim_model.py` | Explicit upstream/idealized matched plant profiles and independent native dynamics comparison. |
| `runtime_cbf.py` | Reuse FRAX/CBFpy with runtime sphere arrays, masks, time/velocity terms, and a sampled-control inequality guard. |
| `online.py` | Ground-truth snapshots, saved background requests, admissible policy installation, and control-loop hooks. |
| `run_showcase.py` / `showcase_gallery.py` | Frozen dynamic fixtures, off/all/once/repeated runs, independent audits, and synchronized recorded playback. |
| `pickup.py` | Native gripper/free-target task sequence, task-defined contact masks, current-state features, and pickup verification around the original arm controller. |
| `run_pickup.py` | Frozen static pickup experiments, source snapshots, six-method comparisons, safe completed-run resume, and layout preparation. |
| `api_budget.py` | Persistent dollar-cap reservations and reported-usage estimates around the existing Jev SDK. |
| `analyze_pickup.py` / `pickup_gallery.py` | Independent native audits, geometry costs, CPU profiling, recorded-answer diagnosis, and synchronized pickup review. |
| `perception.py` | Calibrated simulated RGB/depth, oracle-instance segmentation, depth-only clutter fits and accepted measurement updates; known target/robot retained. |
| `camera_io.py` / `realsense_camera.py` | Shared metric frames, explicit calibration, portable replay and worker I/O; optional SDK adapter tested with doubles only, not connected hardware. |
| `inspect_camera.py` | Offline RGB/depth/mask/geometry inspection and a separate unverified camera-only live entry point; no robot commands. |
| `pickup_layouts.py` / `study_pickup.py` | Geometry-qualified fixtures and frozen count/context, unseen-layout and camera/fault studies using the existing runner. |

This is infrastructure exercised by static pickup and earlier reaching fixtures, not a finished
perception-driven robot architecture. Editable scene/question JSON and ordinary function boundaries are
the substitution points; no speculative plugin framework is required.

When selection work begins, start with inspectable examples and easily editable
questions/orchestration. Review its outputs before committing to a taxonomy,
call structure, confidence policy, or refresh strategy.

### Engineer discretion and owner decisions

Normal file organization, helper functions, implementation mechanisms, and local
refactors can follow what is simple and natural. The example layout is not an
acceptance checklist. Avoid speculative abstractions and do not require user
approval for every routine coding detail.

Ask Nihar before consequential changes to research scope, safety/contact
assumptions, model authority, or upstream reuse. Record meaningful assumptions
and unresolved ambiguities. If an upstream interface is incompatible, explain
the mismatch and options rather than silently editing the dependency.

For each agreed step, keep its code, relevant settings, a short usage explanation,
and the evidence/status needed for review. Apply Section 11's checks as their
features are introduced, not as a demand to implement the final system first.
Nihar's latest authorization included the four pickup items and seven follow-on
items, with autonomous diagnosis/fixes and up to $7.50 total new Jev usage. That
software package is complete. The subsequent D18 authorization includes selection,
unseen static layouts, depth-derived clutter and imperfect-information/model
failure studies, plus richer sphere counts. The existing cumulative $7.50 cap
still applies. Hardware, frontier-provider comparisons, new push/publication and
unselected research extensions remain outside that scope. Report actual results
in PROGRESS.md; preserve failures rather than silently replacing old evidence.
