# Jev-selected CBFs: project architecture and planning reference

Status: planning document, not a running system or authorization to start building.
Updated 20 October 2026. This is the written reference for the project discussion;
the immediate priority is code design and architecture, not test runs or deployment.

The agreed direction is a **new standalone repository** that imports and calls
existing FRAX, OSCBF, and CBFpy implementations. It owns the task-conditioned
constraint-selection framework and small integration adapters, not replacement
robotics libraries. Leave the three original repositories unchanged.

The research idea added to the brief is to generate several geometrically valid,
conservative representations deterministically, then let Jev choose among their
IDs using the task and measured geometric/computational costs. Jev does not need
to generate geometry, Python code, or arbitrary CBF equations.

### How to use this document

- Sections 2, 3, and 10 define the proposed code layout, interfaces, and reuse plan.
- Sections 4-8 record geometry, selection, interaction, and update constraints that
  the implementation must account for; numerical settings remain proposals.
- Sections 1, 9, and 11 preserve future milestones and evaluation ideas. Experiment
  design, task thresholds, API setup, dependency installation, and test runs are
  deferred; their inclusion is not an instruction to execute them now.
- Section 13 distinguishes architectural choices from outstanding configuration
  choices and empirical questions. Do not silently turn a suggested default below
  into an agreed requirement or a measured result.
- Section 14 is the engineer handoff: the first coding deliverable, architecture
  baseline to follow, deferred work, and decisions requiring owner review.

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

Initial simplifications: simulation, known object identities/poses/bounds, known
robot state, and stationary scene obstacles. Panda/MuJoCo is the recommended first
integration because the supplied FRAX demo already uses it, not an architectural
restriction on every future robot or simulator. Actual sensor noise, independent
moving obstacles, arbitrary meshes, learned policies, and active perception are
extensions with explicit gates, not implicit features of the first result.

### Future milestones and task choice (provisional)

- M0: reproduce the original FRAX demo and verify robot/simulator agreement.
- M1: reach a prescribed end-effector pose through static clutter. Demonstrate live
  changes to constraints through task/selection updates without moving obstacles.
- M2, proposed first full manipulation demonstration: grasp a small box, lift it,
  move it to a supplied placement region, release, and retreat. Use scripted task
  phases and the same OSC controller, not a newly trained policy or an implicit
  global path planner.
- M3: replace ground-truth observations with RGB-D estimates; subsequently evaluate
  noise, active views, and moving scenes as separate extensions.

The user was asked whether the first full demonstration should instead be pushing
or reaching only, and which execution machine/API access are available. Pending
those preferences, Panda/MuJoCo and the milestones above are the proposed defaults.
The module contracts do not depend on that preference.

Illustrative M1 settings, to decide when designing runs: position error <= 0.02 m
and rotation error <= 5 degrees for 0.5 s;
episode timeout 15 s. M2 success requires the correct object, a verified lift of at
least 0.05 m, placement inside the supplied goal region, release, and 0.5 s of
settling. Report goal completion independently from safety, and also their joint
rate. These thresholds are configurable task proposals, not agreed acceptance
criteria or measured results.

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

Use ordinary functions and small dataclasses; no dependency-injection framework,
plugin registry, generic planning framework, or base class for every function.
The following is a proposed small layout, not files already implemented. Repository
and Python package names remain placeholders until repository creation is requested.

```text
<new-repository>/
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
Add further modules only when a concrete need warrants them. Lightweight recording
can initially live in `run.py`; a later evaluator can be split out when implemented.

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

Configuration has one explicit construction point in `run.py`, using small typed
records from `types.py`. Keep robot/task settings, geometry choices, model/prompt
versions, margins/gains, solver settings, capacity policy, and update/failure
settings visible there. Do not scatter magic constants or introduce a generic
configuration framework. Secrets come from the environment and are never logged.

## 3. Data and action contracts

All geometry uses metres, radians, seconds, and a right-handed world frame equal
to the initial fixed robot-base frame. `T_A_B` maps coordinates in B into A.
Store transforms as 4x4 matrices; SDK/serialization adapters must explicitly
convert quaternion conventions rather than infer their ordering.

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
after this mapping. A future learned policy must
implement this command contract or supply an explicit action adapter. A policy
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
and chosen precompiled capacity. No mutable registered-static config state.

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

Implement sphere covers first for M1 plumbing. Then add one ellipsoid candidate
before the representation-selection experiment. Initial candidate family:

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
optimal approximations of a mesh. Handle degenerate extents explicitly with a
documented positive minimum and reject nonfinite/negative dimensions.

Coverage is a hard candidate-admissibility condition in the ground-truth stage.
Estimate excess **union** volume against actual simulator geometry using shared,
seeded samples; summing overlapping primitive volumes would double-count overlap.
Report that estimate separately from the analytic coverage certificate. Compute
candidate features outside Jev, including primitive/pair counts and geometric
costs. Use the same samples across methods and record sampling uncertainty.

### Smooth barriers and robot-body size

For robot sphere centre p_i(q), radius r_i, obstacle sphere centre c_j and radius
R_j, use the dimensionless quadratic barrier:

`h_ij = ||p_i(q) - c_j||^2 / (r_i + R_j + delta)^2 - 1`.

Use one barrier per enabled pair, not hard min/max aggregation. Initial geometric
clearance margin proposal: `delta = 0.01 m`, to confirm before runs and then fix
across methods. Future perception-error
inflation is additional and must come from calibrated uncertainty, not Jev's
semantic confidence. Changing this margin is an explicit experiment parameter.

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

Reuse the demo's alpha/alpha_2 linear gains of 10 for initial comparisons and keep
them fixed. Primitive quadratics have continuous second derivatives for positive
sizes, but switching candidates is a separate hybrid-system event, not a smooth
operation. Warm up with a valid nonsingular robot configuration and EE rotation,
not an all-zero orientation or an unchecked singular pose.

## 5. Jev questions, selection, and semantic authority

Use the TypeSafe SDK and a pinned model ID, initially `jev-1.13.0` (documented
stable alias target at review time). Do not use a moving `jev-latest` alias in
recorded comparisons. Log the model actually returned. New model versions require
separate threshold validation. Do not install an SDK or make billed API requests
as part of this architecture review; access/setup remains an implementation gate.

One request contains narrow questions for each object: a role Choice, a candidate
Choice, and avoidance questions for named robot groups. Candidate descriptions
include their measured costs and current task-relative spatial context. Every
question is evaluated independently against shared state; geometry/pair questions
must be self-contained conditional questions, not assume they see the role answer.
Combine/gate results in code after receiving them.

Initial roles: target, avoid, support, unnecessary-for-current-task, uncertain.
Role does not grant contact permission. Jev proposes avoidance of robot groups
(upper arm, forearm, wrist/hand; refined palm/finger groups only after M2 modeling
is implemented). Code maps named groups to verified collision-sphere indices.
Avoid one API question per individual robot sphere in the first version.

Jev selects candidates directly for the main method. Numeric geometry features
remain deterministic; a separate baseline picks the lowest normalized excess
volume plus normalized pair-count cost, with equal weights as the initial declared
setting. Publish both axes and a weight sweep rather than claiming one universally
best tradeoff. The numerical baseline uses the same admissible candidates and
proposed avoidance masks as the corresponding geometry ablation.

Keep generation, CBF gains, numeric sizing, validation, and allowed-contact
authorization in code. Jev may propose a role or relevant robot part; it does not
modify a Python barrier function or authorize an arbitrary contact.

**Important limit:** validating geometric coverage of selected objects does not
prove that Jev selected every object/pair that needed protection. The simulation
benchmark must apply and evaluate its selections honestly, count false omissions,
and avoid claiming full-scene safety for an incomplete selected set. Do not add an
unreported oracle that repairs masks. A future deployment guard can override unsafe
omissions, but its overrides and its independent contribution must be evaluated.

For the proposed uncertainty protocol, retain/enable avoidance for uncertain
objects and pairs. A role label or confident geometry Choice alone never authorizes
dropping an avoidance pair. Require an explicit pair-omission decision with a
declared probability threshold and debounce across compatible updates. For example,
two responses with Noul omission probability >= 0.95 are a setting to evaluate,
not an agreed or calibrated robot-safety guarantee. Tune thresholds on a development
split, freeze for held-out evaluation, and report coverage/omission versus threshold.
Repeated responses are debounce, not independent evidence.
An invalid or ambiguous candidate selection retains the current valid candidate,
or uses the declared conservative baseline candidate if admission checks pass.

## 6. Interactions, grasping, and object motion

Permanent filtered-method protections: torque bounds, joint limits, self-collision,
floor/table clearance with explicit mounting/support exclusions, and the demo's
singularity avoidance. The latter also requires numerical checks near rank loss;
sphere/ellipsoid smoothness does not cure an unstable task-inertia inverse or SVD.
Do not silently remove fixed protections because Jev labels an object harmless.

For M1, no target contact is permitted; the goal is a free-space EE pose. Scripted
waypoints are shared across methods. An OSC safety filter is local and can stop
at a barrier; it is not expected to discover a route around every obstruction.
Record stalled/blocked episodes as such, not automatically as safety failures.

M2 task phases: approach, descend, close, lift, transport, lower, release, retreat.
The explicit task contact table permits fingertip-pad/target contact only during
the designated interaction phases, and target/table or target/placement-surface
contact where required. Palm, wrist, arm, unrelated objects, and payload/environment
pairs remain subject to the declared avoidance policy. Gripper motion/force limits
are controlled by the task/simulator adapter; the seven-torque arm QP does not
automatically constrain finger motors or guarantee grasp quality.

The current FRAX model merges hand/finger geometry into an arm-link collision
model. M2 therefore requires a small, separately validated palm/finger model or
conservative gripper swept envelope; disabling the whole hand's target constraints
and calling that fingertip-only contact is not acceptable. Map named simulator
geoms and FRAX sphere owners explicitly. The simulator already has a gripper
actuator, but the demo does not operate it as a grasping routine.

During verified rigid transport, represent the payload as collision primitives
attached to the EE through a measured frozen relative transform; their centres
are functions of q and can be differentiated by CBFpy. Check actual payload pose
for slip. Incorporate known payload mass/inertia in a prebuilt loaded robot-model
variant before claiming a matched dynamics model. Contact acquisition/release and
any unmodeled payload dynamics are empirically evaluated hybrid phases, not a
formal whole-task safety certificate. M2 is gated on demonstrating these additions.

Independent obstacle motion is disabled in M1/M2. Unexpected non-target movement
invalidates the static-scene assumption and is logged as a failed/aborted episode.
For moving scenes, extend the state/drift to include relevant obstacle motion or
an explicit time-dependent formulation, then validate velocity/acceleration and
latency assumptions. Replacing a centre passed as an extra argument each tick does
not by itself add the missing time derivatives to current CBFpy.

## 7. Timing, updates, and JAX shape management

Architectural choices here are separate control/selection loops, bounded work,
immutable snapshots, and fixed-shape numeric CBF arguments. Exact rates, timeouts,
expiry periods, and capacity buckets below are provisional run settings to decide
after compatibility checks and profiling, not architecture requirements.

Proposed operating targets: MuJoCo physics at 1000 Hz, arm control at 500 Hz,
selector requests up to 2 Hz, and viewer/rendering independently at <= 30 Hz.
These are initial budgets to benchmark, not existing performance measurements or
hard real-time guarantees. Run headless timed trials separately from GUI demos.

Use one background worker with at most one Jev request in flight and a latest-only
input slot. No unbounded queue. Start with a 1 s end-to-end request timeout and
SDK retries disabled; retry a fresh snapshot on the next scheduled opportunity.
Record HTTP errors/rate limits and response times. Model failure never blocks the
control thread. Read credentials from the environment and do not record keys.

Distinguish per-tick snapshot ID from scene epoch (object identity/topology or
geometry revision) and task epoch (instruction/phase/contact rules). Reject results
for obsolete epochs/candidate sets. Robot motion alone must not invalidate every
pending query; revalidate a returned proposal against current q/v and current poses.
Transforms are refreshed by the scene provider; a semantic result is not a cache
of stale world coordinates. Static-scene operation avoids pretending this also
solves the moving-obstacle derivative problem.

Before installing any change, verify IDs, dimensions, coverage, allowed contacts,
and finite QP data. Check current h >= 0 and HOCBF intermediate
`psi = L_f h + alpha_2(h) >= 0` for newly enabled/changed RD2 rows, then check the
trial filtered command and applied-command residuals. Reject unsafe transitions
and retain the last still-valid snapshot. A geometry switch can newly exclude
the current robot pose even when both candidates enclose the same object.

One update manager owns acceptance, rejection, expiry, fallback, and revision
numbering. The selector only proposes changes; the control loop reads one complete
accepted snapshot, never a half-updated list. Keep Python-side descriptive records
separate from the immutable numeric arrays passed to JAX.

Install snapshots atomically at control boundaries. Geometry changes and task
phase changes require immediate review; omissions/cost-only changes use the
debounce rule above. New observed objects start with baseline avoidance pending
classification. Expire semantic decisions after 2 s without a compatible refresh;
attempt the declared all-object fallback with current valid covers/contact rules.
If that fallback cannot pass admission checks, end the simulation episode as an
abort/failure. Do not claim that zero torque or an unvalidated hold is a safe stop.

Pack selected robot-sphere/obstacle-primitive pairs into numeric arrays passed as
runtime args to h_2; do not mutate registered-static config fields. Cache/prewarm
a bounded bank of capacity variants, each including the same mandatory rows.
A possible bank is 0 scene rows and powers of two from 32 up to the configured
scene's maximum possible pair count, rounded up; choose the actual buckets later.
Choose the smallest adequate prewarmed capacity; fill padding with finite,
constant satisfied barriers, never NaN/zero-axis placeholder ellipsoids. Keep pair
ordering deterministic. Capacity overflow is an explicit failure, not truncation.

Measure enabled rows, allocated QP rows, binding rows, and actual compute time.
Fewer enabled rows within the same capacity need not save time. No compilation,
network I/O, logging, rendering, or Python geometry fitting inside the timed jitted
control function. Record compile time separately and discard warmup from steady
latency statistics without hiding runtime recompilation events.

## 8. Solver behavior and numerical failure

The solver path and need for explicit diagnostics are architectural; exact
tolerances, penalties, and acceptance thresholds must be confirmed before runs.

Initial backend: the FRAX port's ElastiQP path through CBFpy, float64 CPU,
solver_tol 1e-5, inherited CBF relaxation penalty 1e3 and control penalty 1e5.
Keep weights/gains/normalization fixed across method comparisons. If installation
on the selected machine fails, test CBFpy's qpax backend as an explicitly recorded
alternative; do not silently switch versions/solvers during experiments.

ElastiQP relaxes constraints, including control bounds. Apply physical motor
limits regardless of backend, then evaluate the CBF residual for the command
actually applied. A clipped returned torque can invalidate a previously satisfied
CBF inequality. Record nominal, returned, requested-applied, and simulator actuator
torques where available. The initial arm motors have gear 1; verify their mapping.

The public CBFpy wrapper returns the control vector, not full solve diagnostics.
Add a thin instrumented adapter around the existing QP-data/solver path, preserving
its math and using backend status where available; do not write a new optimizer.
Record per-row `max(0, G_i*tau - b_i)` and the scaled residual obtained by dividing
by `1 + abs(b_i) + ||G_i||_1 * ||tau||_inf`. Initial unacceptable scaled residual:
1e-4, with raw residuals also published. Treat this as a declared numerical
acceptance threshold to validate, not a physical clearance guarantee.

Nonfinite data/controls, an invalid initial state, unacceptable residual, or missed
physical limits cause an explicit failed/aborted simulation episode with a saved
reason. Do not count aborted episodes as safe successes or keep advancing the
simulation with arbitrary replacement torques. Hardware stopping behavior is out
of scope of this simulation design and requires a separately validated controller.

## 9. Experiments and independent evaluation

Future evaluation plan only. No scenes, episode counts, numerical thresholds, or
test runs in this section are finalized by the architecture discussion.

Use small scenes with 3-8 objects: compact and elongated shapes, rotated objects,
and narrow passages. Task/target changes in the same scene test contextual
decisions. Keep a development split for thresholds/prompts and at least 30 seeded
held-out scene/task episodes per method for the initial report. Use identical
initial states, task waypoints, actuator limits, and contact rules across methods.
Record stochastic/model differences; a seed does not make a hosted API reproducible.

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

Evaluate unintended contacts, signed clearance, contact forces, task completion,
and wall-clock timing against actual simulator collision geoms, not only the fitted
CBF geometry. Initial scenes use analytic or decomposed convex collision geometry
so the evaluator does not confuse visual meshes with simulator collision hulls.
Inspect contacts at every physics substep; report numerical/contact tolerance and
sampling resolution. Discrete checks are not continuous-time collision proofs.

Metrics: collision-free episode rate; goal completion and safe goal completion;
minimum true clearance; omission of required object/body pairs; selected-cover
undercount separately from semantic omissions; raw excess union volume and
robot-expanded conservatism; primitive/enabled/allocated/binding counts; QP
residuals; selection changes/rejections/fallbacks; nominal-control deviation;
control and selector p50/p95/p99/max latency; missed deadlines; scene/proposal age;
recompilation count; API token use and cost. Publish numerators/denominators and
confidence intervals, not only one average score.

If every non-target object must be avoided, obstacle-role selection can be nearly
trivial. Include meaningful instruction/contact distinctions and a rule-based
selection baseline where needed. Do not infer that Jev adds value simply because
CBFs or deterministic fitted geometry help. Negative findings remain informative.

Log configurations, scene seeds, task/contact rules, snapshots, candidate features,
request/response payloads (without credentials), full probabilities, selected and
accepted constraint revisions, fallback/override reasons, actual actions, metrics,
machine details, source SHAs, dependency lock, and prompt/model versions. Provide
an offline replay selector for development/latency isolation; label it as replay,
not live Jev evaluation. A mock selector is plumbing only, not research evidence.

## 10. Reuse and dependency plan

Create the project as its own repository when implementation/repository creation
is requested. Reference the three upstream repositories through pinned git
submodules under `third_party/`; do not nest the new project inside an upstream
repository or copy their source trees into the new package. Pin commits and
record local modifications if a future upstream change is separately authorized.

All three supply runtime code, but they need not all be installed as complete
Python distributions. FRAX and CBFpy use normal package imports. Reuse OSCBF's
standalone `oscbf/core/controllers.py` from its pinned checkout through one explicit
source-module adapter. That module consumes JAX arrays and does not depend on the
older OSCBF robot model. This avoids installing OSCBF's legacy pinned JAX/CBFpy
stack, which conflicts with the newer FRAX examples. Source-only reuse is a
deliberate compatibility boundary, not a claim that the whole old stack is compatible.

Proposed environment: project-local Python 3.12, compatible JAX >= 0.9.1,
MuJoCo >= 3.3.1, ElastiQP, current checked-out FRAX/CBFpy, and the TypeSafe SDK.
Record these as compatibility requirements to check, not a resolved environment.
Lock exact versions only after a smoke test on the selected machine; do not
install anything as part of this documentation task.

Observed source revisions (20 Oct 2026):

| Repository | Local HEAD |
| --- | --- |
| FRAX | `cc0a066b5b539c729838308496ae1b8a1d6aeada` |
| CBFpy | `0ab24ecd431ca151a80cd7e4f2a5d3967e1c4b5a` |
| OSCBF | `6082329f8e1d61ee8e875a63a5cd80a14d513e12` |

Specific adapters, confined to `upstream.py`:

1. Import FRAX's `examples/cbf_utils.py::OSCBFTorqueConfig` from its pinned source
   path. It already uses FRAX's `Manipulator`; subclass it in the new `control.py`
   to supply the dynamic scene barriers. Its dynamics and task-consistent QP
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

Before research runs:

1. Match FRAX/MuJoCo EE transforms, joint order, torque limits, mass/gravity terms,
   and robot collision sphere ownership at valid configurations.
2. Check analytic candidate coverage and finite first/second barrier derivatives;
   compare smooth sphere/ellipsoid derivatives against numerical directional checks.
3. Verify an obstacle/pair parameter update changes the QP without recompiling;
   verify capacity switches only use prewarmed variants and preserve row identity.
4. Verify h/psi admission, stale-epoch rejection, uncertain results, API failure,
   timeout, capacity overflow, saturation, and unacceptable-residual paths.
5. Complete baseline reaching in clutter and an online task/selection change.
6. Gate M2 on gripper/palm geometry, contact permissions, grasp verification,
   payload collision/dynamics handling, and an independent physical contact check.
7. Gate camera-based and moving-obstacle stages on their own geometry/calibration
   and dynamics checks; do not silently claim these from the ground-truth study.

Executed during this review, using available NumPy only: 100,800 interior/corner
points across 100 boxes passed coverage probes for the 1/2/4-sphere family and
enclosing ellipsoid; 100,000 support-direction probes passed outer-ellipsoid
inflation checks. An explicit counterexample showed naive axis-plus-radius
inflation under-covering. Analytic containment arguments are above; sampling is
an implementation sanity check, not a replacement for those arguments.

The current system interpreter is Python 3.12.7 on arm64 macOS with NumPy 1.26.4.
JAX, MuJoCo, qpax, ElastiQP, and typesafe-sdk are not installed in that interpreter;
no project virtualenv was found in this workspace. Therefore no controller,
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

## 13. Decision register and follow-through

### Requirements and proposed architecture

| Topic | Status / direction |
| --- | --- |
| Repository ownership | User requirement: independent project repo; import/call the three originals rather than recreate them. |
| Initial inputs | User requirement: ground-truth objects and robot kinematics are allowed; include dimensions/bounds so geometry is computable. |
| Initial primitives | User requirement: spheres/ellipsoids, with multiple primitives allowed per object. |
| Generation versus selection | Project idea: deterministic conservative candidate generation; Jev selects candidate IDs, roles, and robot groups. |
| Nominal controller | Reuse required; proposed integration is OSCBF's existing pose controller with FRAX matrices, matching the FRAX demo's computation. |
| CBF backend | User requirement: CBFpy; reuse FRAX's compatible OSCBF configuration rather than the older incompatible robot/config pair. |
| Dependency boundary | Proposed design: pinned submodules, normal FRAX/CBFpy imports, two centralized source-module adapters; no upstream edits. |
| New-code organization | Proposed design: seven small modules and shared dataclasses; substitute functions/adapters without a plugin framework. |
| Decision authority | Proposed design: selector proposes, one validator/update manager accepts, control consumes immutable numeric snapshots. |
| Runtime separation | Proposed design: independent bounded selection worker and control loop; no model/network calls or geometry fitting inside JIT. |
| Contact permission | Explicit task/phase and body-pair rules; a target label is never a global collision exemption. |
| Failure behavior | Explicit rejection/fallback/abort with reasons; no silent barrier removal or unvalidated replacement torque. |

These choices make the implementation direction explicit. Rows marked proposed
are recommendations for the initial build, not a claim that every detail has
already been approved or tested. Revisit them deliberately if upstream interfaces
or the eventual task make them unsuitable.

### Decisions deliberately deferred

- Repository/package name and GitHub visibility, at repository-creation time.
- First exact task, scene assets, success thresholds, and grasping scope; reaching
  is the proposed integration milestone, not the final manipulation claim.
- Execution machine, Jev credentials/access, SDK version, and exact dependency
  lock; none is needed to write the architecture, all are needed before runs.
- Control/selection rates, API timeout/retry/expiry values, capacity buckets,
  probability/debounce thresholds, and numerical acceptance tolerances.
- Final candidate family, cost normalization/weights, robot-group mapping, and
  prompt/question wording; initial proposals are recorded above for review.
- Detailed gripper/payload modeling, camera calibration/noise treatment, moving
  obstacles, and any real-robot stopping or deployment policy.
- Benchmark sizes and statistical reporting details. Performance, Jev selection
  quality, conservatism/safety tradeoffs, and novelty need evidence, not a design
  decision declaring them solved.

### Extension seams to preserve, not implement now

For RGB-D, the scene provider should handle calibrated depth back-projection,
segmentation/tracking, metric object bounds, coordinate transforms, and uncertainty.
It supplies the same scene contract as ground truth. RGB images or a visualized
depth image may provide Jev with context, but are not a replacement for metric
geometry processing. A ground-truth coverage certificate does not automatically
transfer to an incomplete point cloud or an occluded object.

For active perception, retain observation timestamps, object revisions, optional
images/uncertainty, and camera transforms. A future task/policy extension can request
another viewpoint when geometry is inadequate. This requires explicit observation
actions and safe motion; merely updating a constraint list is not active perception.
Do not introduce an agent/planner framework before that capability is requested.

For learned policies, preserve the action adapter and filter boundary. For richer
primitives, preserve candidate generation/validation separately from selection.
For moving obstacles, extend the dynamics/time-derivative model explicitly rather
than only changing the perception implementation.

### Intended build order, once implementation is requested

1. Create the independent repository and pinned dependency references; establish
   the single upstream-adapter boundary without changing the originals.
2. Define the shared records, configuration construction, and replaceable provider,
   task, policy, selector, and control signatures.
3. Connect existing FRAX/OSCBF/CBFpy implementations through the adapter, keeping
   upstream dynamics, nominal-controller math, and optimization in their owners.
4. Add conservative candidate generation and a deterministic/replay selector for
   plumbing, then the interchangeable Jev selector. Mock output is not Jev evidence.
5. Add proposal validation, immutable constraint snapshots, bounded update ownership,
   and explicit diagnostics/failure reporting.
6. Only then proceed to the separately agreed compatibility checks, simulation
   runs, and evaluation gates in Sections 9 and 11.

Current deliverable: this planning document. No repository creation, dependency
installation, implementation, billed model calls, or new test runs are authorized
by the documentation request itself.

## 14. Engineer handoff: initial implementation contract

Use this section as the bounded coding brief when the project owner explicitly
commissions implementation. It sets the initial engineering baseline; it does not
authorize coding, GitHub publication, model spending, or runs merely because this
document was shared. Earlier proposals outside this baseline remain proposals.

### First deliverable

Build the standalone framework for **ground-truth, static-scene reaching with the
Panda/MuJoCo integration**, before adding grasping or camera-based perception.
This is an integration milestone toward the manipulation brief, not its full
completion. The code should connect:

```text
ground-truth scene + task -> candidate geometry -> selector proposal
    -> validation/update manager -> accepted constraint snapshot
scripted EE command -> existing OSC -> nominal torque -> CBFpy -> simulator adapter
```

Include the contracts from Section 3, the candidate family from Section 4 (implement
sphere covers first, then the enclosing ellipsoid), a deterministic/replay selector,
and an interchangeable Jev adapter. Keep live Jev optional so API access is not
required to assemble the framework; never silently substitute mock output for Jev.
Support explicit all-off/all-on/adaptive modes for later evaluation, with physical
actuator limits in every mode. This deliverable is source integration, not a
completed benchmark or evidence of safe manipulation.

### Architecture baseline to follow

- Create an independent package using the small layout in Section 2. Reference
  pinned upstream checkouts as in Section 10; do not copy implementations or edit
  the original three repositories. Confirm repository name/visibility before
  creation or publication rather than inventing them.
- Reuse FRAX robot computations and its adapted `OSCBFTorqueConfig`, OSCBF's
  standalone pose controller, and CBFpy's differentiation/QP/solver path. Keep
  source-loading compatibility code in `upstream.py`. Do not add a second robot
  model, controller implementation, or optimizer.
- Generate numeric geometry deterministically; Jev selects valid candidate IDs
  and avoidance groups. Keep CBF gains fixed initially and contact authorization
  in explicit task rules, not model-generated code or unrestricted model output.
- Separate policy, scene/task providers, geometry generation, selection, and
  filtering through the documented records/functions. Use one owner for accepted
  constraints, immutable snapshots, and a bounded background selector. No network
  calls or changing Python lists inside the jitted control calculation.
- Implement explicit rejection/fallback/failure reporting. Never silently discard
  excess constraints, disable protections, change solvers, or apply an unvalidated
  replacement torque to continue an episode.

### Deferred from this deliverable

Camera detection/RGB-D reconstruction, active perception, learned policies,
grasping/contact transitions, finger/payload modeling, independent moving obstacles,
real-robot deployment, benchmark runs, and tuning are later work. Preserve extension
interfaces; do not implement these speculatively. Future camera observations must
distinguish wrist and scene views, including each view's timestamp, frame, calibrated
intrinsics, and camera-to-world transform; reserve this contract without building
the perception stack now.

### Engineer discretion versus owner review

The engineer may choose local variable/helper names, private function organization,
serialization details, and ordinary packaging mechanics consistent with the stated
contracts. Keep settings centralized and document assumptions.

Bring back changes to research scope, public data/action semantics, units/frames,
upstream reuse strategy, primitives/coverage rules, Jev authority, contact permissions,
mandatory protections, failure behavior, or dependency/backend strategy. If an
upstream adapter is incompatible, report the exact interface mismatch and proposed
alternative; do not quietly replace it with copied code or another library.

Numerical rates, timeouts, confidence thresholds, margins, capacity limits, and
solver acceptance thresholds remain configurable and unvalidated. Earlier examples
must not become hidden requirements. Missing run-critical settings should be
reported explicitly before execution, not filled with undocumented guesses.

### Definition of done for the coding handoff

Deliver the source package, explicit dependency references, configuration schema,
and documented entry point assembling the full initial pipeline. Include a short
README describing setup, data flow, selector/mode selection, swap points, unresolved
compatibility issues, and which settings still need confirmation before runs.
Record any deviations from this contract. No placeholders may be presented as
completed Jev, geometry, or CBF integrations.

Code delivery and runtime acceptance are separate checkpoints. The verification
gates in Section 11 still apply before research runs; unexecuted integrations must
be labeled unverified. Installing dependencies, using billed APIs, and running
simulations/benchmarks remain subject to the separately agreed execution scope.
