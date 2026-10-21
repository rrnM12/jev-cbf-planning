# Jev-selected CBFs

This repository adds task-conditioned Jev constraint selection around the original
FRAX/CBFpy controller, without editing the three upstream repositories. Reusable
infrastructure now prepares actual-Panda scene/motion context, generates sphere
cover candidates, records Jev choices, runs comparisons, and renders review
galleries. The current proof of concept performs physical static-clutter pickup:
fresh Jev answers can add/update/remove enabled constraints while the original
arm controller keeps running. Perception and hardware are not implemented.
Older isolated geometry and proxy-selection tools remain available. The larger,
evolving research plan is in [ARCHITECTURE.md](ARCHITECTURE.md).
Current progress, verification, blockers, and decision history are in
[PROGRESS.md](PROGRESS.md); repository working rules are in [AGENTS.md](AGENTS.md).

## Start here: static-clutter pickup

```bash
open artifacts/pickup/review/index.html
```

One task family: maneuver through fixed clutter, pick up the designated target,
then stop. The review links original failures, independent audits, geometry and
prompt studies, and a six-method corrected replication on 12 static layouts.

| Corrected replication | Safe pickups | Mean enabled scene pairs |
| --- | --- | --- |
| Scene CBFs off | 11/12 | 0 |
| All-on | 12/12 | 296.0 |
| Geometric rule (no model) | 12/12 | 151.7 |
| All objects / Jev geometry | 6/12 | 172.8 |
| Jev once | 6/12 | 146.2 |
| Jev repeated | 12/12 | 156.9 |

Repeated queries correct coarse initial covers that stall one-shot runs. The
simple rule remains stronger on geometric costs; Jev superiority is not shown.
The original held-out evaluation and engineering failures are preserved at
`artifacts/pickup/heldout_v1`. The corrected replication uses the same layouts
after decoder/grasp-sequence fixes, not a new unseen benchmark. The first
three-layout gallery remains at `artifacts/pickup/showcase_v1`.

The gripper is articulated and the target is a free rigid body, with no weld or
scripted lift. Success checks the target's lowest point and bilateral finger
contact. Only finger-target contact is allowed by the native evaluator. The
seven-DOF FRAX dynamics omit finger/contact/payload effects; this is empirical
simulation evidence, not a contact-safety or real-time guarantee. The task aligns
above the target, backs off lateral descent drift, and permits lifting only after
closing produces bilateral contact. Target/body masks are task-defined, not learned. Fewer
enabled constraints do not shrink the allocated 348 scene-pair slots.

Rebuild playback/status without API calls or new physics:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B run_pickup.py gallery --output artifacts/pickup/corrected_replication_v2
uv run --offline --frozen --cache-dir .uv-cache python -B run_pickup.py status --output artifacts/pickup/corrected_replication_v2
uv run --offline --frozen --cache-dir .uv-cache python -B analyze_pickup.py audit --output artifacts/pickup/corrected_replication_v2
uv run --offline --frozen --cache-dir .uv-cache python -B analyze_pickup.py report --output artifacts/pickup/corrected_replication_v2
```

For a deliberately new experiment, choose a new output directory:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B run_pickup.py prepare --scenes examples/pickup_review_scenes.json --output artifacts/pickup/my_run
uv run --offline --frozen --cache-dir .uv-cache python -B run_pickup.py baselines --output artifacts/pickup/my_run
# This command spends API usage; only run for an authorized experiment:
uv run --frozen --extra jev --env-file .env.local --cache-dir .uv-cache python -B run_pickup.py run --output artifacts/pickup/my_run --live
uv run --offline --frozen --cache-dir .uv-cache python -B run_pickup.py gallery --output artifacts/pickup/my_run
```

The three-layout `examples/pickup_scenes.json` includes a retained blocked-posture
case; `examples/pickup_review_scenes.json` versions the 12 review layouts.
Questions are in `prompts/pickup_selection.json`; settings are in `pickup.DEFAULTS`
and optional scene-JSON overrides. `prepare` snapshots inputs and local sources;
changed sources require a new run before new physics/model calls. Completed-run
viewing/resume does not make new calls. Offline preparation does not require the
optional Jev SDK.

The approved $7.50 budget is tracked conservatively in ignored
`artifacts/usage/pickup_research_v1.json`. This work used 114 logical requests:
estimated token cost $0.067044516, conservative reservations $0.941359104.
Credentials and generated artifacts are not committed; the gallery path exists
on the experiment machine, not automatically in a fresh clone.

Final checks: 89 offline tests; independent native audit of all 72 corrected
rollouts; playback controls checked for all 12 layouts; 908 local links checked;
completed-run resume makes no new requests or rollouts. The handoff record is
`artifacts/pickup/handoff_checks_v1.json`. Source, tests, editable fixtures and
documentation are versioned; generated evidence and credentials remain local.

Recorded-answer diagnostic replay makes new physics, but no API requests:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B analyze_pickup.py replay --output artifacts/pickup/heldout_v1 --case layout_02 --task-protocol archived --destination artifacts/pickup/new_decoder_replay
```

`archived` uses hash-checked saved task code with the current decoder; `current`
uses the current task sequence. After controls change, old query context may no
longer match: replay is not fresh model-performance evaluation.

The optional geometry-only comparison adds shape-aware sphere candidates:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B inspect_geometry.py --shape-aware --no-view
```

## Historical dynamic simulation showcase

The recorded 21 October run is at:

```bash
open artifacts/showcase/dynamic_v1/index.html
```

Open **“One obstacle leaves while another approaches”** first. Four synchronized
panels compare scene CBFs off, all-on, Jev queried once, and Jev queried repeatedly.
Pause/scrub around 4–8 seconds: repeated selection adds `object_b` at 4.17 s, while
one-shot selection leaves it unconstrained and registers contact. Orange spheres
are enabled object covers; cyan objects are physical; red is the goal. Per-frame
goal error, physical clearance, sphere clearance, and enabled pair counts update
with the slider. Links expose every saved request/answer and diagnostic.

Recorded outcomes (two fixtures, not a statistical benchmark):

| Fixture | Off | All-on | Jev once | Jev repeated |
| --- | --- | --- | --- | --- |
| Distant object approaches/leaves | Contact | Pass | Pass | Pass |
| Obstacle handoff | Contact | Pass | Contact | Pass |

“Pass” requires a complete run, final goal error <20 mm, no sampled native scene
contact/nonpositive distance, and no negative modeled h2. All eight rollouts
completed 6,000 steps / 12 simulated seconds. Repeated selection's mean enabled
scene pairs were 84 versus all-on's 168, and 138.8 versus 252. **Allocated QP size
does not shrink**: dormant slots keep array shapes fixed. No speedup is claimed.
Jev retained previously selected obstacles after they moved away; effective
late-stage dropping is not demonstrated by these live results, although removal
is supported and tested. No prompt/scene tuning followed the answers.

### Reuse and reproduction

`online.py` supplies measured snapshots, a background selector, and runtime
parameters. `runtime_cbf.py` delegates barrier/dynamics/objective/solver work to
FRAX/CBFpy, adding time/velocity terms and masks. `run_static.py` still executes
the original nested nominal policy; filtering is outside its JIT so new arrays
actually take effect. `run_showcase.py` coordinates phases; `showcase_gallery.py`
renders recorded states into synchronized playback. Scenes/questions are plain
JSON in `examples/dynamic_scenes.json` and `prompts/dynamic_selection.json`.

View/status/refresh existing results without a key, network request, or rollout:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B run_showcase.py status
uv run --offline --frozen --cache-dir .uv-cache python -B run_showcase.py gallery
```

For an intentionally new experiment, use a new directory. Preparation and
baselines are offline; `run --live` uses the configured local key and the SDK's
ordinary retry/timeout behavior. The default two-fixture setup plans at most 12
logical invocations, sharing each initial answer across once/repeated. Each
snapshot is saved before its one logical attempt; failures are retained, never
re-queried to obtain preferred answers. A completed directory skips recorded
rollouts/calls. A fresh clone needs the sibling sources/environment plus either
the ignored result bundle or a deliberately new experiment; keys are never shared.

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_showcase.py prepare --output artifacts/showcase/NEW_RUN
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_showcase.py baselines --output artifacts/showcase/NEW_RUN
uv run --offline --frozen --extra jev --cache-dir .uv-cache --env-file .env.local python -B -u run_showcase.py run --live --output artifacts/showcase/NEW_RUN
```

### Explicit modeling choices and limitations

- `sim_model.py` builds an **idealized FRAX-matched plant in memory**. The original
  XML had different tool inertia plus damping/rotor inertia absent from FRAX.
  Native mass/bias/acceleration parity is checked independently; fingers are rigid
  and closed. This is not a calibrated physical Panda. The `upstream` profile and
  original results remain unchanged and available.
- A 0.05 m/s² tightening of scene acceleration inequalities addresses remaining
  sampled-control error in these checks. Geometry and alpha gains are unchanged.
  It is an empirical numerical allowance, not a calibrated uncertainty margin or
  a formal robust-CBF bound. The six-scene correction replay has 12 constrained
  runs with nonnegative sampled barriers; four conservatively blocked goals remain.
- Physical objects follow prescribed mocap paths with piecewise constant velocity.
  Explicit time makes CBFpy include obstacle velocity in RD2 derivatives. Future
  scripts/titles are not passed to Jev. Velocity jumps, arbitrary latency, model
  mismatch, perception errors, and infeasibility are not formally certified.
- Requests use current oracle state and a labeled heuristic joint-interpolation
  forecast, not vision or a simulated future policy rollout. All 21 FRAX moving
  spheres are paired with selected primitives; Jev does not choose robot bodies.
  Independent native checks include all 23 robot collision geoms and every object.
- Failed/review/inadmissible updates retain previous choices, initially all-on.
  This is explicit prototype behavior, not a certified safe fallback. Safety of
  semantic omissions is not established by response-schema validation.

No new runtime dependencies were needed. All implementation remains local and
unpublished; upstream repositories are read-only. See PROGRESS.md for the full
diagnosis, retained failures/audits, tests, and decision history.

## Prior checkpoint: robot-aware static workflow

The reusable code is separate from its first evaluation: `robot_selection.py`
builds actual robot/motion features, `run_static.py` adapts the unchanged FRAX
demo, `run_suite.py` coordinates saved phases, and `static_gallery.py` renders
recorded trajectories. `inspect_selection.py` supplies shared geometry requests,
SDK access, and validation. No controller or CBF equations are reimplemented.

On this machine, inspect the completed run without a key, network request, or
new simulation:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B run_suite.py status
open artifacts/suite/panda_static_v2/index.html
```

Click a scene name for off/all/Jev animations, object IDs, saved choices and
probabilities, physical-contact diagnostics, and modeled barriers. All six calls
authorized for this checkpoint are already spent; a new experiment needs agreement.
Reports and animations are ignored local artifacts, so a fresh clone needs a copy
of this run or a separately authorized new run. Credentials are never included.

The six variations use one no-contact Panda reaching task, not six independent
tasks. Off retains robot protections and physical collisions. All-on uses four
spheres per object independent of Jev; Jev selects objects and their generated
1/2/4-sphere covers. This compares combined role/geometry selection, not role alone.

Results: off has sampled contacts in three cases; all-on and Jev have none. Jev
uses 33–75% fewer scene pairs, but both constrained methods succeed in only two
of six cases. Four stop short and have negative modeled scene barriers (about
0.4–2.8 mm). A read-only probe observes differing FRAX/MuJoCo accelerations at the
same states/controls, without establishing a unique cause. No controller, gains,
solver, or simulator settings were changed to hide these issues. These are
exploratory observations, not a safety, superiority, or real-time guarantee.

### Rerunning or changing a future experiment

Scene geometry is editable in `examples/robot_scenes.json`; question wording is in
`prompts/robot_selection.json`. The current provider supports axis-aligned boxes/
Z-cylinders, default Panda joints, and one shared goal. Ground-truth input and
nominal motion features are ordinary replaceable functions, not vision outputs.

After agreeing on a new experiment, use a new directory and run the phases below.
`prepare` freezes inputs/requests; `baselines` makes no model calls. Only `select`
needs the optional SDK/key and explicit `--live`. The attempt ledger is written
before each invocation and prevents re-querying the same scene on resume; ordinary
SDK retries still apply. Failures are saved, never replaced with guessed answers.

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_suite.py prepare --output artifacts/suite/NEW_RUN
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_suite.py baselines --output artifacts/suite/NEW_RUN
uv run --offline --frozen --extra jev --cache-dir .uv-cache --env-file .env.local python -B -u run_suite.py select --live --output artifacts/suite/NEW_RUN
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_suite.py evaluate --output artifacts/suite/NEW_RUN
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_suite.py gallery --output artifacts/suite/NEW_RUN
```

`run --live` combines these phases; it is not permission to start another research
experiment automatically. Reusing a completed directory skips recorded attempts
and rollouts, while changed frozen inputs are rejected. `gallery --html-only`
refreshes report text/tables using existing images without rerendering, stepping
physics, or calling Jev. Individual rendering settings are exposed by
`static_gallery.py --help`. Generated reports and trajectories remain local/ignored.

## Setup

Use the existing sibling checkouts:

```text
ASL_Lab_Research/
  frax/                 # Read-only
  cbfpy/                # Read-only
  oscbf/                # Read-only; not required by this particular FRAX demo
  jev-cbf-planning/     # This repository and its isolated environment
```

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) if needed
(`brew install uv` is an option on this Mac). Then, from this repository:

```bash
uv sync --cache-dir .uv-cache --python 3.12
uv run --cache-dir .uv-cache python run_demo.py --check
uv run --cache-dir .uv-cache python run_demo.py
```

The launcher automatically uses MuJoCo's `mjpython` on macOS, as required for its
[passive viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer).
After the original demo prints “Demo ready,” double-click its target and
right-click/drag to move it. Close the viewer to finish.

The upstream `--robot` option is preserved (default: Panda). If the checkouts live
elsewhere, supply `--frax-dir /path/to/frax --cbfpy-dir /path/to/cbfpy`.

## What the launcher changes

It checks source paths and runtime-package presence, then executes the original
`frax/examples/oscbf_demo.py`. A child-process-only `PYTHONPATH` points to the local
FRAX and CBFpy checkouts. This avoids running package builds/editable installs in
those read-only directories. Bytecode writes are disabled for the demo process,
and its working directory stays here. No controller math, barrier functions,
robot models, or simulator assets are copied or modified.

`pyproject.toml` lists the external runtime requirements from those checkouts and
the demo. It intentionally does not install the older OSCBF package dependency
stack or the unrelated vision/visualization extras. The environment and uv cache
are local/ignored. Once dependencies resolve, keep the generated `uv.lock` in Git.

## Verification

Run the suite in the project's installed environment:

```bash
uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B -m unittest discover -s tests -v
python3 -B run_demo.py --help
```

All 71 tests pass, covering launch/setup, geometry, SDK contracts, runtime
updates/admission/failure handling, native dynamics parity, time derivatives,
report provenance, and sampled protection/frame parity—not general robot safety.
The seven launcher-only tests also run without dependencies using
`python3 -B -m unittest discover -s tests -p test_run_demo.py -v`.
`run_demo.py --check` verifies presence only. For a bounded run of the actual Panda
demo on macOS:

```bash
uv run --offline --frozen --cache-dir .uv-cache mjpython -B -u tests/smoke_demo.py --steps 1000
```

The real viewer opens; leave its default target unchanged. The check calls the
upstream `main()` and its original nominal controller and safety filter. Test-only
wrappers record states/torques, retain the actual CBF configuration, and stop the
run through its existing cleanup path. No controller, gains, geometry, or barrier
equations are replaced. The viewer closes automatically, and the command prints a
`BASELINE_RESULT=` JSON summary. Change `--steps` to adjust the run length.

The verified 1,000-step run covers 2 simulated seconds. Position error fell from
0.154486 m to 0.003087 m; states, torques, and all 64 model barriers stayed finite.
The sampled model barriers remained positive, with no measured torque-limit
excess. Detailed values are in PROGRESS.md. This default-target smoke check is not
an independent collision evaluation, a safety guarantee, or a stress test of CBF
intervention. Interactive dragging and richer scenes are separate checks.

The offline/frozen flags avoid network resolution and lockfile updates after
setup. The earlier network/GUI failures were session-permission issues, not uv
failures. The agent now runs the tools and viewer directly under the active
permissions; manual Terminal handoffs are not the normal workflow. If a future
session is restricted, use its supported approval mechanism rather than bypass
the restriction. No hardware, cameras, or Jev API key are needed for this step.

## Inspect sphere-cover candidates

Start with a cylinder standing in for a bottle (7 cm diameter, 30 cm high):

```bash
uv run --offline --frozen --cache-dir .uv-cache mjpython -B inspect_geometry.py
```

The viewer shows the 1-, 2-, and 4-sphere candidates side by side, left to right.
Grey is the reference object; transparent colours show the sphere unions. Orbit
and zoom to inspect. In the viewer's geom-group controls, group 0 contains the
objects and group 1 the covers, so either can be hidden. Close the window to exit;
`--seconds 5` is an optional bounded preview. Nothing is connected to the robot.

For numeric results without a window, optionally saving a JSON report:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B inspect_geometry.py --no-view --report artifacts/geometry/cylinder.json
uv run --offline --frozen --cache-dir .uv-cache python -B inspect_geometry.py --no-view --shape box --size 0.08 0.08 0.08 --report artifacts/geometry/box.json
```

Change `--size X Y Z` (full dimensions in metres), `--counts 1 2 4`, `--samples`,
or `--seed` to rerun. A cylinder is Z-aligned and requires equal X/Y dimensions.
Reports store the parameters, candidate IDs, object-local centres/radii, sampling
domain, and measurements. Generated reports under `artifacts/` are ignored by Git.
No dependencies were added for this experiment.

`geometry.py` splits the known object-local bounding box along its longest axis
and circumscribes each equal section with a sphere. Every section is contained in
its sphere, so their **union** covers the whole box, not just sampled surface
points. This is an analytic construction in exact arithmetic, not a formal
floating-point certificate. Zero/nonfinite/negative dimensions are rejected;
one-ULP outward radius rounding is numerical padding, not a clearance margin.
The current inputs are oracle bounds, not perception or fitted meshes.

The evaluator uses the same seeded random points for every candidate to estimate
union volume and volume outside the exact reference box/cylinder. Overlap is
counted once. Reported standard errors describe Monte Carlo sampling noise only.
Zero sampled misses are a diagnostic, not the proof of coverage. The viewer uses
those same reference dimensions but performs no physics or contact evaluation.

With 100,000 points and seed 0, estimated excess volumes were:

| Reference | 1 sphere | 2 spheres | 4 spheres |
| --- | --- | --- | --- |
| Cylinder, 7 × 7 × 30 cm bounds | 15.423 ± 0.055 L | 4.770 ± 0.038 L | 2.210 ± 0.027 L |
| Cube, 8 × 8 × 8 cm | 0.878 ± 0.005 L | 0.835 ± 0.005 L | 0.905 ± 0.005 L |

The ± values are one estimated standard error, not confidence bounds or geometric
uncertainty. More spheres help the elongated example but do not monotonically
improve the compact one. These are deliberately simple, sometimes loose covers,
not optimal shape fits. Primitive count is a constraint-cost proxy; QP runtime,
robot-expanded keep-out volume, margins, ellipsoids, and Jev selection are untested.

## Inspect a tiny Jev selection request

The original standalone experiment is deliberately separate from the robot. Its inputs
are [a synthetic scene](examples/selection_scene.json) and
[editable questions](prompts/selection.json), not camera detections. The scene has
a bottle-shaped cylinder, a nearby cube, and a distant cube. A spherical
end-effector proxy follows a nominal straight segment; this is neither an actual
FRAX rollout nor a whole-arm collision model. No object contact is authorized in
this starting fixture. This choice is an inspectable starting case, not the final
manipulation task or contact policy.

Preview the actual request without sending it anywhere:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B inspect_selection.py --output artifacts/selection/request_preview.json
```

This reuses `geometry.py`, computes the existing 1/2/4-sphere candidates and their
excess-volume estimates, and adds each cover's clearance from the nominal swept
proxy. A negative clearance means those modeled volumes overlap, not necessarily
that the actual object collides: a cover may block empty space. No extra clearance
margin is introduced. Candidate counts are only a cost proxy, not QP timings.

The generated request contains two Choice questions per object:

- A proposed role: `avoid`, `omit`, or `review` for this proxy and task phase.
- A **conditional** geometry choice: assuming the object needs avoidance, choose
  one of its generated candidate IDs. It cannot depend on the role answer from
  the same request because Jev answers questions independently.

These are starting questions to iterate on, not a settled prompt taxonomy.
Change wording in `prompts/selection.json`, facts/task/positions in
`examples/selection_scene.json`, or pass `--scene`, `--questions`, `--task`,
`--counts`, `--samples`, `--seed`, and `--model`. `--task` changes text only; it
does **not** move numeric goals or objects. Object rotations are not supported by
this axis-aligned starting fixture. No expected avoidance labels are supplied.

For a live call, install the optional official SDK:

```bash
uv sync --extra jev --cache-dir .uv-cache
```

Configure `TYPESAFE_API_KEY` locally, not in chat or source. An ignored
`.env.local` file can contain it; if using one, the invocation is:

```bash
uv run --offline --frozen --extra jev --cache-dir .uv-cache --env-file .env.local python -B inspect_selection.py --live --output artifacts/selection/live.json
```

Omit `--env-file` if the key is already in the process environment. `--offline`
controls uv package resolution; `--live` still makes a network call to TypeSafe.
One SDK invocation is made; ordinary SDK retries/timeouts apply, not a bespoke
real-time scheduling policy. The default is the SDK's documented stable alias,
`jev-latest`; the actual response model ID is recorded. See the official
[Choice API](https://docs.typesafe.ai/primitives/choice),
[SDK client](https://docs.typesafe.ai/sdk/python/api/clients/sync), and
[model documentation](https://docs.typesafe.ai/models).

The report records the complete generated request, geometry sampling settings,
and, for a successful live invocation, the SDK response, token usage, wall time,
probabilities/confidence, and decoded proposals. Question/option IDs and numeric
distributions are checked. An `omit` or `review` answer has no selected avoidance
geometry; its conditional geometry answer is retained for inspection. There are
no automatic confidence cutoffs, oracle repairs, or fallback decisions. A failed
call yields no proposal; an invalid returned selection is recorded as a failure.
`proposal_only: true` describes that standalone generation step. The separate
static simulation runner below explicitly replays saved answers without changing
the original report; the flag is not permission to deploy them to hardware.

The first live request to `jev-latest` returned `jev-1.13.0` in 0.629 s (one SDK
invocation, 6,021 input / 277 output tokens). It proposed avoiding the bottle and
nearby cube with four spheres each, and omitting the far cube for this proxy.
The cube geometry choice was uncertain: P(4 spheres)=0.46 versus P(2 spheres)=0.37,
with reported confidence 0.19. Four spheres have slightly better nominal-segment
clearance but worse global excess volume and higher primitive count for this
cube; this is a tradeoff to investigate, not an optimality result or an explanation
generated by Jev. Full initial input/output is stored locally in
`artifacts/selection/live_initial.json`. This single case proves neither useful
robotics performance nor real-time safety; no QP or robot control ran here.

The TypeSafe SDK 0.7.5 also passes fake-HTTP-transport integration checks with
fabricated test responses. Those are separate from the live observation above.
Run the offline suite including that optional adapter check with:

```bash
uv run --offline --frozen --extra jev --cache-dir .uv-cache python -B -m unittest discover -s tests -v
```

## View a saved selection in its scene

Open the first live result on this Mac:

```bash
uv run --offline --frozen --cache-dir .uv-cache mjpython -B -u view_selection.py
```

This reads `artifacts/selection/live_initial.json`, including its recorded object
poses, candidate geometry, and answers. It makes no API call and needs no key or
optional SDK. Saved reports are local/ignored, so a fresh clone needs an existing
successful report supplied with `--report /path/to/report.json`.

Grey objects sit at their recorded world positions; orange transparent spheres
show only selected avoidance covers. The omitted far cube remains visible without
a cover. Labels show roles, sphere counts, and reported geometry confidence. Blue
and purple spheres mark the start and goal; cyan shows the **unfiltered nominal
swept proxy**, not an avoidance trajectory. Intersections are expected where that
nominal motion is blocked. The table is only a Z=0 display reference, not a model
decision or enforced constraint. No physics, robot, or CBF controller runs here.

Orbit/zoom normally, use `--top` for a top-down initial view, or hide geom groups:
0 objects, 1 covers, 2 nominal sweep, 3 start/goal proxies, 4 display table. Close
the window to exit; `--seconds 5` is an optional bounded preview. To validate a
saved report and compile its scene without opening a window:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B view_selection.py --check
```

Both camera launch checks passed. Nihar inspected the viewer and said "looks
good." This is user visual review, not an agent screenshot-based check or a
closed-loop safety evaluation.

## Run the static Panda comparison

This checkpoint connects saved covers to the unchanged FRAX OSCBF demo. It calls
the upstream `main()`, nominal torque controller, `DemoOSCBFConfig`, and CBFpy
safety filter. Small, restored process-local bindings supply the scene and
obstacle arrays and enable headless execution/recording; upstream files are not
edited. No controller equations, barriers, gains, or solver are copied/replaced.

Important context: the saved model evaluated a 2.5 cm end-effector sphere starting
at [0.10, 0, 0.06] m, **not Panda**. The experiment keeps the valid default Panda
joints (EE approximately [0.375, 0, 0.409] m), saved object poses, and saved goal
[0.60, 0, 0.06] m. Thus this is a labeled **proxy-decision replay integration test**,
not evidence that Jev correctly selected constraints for the whole arm. No new
API call or key is needed. Do not edit a recorded request and present its old
answers as decisions made for the changed context.

From this repository, use a new output directory for each rollout:

```bash
uv run --offline --frozen --cache-dir .uv-cache python -B -u run_static.py --output artifacts/static/my_run
uv run --offline --frozen --cache-dir .uv-cache python -B -u static_gallery.py --run-dir artifacts/static/my_run
open artifacts/static/my_run/index.html
```

The three modes use identical initial states, object geometry/contact settings,
fixed targets, and run lengths:

- `off`: disable only scene-object CBFs, **not** robot protections or physical collisions.
- `all`: constrain every object using its saved conditional geometry answer.
- `replay`: constrain only saved `avoid` objects using their selected covers.
  An unresolved `review` answer stops the replay rather than silently omitting it.

All-on uses the same bottle/cube covers as replay; the omitted far cube's retained
two-sphere answer supplies its all-on cover. This initial baseline isolates the
role-selection difference rather than changing geometry at the same time.
`--all-geometry box-split-1` explicitly requests a different fixed-geometry
baseline; only recorded candidate IDs are accepted. This option has not been
evaluated in the initial comparison.

Joint limits, singularity avoidance, floor avoidance, self-collision, and torque
constraints remain present in every mode. All selected scene spheres pair with
all 21 moving-link spheres in the original robot model; Jev has not selected robot
bodies. Objects are fixed and physically collidable; translucent covers are only
display geometry. The original Z=0 floor acts as a floor-level tabletop proxy,
not a raised table or movable-object/contact task. Initial modeled-safe-set
violations, numerical failures, and simulator warnings are not treated as success.
The inherited ElastiQP problem is relaxed, so check violations rather than assume
every inequality was enforced exactly.

`--steps` sets the bounded run length (default 3,000 × 0.002 s = 6 simulated
seconds); `--modes off all replay`, `--report`, and source-path overrides are
available. To watch a bounded physics run on this Mac, use `mjpython` instead of
`python` and add `--view`. The target is fixed for comparisons; do not perturb the
robot through the viewer. Close early to record an incomplete run.

Each output contains the unchanged selection source, `comparison.json`, and
per-mode `.npz` state/torque/EE trajectories. Diagnostics include final error,
modeled barriers, QP inequality violations, requested torque-limit excess,
MuJoCo warnings, and synchronized host-side controller times. Independent
collision checks use all 23 MuJoCo robot collision geoms, including gripper/base,
against **every** scene object, including omitted objects. Mesh distances refer
to MuJoCo collision shapes/convex hulls, not visual-mesh truth. These are sampled
checks, not continuous-time or physical-robot safety guarantees.
Distance queries are supplemented with contact depths, and zero distances are
flagged rather than considered clear: a deliberate robot-mesh overlap returned
zero in a positive-control test. The corrected checker passes both fixed-base
and moving-link overlap controls. Rechecking the initial recorded trajectories
left their strictly positive distances and no-contact results unchanged; that
audit is saved separately as `collision_recheck.json` beside the review gallery.

The gallery renders recorded simulation states into looping animated PNGs, using
MuJoCo plus the standard library, without extra video dependencies. It also saves
final-frame images and links the diagnostics. `--fps`, `--width`, `--height`,
`--azimuth`, and `--elevation` change rendering only, not the rollout. Separate
animations are approximately timed, not frame-synchronized. If no GL rendering
context is available, numeric rollouts still work; gallery generation reports the
rendering failure rather than substituting fake images.

### First static comparison: integration works, scene is not a safety challenge

The review gallery is locally at `artifacts/static/review/index.html`. Each mode
ran 3,000 physics steps; all reached about 1.4 mm final EE error with no sampled
object penetration or negative modeled barriers. Scene constraint-pair counts
were 0 / 210 / 168 for off / all / replay. All and replay trajectories matched to
floating-point precision; the maximum EE-path difference between off and replay
was about 8.2 mm. The omitted far cube stayed at least 0.377 m from robot collision
geometry. This is not evidence of improved safety or completion: off also worked.
The 20% pair-count reduction is real for this replay, but single ordered timing
observations are not a speedup benchmark. Nihar reviewed all animations and said
they look good and normal. See PROGRESS.md for evidence and the next research
questions. No prompts, gains, objects, or goals were tuned
to obtain a preferred comparison.
