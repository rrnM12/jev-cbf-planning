# Jev-selected CBFs

The original FRAX OSCBF demo runs from this repository. Separate experiments
inspect conservative sphere covers and prepare task-conditioned Jev proposals
using known scene geometry. The first live Jev request succeeded; its results are
recorded in PROGRESS.md. No proposals control the robot. The larger,
evolving research plan is in [ARCHITECTURE.md](ARCHITECTURE.md).
Current progress, verification, blockers, and decision history are in
[PROGRESS.md](PROGRESS.md); repository working rules are in [AGENTS.md](AGENTS.md).

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
uv run --offline --frozen --cache-dir .uv-cache python -B -m unittest discover -s tests -v
python3 -B run_demo.py --help
```

The suite covers launch/setup handling and isolated geometry, not robot safety.
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

The next approved experiment is deliberately separate from the robot. Its inputs
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
`proposal_only: true` always means nothing is connected to control.

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
Run all 39 offline tests including that optional adapter check with:

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
