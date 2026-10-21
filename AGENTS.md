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

- The original FRAX OSCBF baseline and isolated sphere-cover checkpoint are
  complete. Nihar approved a tiny, standalone text-only Jev selection experiment
  using known scene objects and generated candidate IDs, followed by a viewer of
  the saved result in its scene. Both are implemented; Nihar reviewed the new
  scene viewer and said "looks good." This checkpoint is complete. Agree on the
  next step before expanding scope. See PROGRESS.md.
- Keep current outputs inspectable proposals only. Do not connect them to robot
  control, grant real contact exemptions, add online constraint updates, new robot
  tasks, other geometry families, or hardware ahead of the current checkpoint.
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
  `uv run --offline --frozen --cache-dir .uv-cache python -B -m unittest discover -s tests -v`.
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
- Passing launcher tests or package-presence checks is not a successful simulation,
  controller-parity result, or safety guarantee. State exactly what was run.
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
