# Experiment methods and results

The completed corrected comparison uses static-clutter pickup: easy has one
clutter obstacle, medium two, hard three, plus a separate orange pickup target.
Configured obstacle counts are validated before perception; they do not supply an
oracle inventory to control. RGBD independently produced 1/2/3 clutter tracks.
The earlier `gallery_v2` and `repeated_v1` had 2/2/3 clutter counts; preserve and
label those historical rather than pooling them with corrected results.

## Five methods

| Method ID | Clutter roles | Cover choice |
| --- | --- | --- |
| `off` | Scene-object barriers disabled | No scene cover; intrinsic robot protections remain |
| `all_single` | Every observed clutter object | One enclosing sphere per object |
| `all_jev` | Every observed clutter object | Jev's conditional adaptive-cover choice |
| `jev_single` | Jev roles | One enclosing sphere per retained object |
| `jev_adaptive` | Jev roles | Jev adaptive-cover choice per retained object |

Non-off methods retain mandatory arm-target protection with the agreed intended
finger-target contact exemption. A deterministic all-adaptive feasibility witness
qualifies scenes; it is separately labeled, not a sixth crossed Jev method.
Each scene must have nonpenetrating initial geometry and an actual unwanted off
collision. Qualification is independent of comparative Jev answers.

## Corrected startup comparison

Local gallery: `artifacts/rgbd/count123_startup_v1/index.html`.
Fifteen runs, with one actual startup response per scene shared across its three
Jev conditions. Controller/physics step is 1 ms; no repeated updates in these runs.

| Method | Easy | Medium | Hard | Mean enabled clutter pairs, easy / medium / hard |
| --- | --- | --- | --- | --- |
| `off` | Collision | Collision | Collision | 0 / 0 / 0 |
| `all_single` | Stalled | Stalled | Stalled | 29 / 58 / 87 |
| `all_jev` | Safe pickup | Safe pickup | Safe pickup | 145 / 377 / 522 |
| `jev_single` | Stalled | Stalled | Stalled | 29 / 29 / 29 |
| `jev_adaptive` | Safe pickup | Safe pickup | Safe pickup | 145 / 145 / 145 |

A stall means no verified pickup within the 20-second task window. It is not
success. Counts above exclude the 16 mandatory arm-target pairs in non-off modes
and 51 intrinsic barriers. Allocated scene rows are fixed at 522/783/1044 by scene,
so fewer enabled pairs do not automatically mean a smaller QP or runtime speedup.

## Corrected repeated-Jev comparison

Local gallery: `artifacts/rgbd/count123_repeated_v1/index.html`.
Nine new rollouts repeat the three Jev conditions. Non-Jev baselines, qualification,
observations and startup answers are copied from the startup comparison and
hash-bound to it; the six baseline panels are not additional independent trials.

All `all_jev` and `jev_adaptive` runs again achieve safe pickup (3/3 each).
All `jev_single` runs again stall (0/3). Mean enabled clutter pairs are
173.72 / 397.38 / 611.22 for `all_jev`, 29 / 29 / 29 for `jev_single`, and
160.22 / 160.31 / 193.32 for `jev_adaptive`.

The recorded completion-driven scheduler has one request in flight, at least
500 ms wall-clock spacing, fresh snapshots and no blocking call in the control
loop. Achieved dispatch rates range 0.854–1.877 Hz; the minimum recorded gap is
0.526919 s and maximum request latency about 1.168 s. Four late replies and zero
invalid replies are retained. There are 22 accepted policy changes including
startup installations; 14 occur after startup. Timing is asynchronous, so changes
in outcomes/cost cannot alone be attributed to a policy change.

The repeated runs use 93 new logical requests, estimated at $0.066176502.
The three shared startup calls bring the corrected comparison to 96 requests and
$0.068243616 estimated usage. The original cumulative ledger across research
checkpoints has 595 calls, $4.913233920 reservations and $0.534521820 estimated
usage under the same $7.50 cap. Reservations and usage estimates are distinct.
No calls or controlled research reruns were added for repository reorganization.

## Evidence and limits

Both corrected galleries have independent native audits of all 15 method panels
and three qualification trajectories each. Repeated non-Jev/qualification records
are inherited and explicitly checked. Audit checks include trajectory hashes,
full native collision/contact/lift evaluation, enabled-row counts, 1 ms timing and
first-step parity with the upstream nominal controller. Offline Node checks
exercise every shared timeline tick, play/pause/restart/scrub/speed controls and
all atlas rectangles; this is not a live-browser visual QA claim.

These are designed simulation scenes with one rollout per condition, not a
statistical repeatability/generalization study. RGBD uses known calibration,
color components and table support. Enclosing observed/completed cells does not
prove coverage of an occluded physical object. Dynamics/contact assumptions,
empirical admission guards and retained failures remain material limitations.
Deterministic adaptive witnesses also solve these scenes; the study establishes
no unique Jev advantage, hard real-time guarantee, formal safety or hardware QA.
Raw answers, source snapshots, manifests, trajectories and original failures stay
in ignored local artifact folders. Full earlier results and decisions are in
[the progress archive](history/progress-2026-10-22.md).
