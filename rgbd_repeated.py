"""Repeated-Jev comparison using exact inputs and startup answers from a saved gallery.

Only the three Jev conditions run again. Non-Jev baselines and qualifications are
copied with provenance; no new scene fitting, startup query, or scene tuning occurs.
"""

import argparse
from collections import Counter
from decimal import Decimal
from pathlib import Path
import shutil

import numpy as np

from api_budget import JevBudget
from inspect_selection import ROOT
from rgbd_pickup import METHODS
from run_rgbd_pickup import SOURCES, execute, verify_inputs
from run_pickup import BUDGET_PATH
from run_suite import digest, read, save


JEV_METHODS = ('all_jev', 'jev_single', 'jev_adaptive')


def usage():
    if not BUDGET_PATH.is_file():
        raise ValueError('existing cumulative usage ledger is missing; do not reset it')
    ledger = read(BUDGET_PATH)
    if Decimal(ledger['cap_usd']) != Decimal('7.50'):
        raise ValueError('existing cumulative cap differs')
    return dict(calls=len(ledger['attempts']), cap_usd=ledger['cap_usd'],
                reserved_usd=str(sum((Decimal(a['reserved_usd']) for a in ledger['attempts']), Decimal(0))),
                estimated_usd=str(sum((Decimal(a.get('reported_usage_estimated_usd', '0')) for a in ledger['attempts']), Decimal(0))),
                ledger_sha256=digest(BUDGET_PATH))


def check_reference(folder):
    manifest = verify_inputs(folder, source=False)
    reference = manifest['startup_reference']
    source = (ROOT/reference['directory']).resolve()
    if not source.is_relative_to(ROOT):
        raise ValueError('reference must stay inside this repository')
    for name, expected in reference['files'].items():
        if digest(source/name) != expected:
            raise ValueError('startup reference changed: '+name)
    return manifest, source


def prepare(folder, reference):
    folder, reference = Path(folder).resolve(), Path(reference).resolve()
    if not folder.is_relative_to(ROOT) or not reference.is_relative_to(ROOT):
        raise ValueError('outputs and references must stay inside this repository')
    if folder.exists() and any(folder.iterdir()):
        raise ValueError('prepare requires a fresh directory')
    original = verify_inputs(reference, source=False)
    if original.get('query_schedule') == 'completion_driven_repeated':
        raise ValueError('reference must be the startup-only comparison')
    source_hashes = {name: digest(ROOT/name) for name in (*SOURCES, 'rgbd_repeated.py', 'verify_rgbd_pickup.py')}
    reference_files = {'manifest.json': digest(reference/'manifest.json'), **original['frozen_files']}
    frozen = []
    for name in ('scenes.json', 'settings.json', 'questions.json'):
        target = folder/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(reference/name, target)
        frozen.append(name)
    for name in original['scene_ids']:
        case, source = folder/name, reference/name
        if not read(source/'qualification.json')['qualified']:
            raise ValueError('reference scene was not qualified: '+name)
        for method in METHODS:
            result_path, trajectory = source/method/'result.json', source/method/'trajectory.npz'
            result = read(result_path)
            reference_files[str(result_path.relative_to(reference))] = digest(result_path)
            if not result.get('startup_refusal'):
                if digest(trajectory) != result['trajectory_sha256']:
                    raise ValueError('reference trajectory fingerprint mismatch')
                reference_files[str(trajectory.relative_to(reference))] = digest(trajectory)
        for item in ('scene.json', 'request.json', 'estimate.json', 'qualification.json',
                     'observation', 'initial', 'qualification', 'off', 'all_single'):
            src, dst = source/item, case/item
            if src.is_dir():
                shutil.copytree(src, dst)
                frozen.extend(str(p.relative_to(folder)) for p in dst.rglob('*') if p.is_file())
            else:
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(src, dst)
                frozen.append(str(dst.relative_to(folder)))
        initial = read(case/'initial/selection.json')
        if 'error' in initial or initial['request'] != read(case/'initial/request.json'):
            raise ValueError('reference startup answer is failed or belongs to different inputs')
        reference_files[f'{name}/initial/selection.json'] = digest(source/'initial/selection.json')
    snapshot = folder/'source_snapshot'
    snapshot.mkdir()
    for name in source_hashes:
        shutil.copyfile(ROOT/name, snapshot/name)
    if any(digest(ROOT/name) != expected for name, expected in source_hashes.items()):
        raise ValueError('sources changed during preparation; retain this draft')
    manifest = dict(original, source_sha256=source_hashes,
                    frozen_files={name: digest(folder/name) for name in frozen},
                    query_schedule='completion_driven_repeated',
                    schedule='Same shared recorded startup answers; each Jev condition then queries its own latest measured state, one request in flight, at least 500 ms between wall-clock starts.',
                    reused_baselines=['off', 'all_single'],
                    startup_reference=dict(directory=str(reference.relative_to(ROOT)), files=reference_files),
                    provenance_note='Qualification and non-Jev baseline trajectories are inherited, not new trials. Numerical control, scenes, beliefs, contact rules and startup answers are unchanged. Producer-source differences are retained; prior post-run event serialization and render/workflow hardening do not change the controller.')
    save(folder/'manifest.json', manifest)
    save(folder/'usage_before.json', usage())
    print('Prepared repeated comparison:', folder, flush=True)


def run(folder, *, live=False):
    manifest, _ = check_reference(folder)
    for name in manifest['scene_ids']:
        case = folder/name
        for method in JEV_METHODS:
            if (case/method/'result.json').exists():
                continue
            verify_inputs(folder)
            if not live:
                raise ValueError('new repeated rollouts require --live')
            usage()  # Require the original cumulative ledger before every new run.
            execute(case, method, initial=read(case/'initial/selection.json'), repeated=True, ask=JevBudget(BUDGET_PATH))
    save(folder/'usage_after.json', usage())
    return report(folder)


def report(folder):
    manifest, source = check_reference(folder)
    rows, calls, total_cost = [], 0, Decimal(0)
    for name in manifest['scene_ids']:
        for method in JEV_METHODS:
            path = folder/name/method
            if not (path/'result.json').exists():
                raise ValueError('comparison is incomplete: '+name+'/'+method)
            result, baseline = read(path/'result.json'), read(source/name/method/'result.json')
            timings = result.get('model_request_timings', [])
            starts = np.array([t['request_start_monotonic_s'] for t in timings])
            completions = np.array([t['completion_monotonic_s'] for t in timings])
            snapshots = np.array([t['snapshot_at_s'] for t in timings])
            gaps = np.diff(starts)
            if len(gaps) and (np.any(gaps < .5) or np.any(starts[1:] < completions[:-1]) or np.any(np.diff(snapshots) <= 0)):
                raise ValueError('repeated schedule violated the start ceiling, one-in-flight rule, or fresh-snapshot ordering')
            events = result.get('selection_events', [])
            if (path/'events.json').exists() and read(path/'events.json') != events:
                raise ValueError('saved events differ from result log')
            selections = [e for e in events if e.get('kind') == 'selection']
            changed = [e for e in selections if e.get('status') == 'accepted' and e.get('policy') != e.get('previous_policy')]
            queries = sorted((path/'queries').glob('*/selection.json'))
            if len(queries) != len(timings):
                raise ValueError('query/timing count differs, including late responses')
            invalid = [dict(source=str(p.relative_to(path)), error=read(p)['error']) for p in queries if 'error' in read(p)]
            cost = sum((Decimal(read(p).get('response', {}).get('usage', {}).get('input_tokens', 0))*Decimal('.042')/1_000_000 for p in queries), Decimal(0))
            calls += len(queries); total_cost += cost
            rows.append(dict(scene=name, method=method,
                startup_only={k: baseline[k] for k in ('unwanted_collision', 'pickup_success', 'safe_pickup', 'simulation_seconds', 'cbf_counts', 'warm_loop')},
                repeated={k: result[k] for k in ('unwanted_collision', 'pickup_success', 'safe_pickup', 'simulation_seconds', 'cbf_counts', 'warm_loop')},
                new_logical_requests=len(queries), new_provider_estimated_usd=str(cost),
                invalid_responses=invalid,
                selection_statuses=dict(Counter(e.get('status') for e in selections)),
                accepted_policy_changes=len(changed),
                changes_after_startup=sum(e.get('source') != 'initial/selection.json' for e in changed),
                late_responses=sum(e.get('kind') == 'late_response' for e in events),
                minimum_request_start_gap_s=float(gaps.min()) if len(gaps) else None,
                achieved_start_rate_hz=float((len(starts)-1)/(starts[-1]-starts[0])) if len(starts)>1 else None,
                latency_range_s=[float((completions-starts).min()), float((completions-starts).max())] if len(starts) else [],
                snapshot_sim_times_s=snapshots.tolist(), one_request_in_flight=True,
                evidence_sha256={str(p.relative_to(path)): digest(p) for p in [path/'result.json', *queries]}))
    summary = dict(rows=rows, new_rollouts=len(rows), reused_baselines=2*len(manifest['scene_ids']),
                   new_logical_requests=calls, new_provider_estimated_usd=str(total_cost),
                   scope='One repeated rollout per Jev condition per designed scene; paired to recorded startup-only results, same startup answers and geometry. Asynchronous wall-clock timing differs, so outcome differences cannot alone be attributed to a selection change. This is not independent scene generalization or a statistical repeatability study.')
    save(folder/'schedule_comparison.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'run', 'report', 'gallery'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, default=ROOT/'artifacts/rgbd/gallery_v2')
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args()
    folder = args.output.resolve()
    if not folder.is_relative_to(ROOT):
        parser.error('all outputs must stay inside this repository')
    if args.command == 'prepare':
        prepare(folder, args.reference)
    elif args.command == 'run':
        run(folder, live=args.live)
    elif args.command == 'report':
        result = report(folder)
        print('Compared', len(result['rows']), 'startup/repeated condition pairs')
    else:
        from rgbd_gallery import gallery
        report(folder)
        gallery(folder)


if __name__ == '__main__':
    main()
