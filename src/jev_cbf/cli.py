"""One command surface; parsing/help does not initialize JAX, GL, cameras or SDKs."""

import argparse
import importlib
from pathlib import Path
from jev_cbf.paths import CONFIGS, ROOT, inside_repository

def parser():
    p = argparse.ArgumentParser(prog='jev-cbf', description='RGBD pickup experiments and saved-result inspection')
    groups = p.add_subparsers(dest='group', required=True)
    rgbd = groups.add_parser('rgbd', help='Startup comparison and independent saved-trajectory audit')
    actions = rgbd.add_subparsers(dest='action', required=True)
    for action in ('prepare', 'qualify', 'run', 'gallery', 'verify', 'status'):
        command = actions.add_parser(action)
        command.add_argument('--output', type=Path, required=True)
        if action == 'prepare': command.add_argument('--scenes', type=Path, default=CONFIGS/'scenes/rgbd_pickup.json')
        if action == 'run': command.add_argument('--live', action='store_true', help='Allow new calls against the existing cumulative ledger')
        if action == 'verify': command.add_argument('--report', type=Path, help='Write audit here instead of updating the run\'s derived verification.json')
    repeated = groups.add_parser('repeated', help='Paired completion-driven repeated-Jev comparison')
    actions = repeated.add_subparsers(dest='action', required=True)
    for action in ('prepare', 'run', 'report', 'gallery'):
        command = actions.add_parser(action)
        command.add_argument('--output', type=Path, required=True)
        if action == 'prepare': command.add_argument('--reference', type=Path, default=ROOT/'artifacts/rgbd/count123_startup_v1')
        if action == 'run': command.add_argument('--live', action='store_true')
    inspect = groups.add_parser('inspect', help='Camera replay and geometry/selection inspection')
    actions = inspect.add_subparsers(dest='action', required=True)
    camera = actions.add_parser('camera')
    cameras = camera.add_subparsers(dest='camera_action', required=True)
    replay = cameras.add_parser('replay', help='Inspect saved frames without a device')
    replay.add_argument('--recording', type=Path, required=True)
    replay.add_argument('--output', type=Path, required=True)
    replay.add_argument('--calibration', type=Path)
    live = cameras.add_parser('live', help='Explicit hardware-unverified camera-only capture')
    live.add_argument('--output', type=Path, required=True)
    live.add_argument('--config', type=Path, required=True)
    live.add_argument('--frames', type=int, default=30)
    geometry = actions.add_parser('geometry')
    geometry.add_argument('--shape', choices=('box','cylinder'), default='cylinder')
    geometry.add_argument('--size', type=float, nargs=3, default=(.07,.07,.30))
    geometry.add_argument('--counts', type=int, nargs='+', default=(1,2,4))
    geometry.add_argument('--shape-aware', action='store_true')
    geometry.add_argument('--samples', type=int, default=100_000)
    geometry.add_argument('--seed', type=int, default=0)
    geometry.add_argument('--no-view', action='store_true')
    geometry.add_argument('--seconds', type=float)
    geometry.add_argument('--report', type=Path)
    selection = actions.add_parser('selection', help='Offline historical proxy request preview')
    selection.add_argument('--scene', type=Path, default=CONFIGS/'legacy/scenes/selection_scene.json')
    selection.add_argument('--questions', type=Path, default=CONFIGS/'legacy/prompts/selection.json')
    selection.add_argument('--model', default='jev-latest')
    selection.add_argument('--task')
    selection.add_argument('--counts', type=int, nargs='+', default=(1,2,4))
    selection.add_argument('--samples', type=int, default=100_000)
    selection.add_argument('--seed', type=int, default=0)
    selection.add_argument('--output', type=Path)
    demo = groups.add_parser('demo', help='Launch unchanged upstream demo, or check setup')
    demo.add_argument('--robot', choices=('panda','iiwa'), default='panda')
    demo.add_argument('--frax-dir', type=Path, default=ROOT.parent/'frax')
    demo.add_argument('--cbfpy-dir', type=Path, default=ROOT.parent/'cbfpy')
    demo.add_argument('--check', action='store_true')
    return p

def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        if getattr(args, 'output', None) is not None:
            args.output = inside_repository(args.output)
        if getattr(args, 'report', None) is not None:
            args.report = inside_repository(args.report)
        if args.group == 'rgbd' and args.action == 'verify':
            importlib.import_module('jev_cbf.experiments.verify').verify(args.output, report_path=args.report)
            return 0
        if args.group in ('rgbd', 'repeated'):
            module = importlib.import_module('jev_cbf.experiments.'+args.group)
            forwarded = [args.action, '--output', str(args.output)]
            for name in ('scenes','reference'):
                if hasattr(args, name): forwarded += ['--'+name, str(getattr(args,name))]
            if getattr(args,'live',False): forwarded += ['--live']
            module.main(forwarded)
            return 0
        values = vars(args).copy()
        group = values.pop('group'); action = values.pop('action', None)
        if group == 'demo': target = 'jev_cbf.upstream'
        elif action == 'geometry': target = 'jev_cbf.reporting.inspection'
        elif action == 'selection': target = 'jev_cbf.legacy.selection'
        else: target = 'jev_cbf.experiments.camera'
        forwarded = []
        if 'camera_action' in values: forwarded.append(values.pop('camera_action'))
        for name, value in values.items():
            if value is None or value is False: continue
            forwarded.append('--'+name.replace('_','-'))
            if value is not True:
                forwarded.extend(str(v) for v in value) if isinstance(value,(list,tuple)) else forwarded.append(str(value))
        return importlib.import_module(target).main(forwarded) or 0
    except (ValueError, FileNotFoundError) as error:
        p.error(str(error))

if __name__ == '__main__':
    raise SystemExit(main())
