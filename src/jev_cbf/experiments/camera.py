"""Offline camera replay/inspection; live capture is explicit and UNVERIFIED.

Replay never imports RealSense or connects to devices; no command controls a
physical robot or calls Jev. Raw frames/masks retain separate provenance.
"""

import argparse
from pathlib import Path
from jev_cbf.io import read
from jev_cbf.reporting.camera import inspect_recording
from jev_cbf.sensing.capture import record_live

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('replay','live'))
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--recording',type=Path)
    parser.add_argument('--calibration',type=Path)
    parser.add_argument('--config',type=Path,help='SDK camera configuration; only live connects to a device')
    parser.add_argument('--frames',type=int,default=30,help='live capture count, not a requested device frame rate')
    args=parser.parse_args(argv)
    from jev_cbf.paths import inside_repository
    args.output = inside_repository(args.output)
    if args.action=='replay':
        if args.recording is None:parser.error('replay needs --recording')
        config=read(args.calibration) if args.calibration else {}
        inspect_recording(args.recording,args.output,calibration=config.get('cameras'),support_z=config.get('support_z_m'),
                          margin_m=config.get('margin_m',.005),excluded_ids=config.get('excluded_ids',()))
    else:
        if args.config is None or args.frames<=0:parser.error('live needs --config and positive --frames')
        record_live(read(args.config),args.output,frames=args.frames)
    return 0

if __name__=='__main__':raise SystemExit(main())
