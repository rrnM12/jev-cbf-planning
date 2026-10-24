"""Independent saved-trajectory audit; never feeds truth back to control."""

import argparse
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlparse
import numpy as np
from jev_cbf.paths import ROOT
from jev_cbf.evaluation import pickup_collision_diagnostics
from jev_cbf.control.rgbd import METHODS
from jev_cbf.control.scene import plant_model
from jev_cbf.provenance import verify_inputs
from jev_cbf.upstream import load_upstream
from jev_cbf.io import read, save, digest

class Links(HTMLParser):
    def __init__(self):
        super().__init__(); self.links=[]
    def handle_starttag(self, tag, attrs):
        for key,value in attrs:
            if key in ('href','src') and value:
                self.links.append(value)

def verify(folder, *, report_path=None):
    report_path = folder/'verification.json' if report_path is None else report_path
    save(report_path,dict(status='audit_running'))
    manifest = verify_inputs(folder, source=False)
    demo, utils = load_upstream(ROOT.parent/'frax', ROOT.parent/'cbfpy')
    robot, mj, rows = demo.load_panda(), utils.mujoco, []
    settings = read(folder/'settings.json')
    for name in manifest['scene_ids']:
        case = folder/name; scene = read(case/'scene.json')
        model = plant_model(scene, robot, mj)
        auxiliary=[m for m in ('qualification/adaptive','repeated_diagnostic') if (case/m/'result.json').exists()]
        for method in [*METHODS,*auxiliary]:
            path = case/method
            result = read(path/'result.json')
            if result.get('startup_refusal'):
                assert not (path/'trajectory.npz').exists()
                rows.append(dict(scene=name, method=method, startup_refusal=True, audited=True)); continue
            assert digest(path/'trajectory.npz') == result['trajectory_sha256']
            with np.load(path/'trajectory.npz') as trace:
                collision, distances, contacts = pickup_collision_diagnostics(mj, model, trace['qpos'], trace['qvel'], scene['objects'], scene['target_id'])
                assert collision['any_sampled_scene_violation'] == result['unwanted_collision']
                np.testing.assert_allclose(distances, trace['object_distance_m'], atol=1e-12)
                target = next(o for o in scene['objects'] if o['id'] == scene['target_id'])
                geom = model.geom('scene/'+target['id']).id
                data, bottom = mj.MjData(model), []
                half = np.array(target['size_m'])/2
                for q,v in zip(trace['qpos'],trace['qvel'],strict=True):
                    data.qpos[:],data.qvel[:]=q,v;mj.mj_forward(model,data)
                    rot=data.geom_xmat[geom].reshape(3,3)
                    support=abs(rot[2])@half if target['shape']=='box' else half[0]*np.linalg.norm(rot[2,:2])+half[2]*abs(rot[2,2])
                    bottom.append(data.geom_xpos[geom][2]-support)
                np.testing.assert_allclose(bottom, trace['target_bottom_m'], atol=1e-12)
                valid=(np.array(bottom)>=settings['pickup_bottom_lift_m'])&contacts['bilateral_finger_contact']
                longest=current=0
                for value in valid:
                    current=current+1 if value else 0;longest=max(current,longest)
                success=longest>=round(settings['pickup_confirmation_s']/settings['control_dt_s'])
                assert success == result['pickup_success']
                assert bool(success and not collision['any_sampled_scene_violation']) == result['safe_pickup']
                enabled=trace['cover_enabled'][:-1]
                counts=np.count_nonzero(enabled,axis=(1,2))
                np.testing.assert_array_equal(counts,trace['enabled_scene_pairs'][:-1])
                np.testing.assert_allclose(np.diff(trace['time_s']),.001,atol=1e-12)
                assert result['oscbf_nominal_first_step_max_difference_Nm']<1e-7
                rows.append(dict(scene=name,method=method,audited=True,steps=len(trace['torques_Nm']),
                                 unwanted_collision=bool(collision['any_sampled_scene_violation']),pickup_success=bool(success),
                                 longest_true_lift_bilateral_window_s=longest*.001,trajectory_sha256=result['trajectory_sha256']))
            print('Audited',name,method,flush=True)
    checked,broken=0,[]
    for page in folder.rglob('*.html'):
        parser=Links();parser.feed(page.read_text())
        for link in parser.links:
            parsed=urlparse(link)
            if parsed.scheme or not parsed.path:
                continue
            target=(page.parent/unquote(parsed.path)).resolve()
            checked+=1
            if not target.exists():broken.append(dict(page=str(page.relative_to(folder)),link=link))
    report=dict(audited_conditions=sum(r['method'] in METHODS for r in rows),auxiliary_trajectories=sum(r['method'] not in METHODS for r in rows),
                verifier_sha256=digest(Path(__file__)),rows=rows,local_links_checked=checked,broken_links=broken,
                scope='Independent native geometry/contact/lift re-evaluation of saved trajectories. No new control, fitting, provider request or oracle correction. Whole-cell/observed/true-volume checks are separately linked in coverage.json.')
    save(report_path,report)
    if broken:raise ValueError('broken local gallery links; see '+str(report_path))
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    verify(parser.parse_args().output.resolve())
