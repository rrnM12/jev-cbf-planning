from dataclasses import asdict
import html
import json
import time
import numpy as np
from jev_cbf.sensing.camera_io import ReplaySource, deproject
from jev_cbf.sensing.geometry import FrameGeometry
from jev_cbf.io import save
from jev_cbf.reporting.images import write_apng
from jev_cbf.reporting.images import depth_image

def geometry_svg(objects,axes=(0,2)):
    if not objects:return '<p>No calibrated object geometry available.</p>'
    limits=np.array([[np.array(o['center_m'])-np.array(o['size_m'])/2,np.array(o['center_m'])+np.array(o['size_m'])/2] for o in objects])
    lower,upper=limits[:,0].min(axis=0)-.08,limits[:,1].max(axis=0)+.08
    scale=400/max(upper[axes[0]]-lower[axes[0]],upper[axes[1]]-lower[axes[1]])
    def xy(p):return 20+(p[axes[0]]-lower[axes[0]])*scale,440-(p[axes[1]]-lower[axes[1]])*scale
    shapes=[]
    for obj in objects:
        center=np.array(obj['center_m']);half=np.array(obj['size_m'])/2
        x,y=xy(center+[-half[0],half[1],half[2]]) if axes==(0,2) else xy(center+[-half[0],half[1],0])
        shapes.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{obj["size_m"][axes[0]]*scale:.2f}" height="{obj["size_m"][axes[1]]*scale:.2f}" fill="#4fc3c7" fill-opacity=".25" stroke="#16888b"/>')
        if obj.get('candidates'):
            candidate=min(obj['candidates'],key=lambda c:c['excess_volume_m3'])
            for offset,radius in zip(candidate['centers_object_m'],candidate['radii_m'],strict=True):
                cx,cy=xy(center+offset)
                shapes.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{radius*scale:.2f}" stroke="#b57a00" fill="#ffc34d" fill-opacity=".15"/>')
        tx,ty=xy(center)
        shapes.append(f'<text x="{tx:.2f}" y="{ty:.2f}" font-size="12">{html.escape(obj["id"])}</text>')
    labels='XYZ'
    return f'<svg viewBox="0 0 480 480" width="440" role="img" aria-label="Robot-frame estimated geometry"><text x="15" y="15">Robot-frame {labels[axes[0]]}/{labels[axes[1]]} · metric orthographic projection</text>{"".join(shapes)}</svg>'

def inspect_recording(recording,output,*,calibration=None,support_z=None,margin_m=.005,excluded_ids=()):
    if output.exists() and any(output.iterdir()):raise ValueError('inspection needs a new/empty output directory')
    output.mkdir(parents=True,exist_ok=True)
    source=ReplaySource(recording)
    processor=FrameGeometry(calibration,lambda f,s:s.annotations(f),support_z=support_z,margin_m=margin_m,
                            excluded_ids=excluded_ids) if calibration is not None and support_z is not None else None
    cards=[];rows=[]
    try:
        while True:
            try:frame=source.read(.1)
            except StopIteration:break
            index=len(rows);folder=output/f'{index:04d}';folder.mkdir()
            write_apng(folder/'rgb.png',[frame.rgb],frame_count=1,fps=1)
            write_apng(folder/'depth.png',[depth_image(frame.depth_m)],frame_count=1,fps=1)
            overlay=None;identity=None
            try:
                masks,identity=source.annotations(frame)
                registered=frame.metadata.get('rgb_depth_registered',False) and frame.rgb.shape[:2]==frame.depth_m.shape
                overlay=frame.rgb.copy() if registered else np.zeros((*frame.depth_m.shape,3),dtype=np.uint8)
                for i,mask in enumerate(masks.values()):
                    color=np.array(([255,90,70],[40,210,160],[130,110,255])[i%3])
                    overlay[mask]=(.5*overlay[mask]+.5*color).astype(np.uint8)
                write_apng(folder/'masks.png',[overlay],frame_count=1,fps=1)
            except ValueError:pass
            error=None;geometry=None
            if processor is not None:
                try:geometry=processor(frame,source)
                except ValueError as exc:error=str(exc)
            else:error='Geometry disabled: supply explicit robot calibration AND table support height. Raw camera-frame preview remains available.'
            try:
                points=deproject(frame)
                np.savez_compressed(folder/'points_optical.npz',points_optical_m=points)
            except ValueError as exc:error=str(exc)
            row=dict(camera_id=frame.camera_id,frame_number=frame.frame_number,capture_time_s=frame.capture_time_s,
                     clock_domain=frame.clock_domain,identity_source=identity,geometry=geometry,error=error,
                     intrinsics=asdict(frame.intrinsics),optical_frame=frame.optical_frame,
                     metadata=frame.metadata,inspection_receipt_age_host_s=time.monotonic()-frame.received_monotonic_s)
            save(folder/'observation.json',row);rows.append(row)
            images=''.join(f'<figure><img src="{index:04d}/{name}.png" width="320" alt="{name}"><figcaption>{name}</figcaption></figure>' for name in ('rgb','depth','masks') if (folder/(name+'.png')).exists())
            objects=geometry['estimated_objects'] if geometry else []
            point_link=f' · <a href="{index:04d}/points_optical.npz">Metric camera-frame points</a>' if (folder/'points_optical.npz').exists() else ''
            cards.append(f'<section><h2>{html.escape(frame.camera_id)} / frame {frame.frame_number}</h2><p>Capture: {frame.capture_time_s:.6f} s ({html.escape(frame.clock_domain)}). Identity: {html.escape(str(identity))}.</p><div class="row">{images}</div><p>{html.escape(error or geometry["status"])}</p><div class="row">{geometry_svg(objects)}{geometry_svg(objects,(0,1))}</div><a href="{index:04d}/observation.json">Measurements/calibration status/candidates</a>{point_link}</section>')
    finally:source.close()
    normalized=json.loads(json.dumps(calibration,default=lambda a:a.tolist()))
    save(output/'inspection.json',dict(frames=rows,recording=str(recording),calibration_used=normalized,
         support_z_m=support_z,margin_m=margin_m,excluded_ids=list(excluded_ids),
         scope='Offline replay inspection. Mask labels are supplied on the depth grid, not detected; RGB overlay is used only for explicitly registered frames. Visible-point bounds and sphere covers do not certify whole-object coverage. No hardware validation, physical actuation or model calls.'))
    (output/'index.html').write_text(f'<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Camera plumbing inspection</title><style>body{{font:16px system-ui;max-width:1150px;margin:35px auto;padding:15px;background:#f4f7fa;color:#193042}}section{{background:white;padding:20px;margin:20px 0;border-radius:12px}}.row{{display:flex;flex-wrap:wrap;gap:15px}}figure{{margin:0}}img,svg{{max-width:100%;height:auto}}svg{{background:#fafafa}}a{{color:#075ec0}}</style><h1>Camera input / geometry inspection</h1><p>Offline recording replay. RGB, display-scaled depth and supplied masks; fitted boxes and least-excess sphere candidates below. Geometry is estimated, not a verified full-object enclosure. Orange circles are orthographic sphere projections, not new CBF rollouts.</p><p>Camera-only preview is allowed without robot calibration. Robot-frame geometry is not guessed. Capture clocks and host receipt ages are different quantities.</p>{"".join(cards)}<a href="inspection.json">Full inspection data</a></html>')
    print('Camera inspection:',output/'index.html',flush=True)
    return rows
