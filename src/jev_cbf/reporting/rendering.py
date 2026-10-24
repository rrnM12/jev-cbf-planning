import numpy as np

def add_scene_labels(scene, labels, *, append=False):
    """Add display-only text without changing model/physics geometry."""
    import mujoco

    start = scene.ngeom if append else 0
    if start + len(labels) > scene.maxgeom:
        raise ValueError("too many labels for the viewer scene")
    for index, (text, position) in enumerate(labels, start=start):
        geom = scene.geoms[index]
        mujoco.mjv_initGeom(
            geom, mujoco.mjtGeom.mjGEOM_LABEL, np.zeros(3),
            np.asarray(position, dtype=float), np.eye(3).ravel(), np.ones(4),
        )
        geom.label = text[:90]
    scene.ngeom = start + len(labels)

def render_frames(mj, model, trace, indices, objects, target_id, *, width=480, height=360, camera_options=None, object_labels=None):
    data = mj.MjData(model)
    option = mj.MjvOption(); option.geomgroup[3] = 0; option.geomgroup[4] = 0
    camera = mj.MjvCamera(); camera.lookat[:] = [.35, 0., .25]
    camera.distance, camera.azimuth, camera.elevation = 1.35, 135, -35
    for key, value in (camera_options or {}).items():
        if key == 'lookat':
            camera.lookat[:] = value
        elif key in ('distance', 'azimuth', 'elevation'):
            setattr(camera, key, value)
        else:
            raise ValueError('unknown playback camera option: '+key)
    model.geom("target_geom").rgba[3] = 0
    with mj.Renderer(model, width=width, height=height) as renderer:
        for index in indices:
            data.qpos[:], data.qvel[:] = trace["qpos"][index], trace["qvel"][index]
            mj.mj_forward(model, data)
            renderer.update_scene(data, camera=camera, scene_option=option)
            scene = renderer.scene
            enabled = np.any(trace["cover_enabled"][index], axis=0)
            for point, radius in zip(trace["cover_centers_m"][index][enabled], trace["cover_radii_m"][index][enabled], strict=True):
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0, 0]),
                               point, np.eye(3).ravel(), np.array([.95, .65, .12, .13]))
                scene.ngeom += 1
            labels = []
            for obj in objects:
                geom = model.geom(f"scene/{obj['id']}").id
                label = (object_labels or {}).get(obj['id'], "TARGET" if obj["id"] == target_id else obj["id"])
                labels.append((label, data.geom_xpos[geom]+[0, 0, obj["size_m"][2]/2+.025]))
            add_scene_labels(scene, labels, append=True)
            for point in trace["ee_m"][np.linspace(0, index, min(index+1, 70), dtype=int)]:
                mj.mjv_initGeom(scene.geoms[scene.ngeom], mj.mjtGeom.mjGEOM_SPHERE,
                               np.array([.003, 0, 0]), point, np.eye(3).ravel(), np.array([.9, .2, .8, 1.]))
                scene.ngeom += 1
            yield renderer.render().copy()
