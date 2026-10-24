import numpy as np
from jev_cbf.control.simulation import collision_diagnostics

def pickup_collision_diagnostics(mj, model, qpos, qvel, objects, target_id):
    """Clutter versus all native robot geometry; target versus non-finger robot.

    Only target-finger contact is exempt. Target/clutter and target/support are
    separately inspected. Evaluation uses independent MjData and full qpos.
    """
    clutter = [o["id"] for o in objects if o["id"] != target_id]
    # The shared evaluator must exclude the free target geom from robot geoms.
    # Pass all scene IDs, then select the clutter-only metrics below.
    summary, distances = collision_diagnostics(mj, model, qpos, qvel, [o["id"] for o in objects])
    finger_ids = {model.body(n).id for n in ("left_finger", "right_finger")}
    target_geom = model.geom(f"scene/{target_id}").id
    scene_geoms = {model.geom(f"scene/{o['id']}").id for o in objects}
    robot_geoms = [i for i in range(model.ngeom) if model.geom_bodyid[i] != 0
                   and i not in scene_geoms and model.geom_bodyid[i] != model.body("target").id
                   and (model.geom_contype[i] or model.geom_conaffinity[i])]
    data = mj.MjData(model)
    forbidden_target, carried_contacts, bilateral = [], [], []
    for q, v in zip(qpos, qvel, strict=True):
        data.qpos[:], data.qvel[:] = q, v
        mj.mj_forward(model, data)
        forbidden_target.append(min(mj.mj_geomDistance(model, data, target_geom, g, 1000., None)
                                    for g in robot_geoms if model.geom_bodyid[g] not in finger_ids))
        fingers, target_clutter = set(), False
        for contact in data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            if target_geom in pair:
                other = next(g for g in pair if g != target_geom)
                body = int(model.geom_bodyid[other])
                if body in finger_ids and contact.dist <= 0:
                    fingers.add(body)
                if other in scene_geoms - {target_geom}:
                    target_clutter = True
        bilateral.append(fingers == finger_ids)
        carried_contacts.append(target_clutter)
    columns = [i for i, o in enumerate(objects) if o["id"] != target_id]
    clutter_distances = distances[:, columns]
    forbidden_target = np.asarray(forbidden_target)
    violation = bool(np.any(clutter_distances <= 0) or np.any(forbidden_target <= 0) or any(carried_contacts))
    summary.update(any_sampled_scene_violation=violation,
                   any_unwanted_robot_clutter_contact=bool(np.any(clutter_distances <= 0)
                       or any(summary["contact_sample_count_by_object"][name] for name in clutter)),
                   any_forbidden_robot_target_contact=bool(np.any(forbidden_target <= 0)),
                   any_target_clutter_contact=bool(any(carried_contacts)),
                   allowed_contact="Only the two native fingers with the designated target; no arm/palm-target or robot/target-clutter exemption.",
                   target_raw_distance_includes_intended_finger_contact=True)
    return summary, distances, {"forbidden_target_distance_m": forbidden_target,
                               "bilateral_finger_contact": np.asarray(bilateral),
                               "target_clutter_contact": np.asarray(carried_contacts)}
