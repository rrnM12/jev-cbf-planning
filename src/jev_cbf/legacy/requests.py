from copy import deepcopy
import numpy as np
from jev_cbf.legacy.selection import prepare_request
from jev_cbf.selection.context import robot_context

def pickup_request(scene, settings, robot, template, *, samples=10_000, model="jev-1.13.0", shape_aware=False,
                   counts=(1,2,4), terminal_features=False):
    q = np.asarray(robot.default_configuration)
    objects = deepcopy(scene["objects"])
    target = scene["target_id"]
    if len([o for o in objects if o["id"] == target]) != 1:
        raise ValueError("pickup needs exactly one designated target")
    if any(o.get("motion") for o in objects):
        raise ValueError("pickup clutter must be static")
    obj = next(o for o in objects if o["id"] == target)
    goal = [*obj["center_m"][:2], settings["grasp_height_m"]]
    facts = {
        "description": "Static ground-truth clutter on a Z=0 tabletop, with a physical free-body pickup target. The robot base is mounted at tabletop height. No perception.",
        "frame": "robot base/world, metres, Z up, initially axis-aligned objects; tabletop Z=0",
        "task": f"Maneuver through static clutter, grasp {target} with the fingers, lift it just enough to verify pickup, then stop. No placement, transport, pushing or other task. Finger-target contact is intended; other robot-target contact and all robot-clutter contact are unwanted. Obstacles do not move.",
        "motion": {"start_m": np.asarray(robot.ee_transform(q))[:3, 3].tolist(), "goal_m": goal},
        "objects": objects,
    }
    context = robot_context(robot, {"qpos": q[None], "qvel": np.zeros((1, 7))})
    context["limitations"] = "Original 21 arm/tool collision spheres; articulated fingers and contact/payload forces are not states in the seven-DOF FRAX dynamics. Independent native evaluation includes the actual fingers. No formal contact-phase safety guarantee."
    context["scene_pairing"] = "Clutter pairs with all original 21 spheres. The explicitly designated pickup target always protects the 16 upstream arm spheres; only the five terminal tool spheres are exempt for intended grasping. Native evaluation still rejects palm/arm-target contact."
    if 4 not in counts:
        raise ValueError("pickup candidate counts must include the fixed four-sphere arm-target cover")
    request = prepare_request(facts, template, model=model, samples=samples, robot_context=context, shape_aware=shape_aware, counts=counts)
    if terminal_features:
        request["state"]["terminal_clearance_features"] = True
    request["state"].update(target_id=target, phase="approach", pickup_settings=deepcopy(settings))
    for supplied, original in zip(request["state"]["objects"], objects, strict=True):
        supplied["description"] = original.get("description", "")
        supplied["designated_pickup_target"] = supplied["id"] == target
    return request
