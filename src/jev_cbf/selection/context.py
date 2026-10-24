import numpy as np

def robot_context(robot, nominal):
    """Describe actual initial joints and the controller's moving-link spheres."""
    q = np.asarray(nominal["qpos"])[0, :7]
    centers, radii = map(np.asarray, robot.link_collision_data(q))
    groups, index = [], 0
    for name, sizes in zip(robot.joint_names, robot.collision_radii, strict=True):
        count = len(sizes)
        groups.append({"joint": name, "sphere_indices": list(range(index, index + count))})
        index += count
    if index != len(radii):
        raise ValueError("robot sphere grouping does not match the collision model")
    return {
        "name": "Franka Panda", "initial_joint_positions_rad": q.tolist(),
        "initial_joint_velocities_rad_s": np.asarray(nominal["qvel"])[0, :7].tolist(),
        "controller": "Original FRAX OSCBF pose-tracking torque controller; fixed down-facing orientation; not a path planner.",
        "moving_link_sphere_centers_world_m": centers.round(6).tolist(),
        "moving_link_sphere_radii_m": radii.tolist(), "sphere_groups": groups,
        "scene_pairing": "Every selected obstacle primitive pairs with all 21 moving-link spheres; the model does not select robot bodies in this experiment.",
        "limitations": "Controller spheres are an approximation and do not model the entire fixed base/gripper. Native collision features separately use all 23 MuJoCo robot collision geoms. No calibrated uncertainty or added margin.",
    }
