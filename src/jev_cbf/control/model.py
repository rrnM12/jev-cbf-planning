"""Explicit in-memory simulator profiles; upstream XML is never edited.

The demo XML and FRAX URDF are different dynamics models. ``frax`` is an
idealized seven-joint plant using FRAX's inertial parameters, no passive joint
damping/rotor inertia, and a rigid closed gripper. It is NOT a calibrated Panda.
``upstream`` preserves the original XML, including its two finger joints.
"""

import numpy as np

def panda_spec(mujoco, frax_dir, *, profile="upstream", robot=None):
    spec = mujoco.MjSpec.from_file(str(frax_dir / "examples/xml/panda.xml"))
    if profile == "upstream":
        return spec
    if profile not in ("frax", "frax_gripper") or robot is None:
        raise ValueError("expected upstream, frax, or frax_gripper; matched profiles require their robot model")
    # The seven-DOF FRAX model already folds the fixed tool into its last link.
    # Keep all original visual/collision geometry but avoid counting tool mass
    # twice. Remove only finger actuation/joints and their dependent elements.
    removed = list(spec.keys)
    if profile == "frax":
        removed += (list(spec.actuators)[7:] + list(spec.equalities)
                    + list(spec.tendons)
                    + [spec.joint("finger_joint1"), spec.joint("finger_joint2")])
    # Split out the native finger inertias at a reference half-opening of 2 cm.
    # Their motion/contact means acceleration parity is no longer exact. Do not
    # count their mass twice or pretend the 7-DOF model includes finger dynamics.
    if profile == "frax_gripper":
        original = spec.compile()
        reference = mujoco.MjData(original)
        reference.qpos[7:9] = .02
        mujoco.mj_forward(original, reference)
    for element in removed:
        spec.delete(element)
    for index in range(7):
        body = spec.body(f"link{index + 1}")
        rotation = robot.link_local_inertia_rotations[index]
        inertia = rotation @ robot.link_local_inertias[index] @ rotation.T
        body.mass = robot.link_masses[index]
        body.ipos = robot.link_local_inertia_positions[index]
        body.fullinertia = [inertia[0, 0], inertia[1, 1], inertia[2, 2],
                            inertia[0, 1], inertia[0, 2], inertia[1, 2]]
    for name in (("hand", "left_finger", "right_finger") if profile == "frax" else ("hand",)):
        body = spec.body(name)
        body.mass, body.inertia = 0., [0., 0., 0.]
    for joint in spec.joints:
        joint.armature, joint.damping = 0., np.zeros(3)
    if profile == "frax_gripper":
        body = spec.body("link7")
        mass, center = float(body.mass), np.array(body.ipos)
        rotation = robot.link_local_inertia_rotations[-1]
        inertia = rotation @ robot.link_local_inertias[-1] @ rotation.T
        shift = lambda p: np.dot(p, p)*np.eye(3) - np.outer(p, p)
        origin_inertia = inertia + mass*shift(center)
        moment = mass*center
        link = original.body("link7").id
        local = reference.xmat[link].reshape(3, 3).T
        for name in ("left_finger", "right_finger"):
            finger = original.body(name).id
            m = float(original.body_mass[finger])
            c = local @ (reference.xipos[finger] - reference.xpos[link])
            r = local @ reference.ximat[finger].reshape(3, 3)
            origin_inertia -= r @ np.diag(original.body_inertia[finger]) @ r.T + m*shift(c)
            moment -= m*c
            mass -= m
        center = moment/mass
        inertia = origin_inertia - mass*shift(center)
        body.mass, body.ipos = mass, center
        body.fullinertia = [inertia[0, 0], inertia[1, 1], inertia[2, 2],
                            inertia[0, 1], inertia[0, 2], inertia[1, 2]]
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    return spec

def dynamics_comparison(mujoco, model, robot, q, v, u):
    """Independent native mass/bias/acceleration comparison at the same state.

    No scene contacts should be present when using this as a parity gate.
    Contact/constraint/passive forces are reported separately, never concealed.
    """
    data = mujoco.MjData(model)
    data.qpos[:7], data.qvel[:7], data.ctrl[:7] = q, v, u
    mujoco.mj_forward(model, data)
    matrix = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, matrix)
    mass = np.asarray(robot.mass_matrix(q))
    bias = np.asarray(robot.gravity_vector(q) + robot.centrifugal_coriolis_vector(q, v))
    acceleration = np.linalg.solve(mass, np.asarray(u) - bias)
    return {
        "max_mass_matrix_difference": float(np.max(np.abs(matrix[:7, :7] - mass))),
        "max_bias_force_difference_Nm": float(np.max(np.abs(data.qfrc_bias[:7] - bias))),
        "max_acceleration_difference_rad_s2": float(np.max(np.abs(data.qacc[:7] - acceleration))),
        "max_passive_force_Nm": float(np.max(np.abs(data.qfrc_passive[:7]))),
        "max_constraint_force_Nm": float(np.max(np.abs(data.qfrc_constraint[:7]))),
        "max_applied_torque_difference_Nm": float(np.max(np.abs(data.qfrc_actuator[:7] - u))),
    }
