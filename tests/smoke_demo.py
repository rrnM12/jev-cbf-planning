"""Bounded GUI smoke check of the original FRAX demo, not a safety evaluation."""

import argparse
import json
from pathlib import Path
import sys
from unittest.mock import patch


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args(argv)
    if args.steps < 1:
        parser.error("--steps must be positive")

    # Import the read-only sibling sources; the demo configures JAX before imports.
    root = Path(__file__).resolve().parents[2]
    sys.dont_write_bytecode = True
    sys.path[:0] = [
        str(root / "frax"),
        str(root / "frax" / "examples"),
        str(root / "cbfpy"),
    ]
    import oscbf_demo as demo

    states, torques = [], []
    captured = {}

    class Finished(Exception):
        """End the bounded run through the original main() cleanup path."""

    class CheckedEnv(demo.ManipulatorEnv):
        """Keep the real upstream viewer/physics; only record and bound the run."""

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            captured["env"] = self
            captured["target"] = self.get_desired_ee_state()[:3].copy()
            states.append(self.get_joint_state().copy())

        def apply_control(self, u):
            u = demo.np.asarray(u)
            if not demo.np.isfinite(u).all():
                raise RuntimeError("Nonfinite torque")
            torques.append(u.copy())
            super().apply_control(u)

        def step(self):
            super().step()
            z = self.get_joint_state().copy()
            if not demo.np.isfinite(z).all():
                raise RuntimeError("Nonfinite robot state")
            states.append(z)
            if self.step_count >= args.steps:
                raise Finished

    original_config = demo.DemoOSCBFConfig

    def capture_config(*args, **kwargs):
        config = original_config(*args, **kwargs)
        captured["config"] = config
        return config

    # Reuse main(), including its nested nominal controller and safety filter.
    # These test-only substitutions do not change gains, geometry, or barriers.
    with (
        patch.object(demo, "ManipulatorEnv", CheckedEnv),
        patch.object(demo, "DemoOSCBFConfig", side_effect=capture_config),
    ):
        try:
            demo.main("panda")
        except Finished:
            pass

    if len(torques) != args.steps:
        raise RuntimeError("Viewer closed before the requested check finished")

    config = captured["config"]
    robot = config.robot
    zs = demo.np.asarray(states)
    hs = demo.np.asarray(demo.jax.jit(demo.jax.vmap(config.h_2))(zs))
    if not demo.np.isfinite(hs).all():
        raise RuntimeError("Nonfinite barrier value")
    ee = demo.np.asarray(demo.jax.jit(demo.jax.vmap(
        lambda q: robot.ee_transform(q)[:3, 3]
    ))(zs[:, :robot.nv]))
    errors = demo.np.linalg.norm(ee - captured["target"], axis=1)

    # This grouping follows the inspected DemoOSCBFConfig.h_2 concatenation order.
    # The values are from that model, not independent mesh collision measurements.
    sphere_count = int(robot.link_collision_data(zs[0, :robot.nv])[1].size)
    obstacle_start = 2 * robot.nv + 1
    obstacle_end = obstacle_start + sphere_count * config.num_collision_bodies
    excess = demo.np.maximum(
        demo.np.abs(demo.np.asarray(torques))
        - demo.np.asarray(robot.actuated_joint_max_forces), 0,
    )
    summary = {
        "robot": "panda",
        "steps": len(torques),
        "simulation_seconds": float(captured["env"].data.time),
        "states_and_torques_finite": True,
        "barrier_count": int(hs.shape[1]),
        "barriers_finite": True,
        "initial_position_error_m": float(errors[0]),
        "final_position_error_m": float(errors[-1]),
        "max_torque_limit_excess_Nm": float(excess.max()),
        "minimum_joint_limit_barrier_rad": float(hs[:, :2 * robot.nv].min()),
        "minimum_singularity_barrier": float(hs[:, 2 * robot.nv].min()),
        "minimum_obstacle_sphere_clearance_m": float(
            hs[:, obstacle_start:obstacle_end].min()
        ),
        "minimum_self_collision_barrier_m": float(
            hs[:, obstacle_end:-sphere_count].min()
        ),
        "minimum_floor_barrier_m": float(hs[:, -sphere_count:].min()),
        "scope": "Default static target; upstream model barriers at sampled states. "
                 "Not an independent safety evaluation or interactive stress test.",
    }
    print("BASELINE_RESULT=" + json.dumps(summary, sort_keys=True), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
