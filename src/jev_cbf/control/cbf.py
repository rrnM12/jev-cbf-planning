"""Small CBFpy adapters: sampled-control guard and runtime sphere parameters.

All barrier geometry, robot protections, dynamics, objectives, gains, and the QP
solver are delegated to the existing FRAX/CBFpy implementations.
"""

import numpy as np

def guarded_cbf(demo, config, *, scene_guard=0.):
    """Tighten scene acceleration inequalities, not obstacle geometry.

    For metre-valued RD2 sphere barriers, guard has units m/s². It is an
    empirical sampled-control allowance, NOT calibrated perception uncertainty
    or a formal continuous-time robustness bound. Zero preserves upstream.
    """
    if scene_guard < 0:
        raise ValueError("scene guard must be nonnegative")
    if not scene_guard:
        return demo.CBF.from_config(config)

    @demo.jax.tree_util.register_static
    class GuardedCBF(demo.CBF):
        def h_qp(self, z, u_des, *args, **kwargs):
            value = super().h_qp(z, u_des, *args, **kwargs)
            first = 2 * config.num_joints + 1
            last = first + getattr(config, "scene_pair_count", config.num_collision_bodies * len(config.robot.link_collision_data(z[:7])[1]))
            return value.at[first:last].add(-scene_guard)

    return GuardedCBF.from_config(config)

def sphere_runtime_config(demo, robot, capacity, *, pairwise=False, tool_spheres=None):
    """Reuse demo barriers with fixed-shape runtime covers and explicit time.

    State is [q(7), v(7), time]. Arguments are reference world centers,
    radii, constant velocities, enabled flags, and reference time. The time
    state makes CBFpy differentiate obstacle motion in its RD2 Lie derivatives.
    No min/union reduction is used: every enabled robot/obstacle pair remains
    a separate smooth constraint. Dormant slots are constant positive barriers.
    """
    from cbfpy import CBFConfig
    from jev_cbf.upstream import Overlay
    import jax.numpy as jnp

    if not isinstance(capacity, int) or capacity < 1:
        raise ValueError("capacity must be a positive integer")
    base = demo.DemoOSCBFConfig(robot, np.full((capacity, 3), 10.), np.full(capacity, .01))
    tool_centers, tool_radii = tool_spheres if tool_spheres is not None else (np.empty((0, 3)), np.empty(0))
    count = len(robot.link_collision_data(robot.default_configuration)[1]) + len(tool_radii)
    seeds = (np.full((capacity, 3), 10.), np.full(capacity, .01),
             np.zeros((capacity, 3)), np.zeros((count, capacity) if pairwise else capacity, dtype=bool), 0.)
    first, last = 15, 15 + capacity*count

    @demo.jax.tree_util.register_static
    class RuntimeConfig(CBFConfig):
        def __init__(self):
            self.robot, self.num_joints, self.num_collision_bodies = robot, 7, capacity
            self.scene_robot_sphere_count, self.scene_pair_count = count, count*capacity
            super().__init__(n=15, m=7, u_min=base.u_min, u_max=base.u_max,
                             backend=base.backend, solver_tol=base.solver_tol,
                             relax_qp=base.relax_qp,
                             cbf_relaxation_penalty=base.cbf_relaxation_penalty,
                             control_relaxation_penalty=base.control_relaxation_penalty,
                             init_args=seeds)

        def f(self, z, *args):
            return jnp.concatenate([base.f(z[:14]), jnp.ones(1)])

        def g(self, z, *args):
            return jnp.concatenate([base.g(z[:14]), jnp.zeros((1, 7))])

        def h_2(self, z, centers, radii, velocities, enabled, reference_time):
            moving = centers + velocities * (z[14] - reference_time)
            raw = type(base).h_2(Overlay(base, collision_positions=moving, collision_radii=radii), z[:14])
            if len(tool_radii):
                transform = robot.ee_transform(z[:7])
                points = jnp.asarray(tool_centers) @ transform[:3, :3].T + transform[:3, 3]
                clearance = jnp.linalg.norm(points[:, None]-moving[None], axis=-1)-jnp.asarray(tool_radii)[:, None]-radii[None]
                raw = jnp.concatenate([raw[:15+21*capacity], clearance.reshape(-1),
                                       raw[15+21*capacity:], points[:, 2]-jnp.asarray(tool_radii)-.01])
            active = jnp.broadcast_to(enabled, (count, capacity)).reshape(-1)
            return raw.at[first:last].set(jnp.where(active, raw[first:last], 1.))

        def alpha(self, h, *args):
            return base.alpha(h)

        def alpha_2(self, h, *args):
            return base.alpha_2(h)

        def P(self, z, u_des, *args):
            return base.P(z[:14], u_des)

        def q(self, z, u_des, *args):
            return base.q(z[:14], u_des)

    return RuntimeConfig()
