"""Independent native dynamics checks, not a real-robot calibration claim."""

import unittest
import numpy as np

from inspect_selection import ROOT
from run_static import load_upstream
from sim_model import dynamics_comparison, panda_spec
from runtime_cbf import guarded_cbf, sphere_runtime_config


class SimulationModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.demo, cls.utils = load_upstream(ROOT.parent / "frax", ROOT.parent / "cbfpy")
        cls.robot = cls.demo.load_panda()

    def test_matched_mass_bias_acceleration_and_preserved_collision_geometry(self):
        mj, robot = self.utils.mujoco, self.robot
        original = panda_spec(mj, ROOT.parent / "frax").compile()
        matched = panda_spec(mj, ROOT.parent / "frax", profile="frax", robot=robot).compile()
        self.assertEqual((original.nq, matched.nq), (9, 7))
        self.assertEqual(original.ngeom, matched.ngeom)
        np.testing.assert_array_equal(original.geom_contype, matched.geom_contype)
        np.testing.assert_array_equal(original.geom_conaffinity, matched.geom_conaffinity)
        for offset in (0., .01, -.02):
            values = dynamics_comparison(mj, matched, robot, robot.default_configuration + offset,
                                         np.arange(7) * .03, np.arange(7))
            for key in ("max_mass_matrix_difference", "max_bias_force_difference_Nm",
                        "max_acceleration_difference_rad_s2"):
                self.assertLess(values[key], 1e-8)
            self.assertEqual(values["max_passive_force_Nm"], 0.)
            self.assertEqual(values["max_constraint_force_Nm"], 0.)
            self.assertEqual(values["max_applied_torque_difference_Nm"], 0.)
        mismatch = dynamics_comparison(mj, original, robot, robot.default_configuration,
                                       np.zeros(7), np.arange(7))
        self.assertGreater(mismatch["max_acceleration_difference_rad_s2"], 1.)

    def test_unknown_or_missing_model_profile_rejected(self):
        for profile in ("invalid", "frax"):
            with self.assertRaises(ValueError):
                panda_spec(self.utils.mujoco, ROOT.parent / "frax", profile=profile)

    def test_guard_only_tightens_scene_rows_and_runtime_motion_is_differentiated(self):
        demo, robot = self.demo, self.robot
        config = sphere_runtime_config(demo, robot, 2)
        plain = guarded_cbf(demo, config)
        guard = guarded_cbf(demo, config, scene_guard=.05)
        z = np.r_[robot.default_configuration, np.zeros(7), 2.]
        args = (np.array([[.6, .3, .2], [10., 10., 10.]]), np.array([.05, .01]),
                np.array([[0., -.1, 0.], [0., 0., 0.]]), np.array([True, False]), 2.)
        u = np.zeros(7)
        delta = np.asarray(plain.h_qp(z, u, *args) - guard.h_qp(z, u, *args))
        np.testing.assert_allclose(delta[15:57], .05)
        np.testing.assert_array_equal(np.r_[delta[:15], delta[57:]], 0.)
        # A native barrier derivative matches finite differences when BOTH robot
        # and obstacle motion are advanced. Removing obstacle velocity changes it.
        h = np.asarray(plain.h(z, *args))
        stopped = list(args); stopped[2] = np.zeros((2, 3))
        self.assertGreater(np.max(abs(h[15:57]-np.asarray(plain.h(z, *stopped))[15:57])), .01)
        np.testing.assert_array_equal(np.asarray(config.h_2(z, *args))[16:57:2], 1.)
        epsilon = 1e-6
        advanced = z.copy(); advanced[-1] += epsilon
        dh = (np.asarray(config.h_2(advanced, *args))-np.asarray(config.h_2(z, *args)))/epsilon
        analytic = h - np.asarray(config.alpha_2(config.h_2(z, *args)))
        np.testing.assert_allclose(dh, analytic, atol=1e-6)
