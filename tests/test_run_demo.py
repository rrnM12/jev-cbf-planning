"""Offline checks for the launcher; these do not test robot/CBF behavior."""

import io
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import run_demo


class LauncherTests(unittest.TestCase):
    def test_child_environment_uses_source_checkouts_and_disables_bytecode(self):
        parent = {"PYTHONPATH": "/existing", "UNCHANGED": "value"}
        env = run_demo.child_environment(Path("/frax"), Path("/cbfpy"), parent)
        self.assertEqual(
            env["PYTHONPATH"], os.pathsep.join(("/frax", "/cbfpy", "/existing"))
        )
        self.assertEqual(env["PYTHONDONTWRITEBYTECODE"], "1")
        self.assertEqual(env["UNCHANGED"], "value")
        self.assertNotIn("PYTHONDONTWRITEBYTECODE", parent)
        self.assertEqual(parent["PYTHONPATH"], "/existing")

    def test_macos_selects_active_environment_mjpython(self):
        with (
            patch.object(sys, "platform", "darwin"),
            patch.object(sys, "prefix", "/env"),
        ):
            self.assertEqual(run_demo.viewer_python(), Path("/env/bin/mjpython"))

    def test_other_platforms_use_active_python(self):
        with patch.object(sys, "platform", "linux"):
            self.assertEqual(run_demo.viewer_python(), Path(sys.executable))

    def test_missing_sources_are_reported(self):
        with (
            patch.object(Path, "is_file", return_value=False),
            patch.object(run_demo.importlib.util, "find_spec", return_value=object()),
        ):
            errors = run_demo.validation_errors(Path("/frax"), Path("/cbfpy"))
        self.assertTrue(
            any("/frax/examples/oscbf_demo.py" in error for error in errors)
        )
        self.assertTrue(any("/cbfpy/cbfpy/__init__.py" in error for error in errors))

    def test_check_mode_does_not_launch_or_change_directory(self):
        with (
            patch.object(run_demo, "validation_errors", return_value=[]),
            patch.object(run_demo.os, "execve") as launch,
            patch.object(run_demo.os, "chdir") as chdir,
            patch("sys.stdout", new_callable=io.StringIO),
        ):
            self.assertEqual(run_demo.main(["--check"]), 0)
            launch.assert_not_called()
            chdir.assert_not_called()

    def test_incomplete_setup_prevents_launch(self):
        with (
            patch.object(
                run_demo, "validation_errors",
                return_value=["Missing runtime modules: jax"],
            ),
            patch.object(run_demo.os, "execve") as launch,
            patch("sys.stderr", new_callable=io.StringIO) as stderr,
        ):
            self.assertEqual(run_demo.main([]), 1)
            launch.assert_not_called()
            self.assertIn("uv sync", stderr.getvalue())

    def test_launch_executes_original_script_with_requested_robot(self):
        with (
            patch.object(run_demo, "validation_errors", return_value=[]),
            patch.object(run_demo.os, "execve") as launch,
            patch.object(run_demo.os, "chdir"),
            patch("sys.stdout", new_callable=io.StringIO),
        ):
            result = run_demo.main([
                "--robot", "iiwa", "--frax-dir", "/frax", "--cbfpy-dir", "/cbfpy",
            ])
        self.assertEqual(result, 0)
        executable, command, env = launch.call_args.args
        self.assertEqual(
            command, [executable, "/frax/examples/oscbf_demo.py", "--robot", "iiwa"]
        )
        self.assertEqual(env["PYTHONDONTWRITEBYTECODE"], "1")


if __name__ == "__main__":
    unittest.main()
