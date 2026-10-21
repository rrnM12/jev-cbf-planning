"""Launch the unchanged FRAX OSCBF demo using read-only sibling checkouts."""

import argparse
import importlib.util
import os
from pathlib import Path
import sys


PROJECT_DIR = Path(__file__).resolve().parent
RUNTIME_MODULES = (
    "elastiqp",
    "jax",
    "jaxlib",
    "lxml",
    "mujoco",
    "networkx",
    "numpy",
    "packaging",
    "qpax",
    "yaml",
)


def viewer_python():
    """MuJoCo's passive viewer needs mjpython on macOS, not ordinary Python."""
    if sys.platform == "darwin":
        return Path(sys.prefix) / "bin" / "mjpython"
    return Path(sys.executable)


def validation_errors(frax_dir, cbfpy_dir):
    """Check source paths and package presence without initializing JAX or a GUI."""
    errors = []
    required_files = (
        frax_dir / "frax" / "__init__.py",
        frax_dir / "examples" / "oscbf_demo.py",
        frax_dir / "examples" / "example_utils.py",
        frax_dir / "examples" / "cbf_utils.py",
        cbfpy_dir / "cbfpy" / "__init__.py",
    )
    for path in required_files:
        if not path.is_file():
            errors.append(f"Missing upstream source: {path}")

    missing = [
        name for name in RUNTIME_MODULES if importlib.util.find_spec(name) is None
    ]
    if missing:
        errors.append("Missing runtime modules: " + ", ".join(missing))
    if not viewer_python().is_file():
        errors.append(f"Missing viewer interpreter: {viewer_python()}")
    return errors


def child_environment(frax_dir, cbfpy_dir, parent):
    """Use the local sources only in the demo process; don't write upstream pyc files."""
    env = dict(parent)
    paths = [str(frax_dir), str(cbfpy_dir)]
    if env.get("PYTHONPATH"):
        paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(paths)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return env


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("panda", "iiwa"), default="panda")
    parser.add_argument("--frax-dir", type=Path, default=PROJECT_DIR.parent / "frax")
    parser.add_argument("--cbfpy-dir", type=Path, default=PROJECT_DIR.parent / "cbfpy")
    parser.add_argument(
        "--check", action="store_true", help="Check setup without opening a viewer"
    )
    args = parser.parse_args(argv)
    frax_dir = args.frax_dir.expanduser().resolve()
    cbfpy_dir = args.cbfpy_dir.expanduser().resolve()

    errors = validation_errors(frax_dir, cbfpy_dir)
    if errors:
        print("Demo setup is incomplete:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        print(
            "Run `uv sync --cache-dir .uv-cache --python 3.12`, then "
            "`uv run --cache-dir .uv-cache python run_demo.py --check`.",
            file=sys.stderr,
        )
        return 1

    print(f"FRAX source: {frax_dir}")
    print(f"CBFpy source: {cbfpy_dir}")
    print(f"Viewer interpreter: {viewer_python()}")
    if args.check:
        print(
            "Setup presence checks passed; "
            "controller/physics compatibility is not yet tested."
        )
        return 0

    command = [
        str(viewer_python()),
        str(frax_dir / "examples" / "oscbf_demo.py"),
        "--robot", args.robot,
    ]
    print(f"Launching the unchanged FRAX OSCBF demo ({args.robot}).", flush=True)
    # Execute the original script so its sibling imports and asset paths work as authored.
    # Its main(), nominal controller, CBF configuration, and environment remain upstream.
    env = child_environment(frax_dir, cbfpy_dir, os.environ)
    os.chdir(PROJECT_DIR)
    try:
        os.execve(command[0], command, env)
    except OSError as error:
        print(f"Could not launch the upstream demo: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
