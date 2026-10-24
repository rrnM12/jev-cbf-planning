"""Frozen input and producer checks for current and historical experiment data."""

import importlib.metadata
from jev_cbf.paths import ROOT
from jev_cbf.io import digest, read

def source_files(*, legacy=False):
    return tuple(str(p.relative_to(ROOT)) for p in sorted((ROOT/'src/jev_cbf').rglob('*.py'))
                 if legacy or 'legacy' not in p.relative_to(ROOT/'src/jev_cbf').parts)

def _contained_file(base, name):
    path = (base/name).resolve()
    if not path.is_relative_to(base.resolve()) or not path.is_file():
        raise ValueError('missing or invalid provenance path: '+name)
    return path

def check_manifest(folder):
    manifest = read(folder / "manifest.json")
    for name, expected in manifest["frozen_files"].items():
        if digest(_contained_file(folder, name)) != expected:
            raise ValueError(f"frozen input changed: {name}; create a new run, do not reuse answers")
    return manifest

def runtime_versions():
    versions={}
    for name in ("mujoco","jax","elastiqp","typesafe-sdk"):
        try:
            versions[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            if name != "typesafe-sdk":
                raise
            versions[name]=None  # Optional for preparation/offline baselines.
    return versions

def verify_inputs(folder, *, source=True):
    manifest = check_manifest(folder)
    version = manifest.get('format_version', 1)
    if version not in (1, 2):
        raise ValueError('unsupported manifest format: '+str(version))
    hashes = manifest.get('source_sha256', {})
    # Both formats retain the original producer bytes. Historical root names
    # are never remapped to new code or rewritten until their hashes match.
    for name, expected in hashes.items():
        if digest(_contained_file(folder/'source_snapshot', name)) != expected:
            raise ValueError('archived source changed: '+name)
    if source:
        if version == 1 and hashes:
            raise ValueError('legacy producer requires its original revision; read saved results or prepare a fresh run')
        for name, expected in hashes.items():
            if digest(_contained_file(ROOT, name)) != expected:
                raise ValueError('source changed; use a fresh run: '+name)
    return manifest
