"""Repository-owned paths, independent of the caller's working directory.

This research package requires the checkout and its read-only sibling sources.
An installation outside a checkout fails explicitly instead of using another
directory's artifacts or silently creating a new usage ledger.
"""

from pathlib import Path

def repository_root(start=None):
    location = Path(start or __file__).resolve()
    for candidate in (location, *location.parents):
        metadata = candidate / 'pyproject.toml'
        if metadata.is_file() and (candidate / 'uv.lock').is_file():
            import tomllib
            if tomllib.loads(metadata.read_text()).get('project', {}).get('name') == 'jev-cbf-planning':
                return candidate
    raise RuntimeError('jev-cbf requires its research checkout; install with uv sync in that repository')

ROOT = repository_root()

CONFIGS = ROOT / 'configs'

BUDGET_PATH = ROOT / 'artifacts/usage/pickup_research_v1.json'

def inside_repository(path):
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_relative_to(ROOT):
        raise ValueError('all outputs must stay inside this repository')
    return resolved
