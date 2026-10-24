"""Integration boundaries introduced by the src-package migration."""

import ast
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from jev_cbf.cli import main
from jev_cbf.io import digest, save
from jev_cbf.paths import ROOT, BUDGET_PATH, CONFIGS, repository_root
from jev_cbf.provenance import verify_inputs


class PackageTests(unittest.TestCase):
    def test_every_cli_help_stays_lightweight_and_offline(self):
        commands = [[], ['rgbd'], ['repeated'], ['inspect'], ['inspect','camera'], ['demo']]
        commands += [['rgbd',a] for a in ('prepare','qualify','run','gallery','verify','status')]
        commands += [['repeated',a] for a in ('prepare','run','report','gallery')]
        commands += [['inspect',a] for a in ('geometry','selection')]
        code = '''import sys
from jev_cbf.cli import main
for args in COMMANDS:
 try: main(args+['--help'])
 except SystemExit as error: assert error.code == 0
assert not any(m in sys.modules for m in ('jax','numpy','mujoco','typesafe_sdk','pyrealsense2'))
'''.replace('COMMANDS',repr(commands))
        result = subprocess.run([sys.executable,'-B','-c',code], capture_output=True,text=True,cwd=ROOT.parent)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_paths_and_configs_resolve_from_a_different_working_directory(self):
        result = subprocess.run([sys.executable,'-B','-c',
            'from jev_cbf.paths import ROOT,CONFIGS,BUDGET_PATH; import json; '
            'print(json.dumps([str(ROOT),str(BUDGET_PATH),json.loads((CONFIGS/"scenes/rgbd_pickup.json").read_text())["clutter_counts"]]))'],
            cwd=ROOT.parent,capture_output=True,text=True,check=True)
        root,ledger,counts = json.loads(result.stdout)
        self.assertEqual(root,str(ROOT))
        self.assertEqual(ledger,str(BUDGET_PATH))
        self.assertEqual(counts,dict(easy=1,medium=2,hard=3))
        self.assertTrue((CONFIGS/'prompts/rgbd_pickup.json').is_file())
        self.assertTrue((CONFIGS/'cameras/realsense.example.json').is_file())

    def test_outside_checkout_root_discovery_fails_explicitly(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(RuntimeError,'research checkout'):
                repository_root(Path(temp)/'module.py')

    def test_cli_rejects_outside_output_before_loading_workflow(self):
        with patch('jev_cbf.cli.importlib.import_module') as imports, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main(['rgbd','prepare','--output',str(ROOT.parent/'outside-run')])
            self.assertEqual(error.exception.code,2)
            imports.assert_not_called()

    def test_active_libraries_do_not_import_legacy(self):
        for path in (ROOT/'src/jev_cbf').rglob('*.py'):
            if 'legacy' in path.parts: continue
            for node in ast.walk(ast.parse(path.read_text())):
                modules = [node.module or ''] if isinstance(node,ast.ImportFrom) else [a.name for a in node.names] if isinstance(node,ast.Import) else []
                self.assertFalse(any(m.startswith('jev_cbf.legacy') for m in modules),str(path))

    def test_historical_cli_parsers_survive_extraction(self):
        for module in ('static','suite','pickup','run_showcase','study_pickup',
                       'analyze_pickup','pickup_gallery','static_gallery','selection','view_selection'):
            with self.subTest(module=module):
                result = subprocess.run([sys.executable,'-B','-m','jev_cbf.legacy.'+module,'--help'],
                                        capture_output=True,text=True,cwd=ROOT.parent)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('usage:',result.stdout)


class ProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.run = self.root/'run'

    def fixture(self, *, version=1):
        name = 'old_runner.py' if version==1 else 'src/jev_cbf/producer.py'
        source = self.root/name
        source.parent.mkdir(parents=True,exist_ok=True)
        source.write_text('original producer\n')
        snapshot = self.run/'source_snapshot'/name
        snapshot.parent.mkdir(parents=True)
        snapshot.write_bytes(source.read_bytes())
        save(self.run/'input.json',dict(frozen=True))
        manifest = dict(format_version=version, frozen_files={'input.json':digest(self.run/'input.json')}, source_sha256={name:digest(source)})
        save(self.run/'manifest.json',manifest)
        return source,snapshot

    def test_historical_read_checks_archived_bytes_without_requiring_old_root_files(self):
        source,snapshot = self.fixture()
        before = {p:p.read_bytes() for p in self.run.rglob('*') if p.is_file()}
        source.unlink()
        self.assertEqual(verify_inputs(self.run,source=False)['format_version'],1)
        self.assertEqual(before,{p:p.read_bytes() for p in self.run.rglob('*') if p.is_file()})
        with self.assertRaisesRegex(ValueError,'original revision'):
            verify_inputs(self.run)

    def test_historical_snapshot_tamper_is_rejected(self):
        _,snapshot = self.fixture()
        snapshot.write_text('changed\n')
        with self.assertRaisesRegex(ValueError,'archived source changed'):
            verify_inputs(self.run,source=False)

    def test_new_producer_checks_current_and_snapshot_bytes_separately(self):
        source,_ = self.fixture(version=2)
        with patch('jev_cbf.provenance.ROOT',self.root):
            verify_inputs(self.run)
            source.write_text('new producer\n')
            verify_inputs(self.run,source=False)
            with self.assertRaisesRegex(ValueError,'source changed'):
                verify_inputs(self.run)

    def test_frozen_input_change_and_escape_are_rejected(self):
        self.fixture()
        save(self.run/'input.json',dict(frozen=False))
        with self.assertRaisesRegex(ValueError,'frozen input changed'):
            verify_inputs(self.run,source=False)
        save(self.run/'manifest.json',dict(frozen_files={'../old_runner.py':'ignored'}, source_sha256={}))
        with self.assertRaisesRegex(ValueError,'invalid provenance path'):
            verify_inputs(self.run,source=False)

    def test_unsupported_manifest_format_fails_closed(self):
        save(self.run/'manifest.json',dict(format_version=999,frozen_files={},source_sha256={}))
        with self.assertRaisesRegex(ValueError,'unsupported manifest'):
            verify_inputs(self.run,source=False)
