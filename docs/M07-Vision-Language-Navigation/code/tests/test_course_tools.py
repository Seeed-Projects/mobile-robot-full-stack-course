"""Meaningful offline checks for material preparation and evidence analysis."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EvidenceTests(unittest.TestCase):
    def test_real_history_counts_hashes_and_travel(self):
        result = load('analyze_run').analyze(ROOT / 'evidence', ROOT / 'evidence/actual-state.jsonl', 'navigation-0001')
        self.assertEqual(result['predictions_recorded'], 5)
        self.assertFalse(result['strict_success_reported'])
        self.assertFalse(result['success_independently_reverified'])
        self.assertEqual(result['state_samples'], 2385)
        self.assertAlmostEqual(result['actual_path_m_from_samples'], 1.4285164577, places=8)
        self.assertLess(result['state_recording_window_s'], 12)
        self.assertEqual(result['nonempty_contact_samples'], 0)

    def test_wrong_task_cannot_borrow_another_trajectory(self):
        with self.assertRaises(ValueError):
            load('analyze_run').analyze(ROOT / 'evidence', ROOT / 'evidence/actual-state.jsonl', 'another-task')

    def test_changed_input_cannot_be_attached_to_response(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'outcome.json').write_bytes((ROOT / 'evidence/outcome.json').read_bytes())
            p = root / 'prediction-01'
            p.mkdir()
            (p / 'raw.json').write_bytes((ROOT / 'evidence/prediction-01/raw.json').read_bytes())
            (p / 'input.jpg').write_bytes(b'not the captured image')
            with self.assertRaises(ValueError):
                load('analyze_run').analyze(root)

    def test_nonfinite_motion_rejected_even_with_valid_jpeg(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'outcome.json').write_bytes((ROOT / 'evidence/outcome.json').read_bytes())
            p = root / 'prediction-01'
            p.mkdir()
            raw = json.loads((ROOT / 'evidence/prediction-01/raw.json').read_text())
            raw['response']['data']['actions']['actions'][0][0] = float('nan')
            (p / 'raw.json').write_text(json.dumps(raw))
            (p / 'input.jpg').write_bytes((ROOT / 'evidence/prediction-01/input.jpg').read_bytes())
            with self.assertRaises(ValueError):
                load('analyze_run').analyze(root)


class OverlayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / 'project'
        self.project.mkdir()
        self.material = self.root / 'materials'
        (self.material / 'overlay').mkdir(parents=True)
        self.old, self.new = b'upstream source', b'frozen course source'
        self.target = self.project / 'source.py'
        self.target.write_bytes(self.old)
        (self.material / 'overlay/source.py').write_bytes(self.new)
        manifest = {'upstream_commit': 'fixed-commit', 'files': [{
            'path': 'source.py', 'sha256': hashlib.sha256(self.new).hexdigest(),
            'upstream_sha256': hashlib.sha256(self.old).hexdigest()}]}
        (self.material / 'overlay-manifest.json').write_text(json.dumps(manifest))
        self.module = load('install_overlay')
        self.module.MATERIALS = self.material

    def invoke(self, apply=False, commit='fixed-commit'):
        with patch.object(self.module.subprocess, 'check_output', return_value=commit):
            return self.module.install(self.project, apply)

    def test_dry_run_leaves_target_unchanged(self):
        self.assertEqual(self.invoke()['files_to_copy'], 1)
        self.assertEqual(self.target.read_bytes(), self.old)

    def test_apply_and_second_application_are_idempotent(self):
        self.invoke(True)
        self.assertEqual(self.target.read_bytes(), self.new)
        self.assertEqual(self.invoke(True)['files_to_copy'], 0)

    def test_user_edit_is_preserved(self):
        self.target.write_bytes(b'valuable user edit')
        with self.assertRaises(ValueError):
            self.invoke(True)
        self.assertEqual(self.target.read_bytes(), b'valuable user edit')

    def test_wrong_commit_is_rejected(self):
        with self.assertRaises(ValueError):
            self.invoke(True, 'different-commit')

    def test_symlink_does_not_overwrite_external_file(self):
        external = self.root / 'external.py'
        external.write_bytes(self.old)
        self.target.unlink()
        self.target.symlink_to(external)
        with self.assertRaises(ValueError):
            self.invoke(True)
        self.assertEqual(external.read_bytes(), self.old)


class AssetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.material = self.root / 'materials'
        (self.material / 'configs').mkdir(parents=True)
        self.project = self.root / 'project'
        self.project.mkdir()
        self.content = b'fixed mesh fixture'
        manifest = {'files': [{'file': 'part.stl', 'sha256': hashlib.sha256(self.content).hexdigest()}]}
        config = {'geometry': {'wheel_radius_m': .085}, 'sdk_chassis_config': '/old/sdk/config.json'}
        for name, value in [('asset-manifest.json', manifest), ('chassis.original.json', config), ('sdk-chassis.json', config), ('task.original.json', {'instruction': 'Go to the door and stop.'})]:
            (self.material / 'configs' / name).write_text(json.dumps(value))
        self.module = load('prepare_assets')
        self.module.MATERIALS = self.material

    def bundle(self, name='meshes/part.stl', data=None):
        path = self.root / 'assets.tar.gz'
        data = self.content if data is None else data
        with tarfile.open(path, 'w:gz') as archive:
            member = tarfile.TarInfo(name)
            member.size = len(data)
            archive.addfile(member, io.BytesIO(data))
        return path

    def test_asset_dry_run_does_not_create_output(self):
        self.module.prepare(self.project, self.bundle())
        self.assertFalse((self.project / 'course-assets').exists())

    def test_apply_relocates_json_without_real_sdk(self):
        self.module.prepare(self.project, self.bundle(), True)
        config = json.loads((self.project / 'course-assets/dm-h65/config.json').read_text())
        self.assertTrue(Path(config['sdk_chassis_config']).is_file())
        self.assertEqual(config['geometry']['wheel_radius_m'], .085)
        with self.assertRaises(ValueError):
            self.module.prepare(self.project, self.bundle(), True)

    def test_mesh_tamper_rejected(self):
        with self.assertRaises(ValueError):
            self.module.prepare(self.project, self.bundle(data=b'changed'))

    def test_path_traversal_rejected(self):
        with self.assertRaises(ValueError):
            self.module.prepare(self.project, self.bundle('../escaped.stl'), True)
        self.assertFalse((self.root / 'escaped.stl').exists())


if __name__ == '__main__':
    unittest.main()
