"""Meaningful inventory failure cases, using synthetic temporary fixtures only."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import sys

path = Path(__file__).resolve().parents[1] / 'examples/custom-chassis/inspect_urdf.py'
spec = importlib.util.spec_from_file_location('course_urdf', path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)

class URDFInventoryTests(unittest.TestCase):
    def test_mapped_mesh_hash_and_input_unchanged(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / 'mesh.stl').write_bytes(b'synthetic fixture, not genuine CAD')
            urdf = root / 'robot.urdf'
            source = '<robot name="test"><link name="base"><visual><geometry><mesh filename="package://test/mesh.stl"/></geometry></visual></link></robot>'
            urdf.write_text(source)
            result = module.inspect(urdf, {'test': root})
            self.assertTrue(result['valid_inventory'])
            self.assertEqual(result['meshes'][0]['bytes'], 34)
            self.assertEqual(urdf.read_text(), source)

    def test_missing_asset_is_not_success(self):
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp) / 'robot.urdf'
            urdf.write_text('<robot><link name="base"><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link></robot>')
            self.assertFalse(module.inspect(urdf)['valid_inventory'])

    def test_disconnected_cycle_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp) / 'robot.urdf'
            urdf.write_text('<robot><link name="base"/><link name="a"/><link name="b"/><joint name="ab"><parent link="a"/><child link="b"/></joint><joint name="ba"><parent link="b"/><child link="a"/></joint></robot>')
            with self.assertRaises(ValueError):
                module.inspect(urdf)

    def test_duplicate_parent_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp) / 'robot.urdf'
            urdf.write_text('<robot><link name="base"/><link name="a"/><joint name="x"><parent link="base"/><child link="a"/></joint><joint name="y"><parent link="base"/><child link="a"/></joint></robot>')
            with self.assertRaises(ValueError):
                module.inspect(urdf)

    def test_package_escape_is_not_resolved(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            package = root / 'pkg'
            package.mkdir()
            (root / 'secret.stl').write_text('synthetic')
            urdf = package / 'robot.urdf'
            urdf.write_text('<robot><link name="base"><visual><geometry><mesh filename="package://test/../secret.stl"/></geometry></visual></link></robot>')
            self.assertFalse(module.inspect(urdf, {'test': package})['valid_inventory'])

if __name__ == '__main__':
    unittest.main()
