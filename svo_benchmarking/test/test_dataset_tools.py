import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np


def tool(name):
    path = Path(__file__).resolve().parents[1] / 'dataset_tools' / 'python' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DatasetToolsTest(unittest.TestCase):
    def test_stereo_pairing_skips_comments_and_preserves_paths(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'left').write_text('# id stamp image\n0 10.000000000001 img/l.png\n')
            (root / 'right').write_text('# id stamp image\n7 10.000000000001 img/r.png\n')
            count = tool('combine_images').combine(root / 'left', root / 'right', root / 'output')
            self.assertEqual(count, 1)
            self.assertIn('img/l.png img/r.png', (root / 'output').read_text())
            (root / 'right').write_text('7 10.000000000002 img/r.png\n')
            with self.assertRaises(ValueError):
                tool('combine_images').combine(root / 'left', root / 'right', root / 'output')

    def test_single_pose_hand_eye_right_multiplication(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            np.savetxt(root / 'source', [[0, 1.0, 1, 2, 3, 0, 0, 0, 1]])
            matrix = np.eye(4)
            matrix[0, 3] = 0.2
            np.savetxt(root / 'matrix', matrix, delimiter=',')
            count = tool('transform_trajectory').transform(root / 'source', root / 'matrix', root / 'output')
            self.assertEqual(count, 1)
            np.testing.assert_allclose(np.loadtxt(root / 'output')[2:5], [1.2, 2, 3])


if __name__ == '__main__':
    unittest.main()
