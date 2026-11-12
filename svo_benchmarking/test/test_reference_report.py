import tempfile
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from svo_ceres_benchmarking.reference_report import report


class ReferenceReportTest(unittest.TestCase):
    def trajectory(self):
        stamps = np.arange(0., 4.1, 0.1)
        xyz = np.column_stack((stamps, np.sin(stamps), 0.2 * stamps ** 2))
        quat = Rotation.from_rotvec(np.column_stack((stamps * 0, stamps * 0, 0.1 * stamps))).as_quat()
        return np.column_stack((stamps, xyz, quat))

    def test_rigid_alignment_and_one_second_rpe(self):
        reference = self.trajectory()
        estimate = reference.copy()
        rotation = Rotation.from_rotvec([0.2, -0.1, 0.4])
        estimate[:, 1:4] = rotation.apply(reference[:, 1:4]) + [3, -2, 1]
        estimate[:, 4:8] = (rotation * Rotation.from_quat(reference[:, 4:8])).as_quat()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            np.savetxt(root / 'estimate', estimate)
            np.savetxt(root / 'reference', reference)
            result = report(root / 'estimate', root / 'reference', root / 'output')
            self.assertEqual(result['associated_poses'], len(reference))
            self.assertGreater(result['rpe_pairs'], 0)
            self.assertLess(result['ate_position_rmse_m'], 1e-10)
            self.assertLess(result['rpe_translation_rmse_m'], 1e-10)
            self.assertLess(result['rpe_rotation_rmse_rad'], 1e-10)
            self.assertTrue((root / 'output/trajectory_reference.png').exists())
            # Scale is deliberately not fitted: a scale error must remain visible.
            estimate[:, 1:4] *= 2
            np.savetxt(root / 'estimate', estimate)
            scaled = report(root / 'estimate', root / 'reference', root / 'scaled')
            self.assertGreater(scaled['ate_position_rmse_m'], 0.1)

    def test_duplicate_stamp_is_an_error(self):
        reference = self.trajectory()
        reference[1, 0] = reference[0, 0]
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'trajectory'
            np.savetxt(source, reference)
            with self.assertRaises(ValueError):
                report(source, source, Path(temporary) / 'output')


if __name__ == '__main__':
    unittest.main()
