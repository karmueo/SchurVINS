import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import yaml
from vikit_py.ros_node import RosNode
from vikit_py.depthmap_utils import load_depthmap, MapIOError


class ToolContractTest(unittest.TestCase):
    def test_ros2_parameters_and_quoted_paths(self):
        with tempfile.TemporaryDirectory(prefix='svo fixture ') as temporary:
            runner = RosNode('svo_ros', 'svo_benchmark')
            with patch('vikit_py.ros_node.subprocess.run') as process:
                runner.run({'camera': {'delay': 0.0}, 'T_world_imuinit/qw': 1.0,
                            'flags': {'trial_idx': 2}}, log_cpu_usage=False,
                           log_cpu_usage_folder=temporary)
            command = process.call_args.args[0]
            self.assertEqual(command[:4], ['ros2', 'run', 'svo_ros', 'svo_benchmark'])
            self.assertIn('--trial_idx=2', command)
            self.assertEqual(command[-1], str(Path(temporary) / 'params_ros.yaml'))
            params = yaml.safe_load((Path(temporary) / 'params_ros.yaml').read_text())['/**']['ros__parameters']
            self.assertEqual(params['camera.delay'], 0.0)
            self.assertEqual(params['T_world_imuinit.qw'], 1.0)
            self.assertNotIn('flags', params)

    def test_depth_format_failure_is_explicit(self):
        with self.assertRaises(MapIOError):
            load_depthmap('unsupported.txt', 2, 3)


if __name__ == '__main__':
    unittest.main()
