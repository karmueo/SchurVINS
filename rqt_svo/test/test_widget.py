"""GUI lifecycle fixture; QT_QPA_PLATFORM=offscreen or xvfb-run is supported."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import threading
import unittest
import rclpy
from python_qt_binding.QtWidgets import QApplication
from svo_msgs.msg import Info
from rqt_svo.svo_widget import SvoWidget


class WidgetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_create_switch_namespace_and_close(self):
        node = rclpy.create_node('rqt_svo_fixture')
        widget = SvoWidget(node)
        try:
            old_generation = widget._generation
            message = Info(num_matches=42, stage=2, tracking_quality=2)
            callback = threading.Thread(target=widget.info_cb, args=(message, old_generation))
            callback.start()
            callback.join()
            widget.update_info()
            self.assertIn('TRACKING', widget.svo_info_label.text())
            self.assertEqual(widget.num_tracked_bar.value(), 42)
            widget.register('/another_svo')
            widget.info_cb(message, old_generation)
            widget.update_info()
            self.assertEqual(widget.svo_info_label.text(), 'Not Connected')
            widget.close()
            widget.shutdown()
            self.assertFalse(widget._update_info_timer.isActive())
            self.assertIsNone(widget._publisher)
            self.assertIsNone(widget._subscriber)
        finally:
            widget.shutdown()
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
