#!/usr/bin/env python3
import os
import threading
from ament_index_python.packages import get_package_share_directory
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from svo_msgs.msg import Info
from std_msgs.msg import String


class SvoWidget(QWidget):
    def __init__(self, node, svo_namespace='/svo'):
        super().__init__()
        self.setObjectName('SvoWidget')
        self._node = node
        self._lock = threading.Lock()
        self._last_info_msg = None
        self._publisher = None
        self._subscriber = None
        self._closed = False
        self._generation = 0
        self._svo_namespace = ''
        loadUi(os.path.join(get_package_share_directory('rqt_svo'), 'resource', 'widget.ui'), self)
        self.num_tracked_bar.setMaximum(120)
        self.button_start.pressed.connect(self.on_start_button_pressed)
        self.button_reset.pressed.connect(self.on_reset_button_pressed)
        self.button_quit.pressed.connect(self.on_quit_button_pressed)
        self.topic_line_edit.setText(svo_namespace)
        self.topic_line_edit.editingFinished.connect(self._on_topic_changed)
        self._update_info_timer = QTimer(self)
        self._update_info_timer.timeout.connect(self.update_info)
        self.register(svo_namespace)
        self._update_info_timer.start(40)

    @Slot()
    def _on_topic_changed(self):
        try:
            self.register(str(self.topic_line_edit.text()))
        except ValueError as error:
            self.svo_info_label.setText(str(error))

    def register(self, namespace):
        namespace = '/' + namespace.strip('/') if namespace.strip('/') else '/svo'
        from rclpy.validate_namespace import validate_namespace
        validate_namespace(namespace)
        if self._closed:
            return
        self.unregister()
        self._svo_namespace = namespace
        with self._lock:
            generation = self._generation
        # Executor callbacks only store a message. All Qt changes occur in the
        # GUI timer, and callbacks from a previous subscription are ignored.
        self._subscriber = self._node.create_subscription(
            Info, namespace + '/info', lambda msg: self.info_cb(msg, generation), 10)
        self._publisher = self._node.create_publisher(String, namespace + '/remote_key', 10)

    def unregister(self):
        with self._lock:
            self._generation += 1
            self._last_info_msg = None
        if self._subscriber is not None:
            self._node.destroy_subscription(self._subscriber)
            self._subscriber = None
        if self._publisher is not None:
            self._node.destroy_publisher(self._publisher)
            self._publisher = None

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        self._update_info_timer.stop()
        self.unregister()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)

    def info_cb(self, message, generation=None):
        with self._lock:
            if self._closed or (generation is not None and generation != self._generation):
                return
            self._last_info_msg = message

    def update_info(self):
        with self._lock:
            message = self._last_info_msg
        if message is None:
            self.svo_info_label.setText('Not Connected')
            self.num_tracked_bar.setValue(0)
            return
        fps = 1.0 / message.processing_time if message.processing_time > 0 else 0.0
        stages = {0: 'PAUSED', 1: 'INITIALIZING', 2: 'TRACKING', 3: 'RELOCALIZING'}
        quality = {0: 'INSUFFICIENT', 1: 'BAD', 2: 'GOOD'}
        self.svo_info_label.setText(
            f'fps = {fps:.2f}\t #Features = {message.num_matches}\t '
            f'{stages.get(message.stage, "UNKNOWN")}\t {quality.get(message.tracking_quality, "UNKNOWN")}')
        self.num_tracked_bar.setValue(message.num_matches)

    def on_start_button_pressed(self):
        self.send_command('s')

    def on_reset_button_pressed(self):
        self.send_command('r')

    def on_quit_button_pressed(self):
        self.send_command('q')

    def send_command(self, command):
        if self._publisher is not None and not self._closed:
            self._publisher.publish(String(data=command))
