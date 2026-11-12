#!/usr/bin/env python3
import os
import argparse
from qt_gui.plugin import Plugin
from .svo_widget import SvoWidget

class Svo(Plugin):
  """
  Subclass of Plugin to display SVO status
  """
  def __init__(self, context):

    # Init Plugin
    super(Svo, self).__init__(context)
    self.setObjectName('SvoPlugin')

    # Load arguments
    # TODO load topic name from args
    args = self._parse_args(context.argv())

    # Create QWidget
    self._widget = SvoWidget(context.node, svo_namespace=args.topic)

    # Show _widget.windowTitle on left-top of each plugin (when
    # it's set in _widget). This is useful when you open multiple
    # plugins at once. Also if you open multiple instances of your
    # plugin at once, these lines add number to make it easy to
    # tell from pane to pane.
    if context.serial_number() > 1:
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

    # Add widget to the user interface
    context.add_widget(self._widget)


  def _parse_args(self, argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/svo', help='SVO namespace')
    args, unknown = parser.parse_known_args(argv)
    return args

  # def _parse_args(self, argv):
  #   parser = argparse.ArgumentParser(prog='rqt_svo', add_help=False)
  #   group = parser.add_argument_group('Options for rqt_svo plugin')
  #   group.add_argument('--topic', type=argparse.FileType('r'), nargs='*', default=[], help='Svo Info Topic to display')
  #   return parser.parse_args(argv)

  def save_settings(self, plugin_settings, instance_settings):
    instance_settings.set_value('namespace', self._widget._svo_namespace)

  def restore_settings(self, plugin_settings, instance_settings):
    namespace = str(instance_settings.value('namespace', self._widget._svo_namespace))
    self._widget.topic_line_edit.setText(namespace)
    self._widget.register(namespace)

  def shutdown_plugin(self):
    self._widget.shutdown()
