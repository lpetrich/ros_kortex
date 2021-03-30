#!/usr/bin/env python

from .iris_gui import IrisGuiWidget
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer
import rospy

class IrisGuiPlugin(Plugin):
    def __init__(self, context):
        super(IrisGuiPlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("You may not run more than one instance of IrisGuiPlugin.")
        self.setObjectName('Robotic Assistance Interface')
        self._widget = IrisGuiWidget()
        context.add_widget(self._widget)

        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._widget.pub_check)
        self._update_parameter_timer.start(100)

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)