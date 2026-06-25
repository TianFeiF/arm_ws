# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""rqt plugin entry point for the armv7 unified control panel."""
from rqt_gui_py.plugin import Plugin

from .armv7_widget import Armv7Widget


class Armv7Panel(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('Armv7Panel')
        # rqt already spins context.node on its own executor -- reuse it.
        self._widget = Armv7Widget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()
