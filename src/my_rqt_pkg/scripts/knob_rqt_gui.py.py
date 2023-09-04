import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QLabel

class MyRQTPlugin(Plugin):

    def __init__(self, context):
        super(MyRQTPlugin, self).__init__(context)
        self.setObjectName('MyRQTPlugin')

        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('my_rqt_pkg'), 'resource', 'my_rqt_gui.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MyRQTPluginUi')
        context.add_widget(self._widget)

        # Connect the button to the callback function
        self._widget.toggle_button.clicked.connect(self.toggle_state)

        # Assume the ROS node is already initialized
        self._state = False

    def toggle_state(self):
        # Toggle the state
        self._state = not self._state
        rospy.loginfo("State toggled to: " + str(self._state))
        # Update the GUI
        self._widget.status_label.setText("State: " + ("ON" if self._state else "OFF"))

    # TODO: Implement functions to get joint info and TCP position, and display them on the GUI
