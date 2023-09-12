#!/usr/bin/env python3

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QDoubleSpinBox, QLabel, QLineEdit
from PyQt5.QtChart import QChart, QChartView, QLineSeries, QValueAxis
from PyQt5.QtGui import QPainter
from PyQt5.QtCore import Qt


import sys
import math
import rospy
from knob_robot_control.msg import KnobState, KnobCommand
from std_msgs.msg import String, Int32, Float32
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QDoubleSpinBox, QLabel, QLineEdit
from knob_controller import KnobController


class KnobCommandInterface(QWidget):
    def __init__(self):
        super().__init__()

        # Initialize ROS node
        rospy.init_node("knob_command_interface", anonymous=True)

        self.knob_command_pub = rospy.Publisher("/knob_command", KnobCommand, queue_size=10)

        # Create GUI layout
        layout = QVBoxLayout()

        # Initialize chart
        self.chart = QChart()
        self.chart.setTitle("Example Chart")
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRenderHint(QPainter.Antialiasing)
        layout.addWidget(self.chart_view)

        # Create a series for the chart (you can add data points to this series)
        self.series = QLineSeries()
        self.chart.addSeries(self.series)

        # Create and set the axis for the chart
        self.axisX = QValueAxis()
        self.axisY = QValueAxis()
        self.axisX.setTitleText("X Axis")
        self.axisY.setTitleText("Y Axis")
        self.chart.setAxisX(self.axisX, self.series)
        self.chart.setAxisY(self.axisY, self.series)
        self.axisX.setRange(0, 10)  # Example range for x-axis
        self.axisY.setRange(0, 10)  # Example range for y-axis


        # Create widgets for each KnobCommand parameter
        self.num_positions_spinbox = QDoubleSpinBox()
        self.num_positions_spinbox.setRange(0, 100)  # Example range, adjust as needed
        self.num_positions_spinbox.setValue(11)
        layout.addWidget(QLabel("Num Positions"))
        layout.addWidget(self.num_positions_spinbox)

        self.position_spinbox = QDoubleSpinBox()
        self.position_spinbox.setRange(0, 100)  # Example range, adjust as needed
        self.position_spinbox.setValue(0)
        layout.addWidget(QLabel("Position"))
        layout.addWidget(self.position_spinbox)

        self.position_width_radians_spinbox = QDoubleSpinBox()
        self.position_width_radians_spinbox.setRange(0, 100)  # Example range, adjust as needed
        self.position_width_radians_spinbox.setValue(10 * math.pi / 180)
        layout.addWidget(QLabel("Position Width Radians"))
        layout.addWidget(self.position_width_radians_spinbox)

        self.detent_strength_unit_spinbox = QDoubleSpinBox()
        self.detent_strength_unit_spinbox.setRange(0, 100)  # Example range, adjust as needed
        self.detent_strength_unit_spinbox.setValue(0)
        layout.addWidget(QLabel("Detent Strength Unit"))
        layout.addWidget(self.detent_strength_unit_spinbox)

        self.endstop_strength_unit_spinbox = QDoubleSpinBox()
        self.endstop_strength_unit_spinbox.setRange(0, 100)  # Example range, adjust as needed
        self.endstop_strength_unit_spinbox.setValue(1)
        layout.addWidget(QLabel("Endstop Strength Unit"))
        layout.addWidget(self.endstop_strength_unit_spinbox)

        self.snap_point_spinbox = QDoubleSpinBox()
        self.snap_point_spinbox.setRange(0, 100)  # Example range, adjust as needed
        self.snap_point_spinbox.setValue(1.1)
        layout.addWidget(QLabel("Snap Point"))
        layout.addWidget(self.snap_point_spinbox)

        self.text_line_edit = QLineEdit()
        self.text_line_edit.setText("Bounded 0-10\nNo detents")
        layout.addWidget(QLabel("Text"))
        layout.addWidget(self.text_line_edit)


        # Create a publish button and connect it to the publish function
        publish_button = QPushButton("Publish")
        publish_button.clicked.connect(self.publish_knob_command)
        layout.addWidget(publish_button)

        # std_msgs/Float32 tcp_force
        self.tcp_force_spinbox = QDoubleSpinBox()
        self.tcp_force_spinbox.setRange(0, 3)  # Example range, adjust as needed
        self.tcp_force_spinbox.setValue(0)  
        layout.addWidget(QLabel("TCP Force"))
        layout.addWidget(self.tcp_force_spinbox)

        # Create a publish button and connect it to the publish function
        publish_button = QPushButton("Publish Force")
        publish_button.clicked.connect(self.publish_force)
        layout.addWidget(publish_button)


        # Create a publish button and connect it to the publish function
        publish_button = QPushButton("Force Tesing")
        publish_button.clicked.connect(self.force_testing)
        layout.addWidget(publish_button)

        self.setLayout(layout)
        self.setWindowTitle("Knob Command Interface")

    def force_testing(self):
        # 10s of force testing
        for i in range(1000):
            knob_command = KnobCommand()
            knob_command.text.data = "force"
            knob_command.tcp_force.data = math.sin(rospy.Time.now().to_sec()) + 1.5
            self.knob_command_pub.publish(knob_command)
            rospy.sleep(0.01)


    def publish_force(self):
        knob_command = KnobCommand()
        knob_command.text.data = "force"
        knob_command.tcp_force.data = float(self.tcp_force_spinbox.value())
        self.knob_command_pub.publish(knob_command)

    def publish_knob_command(self):
        knob_command = KnobCommand()
        knob_command.header.stamp = rospy.Time.now()
        knob_command.num_positions.data = int(self.num_positions_spinbox.value())
        knob_command.position.data = int(self.position_spinbox.value())
        knob_command.position_width_radians.data = float(self.position_width_radians_spinbox.value())
        knob_command.detent_strength_unit.data = float(self.detent_strength_unit_spinbox.value())
        knob_command.endstop_strength_unit.data = float(self.endstop_strength_unit_spinbox.value())
        knob_command.snap_point.data = float(self.snap_point_spinbox.value())
        knob_command.text.data = self.text_line_edit.text()
        knob_command.tcp_force.data = float(1.0)
        self.knob_command_pub.publish(knob_command)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = KnobCommandInterface()
    window.show()
    sys.exit(app.exec_())
