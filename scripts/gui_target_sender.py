#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QLabel,
)
from PyQt5.QtCore import Qt
import sys


class SliderPublisher(Node):
    def __init__(self):
        super().__init__("slider_publisher")
        self.publisher = self.create_publisher(
            Float64MultiArray, "crocoddyl_controller/target", 10
        )
        self.slider_values = [0.0, 0.0, 0.0]

        self.app = QApplication(sys.argv)

        self.main_window = QMainWindow()
        self.main_widget = QWidget(self.main_window)
        self.main_layout = QVBoxLayout(self.main_widget)
        self.main_window.setCentralWidget(self.main_widget)

        self.slider1 = QSlider(Qt.Horizontal)
        self.slider2 = QSlider(Qt.Horizontal)
        self.slider3 = QSlider(Qt.Horizontal)

        self.value_label1 = QLabel("0.00")
        self.value_label2 = QLabel("0.00")
        self.value_label3 = QLabel("0.00")

        self.slider1.setMinimum(0)
        self.slider1.setMaximum(100)
        self.slider1.setSingleStep(1)

        self.slider2.setMinimum(-100)
        self.slider2.setMaximum(100)
        self.slider2.setSingleStep(1)

        self.slider3.setMinimum(0)
        self.slider3.setMaximum(200)
        self.slider3.setSingleStep(10)

        self.label1 = QLabel("x")
        self.label2 = QLabel("y")
        self.label3 = QLabel("z")

        slider_widget1 = QWidget()
        slider_layout1 = QVBoxLayout()
        slider_layout1.addWidget(self.label1)
        slider_layout1.addWidget(self.slider1)
        slider_layout1.addWidget(self.value_label1)
        slider_widget1.setLayout(slider_layout1)

        slider_widget2 = QWidget()
        slider_layout2 = QVBoxLayout()
        slider_layout2.addWidget(self.label2)
        slider_layout2.addWidget(self.slider2)
        slider_layout2.addWidget(self.value_label2)
        slider_widget2.setLayout(slider_layout2)

        slider_widget3 = QWidget()
        slider_layout3 = QVBoxLayout()
        slider_layout3.addWidget(self.label3)
        slider_layout3.addWidget(self.slider3)
        slider_layout3.addWidget(self.value_label3)
        slider_widget3.setLayout(slider_layout3)

        self.slider1.valueChanged.connect(self.slider_changed)
        self.slider2.valueChanged.connect(self.slider_changed)
        self.slider3.valueChanged.connect(self.slider_changed)

        self.slider1.valueChanged.connect(self.update_value_label1)
        self.slider2.valueChanged.connect(self.update_value_label2)
        self.slider3.valueChanged.connect(self.update_value_label3)

        self.publish_button = QPushButton("Send target")
        self.publish_button.clicked.connect(self.publish_values)

        self.main_layout.addWidget(slider_widget1)
        self.main_layout.addWidget(slider_widget2)
        self.main_layout.addWidget(slider_widget3)
        self.main_layout.addWidget(self.publish_button)

        self.main_window.show()
        self.app.exec_()

    def slider_changed(self):
        self.slider_values[0] = self.slider1.value() / 100.0
        self.slider_values[1] = self.slider2.value() / 100.0
        self.slider_values[2] = self.slider3.value() / 100.0

    def update_value_label1(self, value):
        self.value_label1.setText(f"{value/100}")

    def update_value_label2(self, value):
        self.value_label2.setText(f"{value/100}")

    def update_value_label3(self, value):
        self.value_label3.setText(f"{value/100}")

    def publish_values(self):
        msg = Float64MultiArray(data=self.slider_values)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    slider_publisher = SliderPublisher()
    rclpy.spin(slider_publisher)
    slider_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
