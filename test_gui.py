import sys
import math
import subprocess
from threading import Thread
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QWidget, QMessageBox
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.is_measuring_distance = False
        self.speed = 0.0
        self.current_speed = 0.0
        self.running_time = 0
        self.create_timer(1.0, self.update_running_time)

    def odom_callback(self, msg):
        if self.is_measuring_distance:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            if self.prev_x is not None and self.prev_y is not None:
                distance_increment = math.sqrt((current_x - self.prev_x) ** 2 + (current_y - self.prev_y) ** 2)
                self.total_distance += distance_increment
            self.prev_x = current_x
            self.prev_y = current_y

        self.current_speed = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2 +
            msg.twist.twist.linear.z ** 2
        )

    def update_running_time(self):
        if self.is_measuring_distance:
            self.running_time += 1
            if self.running_time > 0:
                self.speed = (self.total_distance * 0.001) / (self.running_time * (1 / 3600))

    def start_distance_measurement(self):
        self.is_measuring_distance = True
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.speed = 0.0
        self.running_time = 0
        print("Distance measurement START!")

    def stop_distance_measurement(self):
        self.is_measuring_distance = False
        print("Distance measurement STOP!")


class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.move_process = None
        self.is_measuring = False
        self.init_ui()
        rclpy.init()
        self.distance_tracker = DistanceTracker()

        self.ros_thread = Thread(target=rclpy.spin, args=(self.distance_tracker,), daemon=True)
        self.ros_thread.start()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(500)

    def init_ui(self):
        self.setWindowTitle("Robot Control UI")
        self.setGeometry(100, 100, 600, 500)

        self.move_button = QPushButton("Run Move", self)
        self.move_button.setFixedSize(400, 140)
        self.measure_button = QPushButton("Start Measure", self)
        self.measure_button.setFixedSize(400, 140)

        self.running_time_label = QLabel("Running Time: 00:00")
        self.total_distance_label = QLabel("Total Distance: 0.00 m")
        self.average_speed_label = QLabel("Average Speed: 0.00 km/h")
        self.current_speed_label = QLabel("Current Speed: 0.00 m/s")

        for label in [self.running_time_label, self.total_distance_label, self.average_speed_label, self.current_speed_label]:
            label.setStyleSheet("font-size: 40px; font-weight: bold;")
            label.setFixedHeight(60)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.move_button)
        button_layout.addWidget(self.measure_button)

        self.move_button.clicked.connect(self.toggle_move)
        self.measure_button.clicked.connect(self.toggle_measure)

        layout = QVBoxLayout()
        layout.addLayout(button_layout)
        layout.addWidget(self.running_time_label)
        layout.addWidget(self.total_distance_label)
        layout.addWidget(self.average_speed_label)
        layout.addWidget(self.current_speed_label)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def toggle_move(self):
        if self.move_process is None:
            self.run_move()
        else:
            self.stop_move()

    def run_move(self):
        try:
            self.move_process = subprocess.Popen(
                ["python3", "move_limit_speed.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.move_button.setText("Stop Move")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error running Move: {e}")

    def stop_move(self):
        if self.move_process:
            self.move_process.terminate()
            self.move_process = None
        self.move_button.setText("Run Move")

    def toggle_measure(self):
        if not self.is_measuring:
            self.start_measure()
        else:
            self.stop_measure()

    def start_measure(self):
        self.distance_tracker.start_distance_measurement()
        self.is_measuring = True
        self.measure_button.setText("Stop Measure")

    def stop_measure(self):
        self.distance_tracker.stop_distance_measurement()
        self.is_measuring = False
        self.measure_button.setText("Start Measure")

    def update_ui(self):
        if self.distance_tracker.is_measuring_distance:
            time_str = f"{self.distance_tracker.running_time // 60:02}:{self.distance_tracker.running_time % 60:02}"
            self.running_time_label.setText(f"Running Time: {time_str}")
            self.total_distance_label.setText(f"Total Distance: {self.distance_tracker.total_distance:.2f} m")
            self.average_speed_label.setText(f"Average Speed: {self.distance_tracker.speed:.2f} km/h")
        self.current_speed_label.setText(f"Current Speed: {self.distance_tracker.current_speed:.2f} m/s")

    def closeEvent(self, event):
        if self.move_process:
            self.move_process.terminate()
        self.distance_tracker.destroy_node()
        rclpy.shutdown()
        self.ros_thread.join()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    main_window = RobotControlUI()
    main_window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

