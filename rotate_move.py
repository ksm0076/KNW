#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
import time

class RobotController:
    def __init__(self):
        rclpy.init()
        
        self.node = rclpy.create_node('robot_control_node')

        self.cmdvel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.rs_pallet_axis_sub = self.node.create_subscription(
            Quaternion,
            '/pallet_axis',
            self.pallet_callback,
            10
        )
        self.status_sub = self.node.create_subscription(
            String,
            '/human_status',
            self.status_callback,
            10
        )

        self.current_pose = Pose()
        self.pallet_axis = Quaternion()
        
        self.kp_linear = 0.2  # 직선 PID 제어기 상수 
        self.kp_angular = 1  # 각도 PID 제어기 상수

        self.max_linear_speed = 0.75  # 최대 직선 속도 (m/s)
        self.max_angular_speed = 2.0  # 최대 각도 속도 (rad/s)

        self.last_detection_time = time.time()
        self.searching = False

    def pallet_callback(self, msg):
        self.pallet_axis = msg
        distance = self.calculate_distance()
        self.last_detection_time = time.time()  # 마지막 감지 시간 갱신
        if distance > 1:
            print("Following person")
            self.searching = False

            # PID 제어기를 사용하여 속도 계산
            cmd_vel_msg = Twist()
            target_x = self.pallet_axis.x
            target_y = self.pallet_axis.y
            yaw_error = self.calculate_yaw_error()

            linear_speed = self.kp_linear * distance
            angular_speed = self.kp_angular * yaw_error

            linear_speed = self.limit_speed(linear_speed, self.max_linear_speed)
            angular_speed = self.limit_speed(angular_speed, self.max_angular_speed)

            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = angular_speed

            self.cmdvel_pub.publish(cmd_vel_msg)
            print("Speed:", linear_speed, angular_speed)

        elif distance <= 1 and distance > 0:
            print("STOP")
            self.stop_robot()

    def status_callback(self, msg):
        print("status_callback :", msg)
        print(time.time(), self.last_detection_time, self.searching)
        if msg.data == "lost":
            current_time = time.time()
            if not self.searching and current_time - self.last_detection_time > 3:
                print("Human lost. Starting to rotate.")
                self.searching = True
                self.rotate_to_find_person()
        elif msg.data == "found":
            self.searching = False

    def rotate_to_find_person(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.5  # 회전 속도
        self.cmdvel_pub.publish(cmd_vel_msg)

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmdvel_pub.publish(cmd_vel_msg)
        print("STOP")

    def calculate_distance(self):
        dx = self.pallet_axis.x
        dy = self.pallet_axis.y
        return math.sqrt(dx**2 + dy**2) * 0.001

    def calculate_yaw_error(self):
        _, _, current_yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])
        desired_yaw = math.atan2(self.pallet_axis.x, self.pallet_axis.y)
        desired_yaw -= 1.6
        return current_yaw - desired_yaw

    def limit_speed(self, speed, max_speed):
        return min(speed, max_speed)

if __name__ == '__main__':
    try:
        controller = RobotController()
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass
