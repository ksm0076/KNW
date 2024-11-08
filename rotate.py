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

        self.current_pose = Pose()
        self.pallet_axis = None  # 사람이 감지되었는지 여부
        self.last_detection_time = time.time()  # 마지막 감지 시간
        self.detection_timeout = 3  # 사람이 사라졌다고 판단하는 시간 (초)

        self.kp_linear = 0.5  # 직선 PID 제어기 상수 
        self.kp_angular = 1  # 각도 PID 제어기 상수

        self.max_linear_speed = 1.0  # 최대 직선 속도 (m/s)
        self.max_angular_speed = 2.0  # 최대 각도 속도 (rad/s)
        
        print("Robot Controller Initialized")

    def pallet_callback(self, msg):
        self.pallet_axis = msg
        self.last_detection_time = time.time()  # 사람이 감지된 시간을 갱신
        self.follow_person()

    def follow_person(self):
        if self.pallet_axis:
            distance = self.calculate_distance()
            if distance > 1:
                print("Following detected person")
                cmd_vel_msg = Twist()

                target_x = self.pallet_axis.x
                target_y = self.pallet_axis.y
                print("Target:", target_x, target_y)

                yaw_error = self.calculate_yaw_error()

                # PID 제어기를 사용한 속도 계산
                linear_speed = self.kp_linear * distance
                angular_speed = self.kp_angular * yaw_error

                # 속도 제한
                linear_speed = self.limit_speed(linear_speed, self.max_linear_speed)
                angular_speed = self.limit_speed(angular_speed, self.max_angular_speed)

                # Twist 메시지에 속도 값 설정
                cmd_vel_msg.linear.x = linear_speed
                cmd_vel_msg.angular.z = angular_speed

                self.cmdvel_pub.publish(cmd_vel_msg)
                print("Speed:", linear_speed, angular_speed)
            else:
                print("STOP")
                self.stop_robot()
        else:
            self.check_for_person_loss()

    def check_for_person_loss(self):
        current_time = time.time()
        if current_time - self.last_detection_time > self.detection_timeout:
            print("Person lost, rotating...")
            self.rotate_to_find_person()

    def rotate_to_find_person(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 0.5  # 회전 속도 설정
        self.cmdvel_pub.publish(cmd_vel_msg)

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmdvel_pub.publish(cmd_vel_msg)

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
        print("Desired yaw:", desired_yaw, "Current yaw:", current_yaw)
        return current_yaw - desired_yaw

    def limit_speed(self, speed, max_speed):
        return min(speed, max_speed)

if __name__ == '__main__':
    try:
        controller = RobotController()
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass
