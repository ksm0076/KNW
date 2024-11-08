#!/usr/bin/env python
import queue
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

        self.chatter_pub = self.node.create_publisher(String, 'chatter', 10)
        self.cmdvel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.switch_g2p_pub = self.node.create_publisher(String, 'switch/g2p', 10)
        self.rs_pallet_axis_sub = self.node.create_subscription(
            Quaternion,
            '/pallet_axis',
            self.pallet_callback,
            10
        )

        self.current_pose = Pose()
        self.pallet_axis = Quaternion()
        
        self.pallet = 0

        self.kp_linear = 0.5  # 직선 PID 제어기 상수 
        self.kp_angular = 1  # 각도 PID 제어기 상수

        self.max_linear_speed = 1.0  # 최대 직선 속도 (m/s)
        self.max_angular_speed = 2.0  # 최대 각도 속도 (rad/s)
        
        self.target_relative_pose = Pose()
        self.target_relative_pose.position.x = 0.5  # 목표 상대 위치 x 좌표
        self.target_relative_pose.position.y = 0.0  # 목표 상대 위치 y 좌표
        
        print("bbbb")

    def odom_callback(self, data):
        # 현재 위치 업데이트 콜백 함수
        self.current_pose = data.pose.pose
        
    def pallet_callback(self, msg):
        self.pallet_axis = msg
        distance = self.calculate_distance()
        if distance > 1:
            print("cccccccccc")

            # 목표 각도와 현재 각도 간 오차 계산
            cmd_vel_msg = Twist()

            target_x = self.pallet_axis.x
            target_y = self.pallet_axis.y
            print("Target :",target_x, target_y)
            self.target_relative_pose.position.x = target_x
            self.target_relative_pose.position.y = target_y
            yaw_error = self.calculate_yaw_error()

            # PID 제어기를 사용한 속도 계산
            linear_speed = self.kp_linear * distance
            angular_speed = self.kp_angular * yaw_error

            # 속도 제한
            linear_speed = self.limit_speed(linear_speed, self.max_linear_speed)
            angular_speed = self.limit_speed(angular_speed, self.max_angular_speed)

            # Twist 메시지에 속도 값 설정
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = angular_speed
            
            print(linear_speed, angular_speed)

            # cmd_vel 토픽으로 속도 메시지 발행
            self.cmdvel_pub.publish(cmd_vel_msg)
            print(distance)

        elif distance <= 1 and distance > 0:
            print("STOP")
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmdvel_pub.publish(cmd_vel_msg)
        else:
            pass

    def respawn_callback(self, msg):
        self.respawn_data = msg.data

    def calculate_distance(self):
        # 현재 위치와 목표 위치 간 거리 계산
        dx = self.pallet_axis.x
        dy = self.pallet_axis.y
        return math.sqrt(dx**2 + dy**2)*0.001

    def calculate_yaw_error(self):
        # 현재 각도와 목표 각도 간 오차 계산
        _, _, current_yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ])
        desired_yaw = math.atan2(
            self.pallet_axis.x,
            self.pallet_axis.y
        )
        desired_yaw -= 1.6
        print("desire : ", desired_yaw, "current : ", current_yaw)
        return current_yaw - desired_yaw

    def limit_speed(self, speed, max_speed):
        # 속도 제한 함수
        return min(speed, max_speed)

    def move_to_relative_position(self):
        pass

if __name__ == '__main__':
    try:
        controller = RobotController()
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass

