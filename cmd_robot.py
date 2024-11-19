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

        self.kp_linear = 0.4
        self.kp_angular = 2

        self.max_linear_speed = 2
        self.max_angular_speed = 2.0

        self.last_detection_time = time.time()
        self.searching = False
        self.rotation_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
        self.last_status = None
        self.stop_status = None
       
        
    def pallet_callback(self, msg):
        self.pallet_axis = msg
        distance = self.calculate_distance()
        self.last_detection_time = time.time()  # Update last detection time when detected
        self.searching = False  # Stop searching if person is detected

        if distance > 1.5:
            print("Following person")
            self.stop_status = False
            
            cmd_vel_msg = Twist()
            yaw_error = self.calculate_yaw_error()
            linear_speed = self.kp_linear * distance
            angular_speed = self.kp_angular * yaw_error

            linear_speed = self.limit_speed(linear_speed, self.max_linear_speed)
            angular_speed = self.limit_speed(angular_speed, self.max_angular_speed)

            print(f"Distance: {distance}, speed: {linear_speed}")
            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = angular_speed

            if cmd_vel_msg.angular.z > 0:
                self.rotation_direction = 1
            elif cmd_vel_msg.angular.z < 0:
                self.rotation_direction = -1

            self.cmdvel_pub.publish(cmd_vel_msg)
        elif distance <= 1.5 and distance > 0:
            if self.stop_status == False:
                print("STOP")
            self.stop_robot()

    def status_callback(self, msg):
        # 상태가 변경되었을 때만 출력
        if msg.data != self.last_status:
            print("status_callback :", msg)
            self.last_status = msg.data  # 현재 상태를 업데이트
            
        if msg.data == "lost":
            current_time = time.time()
            if current_time - self.last_detection_time > 1:
                print("Human lost. Starting to rotate.")
                self.searching = True
                self.rotate_to_find_person()
        elif msg.data == "found":
            self.searching = False

    def rotate_to_find_person(self):
        print("Rotating to find person...")
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.0 * self.rotation_direction  # Adjust rotation speed as needed

        self.cmdvel_pub.publish(cmd_vel_msg)
        time.sleep(0.3) 
        if self.searching:
            print("Human not found. Stopping rotation and alerting user.")
            self.stop_robot()
            self.node.get_logger().info("Human not found after rotation. Please check.")

    def stop_robot(self):
        self.stop_status = True
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