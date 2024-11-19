#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Quaternion
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
        self.odom_sub = self.node.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.node.create_timer(1, self.time_callback)

        self.current_pose = Pose()
        self.pallet_axis = Quaternion()
        
        self.kp_linear = 0.2
        self.kp_angular = 1

        self.max_linear_speed = 0.75
        self.max_angular_speed = 2.0

        self.last_detection_time = time.time()
        self.current_time = time.time()
        self.rotation_direction = 1  # 1 for clockwise, -1 for counter clockwise
    
    def time_callback(self):
        self.current_time = time.time()
        if self.current_time - self.last_detection_time > 2:
            self.rotate_to_find_person()
    
    def pallet_callback(self, msg):
        self.pallet_axis = msg
        self.last_detection_time = time.time()  # Update last detection time when detected
        distance = self.calculate_distance()        
        
        if distance > 1:
            print("Following person")
            cmd_vel_msg = Twist()
            yaw_error = self.calculate_yaw_error()
            linear_speed = self.kp_linear * distance
            angular_speed = self.kp_angular * yaw_error

            linear_speed = self.limit_speed(linear_speed, self.max_linear_speed)
            angular_speed = self.limit_speed(angular_speed, self.max_angular_speed)

            cmd_vel_msg.linear.x = linear_speed
            cmd_vel_msg.angular.z = angular_speed
            
            if cmd_vel_msg.angular.z > 0:
                self.rotation_direction = 1
            elif cmd_vel_msg.angular.z < 0:
                self.rotation_direction = -1
            
            self.cmdvel_pub.publish(cmd_vel_msg)
        elif distance <= 1 and distance > 0:
            print("STOP")
            self.stop_robot()

    def rotate_to_find_person(self):
        print("Rotating to find person...")
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.0 * self.rotation_direction  # rotation speed
                
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
        return current_yaw - desired_yaw

    def limit_speed(self, speed, max_speed):
        return min(speed, max_speed)
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

if __name__ == '__main__':
    try:
        controller = RobotController()
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass
