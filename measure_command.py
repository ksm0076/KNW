#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DistanceCommandPublisher(Node):
    def __init__(self):
        super().__init__('distance_command_publisher')
        self.publisher = self.create_publisher(
            String,
            '/distance_command',
            10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

def main(args=None):
    rclpy.init(args=args)
    command_publisher = DistanceCommandPublisher()

    while rclpy.ok():
        command = input("Enter command (start/stop): ")
        if command in ["start", "stop"]:
            command_publisher.publish_command(command)
        else:
            print("Invalid command. Please enter 'start' or 'stop'.")

    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
