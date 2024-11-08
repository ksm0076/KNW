import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import argparse
import random

from ultralytics import YOLO



class PersonSegmentationNode(Node):
    def __init__(self, image_topic):
        super().__init__('person_segmentation_node')
        self.img_pub = self.create_publisher(Image, '/yolov8_result', 10)
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

    def image_listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(source=cv_image, device="cuda:0", classes=[0], verbose=False)

        results = results[0].cpu()
        
        if not results.boxes:
            self.get_logger().info('No objects detected')
            return

        for i in range(len(results)):

            if results.boxes :
                bbox_info = results.boxes[i]
                class_name = self.model.names[int(bbox_info.cls)]

                bbox = bbox_info.xywh[0]
                center_x = float(bbox[0])
                center_y = float(bbox[1])
                size_x = float(bbox[2])
                size_y = float(bbox[3])



                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)

                
                color = (r, g, b)

                pt1 = (round(center_x - size_x / 2.0),
                        round(center_y - size_y / 2.0))
                pt2 = (round(center_x + size_x / 2.0),
                        round(center_y + size_y / 2.0))
                cv2.rectangle(cv_image, pt1, pt2, color, 2)
                cv2.putText(cv_image, str(class_name), ((pt1[0]+pt2[0])//2-5, pt1[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness=2)

        
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))


def main(args=None):
    parser = argparse.ArgumentParser(description='Person Segmentation Node')
    parser.add_argument('--image_topic', type=str, default='/camera/color/image_raw',
                        help='The image topic to subscribe to')
    args = parser.parse_args()

    rclpy.init()
    person_segmentation_node = PersonSegmentationNode(args.image_topic)
    rclpy.spin(person_segmentation_node)
    person_segmentation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()