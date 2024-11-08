import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import argparse
import random
from ultralytics import YOLO
from geometry_msgs.msg import Quaternion
import pyrealsense2 as rs2

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
        self._class_to_color = {}

        #
        self.intrinsics = None
        self.target_pixel = None
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.position_pub = self.create_publisher(Quaternion, '/pallet_axis', 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.imageDepthInfoCallback, 1)
        self.intrinsics = None
        #
    def image_listener_callback(self, msg):
        print("GOGO")
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
                #
                self.target_pixel = (int(bbox[0]), int(bbox[1]))
                self.get_logger().info(f"Person detected at pixel: {self.target_pixel}")
                #
                center_x = float(bbox[0])
                center_y = float(bbox[1])
                size_x = float(bbox[2])
                size_y = float(bbox[3])


                if class_name not in self._class_to_color :
                    r = random.randint(0, 255)
                    g = random.randint(0, 255)
                    b = random.randint(0, 255)
                    self._class_to_color[class_name] = (r, g, b)

                color = self._class_to_color[class_name]

                pt1 = (round(center_x - size_x / 2.0),
                        round(center_y - size_y / 2.0))
                pt2 = (round(center_x + size_x / 2.0),
                        round(center_y + size_y / 2.0))
                cv2.rectangle(cv_image, pt1, pt2, color, 2)
                cv2.putText(cv_image, str(class_name), ((pt1[0]+pt2[0])//2-5, pt1[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness=2)


        self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))
    def depth_callback(self, msg):
        """ 깊이 이미지를 통해 대상 픽셀의 깊이 정보 획득 및 3D 상대 좌표 계산 """
        print("***********************")
        if self.target_pixel is None or self.intrinsics is None:
            return

        cv_depth_image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        depth = cv_depth_image[self.target_pixel[1], self.target_pixel[0]]

        if depth == 0:
            self.target_depth = None
            return

        self.target_depth = depth
        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.target_pixel[0], self.target_pixel[1]], depth)

        # 좌표를 ROS Quaternion 메시지로 발행
        person_position = Quaternion()
        #person_position.x = result[0]
        #person_position.y = result[1]
        #person_position.z = result[2]

        person_position.x = result[2]
        person_position.y = -result[0]
        person_position.z = -result[1]
        person_position.w = 1.0  # 회전 정보는 필요 없으므로 기본값으로 설정
        print("***************",person_position)
        self.position_pub.publish(person_position)
        self.get_logger().info(f"Person position published: x={result[0]:.2f}, y={result[1]:.2f}, z={result[2]:.2f}")

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d[:5]]  # 5개의 요소로 제한
        except CvBridgeError as e:
            print(e)
            return
        
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
