import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from firefighting_interfaces.msg import FireDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class FireVisionNode(Node):
    def __init__(self):
        super().__init__('fire_vision_node')
        
        pkg_share = get_package_share_directory('firefighting_pkg')
        model_relative_path = 'models/yolov8n.pt'
        self.yolo_model_path = os.path.join(pkg_share, model_relative_path)
        self.bridge = CvBridge()
        
        # Subscriber for Gazebo RGB Camera
        self.image_sub = self.create_subscription(Image, '/camera/image', self.image_cb, 10)
        
        # Publisher for detection results
        self.fire_pub = self.create_publisher(FireDetection, '/fire_detection', 10)

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame, conf=0.5, verbose=False)
        
        detection_msg = FireDetection()
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.fire_detected = False

        for r in results:
            if len(r.boxes) > 0:
                # Find the largest detection (likely the primary fire)
                detection_msg.fire_detected = True
                box = r.boxes[0].xywh[0] # [x_center, y_center, width, height]
                detection_msg.fire_pixel_position.x = float(box[0])
                detection_msg.fire_pixel_position.y = float(box[1])
                detection_msg.confidence = float(r.boxes[0].conf)
                
        self.fire_pub.publish(detection_msg)

def main():
    rclpy.init()
    node = FireVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()