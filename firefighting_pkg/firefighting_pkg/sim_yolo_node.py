import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class SimYoloNode(Node):
    def __init__(self):
        super().__init__('sim_yolo_node')

        # Use your current package and model layout
        pkg_share = get_package_share_directory('firefighting_pkg')
        model_relative_path = 'models/yolov8n.pt'
        self.yolo_model_path = os.path.join(pkg_share, model_relative_path)

        self.conf_threshold = 0.4

        self.model = YOLO(self.yolo_model_path)
        self.model.conf = self.conf_threshold

        self.bridge = CvBridge()

        # Publishers
        self.pub_fire_detected = self.create_publisher(Bool, 'firedetected', 10)
        self.pub_fire_distance = self.create_publisher(Float32, 'firedistance', 10)
        self.pub_fire_position = self.create_publisher(Point, 'fireposition', 10)

        # Subscribe to bridged RGB camera from Gazebo
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image',      # from depth_bridge.yaml
            self.image_callback,
            10
        )

        self.get_logger().info('Sim YOLO fire node started.')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(frame, imgsz=640, verbose=False)

        nearest_distance = None
        fire_center = None
        fire_detected = False

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy() if len(result.boxes) > 0 else []
            for box in boxes:
                x1, y1, x2, y2 = map(int, box[:4])
                conf = float(box[4]) if len(box) > 4 else 0.0
                if conf < self.conf_threshold:
                    continue

                fire_detected = True
                fire_center = ((x1 + x2) // 2, (y1 + y2) // 2)

                # For the demo: fake distance based on box height
                h = max(1, y2 - y1)
                distance_m = max(1.0, 60.0 / h)  # tweak constant 60.0 to look nice
                if nearest_distance is None or distance_m < nearest_distance:
                    nearest_distance = distance_m

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish to ROS
        msg_detect = Bool()
        msg_detect.data = fire_detected
        self.pub_fire_detected.publish(msg_detect)

        msg_dist = Float32()
        msg_dist.data = nearest_distance if nearest_distance is not None else -1.0
        self.pub_fire_distance.publish(msg_dist)

        msg_pos = Point()
        if fire_center is not None:
            msg_pos.x = float(fire_center[0])
            msg_pos.y = float(fire_center[1])
            msg_pos.z = 0.0
        else:
            msg_pos.x = msg_pos.y = msg_pos.z = 0.0
        self.pub_fire_position.publish(msg_pos)

        if fire_detected:
            self.get_logger().info(
                f"Fire detected at ~{msg_dist.data:.2f} m, center=({msg_pos.x:.1f},{msg_pos.y:.1f})"
            )

        # Optional debug window
        cv2.imshow("Sim Fire YOLO", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SimYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
