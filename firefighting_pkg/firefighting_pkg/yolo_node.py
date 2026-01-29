import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
import depthai as dai
import cv2
import numpy as np
import os
import threading
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory 


class DepthYoloNode(Node):
    def __init__(self):
        super().__init__('depth_yolo_node')
    
        # === User settings ===
        pkg_share = get_package_share_directory('firefighting_uav_nodes')
        model_relative_path = 'models/yolov8n.pt'
        self.YOLO_MODEL_PATH = os.path.join(pkg_share, model_relative_path)

        self.SAVE_FRAMES = True
        self.SAVE_DIR = "detections"
        self.CONF_THRESHOLD = 0.4
        self.YOLO_EVERY_N_FRAMES = 4

        if self.SAVE_FRAMES:
            os.makedirs(self.SAVE_DIR, exist_ok=True)

        # === Load YOLO model ===
        self.model = YOLO(self.YOLO_MODEL_PATH)
        self.model.conf = self.CONF_THRESHOLD
        
        # === Publishers (simple types instead of FireReport) ===
        self.pub_fire_detected = self.create_publisher(Bool, 'firedetected', 10)
        self.pub_fire_distance = self.create_publisher(Float32, 'firedistance', 10)
        self.pub_fire_position = self.create_publisher(Point, 'fireposition', 10)

        # === Initialize DepthAI ===
        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)
        self.qRgb = self.device.getOutputQueue("color", maxSize=4, blocking=False)
        self.qDepth = self.device.getOutputQueue("depth", maxSize=4, blocking=False)

        self.get_logger().info("Fire detection node started — Press ESC to exit.")
        self.frame_count = 0
        self.last_results = []    

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        cam = pipeline.createColorCamera()
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setInterleaved(False)
        cam.setFps(30)

        monoLeft = pipeline.createMonoCamera()
        monoRight = pipeline.createMonoCamera()
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo = pipeline.createStereoDepth()
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        xoutRgb = pipeline.createXLinkOut()
        xoutRgb.setStreamName("color")
        cam.video.link(xoutRgb.input)

        xoutDepth = pipeline.createXLinkOut()
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)

        return pipeline

    def process_frames(self):
        while True:
            inRgb = self.qRgb.tryGet()
            inDepth = self.qDepth.tryGet()
            if inRgb is None or inDepth is None:
                continue

            colorFrame = inRgb.getCvFrame()
            depthFrame = inDepth.getFrame()
            self.frame_count += 1

            # Run YOLO every Nth frame
            if self.frame_count % self.YOLO_EVERY_N_FRAMES == 0:
                results = self.model.predict(colorFrame, imgsz=640, verbose=False)
                self.last_results = results
            else:
                results = self.last_results

            nearest_distance = None
            fire_center = None
            fire_detected = False

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy() if len(result.boxes) > 0 else []
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box[:4])
                    conf = box[4] if len(box) > 4 else 0

                    cv2.rectangle(colorFrame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(colorFrame, f"{conf:.2f}", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # Depth
                    depthCrop = depthFrame[y1:y2, x1:x2]
                    valid = depthCrop[depthCrop > 0]
                    if valid.size > 0:
                        distance_m = np.median(valid) / 1000.0
                        fire_center = ((x1 + x2) // 2, (y1 + y2) // 2)
                        fire_detected = True

                        cv2.putText(colorFrame, f"{distance_m:.2f} m",
                                    (x1, y2 + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                        if nearest_distance is None or distance_m < nearest_distance:
                            nearest_distance = distance_m

            # ROS publish (simple messages)
            # 1) fire detected flag
            msg_detect = Bool()
            msg_detect.data = fire_detected
            self.pub_fire_detected.publish(msg_detect)

            # 2) nearest distance (or -1.0 if none)
            msg_dist = Float32()
            msg_dist.data = nearest_distance if nearest_distance is not None else -1.0
            self.pub_fire_distance.publish(msg_dist)

            # 3) fire position in image pixels (or zeros)
            msg_pos = Point()
            if fire_center is not None:
                msg_pos.x = float(fire_center[0])
                msg_pos.y = float(fire_center[1])
                msg_pos.z = 0.0
            else:
                msg_pos.x = 0.0
                msg_pos.y = 0.0
                msg_pos.z = 0.0
            self.pub_fire_position.publish(msg_pos)

            if nearest_distance is not None:
                text = f"Nearest Fire: {nearest_distance:.2f} m"
                cv2.putText(colorFrame, text, (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.get_logger().info(text)

            cv2.imshow("OAK-D Fire Detection + ROS2", colorFrame)
            if self.SAVE_FRAMES:
                cv2.imwrite(os.path.join(self.SAVE_DIR, f"frame_{self.frame_count:04d}.jpg"), colorFrame)

            if cv2.waitKey(1) == 27:  # ESC
                break


def main(args=None):
    rclpy.init(args=args)
    node = DepthYoloNode()
    
    # 🔥 THREAD so ROS2 works!
    thread = threading.Thread(target=node.process_frames, daemon=True)
    thread.start()
    
    try:
        rclpy.spin(node)  # Services work now!
    except KeyboardInterrupt:
        pass
    finally:
        node.device.close()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()