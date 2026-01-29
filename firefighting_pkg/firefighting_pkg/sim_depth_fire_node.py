#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from firefighting_interfaces.msg import FireDetection
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimDepthFireNode(Node):
    def __init__(self):
        super().__init__('sim_depth_fire_node')
        
        self.bridge = CvBridge()
        self.last_depth = None
        self.hsv_lower = np.array([20, 80, 80], dtype=np.uint8)
        self.hsv_upper = np.array([35, 255, 255], dtype=np.uint8)
        self.min_area = 50
        
        # Publishers
        qos = 10
        self.pub_fire_detected = self.create_publisher(Bool, 'firedetected', qos)
        self.pub_fire_distance = self.create_publisher(Float32, 'firedistance', qos)
        self.pub_fire_position = self.create_publisher(Point, 'fireposition', qos)
        
        # Subscribers - YOUR WORKING TOPICS!
        self.sub_rgb = self.create_subscription(Image, '/camera/image', self.rgb_cb, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth', self.depth_cb, 10)
        
        self.frame_count = 0
        self.get_logger().info('🔥 SimDepthFireNode READY - /camera/image + /camera/depth')

    def depth_cb(self, msg):
        try:
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except:
            pass

    def rgb_cb(self, msg):
        self.frame_count += 1
        display_frame = None
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            display_frame = frame.copy()  # Always have frame for imshow
            
            if self.last_depth is None or frame.shape[:2] != self.last_depth.shape[:2]:
                return
            
            # Fire detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            fire_detected = False
            nearest_distance = -1.0
            fire_center = None
            max_contour_area = 0
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                max_contour_area = area
                
                if area > self.min_area:
                    fire_detected = True
                    x, y, w, h = cv2.boundingRect(c)
                    cx, cy = x + w // 2, y + h // 2
                    fire_center = (cx, cy)
                    
                    # Depth
                    dy, dx = 2, 2
                    patch = self.last_depth[cy-dy:cy+dy+1, cx-dx:cx+dx+1]
                    valid_depth = patch[patch > 0.0]
                    if valid_depth.size > 0:
                        nearest_distance = float(valid_depth.mean())
                    
                    # Visualize
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                    cv2.putText(frame, f"{nearest_distance:.1f}m", (x, y - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # FIXED PUBLISHING
            self.pub_fire_detected.publish(Bool(data=fire_detected))
            self.pub_fire_distance.publish(Float32(data=nearest_distance))
            
            msg_pos = Point()
            if fire_center:
                msg_pos.x = float(fire_center[0])
                msg_pos.y = float(fire_center[1])
                if self.frame_count % 30 == 0:  # Log every second
                    self.get_logger().info(f"🔥 Fire: {nearest_distance:.1f}m ({cx},{cy})")
            self.pub_fire_position.publish(msg_pos)
            
        except Exception as e:
            self.get_logger().warn(f"Processing error: {e}")
        
        # ✅ ALWAYS SHOW WINDOW (even on error)
        if display_frame is not None:
            cv2.imshow("SimDepthFire", display_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SimDepthFireNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
