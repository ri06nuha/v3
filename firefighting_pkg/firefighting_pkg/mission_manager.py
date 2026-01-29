import rclpy
from rclpy.node import Node
from firefighting_interfaces.msg import MissionState, FireDetection
from firefighting_interfaces.srv import StartMission

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        self.state = MissionState.IDLE
        self.state_pub = self.create_publisher(MissionState, '/mission_state', 10)
        
        # Service from WebApp
        self.start_srv = self.create_service(StartMission, '/start_mission', self.handle_start)
        self.fire_sub = self.create_subscription(FireDetection, '/fire_detection', self.fire_cb, 10)
        
        self.timer = self.create_timer(1.0, self.publish_state)

    def handle_start(self, request, response):
        self.get_logger().info(f"Initiating Mission to {request.latitude}, {request.longitude}")
        self.state = MissionState.TAKEOFF
        response.success = True
        response.status = "Mission Started"
        return response

    def fire_cb(self, msg):
        # Logic to transition from Scanning to Approach 
        if msg.fire_detected and self.state == MissionState.SCANNING:
            self.state = MissionState.APPROACHING_FIRE
            self.get_logger().info("Target Acquired: Transitioning to Approach.")

    def publish_state(self):
        msg = MissionState()
        msg.state = self.state
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()