#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, Point
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Bool, Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from firefighting_interfaces.srv import StartMission

import math

class FireMissionNode(Node):
    def __init__(self):
        super().__init__('mavros_mission_node')

        # --- TUNING & CONFIG ---
        self.SEARCH_ALT = 5.0           # Altitude to fly to target
        self.HOVER_ALT = 3.0            # Final drop altitude
        self.HOVER_TOLERANCE = 0.2      # Meters (xy error allowable for drop)
        self.SEARCH_YAW_RATE = 0.4      # Rad/s
        self.FRAME_CENTER_X = 320.0     # 640 / 2
        self.FRAME_CENTER_Y = 240.0     # 480 / 2
        
        # --- STATE MACHINE ---
        # IDLE -> TAKEOFF -> TRANSIT -> SEARCH -> ALIGN -> DROP -> RTL
        self.state = "IDLE" 
        self.mavros_state = State()
        self.current_alt = 0.0
        self.current_yaw = 0.0
        
        # --- TARGETS ---
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.fire_detected = False
        self.fire_dist = -1.0
        self.fire_pos_px = Point() # x,y pixels

        # --- COMMUNICATIONS ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        # MAVROS
        self.sub_state = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)
        self.pub_vel = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.pub_global = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        
        # PERCEPTION (From your sim_depth_fire_node)
        self.create_subscription(Bool, '/firedetected', self.det_cb, 10)
        self.create_subscription(Float32, '/firedistance', self.dist_cb, 10)
        self.create_subscription(Point, '/fireposition', self.pos_cb, 10)

        # SERVICES
        self.cli_arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_cmd = self.create_client(CommandLong, '/mavros/cmd/command')
        
        # MISSION SERVER (Simulates Firebase Trigger)
        self.srv_start = self.create_service(StartMission, 'start_mission', self.start_mission_cb)

        # LOOP
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        self.get_logger().info("🔥 READY: Waiting for coordinates (ros2 service call /start_mission...)")

    # --- CALLBACKS ---
    def state_cb(self, msg): self.mavros_state = msg
    def det_cb(self, msg): self.fire_detected = msg.data
    def dist_cb(self, msg): self.fire_dist = msg.data
    def pos_cb(self, msg): self.fire_pos_px = msg
    
    def pose_cb(self, msg):
        self.current_alt = msg.pose.position.z
        # Extract Yaw
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def start_mission_cb(self, req, res):
        self.target_lat = req.latitude
        self.target_lon = req.longitude
        self.get_logger().info(f"📍 COORDINATES RECEIVED: {self.target_lat}, {self.target_lon}")
        
        if self.state == "IDLE":
            self.state = "TAKEOFF"
            res.success = True
            res.status = "Taking Off"
        else:
            res.success = False
            res.status = f"Busy in state: {self.state}"
        return res

    # --- MAIN LOGIC ---
    def control_loop(self):
        cmd = Twist()

        # 1. ARMING & TAKEOFF
        if self.state == "TAKEOFF":
            if self.mavros_state.mode != "GUIDED":
                self.set_mode("GUIDED")
            elif not self.mavros_state.armed:
                self.arm_drone(True)
            else:
                # Climb straight up
                if self.current_alt < self.SEARCH_ALT - 0.5:
                    cmd.linear.z = 1.0
                    self.pub_vel.publish(cmd)
                else:
                    self.state = "TRANSIT"
                    self.get_logger().info("✈️ ALTITUDE REACHED -> TRANSIT TO GPS")

        # 2. TRANSIT (Go To Coordinates)
        elif self.state == "TRANSIT":
            # For Simulation simplicity, if coordinates are 0,0, skip to search (assume we are there)
            if self.target_lat == 0.0:
                self.state = "SEARCH"
                return

            # Send Global Setpoint
            goal = GeoPoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.latitude = self.target_lat
            goal.pose.position.longitude = self.target_lon
            goal.pose.position.altitude = self.current_alt # Maintain Alt
            self.pub_global.publish(goal)
            
            # Simple check: In real mission, check GPS distance. 
            # For now, we assume arrival if fire is seen OR user manually switches logic.
            # Let's verify arrival by distance logic if needed, but for visual servo demo:
            if self.fire_detected:
                self.state = "ALIGN"
                self.get_logger().info("🔥 FIRE SPOTTED DURING TRANSIT -> ALIGNING")

        # 3. SEARCH (Yaw Spin)
        elif self.state == "SEARCH":
            if self.fire_detected and self.fire_dist > 0:
                self.state = "ALIGN"
                self.get_logger().info(f"🔥 CONTACT: {self.fire_dist:.1f}m -> VISUAL SERVO START")
            else:
                cmd.angular.z = self.SEARCH_YAW_RATE
                self.pub_vel.publish(cmd)

        # 4. ALIGN (Visual Servoing)
        elif self.state == "ALIGN":
            if not self.fire_detected:
                self.state = "SEARCH" # Lost it, spin again
                return

            # --- CALCULATE VELOCITY ---
            # Image Frame: Top-Left (0,0). Center (320, 240)
            # Fire X < 320 (Left) -> Drone needs to move LEFT (Body Y positive?) 
            # NOTE: Body Frame: X=Forward, Y=Left, Z=Up
            
            err_x = (self.FRAME_CENTER_X - self.fire_pos_px.x) # Pos = Fire is Left
            err_y = (self.FRAME_CENTER_Y - self.fire_pos_px.y) # Pos = Fire is Top (Forward)
            
            # Gains
            Kp_xy = 0.002
            Kp_z = 0.5
            
            # Velocity in Body Frame
            vel_fwd = Kp_xy * err_y   # Move Forward if fire is high in image
            vel_left = Kp_xy * err_x  # Move Left if fire is left in image
            
            # Altitude Control (Target 3.0m)
            err_z = self.HOVER_ALT - self.fire_dist
            vel_z = Kp_z * err_z # If err_z is neg (too high), vel_z is neg (descend) -> Wait, Dist is measured by cam.
            # If fire_dist is 5m, we want 3m. Error = 3 - 5 = -2. Descend. Correct.
            
            # Transform to ENU for MAVROS (Rotate by current Yaw)
            # cmd_vel is usually in ENU frame for MAVROS "setpoint_velocity"
            
            cmd.linear.x = vel_fwd * math.cos(self.current_yaw) - vel_left * math.sin(self.current_yaw)
            cmd.linear.y = vel_fwd * math.sin(self.current_yaw) + vel_left * math.cos(self.current_yaw)
            
            # Simple Descent logic: Stop xy movement if we are descending fast
            # Refined: Move XY and Z together
            cmd.linear.z = 0.5 * (self.HOVER_ALT - self.current_alt) # Use Baro alt for stability or Sonar if avail
            
            # CHECK DROP CONDITION
            # If we are close to 3m alt AND centered
            if abs(err_x) < 20 and abs(err_y) < 20 and abs(self.current_alt - self.HOVER_ALT) < 0.5:
                self.state = "DROP"
            
            self.pub_vel.publish(cmd)

        # 5. DROP
        elif self.state == "DROP":
            self.get_logger().info("💣 DROPPING PAYLOAD!")
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            self.pub_vel.publish(cmd)
            
            self.trigger_payload_service()
            
            # Wait loop or simple counter could be added here
            self.state = "RTL"

        # 6. RTL
        elif self.state == "RTL":
            if self.mavros_state.mode != "RTL":
                self.set_mode("RTL")
                self.get_logger().info("🏠 RETURNING HOME")

    # --- HELPERS ---
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.cli_mode.call_async(req)

    def arm_drone(self, arm):
        req = CommandBool.Request()
        req.value = arm
        self.cli_arming.call_async(req)

    def trigger_payload_service(self):
        req = CommandLong.Request()
        req.command = 183   # MAV_CMD_DO_SET_SERVO
        req.param1 = 9.0    # Servo Instance
        req.param2 = 1900.0 # PWM Open
        self.cli_cmd.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = FireMissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()