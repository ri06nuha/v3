#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from firefighting_interfaces.msg import MissionState
from firefighting_interfaces.srv import GoToWaypoint, StartMission, DeployExtinguisher, RTL
from pymavlink import mavutil
import math

class MavlinkControllerNode(Node):
    def __init__(self):
        super().__init__('mavlink_controller_node')
        
        # Parameters
        self.declare_parameter('connection', 'udp:127.0.0.1:14550')
        self.declare_parameter('takeoff_alt', 5.0)
        self.declare_parameter('approach_dist', 3.0)
        self.declare_parameter('hover_alt_offset', 3.0)
        self.declare_parameter('forward_speed', 1.0)
        self.declare_parameter('search_yaw_rate', 30.0)
        
        self.connection_str = self.get_parameter('connection').get_parameter_value().string_value
        self.takeoff_alt = self.get_parameter('takeoff_alt').get_parameter_value().double_value
        self.approach_dist = self.get_parameter('approach_dist').get_parameter_value().double_value
        self.hover_alt_offset = self.get_parameter('hover_alt_offset').get_parameter_value().double_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').get_parameter_value().double_value
        
        # 🔥 MISSION TARGET STORAGE (NEW!)
        self.mission_target_x = 0.0
        self.mission_target_y = 0.0
        self.mission_target_z = 0.0
        
        # Mission state
        self.mission_active = False
        self.phase = "IDLE"
        self.armed_takeoff_done = False
        self.fire_detected = False
        self.fire_distance = -1.0
        self.current_rel_alt = 0.0
        self.yaw_dir = 1.0
        self.last_fire_time = 0
        
        # Publishers
        self.pub_mission_state = self.create_publisher(MissionState, '/mission_state', 10)
        self.publish_mission_state(0, "IDLE - waiting /start_mission", 0.0)
        
        # Subscribers
        self.sub_detect = self.create_subscription(Bool, 'firedetected', self.fire_detect_cb, 10)
        self.sub_dist = self.create_subscription(Float32, 'firedistance', self.fire_distance_cb, 10)
        
        # Services
        self.goto_waypoint_srv = self.create_service(GoToWaypoint, 'goto_waypoint', self.goto_waypoint_cb)
        self.start_mission_srv = self.create_service(StartMission, 'start_mission', self.start_mission_cb)
        self.deploy_srv = self.create_service(DeployExtinguisher, 'deploy_extinguisher', self.deploy_cb)
        self.rtl_srv = self.create_service(RTL, 'rtl', self.rtl_cb)
        
        # MAVLink thread
        self.master = None
        self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.mav_thread.start()
        
        self.get_logger().info('🚀 READY - Use: ros2 service call /start_mission firefighting_interfaces/srv/StartMission "{latitude: 4.0, longitude: 3.0, altitude_msl: 8.0}"')
    
    def publish_mission_state(self, state_id, description, progress):
        """Publish mission state (NO header)"""
        msg = MissionState()
        msg.state = state_id
        msg.state_description = description
        msg.mission_progress = progress
        self.pub_mission_state.publish(msg)
    
    def fire_detect_cb(self, msg):
        self.fire_detected = msg.data
        if msg.data:
            self.last_fire_time = time.time()
    
    def fire_distance_cb(self, msg):
        if msg.data > 0:
            self.fire_distance = msg.data
    
    def mavlink_loop(self):
        """🔥 MAIN MISSION EXECUTION"""
        self.get_logger().info(f'Connecting to {self.connection_str}...')
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.master.wait_heartbeat()
        self.get_logger().info(f'Connected sys={self.master.target_system} comp={self.master.target_component}')
        
        self.set_mode('GUIDED')
        self.publish_mission_state(1, "PREFLIGHT - waiting service", 5.0)
        
        # 🚫 WAIT FOR /start_mission SERVICE
        while rclpy.ok() and self.phase == "IDLE":
            self.publish_mission_state(0, "IDLE - waiting /start_mission", 0.0)
            time.sleep(0.5)
        
        # 🎯 SERVICE TRIGGERED! START MISSION SEQUENCE
        self.phase = "ARMING"
        self.publish_mission_state(1, "ARMING MOTORS", 10.0)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.master.motors_armed_wait()
        
        self.phase = "TAKEOFF"
        self.publish_mission_state(2, "TAKING OFF", 25.0)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.takeoff_alt)
        time.sleep(15)
        
        # 🔥 NEW: GO TO MISSION TARGET FIRST!
        self.phase = "NAVIGATE_TO_ZONE"
        self.publish_mission_state(3, f"GOING TO TARGET ({self.mission_target_x},{self.mission_target_y})", 35.0)
        success = self.goto_mission_waypoint(self.mission_target_x, self.mission_target_y, self.mission_target_z)
        
        if success:
            self.phase = "SCANNING"
            self.publish_mission_state(4, "SCANNING at target zone", 40.0)
        else:
            self.phase = "RTB"
            self.set_mode('RTL')
            return
        
        # 🔥 MAIN MISSION LOOP - FIXED STATE MACHINE
        try:
            while rclpy.ok() and self.phase != "RTB":
                msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                if msg:
                    self.current_rel_alt = -msg.z
                
                if self.phase == "SCANNING":
                    if self.fire_detected and self.fire_distance > 0:
                        # 🔥 FIXED: Go directly to APPROACH (no intermediate state)
                        self.phase = "APPROACHING_FIRE"
                        self.publish_mission_state(6, f"FIRE DETECTED {self.fire_distance:.1f}m → APPROACHING", 60.0)
                        self.get_logger().info(f'🔥 FIRE FOUND! dist={self.fire_distance:.1f}m → APPROACH')
                    else:
                        self.search_yaw()
                        self.publish_mission_state(4, "SCANNING - yaw search", 45.0)
                
                elif self.phase == "APPROACHING_FIRE":
                    if self.fire_distance <= self.approach_dist:
                        self.phase = "POSITIONING"
                        self.publish_mission_state(7, f"POSITIONING 3m above fire ({self.fire_distance:.1f}m)", 80.0)
                    else:
                        self.approach_fire()
                
                elif self.phase == "POSITIONING":
                    if self.fire_distance > 0 and self.fire_distance < self.approach_dist * 1.2:
                        self.phase = "DEPLOYING_PAYLOAD"
                        self.publish_mission_state(8, "DEPLOYING EXTINGUISHER", 90.0)
                        self.deploy_payload()
                    else:
                        self.hover_fire()
                
                elif self.phase == "DEPLOYING_PAYLOAD":
                    self.phase = "RTB"
                    self.publish_mission_state(10, "RTB - Mission Success", 95.0)
                    self.set_mode('RTL')
                
                time.sleep(0.2)
            
            self.publish_mission_state(11, "MISSION COMPLETE - LANDING", 100.0)
            
        except Exception as e:
            self.get_logger().error(f'Loop error: {e}')
            self.publish_mission_state(99, f"EMERGENCY: {str(e)}", 0.0)
    
    def goto_mission_waypoint(self, x_target, y_target, z_target):
        """🔥 NEW: Navigate to user coordinates (SITL local NED)"""
        self.get_logger().info(f'🚁 Navigating to X={x_target}, Y={y_target}, Z={z_target}')
        
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # Position control
            x_target, y_target, -z_target, 0, 0, 0, 0, 0, 0, 0, 0)
        
        timeout = time.time() + 60
        while time.time() < timeout:
            pos = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if pos:
                dist = math.sqrt((pos.x - x_target)**2 + (pos.y - y_target)**2)
                self.publish_mission_state(3, f'NAV: {dist:.1f}m to target', 30.0 + (70-dist)/2)
                if dist < 2.0:  # Arrived!
                    self.get_logger().info(f'✅ ARRIVED AT MISSION TARGET ({x_target:.1f},{y_target:.1f})')
                    return True
            time.sleep(0.2)
        
        self.get_logger().warn('⚠️ MISSION TARGET TIMEOUT - Starting scan anyway')
        return False
    
    # 🔥 MAVLINK MOVEMENT METHODS
    def search_yaw(self):
        """Yaw search pattern at constant altitude"""
        alt_error = self.takeoff_alt - self.current_rel_alt
        vz = max(min(0.3 * alt_error, 1.0), -1.0)
        self.send_body_velocity(0, 0, vz)
        self.condition_yaw_relative(0, self.search_yaw_rate * self.yaw_dir)
        self.yaw_dir *= -1
    
    def approach_fire(self):
        """Approach fire while maintaining safe altitude"""
        target_alt = self.fire_distance + self.hover_alt_offset
        alt_error = target_alt - self.current_rel_alt
        vz = max(min(0.4 * alt_error, 0.8), -0.8)
        self.send_body_velocity(self.forward_speed, 0, vz)
        self.condition_yaw_relative(0, 0)  # Face forward
        self.publish_mission_state(6, f'APPROACH: {self.fire_distance:.1f}m', 70.0)
    
    def hover_fire(self):
        """Fine position above fire"""
        target_alt = self.fire_distance + self.hover_alt_offset
        alt_error = target_alt - self.current_rel_alt
        vz = max(min(0.3 * alt_error, 0.5), -0.5)
        self.send_body_velocity(0, 0, vz)
        self.condition_yaw_relative(0, 0)
    
    def deploy_payload(self):
        """Drop extinguisher"""
        self.get_logger().info('💥 DEPLOYING EXTINGUISHER!')
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
        time.sleep(2)
    
    # MAVLINK HELPERS
    def set_mode(self, mode_name):
        if not self.master: return
        mode_map = self.master.mode_mapping()
        if mode_name in mode_map:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_map[mode_name])
    
    def send_body_velocity(self, vx, vy, vz):
        if not self.master: return
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
    
    def condition_yaw_relative(self, yaw_angle_deg, yaw_rate_dps):
        if not self.master: return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, yaw_angle_deg, yaw_rate_dps, 1, 0, 0, 0, 0)
    
    # 🔥 SERVICES - PROPERLY IMPLEMENT YOUR .SRV FILES
    def start_mission_cb(self, request, response):
        """🚀 STORES coordinates + triggers mission"""
        if self.phase != "IDLE":
            response.success = False
            response.status = 'ALREADY_RUNNING'
            return response
        
        # 🔥 FIX #1: ACTUALLY USE THE COORDINATES!
        self.mission_target_x = request.latitude    # SITL X meters
        self.mission_target_y = request.longitude   # SITL Y meters
        self.mission_target_z = request.altitude_msl
        
        self.get_logger().info(f'🚀 MISSION TARGET SET: X={self.mission_target_x}, Y={self.mission_target_y}, Z={self.mission_target_z}')
        self.phase = "ARMING"  # Unblocks mavlink_loop
        
        response.success = True
        response.status = 'MISSION_TARGET_SET'
        return response
    
    def rtl_cb(self, request, response):
        self.phase = "RTB"
        if self.master:
            self.set_mode('RTL')
        self.publish_mission_state(10, "RTL VIA SERVICE", 95.0)
        response.success = True
        return response
    
    def deploy_cb(self, request, response):
        self.phase = "DEPLOYING_PAYLOAD"
        self.deploy_payload()
        self.publish_mission_state(8, "MANUAL DEPLOY", 90.0)
        response.success = True
        return response
    
    def goto_waypoint_cb(self, request, response):
        if not self.master:
            response.success = False
            response.status = 'NO_MAVLINK'
            return response
        
        x_target, y_target, z_target = request.latitude, request.longitude, request.altitude_msl
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000, x_target, y_target, -z_target, 0,0,0,0,0,0,0,0)
        
        timeout = time.time() + 30
        while time.time() < timeout:
            pos = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if pos:
                dist = math.sqrt((pos.x - x_target)**2 + (pos.y - y_target)**2)
                response.distance_remaining = dist
                if dist < 2.0:
                    response.success = True
                    response.status = 'REACHED'
                    return response
            time.sleep(0.2)
        
        response.success = False
        response.status = 'TIMEOUT'
        response.distance_remaining = 999.0
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
