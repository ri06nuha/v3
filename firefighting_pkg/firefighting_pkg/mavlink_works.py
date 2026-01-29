#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from pymavlink import mavutil
from firefighting_interfaces.srv import GoToWaypoint, StartMission, DeployExtinguisher, RTL
import math


class MavlinkControllerNode(Node):
    def __init__(self):
        super().__init__('mavlink_controller_node')
        
        # YOUR WORKING PARAMETERS
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
        
        # AUTONOMOUS MISSION STATE (NEW)
        self.mission_active = False
        self.phase = "IDLE"  # IDLE→ARMED→TAKEOFF→SEARCH→APPROACH→HOVER→DEPLOY
        self.armed_takeoff_done = False
        self.fire_detected = False
        self.fire_distance = -1.0
        self.current_rel_alt = 0.0
        self.yaw_dir = 1.0
        self.last_fire_time = 0
        
        # YOUR WORKING SUBSCRIBERS
        self.sub_detect = self.create_subscription(Bool, 'firedetected', self.fire_detect_cb, 10)
        self.sub_dist = self.create_subscription(Float32, 'firedistance', self.fire_distance_cb, 10)
        
        # YOUR SERVICES
        self.goto_waypoint_srv = self.create_service(GoToWaypoint, 'goto_waypoint', self.goto_waypoint_cb)
        self.start_mission_srv = self.create_service(StartMission, 'start_mission', self.start_mission_cb)
        self.deploy_srv = self.create_service(DeployExtinguisher, 'deploy_extinguisher', self.deploy_cb)
        self.rtl_srv = self.create_service(RTL, 'rtl', self.rtl_cb)
        
        # YOUR MAVLINK THREAD
        self.master = None
        self.mav_thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.mav_thread.start()
        
        self.get_logger().info('MavlinkControllerNode AUTONOMOUS ready')
    
    def fire_detect_cb(self, msg):
        """FIXED: Keep distance even if fire flickers"""
        self.fire_detected = msg.data
        if msg.data:
            self.last_fire_time = time.time()
        # REMOVED: self.fire_distance = -1.0  ← Keep distance!

    def fire_distance_cb(self, msg):
        """Distance always valid"""
        if msg.data > 0:  # Only positive distances
            self.fire_distance = msg.data
    
    def mavlink_loop(self):
        """YOUR WORKING STARTUP + FIXED PHASE MACHINE"""
        self.get_logger().info(f'Connecting to {self.connection_str}...')
        self.master = mavutil.mavlink_connection(self.connection_str)
        self.master.wait_heartbeat()
        self.get_logger().info(f'Connected sys={self.master.target_system} comp={self.master.target_component}')
        
        self.set_mode('GUIDED')
        time.sleep(1)
        
        # YOUR WORKING AUTO-ARM + TAKEOFF
        self.get_logger().info('AUTO ARM + TAKEOFF')
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.master.motors_armed_wait()
        
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, self.takeoff_alt)
        time.sleep(15)
        
        self.phase = "SEARCH"
        self.get_logger().info('🚁 TAKEOFF COMPLETE → SEARCH PHASE')
        
        try:
            while rclpy.ok():
                msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
                if msg:
                    self.current_rel_alt = -msg.z
                
                # ✅ FIXED PHASE MACHINE
                if self.phase == "SEARCH":
                    if self.fire_detected and self.fire_distance > 0:  # ← TRIGGER!
                        self.phase = "APPROACH"
                        self.get_logger().info(f'🔥 FIRE DETECTED dist={self.fire_distance:.1f}m → APPROACH')
                    else:
                        self.search_yaw()
                
                elif self.phase == "APPROACH":
                    if self.fire_distance <= self.approach_dist:
                        self.phase = "HOVER"
                        self.get_logger().info(f'✅ APPROACH COMPLETE dist={self.fire_distance:.1f}m → HOVER')
                    else:
                        self.approach_fire()
                
                elif self.phase == "HOVER":
                    if self.fire_distance > 0 and self.fire_distance < self.approach_dist * 1.2:
                        self.phase = "DEPLOY"
                        self.get_logger().info('🎯 PERFECT POSITION → DEPLOY!')
                        self.deploy_cb(None, None)
                    else:
                        self.hover_fire()
                
                elif self.phase == "DEPLOY":
                    self.phase = "RTL"
                    self.set_mode('RTL')
                    self.get_logger().info('💥 MISSION COMPLETE → RTL')
                
                time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f'Loop error: {e}')



    def search_yaw(self):
        """YOUR WORKING YAW SEARCH"""
        alt_error = self.takeoff_alt - self.current_rel_alt
        vz = max(min(0.3 * alt_error, 1.0), -1.0)
        self.send_body_velocity(0, 0, vz)
        self.condition_yaw_relative(0, self.search_yaw_rate * self.yaw_dir)
        self.yaw_dir *= -1
        self.get_logger().info(f'SEARCH yaw={self.search_yaw_rate}deg/s alt={self.current_rel_alt:.1f}')


    def approach_fire(self):
        """APPROACH PHASE"""
        target_alt = self.fire_distance + self.hover_alt_offset
        alt_error = target_alt - self.current_rel_alt
        vz = max(min(0.4 * alt_error, 0.8), -0.8)
        self.send_body_velocity(self.forward_speed, 0, vz)
        self.condition_yaw_relative(0, 0)
        self.get_logger().info(f'APPROACH dist={self.fire_distance:.1f}m target_alt={target_alt:.1f}')
        
        if self.fire_distance <= self.approach_dist:
            self.phase = "HOVER"
            self.get_logger().info('🔥 FIRE APPROACHED - HOVERING')


    def hover_fire(self):
        """HOVER PHASE - READY TO DEPLOY"""
        target_alt = self.fire_distance + self.hover_alt_offset
        alt_error = target_alt - self.current_rel_alt
        vz = max(min(0.3 * alt_error, 0.5), -0.5)
        self.send_body_velocity(0, 0, vz)
        self.condition_yaw_relative(0, 0)
        self.get_logger().info(f'HOVER dist={self.fire_distance:.1f} alt={self.current_rel_alt:.1f} READY→DEPLOY')


    # YOUR WORKING MAVLINK HELPERS (UNCHANGED)
    def set_mode(self, mode_name):
        mode_map = self.master.mode_mapping()
        if mode_name in mode_map:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_map[mode_name])
    
    def send_body_velocity(self, vx, vy, vz):
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
    
    def condition_yaw_relative(self, yaw_angle_deg, yaw_rate_dps):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, yaw_angle_deg, yaw_rate_dps, 1, 0, 0, 0, 0)


    # YOUR SERVICES (TRIGGER MISSION)
    def start_mission_cb(self, req, res):
        """TRIGGER AUTONOMOUS MISSION"""
        self.mission_active = True
        self.phase = "SEARCH"  # Start searching immediately
        self.get_logger().info(f'🚀 MISSION STARTED at {req.latitude}, {req.longitude}')
        res.success = True
        res.status = 'SEARCHING_FOR_FIRE'
        return res


    def deploy_cb(self, req, res):
        self.get_logger().info('💥 EXTINGUISHER DEPLOYED!')
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
        res.success = True
        return res
    
    def rtl_cb(self, req, res):
        self.phase = "RTL"
        self.set_mode('RTL')
        res.success = True
        return res


    def goto_waypoint_cb(self, req, res):
        """Navigate to GPS waypoint - FIXED for SITL/HITL"""
        self.get_logger().info(f'GOTO lat={req.latitude:.6f}, lon={req.longitude:.6f}, alt={req.altitude_msl:.1f}')


        # ARM + TAKEOFF if needed
        if not self.armed_takeoff_done:
            self.arm_takeoff()
            time.sleep(2)  # Stabilize


        # Use LOCAL_NED for SITL (treat lat/lon as x,y meters)
        x_target = req.latitude  # SITL: use as local X
        y_target = req.longitude  # SITL: use as local Y  
        z_target = req.altitude_msl


        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # Position mask
            x_target, y_target, -z_target, 0, 0, 0, 0, 0, 0, 0, 0)


        # Simple timeout (no GPS blocking for SITL)
        timeout = time.time() + 30
        while time.time() < timeout:
            pos = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
            if pos:
                dist = math.sqrt((pos.x - x_target)**2 + (pos.y - y_target)**2)
                res.distance_remaining = dist
                if dist < 2.0:
                    res.success = True
                    res.status = 'REACHED'
                    self.mission_active = True
                    self.searching = True
                    return res
            time.sleep(0.2)


        res.success = False
        res.status = 'TIMEOUT'
        return res


    def start_mission_cb(self, req, res):
        """Start mission - FIXED no nested service call"""
        self.get_logger().info(f'START MISSION treating lat={req.latitude}, lon={req.longitude} as LOCAL x,y')


        # Direct mission start (no goto_waypoint_cb nesting)
        self.mission_active = True


        if not self.armed_takeoff_done:
            self.arm_takeoff()


        # Set mission waypoint (fire patrol area)
        self.target_x, self.target_y = req.latitude, req.longitude
        self.searching = True


        res.success = True
        res.status = 'MISSION_ACTIVE_SEARCHING'
        return res



    def gps_distance(self, lat1, lon1, lat2, lon2):
        """distance between GPS coordinates (meters)"""
        R = 6371000  # Earth radius
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi, dlambda = math.radians(lat2-lat1), math.radians(lon2-lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.asin(math.sqrt(a))



    def deploy_cb(self, req, res):
        self.get_logger().info('DEPLOY EXTINGUISHER')
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
        res.success = True
        return res
    
    def rtl_cb(self, req, res):
        self.get_logger().info('RTL')
        self.mission_active = False
        self.set_mode('RTL')
        res.success = True
        return res



def main(args=None):
    rclpy.init(args=args)
    node = MavlinkControllerNode()
    rclpy.spin(node)



if __name__ == '__main__':
    main()
