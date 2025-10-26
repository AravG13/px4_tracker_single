#!/usr/bin/env python3
"""
Fixed PX4 ROS2 Control - Based on proven Gazebo implementation
Critical fixes:
1. Proper axis control matching Gazebo (Y for left/right, X for forward/back)
2. Correct PID tuning from working system
3. Proper backing away state machine
4. Ground safety with altitude estimation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode, 
    TrajectorySetpoint,
    VehicleStatus,
    VehicleLocalPosition
)
from geometry_msgs.msg import PointStamped, Vector3
from std_srvs.srv import Empty

import time
import math
import numpy as np
from enum import Enum
from dataclasses import dataclass

class DroneState(Enum):
    DISARMED = 0
    ARMED = 1
    TAKING_OFF = 2
    HOVERING = 3
    TRACKING = 4
    LANDING = 5
    EMERGENCY = 6

@dataclass
class TrackingData:
    """Matches Gazebo implementation tracking data"""
    pixel_x: int = 0
    pixel_y: int = 0
    frame_width: int = 640
    frame_height: int = 480
    bbox_area: float = 0.0
    confidence: float = 0.0
    boundary_violation: bool = False
    timestamp: float = 0.0

class StablePIDController:
    """Exact PID implementation from working Gazebo code"""
    
    def __init__(self, kp, ki, kd, max_output=1.0, max_integral=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.max_integral = max_integral
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0
        self.prev_time = None
        
    def compute(self, error, dt):
        """Compute PID output with derivative filtering like Gazebo"""
        # Proportional
        proportional = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        integral_term = self.ki * self.integral
        
        # Derivative with heavy filtering (0.8 from Gazebo)
        derivative = (error - self.prev_error) / dt
        derivative = 0.8 * self.prev_derivative + 0.2 * derivative
        self.prev_derivative = derivative
        derivative_term = self.kd * derivative
        
        output = proportional + integral_term + derivative_term
        output = max(min(output, self.max_output), -self.max_output)
        
        self.prev_error = error
        return output
        
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_derivative = 0.0

class ExponentialSmoother:
    """Exact smoother from Gazebo implementation"""
    
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.value = 0.0
        self.initialized = False
        
    def filter(self, new_value):
        if not self.initialized:
            self.value = new_value
            self.initialized = True
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value
        return self.value
        
    def reset(self):
        self.value = 0.0
        self.initialized = False

class FixedPX4Controller(Node):
    def __init__(self):
        super().__init__('fixed_px4_controller')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', 
            self.status_callback, qos_profile)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_callback, qos_profile)
        self.target_sub = self.create_subscription(
            PointStamped, '/detected_target',
            self.target_callback, 10)
        self.bbox_sub = self.create_subscription(
           Vector3, '/target_bbox_info',
           self.bbox_callback, 10)
        
        # Services
        self.create_service(Empty, '/drone/arm', self.arm_callback)
        self.create_service(Empty, '/drone/disarm', self.disarm_callback)
        self.create_service(Empty, '/drone/takeoff', self.takeoff_callback)
        self.create_service(Empty, '/drone/land', self.land_callback)
        self.create_service(Empty, '/drone/emergency', self.emergency_callback)
        self.create_service(Empty, '/drone/start_tracking', self.start_tracking_callback)
        self.create_service(Empty, '/drone/stop_tracking', self.stop_tracking_callback)
        
        # State variables
        self.state = DroneState.DISARMED
        self.armed = False
        self.nav_state = 0
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.home_pos = None
        self.tracking_enabled = False
        
        # Flight parameters (from Gazebo)
        self.takeoff_altitude = -5.0
        self.TARGET_FOLLOW_DISTANCE = 3.5
        self.MIN_ALTITUDE = 3.0
        self.SAFETY_ALTITUDE = 4.0
        self.hover_altitude = -5.0
        
        # EXACT PID TUNING FROM WORKING GAZEBO CODE
        self.pid_x = StablePIDController(0.8, 0.01, 0.1, 1.5, 0.3)  # Forward/back
        self.pid_y = StablePIDController(1.2, 0.02, 0.15, 2.0, 0.4)  # Left/right
        self.pid_z = StablePIDController(0.3, 0.005, 0.02, 0.8, 0.1)  # Up/down
        self.pid_distance = StablePIDController(0.6, 0.015, 0.08, 1.5, 0.25)  # Distance
        
        # EXACT SMOOTHING FROM GAZEBO
        self.smoother_x = ExponentialSmoother(0.30)
        self.smoother_y = ExponentialSmoother(0.20)
        self.smoother_z = ExponentialSmoother(0.10)
        self.smoother_distance = ExponentialSmoother(0.15)
        
        # Tracking data
        self.tracking_data = TrackingData()
        self.last_target_time = None
        self.target_timeout = 2.0
        self.last_control_time = time.time()
        
        # Backing away state (from Gazebo)
        self.backing_away_mode = False
        self.backup_start_time = None
        self.backup_duration = 2.0  # 2 seconds like Gazebo
        
        # State machine
        self.command_start_time = None
        self.sequence_step = 0
        
        # Ground safety
        self.estimated_altitude = 5.0
        self.last_altitude_update = time.time()
        # Bbox size tracking for distance control
        self.prev_bbox_size = 0.0
        self.prev_bbox_time = time.time()
        self.size_rate = 0.0
        
        # Timers
        self.offboard_timer = self.create_timer(0.02, self.publish_offboard_mode)
        self.state_timer = self.create_timer(0.1, self.state_machine)
        self.control_timer = self.create_timer(0.05, self.tracking_control_loop)
        
        self.get_logger().info("Fixed PX4 Controller Started (Gazebo-matched)")
        self.get_logger().info("PID: X=0.8/0.01/0.1, Y=0.8/0.01/0.1, Z=0.3/0.005/0.02")
    
    def status_callback(self, msg):
        old_armed = self.armed
        old_nav_state = self.nav_state
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        
        if old_armed != self.armed:
            status = "ARMED" if self.armed else "DISARMED"
            self.get_logger().info(f"Drone {status}")
        if old_nav_state != self.nav_state:
            nav_states = {14: "OFFBOARD", 4: "AUTO_LOITER", 0: "MANUAL", 2: "POSCTL"}
            state_name = nav_states.get(self.nav_state, f"UNKNOWN({self.nav_state})")
            self.get_logger().warn(f"Nav state changed: {old_nav_state} -> {self.nav_state} ({state_name})")    
        if not self.armed and self.state != DroneState.DISARMED:
            self.state = DroneState.DISARMED
            self.tracking_enabled = False
            self.reset_controllers()
    
    def position_callback(self, msg):
        self.current_pos = np.array([msg.x, msg.y, msg.z])
        self.current_vel = np.array([msg.vx, msg.vy, msg.vz])
        self.estimated_altitude = -msg.z

        self.current_heading = msg.heading  # Yaw angle in radians
        if int(time.time() * 2) % 10 == 0:
         self.get_logger().info(
            f"DRONE: pos=[{msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}] "
            f"vel=[{msg.vx:.2f}, {msg.vy:.2f}, {msg.vz:.2f}] alt={self.estimated_altitude:.2f}m"
        )
        
        if self.home_pos is None and self.armed:
            self.home_pos = self.current_pos.copy()
            self.hover_altitude = msg.z
            self.get_logger().info(f"Home position set: {self.home_pos}")
    
    def target_callback(self, msg):
        """Receive normalized coordinates from tracker"""
        # Convert normalized coordinates back to pixel coordinates
        # Tracker sends: norm_x * 1.5 where norm_x = (pixel_x - center_x) / center_x
        
        cx = self.tracking_data.frame_width / 2  # 320
        cy = self.tracking_data.frame_height / 2  # 240
        
        # Reverse the normalization
        norm_x = msg.point.x / 1.5
        norm_y = msg.point.y / 1.5
        
        self.tracking_data.pixel_x = int(norm_x * cx + cx)
        self.tracking_data.pixel_y = int(norm_y * cy + cy)
        
        self.tracking_data.timestamp = time.time()
        self.last_target_time = self.get_clock().now()
        
        
        self.get_logger().info(
          f"TARGET: norm=[{msg.point.x:.3f}, {msg.point.y:.3f}] -> "
            f"pixels=[{self.tracking_data.pixel_x}, {self.tracking_data.pixel_y}]"
        )
    def bbox_callback(self, msg):
            self.tracking_data.bbox_area = msg.x
            self.tracking_data.confidence = msg.y
            self.tracking_data.boundary_violation = (msg.z > 0.5)
            
            if int(time.time() * 4) % 10 == 0:
             self.get_logger().info(
                f"BBOX: area={self.tracking_data.bbox_area:.1f}, boundary={self.tracking_data.boundary_violation}"
            )
    def tracking_control_loop(self):
            """Complete working control loop with velocity-based distance control"""
            if not self.tracking_enabled or self.state != DroneState.TRACKING:
                return
            
            # Safety check
            if self.tracking_data.bbox_area <= 0:
                setpoint = TrajectorySetpoint()
                setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                setpoint.position = [self.current_pos[0], self.current_pos[1], self.takeoff_altitude]
                setpoint.velocity = [0.0, 0.0, 0.0]
                setpoint.yaw = float('nan')
                self.setpoint_pub.publish(setpoint)
                self.get_logger().warn("No target - hovering", throttle_duration_sec=2.0)
                return
            
            # Timeout check
            if self.last_target_time:
                time_since_target = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
                if time_since_target > self.target_timeout:
                    self.state = DroneState.HOVERING
                    self.tracking_enabled = False
                    return
            
            current_time = time.time()
            dt = current_time - self.last_control_time
            dt = max(min(dt, 0.1), 0.01)
            self.last_control_time = current_time
            
            # Calculate errors
            cx = self.tracking_data.frame_width / 2
            cy = self.tracking_data.frame_height / 2
            
            err_x = (self.tracking_data.pixel_x - cx) / cx
            err_y = (self.tracking_data.pixel_y - cy) / cy
            
            # Current object size
            current_size = math.sqrt(self.tracking_data.bbox_area)
            boundary_violation = self.tracking_data.boundary_violation
            
            # Calculate boundary error (how close to edges)
            boundary_error = 0.0
            if self.tracking_data.bbox_area > 0:
                margin = 50
                frame_w = self.tracking_data.frame_width
                frame_h = self.tracking_data.frame_height
                
                # Estimate bbox edges from center and area
                bbox_half_size = math.sqrt(self.tracking_data.bbox_area) / 2
                bbox_x = self.tracking_data.pixel_x - bbox_half_size
                bbox_y = self.tracking_data.pixel_y - bbox_half_size
                bbox_x2 = self.tracking_data.pixel_x + bbox_half_size
                bbox_y2 = self.tracking_data.pixel_y + bbox_half_size
                
                # Calculate distance violations from each edge
                if bbox_x < margin: 
                    boundary_error += (margin - bbox_x) / frame_w
                if bbox_y < margin: 
                    boundary_error += (margin - bbox_y) / frame_h
                if bbox_x2 > frame_w - margin: 
                    boundary_error += (bbox_x2 - (frame_w - margin)) / frame_w
                if bbox_y2 > frame_h - margin: 
                    boundary_error += (bbox_y2 - (frame_h - margin)) / frame_h
            
            # Backing away state machine with gradual intensity
            if not self.backing_away_mode and (boundary_violation or boundary_error > 0.05):
                self.backing_away_mode = True
                self.backup_start_time = current_time
                self.get_logger().info("STARTING BACKUP MODE - boundary violation")
            elif self.backing_away_mode:
                backup_elapsed = current_time - self.backup_start_time
                if backup_elapsed >= self.backup_duration and not boundary_violation and boundary_error < 0.02:
                    self.backing_away_mode = False
                    self.get_logger().info("BACKUP COMPLETE")
            
            # Distance control with velocity tracking
            if self.backing_away_mode:
                backup_elapsed = current_time - self.backup_start_time
                
                # Gradual ramp-up over first 0.5 seconds, then decay
                if backup_elapsed < 0.5:
                    backup_intensity = 0.2 + (0.6 * (backup_elapsed / 0.5))
                else:
                    backup_intensity = 0.8 * math.exp(-(backup_elapsed - 0.5) / 2.0)
                
                distance_error = backup_intensity
                
                # Maintain minimum backup if still near boundary
                if boundary_violation or boundary_error > 0.03:
                    distance_error = max(distance_error, 0.3)
            else:
                TARGET_DISTANCE_PIXELS = 70.0
                
                # Initialize tracking variables
                if not hasattr(self, 'prev_bbox_size'):
                    self.prev_bbox_size = current_size
                    self.prev_bbox_time = current_time
                    self.size_rate = 0.0
                
                # Calculate bbox size change rate
                size_dt = current_time - self.prev_bbox_time
                if size_dt > 0.1:
                    self.size_rate = (current_size - self.prev_bbox_size) / size_dt
                    self.prev_bbox_size = current_size
                    self.prev_bbox_time = current_time
                
                # Primary: Static size error
                size_error = (current_size - TARGET_DISTANCE_PIXELS) / TARGET_DISTANCE_PIXELS
                
                # Secondary: Dynamic rate error
                rate_threshold = 3.0
                if abs(self.size_rate) > rate_threshold:
                    rate_error = self.size_rate / 30.0
                    distance_error = size_error + rate_error * 0.6
                else:
                    distance_error = size_error
                
                # Clamp
                distance_error = max(min(distance_error, 0.4), -0.4)
            
            # Deadzones
            POSITION_DEADZONE = 0.08
            ALTITUDE_DEADZONE = 0.15
            DISTANCE_DEADZONE = 0.12
            
            # Body frame control computation
            ctrl_x = 0.0
            ctrl_y = 0.0
            
            # Left/right control
            if abs(err_x) > POSITION_DEADZONE:
                ctrl_y = self.pid_y.compute(err_x, dt)
            else:
                self.pid_y.reset()
            
            # Forward/back from distance control
            if abs(distance_error) > DISTANCE_DEADZONE:
                ctrl_x = -self.pid_distance.compute(distance_error, dt)
                
                # Debug logging every 20 iterations
                if not hasattr(self, '_debug_counter'):
                    self._debug_counter = 0
                self._debug_counter += 1
                if self._debug_counter % 20 == 0:
                    self.get_logger().warn(
                        f"DISTANCE: size={current_size:.1f}px target=70px "
                        f"dist_err={distance_error:+.3f} ctrl_x={ctrl_x:+.3f}"
                    )
            else:
                ctrl_x = 0.0
                self.pid_distance.reset()
            
            # Rate limiter to prevent jerky movements
            if not hasattr(self, 'prev_ctrl_x'):
                self.prev_ctrl_x = 0.0
                self.prev_ctrl_y = 0.0
            
            MAX_ACCEL = 0.4  # Maximum change per loop
            delta_x = ctrl_x - self.prev_ctrl_x
            delta_y = ctrl_y - self.prev_ctrl_y
            
            if abs(delta_x) > MAX_ACCEL:
                ctrl_x = self.prev_ctrl_x + (MAX_ACCEL if delta_x > 0 else -MAX_ACCEL)
            if abs(delta_y) > MAX_ACCEL:
                ctrl_y = self.prev_ctrl_y + (MAX_ACCEL if delta_y > 0 else -MAX_ACCEL)
            
            self.prev_ctrl_x = ctrl_x
            self.prev_ctrl_y = ctrl_y
            
            # Apply smoothing
            vx = self.smoother_x.filter(ctrl_x)
            vy = self.smoother_y.filter(ctrl_y)
            
            # Velocity limits
            MAX_VEL_XY = 1.0
            vx = max(min(vx, MAX_VEL_XY), -MAX_VEL_XY)
            vy = max(min(vy, MAX_VEL_XY), -MAX_VEL_XY)
            
            if abs(vx) < 0.05:
                vx = 0.0
            if abs(vy) < 0.05:
                vy = 0.0
            
            # Transform body to NED
            yaw = getattr(self, 'current_heading', 0.0)
            ned_vx = vx * math.cos(yaw) - vy * math.sin(yaw)
            ned_vy = vx * math.sin(yaw) + vy * math.cos(yaw)
            
            # Altitude control
            if not hasattr(self, 'target_altitude'):
                self.target_altitude = self.takeoff_altitude
            
            # Ground safety
            if self.estimated_altitude < 2.0:
                self.target_altitude = -6.0
                self.get_logger().error(f"GROUND SAFETY: {self.estimated_altitude:.1f}m - CLIMBING")
            elif abs(err_y) > ALTITUDE_DEADZONE and self.estimated_altitude > 3.0:
                altitude_adjustment = err_y * 0.8
                self.target_altitude += altitude_adjustment * 0.04
                self.target_altitude = max(min(self.target_altitude, -2.5), -8.0)
            
            # Build setpoint
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            setpoint.position = [float('nan'), float('nan'), self.target_altitude]
            setpoint.velocity = [float(ned_vx), float(ned_vy), float('nan')]
            setpoint.yaw = float('nan')
            
            self.setpoint_pub.publish(setpoint)
            
            # Enhanced logging
            mode = "BACKUP" if self.backing_away_mode else "TRACK"
            size_rate_str = f"rate={self.size_rate:+.1f}px/s" if hasattr(self, 'size_rate') else "rate=N/A"
            self.get_logger().info(
                f"{mode} | err=[{err_x:+.2f},{err_y:+.2f}] size={current_size:.0f}px {size_rate_str} dist_err={distance_error:+.2f} | "
                f"body=[{vx:+.2f},{vy:+.2f}] NED=[{ned_vx:+.2f},{ned_vy:+.2f}] | "
                f"alt={-self.target_altitude:.1f}m",
                throttle_duration_sec=0.5
            )
    def estimate_distance_from_bbox(self, bbox_area):
        """Exact distance estimation from Gazebo"""
        if bbox_area <= 0:
            return -1.0
        
        bbox_diagonal_px = math.sqrt(bbox_area)
        if bbox_diagonal_px < 10.0:
            return -1.0
        
        KNOWN_OBJECT_SIZE_M = 1.0
        FOCAL_LENGTH_PX = 350.0
        
        estimated_distance = (KNOWN_OBJECT_SIZE_M * FOCAL_LENGTH_PX) / bbox_diagonal_px
        return max(1.0, min(50.0, estimated_distance))
    
    def reset_controllers(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        self.pid_distance.reset()
        self.smoother_x.reset()
        self.smoother_y.reset()
        self.smoother_z.reset()
        self.smoother_distance.reset()
    
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)
    
    def send_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
    
    def state_machine(self):
            now = self.get_clock().now()
            
            if self.state == DroneState.ARMED and self.command_start_time:
                elapsed = (now - self.command_start_time).nanoseconds / 1e9
                
                if self.sequence_step == 1 and elapsed > 1.0:
                    self.get_logger().info("Switching to OFFBOARD mode...")
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self.sequence_step = 2
                    
                elif self.sequence_step == 2 and elapsed > 2.0:
                    self.get_logger().info(f"Takeoff command - target altitude: {-self.takeoff_altitude}m")
                    self.state = DroneState.TAKING_OFF
                    self.sequence_step = 0
                    
            elif self.state == DroneState.TAKING_OFF:
                # CRITICAL: Keep sending setpoints during takeoff
                self.send_takeoff_setpoint()
                
                # Also ensure offboard mode stays active
                if self.nav_state != 14:
                    self.get_logger().warn("Not in OFFBOARD! Resending mode command...", throttle_duration_sec=1.0)
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                
                current_altitude = -self.current_pos[2]
                target_altitude = -self.takeoff_altitude
                
                if abs(current_altitude - target_altitude) < 0.8:
                    self.get_logger().info(f"✓ Takeoff complete at {current_altitude:.1f}m - OFFBOARD mode active")
                    self.state = DroneState.HOVERING
                    
            elif self.state == DroneState.HOVERING:
                self.send_hover_setpoint()
            elif self.state == DroneState.TRACKING:    
                # Ensure still in offboard
                if self.nav_state != 14:
                    self.get_logger().warn("Lost OFFBOARD mode!", throttle_duration_sec=2.0)
    def send_takeoff_setpoint(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [self.current_pos[0], self.current_pos[1], self.takeoff_altitude]
        setpoint.velocity = [float('nan')] * 3
        setpoint.yaw = float('nan')
        self.setpoint_pub.publish(setpoint)
    
    def send_hover_setpoint(self):
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.position = [self.current_pos[0], self.current_pos[1], self.takeoff_altitude]
        setpoint.velocity = [0.0, 0.0, 0.0]
        setpoint.yaw = float('nan')
        self.setpoint_pub.publish(setpoint)
    
    # Service callbacks
    def arm_callback(self, request, response):
        if self.state != DroneState.DISARMED:
            return response
        
        self.get_logger().info("ARM sequence...")
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        time.sleep(2.0)
        self.get_logger().info("Streaming initial setpoints...")
        for i in range(100):  # 2.5 seconds at 20Hz
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            setpoint.position = [0.0, 0.0, self.takeoff_altitude] 
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yaw = float('nan')
            self.setpoint_pub.publish(setpoint)
            time.sleep(0.02)
        
        self.get_logger().info("Switching to OFFBOARD mode...")
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        time.sleep(1.0)
        self.state = DroneState.ARMED
        self.command_start_time = self.get_clock().now()
        self.sequence_step = 1
        return response
    
    def disarm_callback(self, request, response):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.state = DroneState.DISARMED
        self.tracking_enabled = False
        return response
    
    def takeoff_callback(self, request, response):
        if self.state == DroneState.DISARMED:
            return self.arm_callback(request, response)
        return response
    
    def land_callback(self, request, response):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.state = DroneState.LANDING
        self.tracking_enabled = False
        return response
    
    def emergency_callback(self, request, response):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0)
        self.state = DroneState.EMERGENCY
        self.tracking_enabled = False
        return response
    
    def start_tracking_callback(self, request, response):
        if self.state not in [DroneState.HOVERING, DroneState.TRACKING]:
            self.get_logger().warn(f"Can only start tracking from HOVER state, currently: {self.state.name}")
            return response
        
        if self.tracking_data.bbox_area <= 0:
            self.get_logger().warn("No target selected yet! Select target in tracker window first.")
            return response
        
        self.get_logger().info("Starting 3D tracking (Gazebo-style)")
        self.tracking_enabled = True
        self.state = DroneState.TRACKING
        self.reset_controllers()
        self.last_control_time = time.time()
        return response
    
    def stop_tracking_callback(self, request, response):
        self.get_logger().info("Stopping tracking")
        self.tracking_enabled = False
        self.state = DroneState.HOVERING
        self.reset_controllers()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FixedPX4Controller()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()