#!/usr/bin/env python3
"""
Comprehensive ROS2 CSV Logger - ALL Topics
Logs every relevant topic from drone and rover in one CSV file
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 messages
from px4_msgs.msg import (
    VehicleLocalPosition, VehicleStatus, VehicleAttitude,
    VehicleGlobalPosition, VehicleControlMode, SensorCombined,
    BatteryStatus, VehicleOdometry, VehicleLandDetected,
    TimesyncStatus, FailsafeFlags
)

# Standard ROS messages
from geometry_msgs.msg import PointStamped, Vector3, Twist, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import csv
import math
from datetime import datetime
from pathlib import Path


class ComprehensiveLogger(Node):
    def __init__(self):
        super().__init__('comprehensive_logger')
        
        # Parameters
        self.declare_parameter('log_rate_hz', 10.0)
        self.declare_parameter('log_filename', 'comprehensive_data.csv')
        
        self.log_rate = self.get_parameter('log_rate_hz').value
        log_filename = self.get_parameter('log_filename').value
        
        # Create timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = Path(f"{log_filename.replace('.csv', '')}_{timestamp}.csv")
        
        self.get_logger().info(f"Comprehensive logging to: {self.log_file.absolute()}")
        self.get_logger().info(f"Log rate: {self.log_rate} Hz")
        
        # QoS profiles
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Storage for latest messages
        self.data = {
            # Drone data
            'vehicle_local_position': None,
            'vehicle_status': None,
            'vehicle_attitude': None,
            'vehicle_global_position': None,
            'vehicle_control_mode': None,
            'sensor_combined': None,
            'battery_status': None,
            'vehicle_odometry': None,
            'vehicle_land_detected': None,
            'timesync_status': None,
            'failsafe_flags': None,
            
            # Tracking data
            'detected_target': None,
            'target_bbox_info': None,
            'enhanced_target_data': None,
            
            # Rover data
            'cmd_vel': None,
            'joint_states': None,
            'goal_pose': None,
            'clicked_point': None
        }
        
        # Initialize CSV
        self._init_csv_file()
        
        # === DRONE SUBSCRIBERS ===
        self.create_subscription(VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            lambda msg: self._store('vehicle_local_position', msg), qos_px4)
        
        self.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            lambda msg: self._store('vehicle_status', msg), qos_px4)
        
        self.create_subscription(VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            lambda msg: self._store('vehicle_attitude', msg), qos_px4)
        
        self.create_subscription(VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            lambda msg: self._store('vehicle_global_position', msg), qos_px4)
        
        self.create_subscription(VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            lambda msg: self._store('vehicle_control_mode', msg), qos_px4)
        
        self.create_subscription(SensorCombined,
            '/fmu/out/sensor_combined',
            lambda msg: self._store('sensor_combined', msg), qos_px4)
        
        self.create_subscription(BatteryStatus,
            '/fmu/out/battery_status_v1',
            lambda msg: self._store('battery_status', msg), qos_px4)
        
        self.create_subscription(VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            lambda msg: self._store('vehicle_odometry', msg), qos_px4)
        
        self.create_subscription(VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            lambda msg: self._store('vehicle_land_detected', msg), qos_px4)
        
        self.create_subscription(TimesyncStatus,
            '/fmu/out/timesync_status',
            lambda msg: self._store('timesync_status', msg), qos_px4)
        
        self.create_subscription(FailsafeFlags,
            '/fmu/out/failsafe_flags',
            lambda msg: self._store('failsafe_flags', msg), qos_px4)
        
        # === TRACKING SUBSCRIBERS ===
        self.create_subscription(PointStamped,
            '/detected_target',
            lambda msg: self._store('detected_target', msg), qos_reliable)
        
        self.create_subscription(Vector3,
            '/target_bbox_info',
            lambda msg: self._store('target_bbox_info', msg), qos_reliable)
        
        self.create_subscription(Vector3,
            '/enhanced_target_data',
            lambda msg: self._store('enhanced_target_data', msg), qos_reliable)
        
        # === ROVER SUBSCRIBERS ===
        self.create_subscription(Twist,
            '/cmd_vel',
            lambda msg: self._store('cmd_vel', msg), qos_reliable)
        
        self.create_subscription(JointState,
            '/joint_states',
            lambda msg: self._store('joint_states', msg), qos_reliable)
        
        self.create_subscription(PoseStamped,
            '/goal_pose',
            lambda msg: self._store('goal_pose', msg), qos_reliable)
        
        self.create_subscription(PointStamped,
            '/clicked_point',
            lambda msg: self._store('clicked_point', msg), qos_reliable)
        
        # Logging timer
        self.log_timer = self.create_timer(1.0 / self.log_rate, self.write_log_entry)
        self.log_count = 0
        self.create_timer(5.0, self.print_status)
        
        self.get_logger().info("Comprehensive logger started - capturing ALL topics")
    
    def _store(self, key, msg):
        """Store latest message"""
        self.data[key] = msg
    
    def _init_csv_file(self):
        """Initialize CSV with comprehensive headers"""
        self.csv_file = open(self.log_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        headers = [
            # Time
            'timestamp_sec',
            
            # Vehicle Local Position (NED frame)
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'acc_x', 'acc_y', 'acc_z',
            'heading', 'delta_heading',
            'ref_lat', 'ref_lon', 'ref_alt',
            
            # Vehicle Global Position (GPS)
            'lat', 'lon', 'alt',
            'alt_ellipsoid',
            
            # Vehicle Attitude
            'q_w', 'q_x', 'q_y', 'q_z',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'rollspeed', 'pitchspeed', 'yawspeed',
            
            # Vehicle Status
            'arming_state', 'nav_state', 'failsafe',
            'system_type', 'system_id',
            
            # Vehicle Control Mode
            'flag_armed', 'flag_multicopter_position_control_enabled',
            'flag_control_manual_enabled', 'flag_control_auto_enabled',
            'flag_control_offboard_enabled', 'flag_control_rates_enabled',
            'flag_control_attitude_enabled', 'flag_control_altitude_enabled',
            'flag_control_climb_rate_enabled', 'flag_control_velocity_enabled',
            'flag_control_position_enabled',
            
            # Sensor Combined (IMU)
            'gyro_x', 'gyro_y', 'gyro_z',
            'accel_x', 'accel_y', 'accel_z',
            'mag_x', 'mag_y', 'mag_z',
            
            # Battery Status
            'voltage_v', 'current_a', 'remaining_percent',
            'discharged_mah', 'temperature',
            
            # Vehicle Odometry
            'odom_pos_x', 'odom_pos_y', 'odom_pos_z',
            'odom_vel_x', 'odom_vel_y', 'odom_vel_z',
            
            # Land Detected
            'landed', 'freefall', 'ground_contact',
            
            # Timesync
            'round_trip_time',
            
            # Failsafe Flags
            'mode_req_angular_velocity', 'mode_req_attitude',
            'mode_req_local_alt', 'mode_req_local_position',
            'mode_req_global_position', 'mode_req_mission',
            
            # Target Detection
            'target_x', 'target_y', 'target_z',
            
            # Bounding Box
            'bbox_area', 'bbox_confidence', 'boundary_violation',
            
            # Enhanced Target Data
            'enhanced_x', 'enhanced_y', 'enhanced_z',
            
            # Rover Command Velocity
            'rover_cmd_linear_x', 'rover_cmd_linear_y', 'rover_cmd_angular_z',
            
            # Rover Joints (up to 6 joints)
            'joint_1_pos', 'joint_2_pos', 'joint_3_pos',
            'joint_4_pos', 'joint_5_pos', 'joint_6_pos',
            'joint_1_vel', 'joint_2_vel', 'joint_3_vel',
            'joint_4_vel', 'joint_5_vel', 'joint_6_vel',
            
            # Goal Pose
            'goal_x', 'goal_y', 'goal_z',
            
            # Clicked Point
            'clicked_x', 'clicked_y', 'clicked_z'
        ]
        
        self.csv_writer.writerow(headers)
        self.csv_file.flush()
    
    def write_log_entry(self):
        """Write comprehensive log entry"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        row = [timestamp]
        
        # Vehicle Local Position
        if self.data['vehicle_local_position']:
            p = self.data['vehicle_local_position']
            row.extend([
                p.x, p.y, p.z,
                p.vx, p.vy, p.vz,
                p.ax, p.ay, p.az,
                p.heading, p.delta_heading,
                p.ref_lat, p.ref_lon, p.ref_alt
            ])
        else:
            row.extend([0.0] * 14)
        
        # Vehicle Global Position
        if self.data['vehicle_global_position']:
            g = self.data['vehicle_global_position']
            row.extend([g.lat, g.lon, g.alt, g.alt_ellipsoid])
        else:
            row.extend([0.0] * 4)
        
        # Vehicle Attitude
        if self.data['vehicle_attitude']:
            a = self.data['vehicle_attitude']
            q = a.q
            roll, pitch, yaw = self._quat_to_euler(q[1], q[2], q[3], q[0])
            # Angular velocity from delta_q if available, otherwise zeros
            rollspeed = getattr(a, 'rollspeed', 0.0)
            pitchspeed = getattr(a, 'pitchspeed', 0.0)
            yawspeed = getattr(a, 'yawspeed', 0.0)
            row.extend([
                q[0], q[1], q[2], q[3],
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                rollspeed, pitchspeed, yawspeed
            ])
        else:
            row.extend([0.0] * 10)
        
        # Vehicle Status
        if self.data['vehicle_status']:
            s = self.data['vehicle_status']
            row.extend([
                s.arming_state, s.nav_state, s.failsafe,
                s.system_type, s.system_id
            ])
        else:
            row.extend([0] * 5)
        
        # Vehicle Control Mode
        if self.data['vehicle_control_mode']:
            c = self.data['vehicle_control_mode']
            row.extend([
                int(c.flag_armed),
                int(c.flag_multicopter_position_control_enabled),
                int(c.flag_control_manual_enabled),
                int(c.flag_control_auto_enabled),
                int(c.flag_control_offboard_enabled),
                int(c.flag_control_rates_enabled),
                int(c.flag_control_attitude_enabled),
                int(c.flag_control_altitude_enabled),
                int(c.flag_control_climb_rate_enabled),
                int(c.flag_control_velocity_enabled),
                int(c.flag_control_position_enabled)
            ])
        else:
            row.extend([0] * 11)
        
        # Sensor Combined
        if self.data['sensor_combined']:
            s = self.data['sensor_combined']
            try:
                row.extend([
                    s.gyro_rad[0], s.gyro_rad[1], s.gyro_rad[2],
                    s.accelerometer_m_s2[0], s.accelerometer_m_s2[1], s.accelerometer_m_s2[2],
                    s.magnetometer_ga[0], s.magnetometer_ga[1], s.magnetometer_ga[2]
                ])
            except (AttributeError, IndexError):
                row.extend([0.0] * 9)
        else:
            row.extend([0.0] * 9)
        
        # Battery Status
        if self.data['battery_status']:
            b = self.data['battery_status']
            row.extend([
                b.voltage_v, b.current_a, b.remaining * 100,
                b.discharged_mah, b.temperature
            ])
        else:
            row.extend([0.0] * 5)
        
        # Vehicle Odometry
        if self.data['vehicle_odometry']:
            o = self.data['vehicle_odometry']
            row.extend([
                o.position[0], o.position[1], o.position[2],
                o.velocity[0], o.velocity[1], o.velocity[2]
            ])
        else:
            row.extend([0.0] * 6)
        
        # Land Detected
        if self.data['vehicle_land_detected']:
            l = self.data['vehicle_land_detected']
            row.extend([int(l.landed), int(l.freefall), int(l.ground_contact)])
        else:
            row.extend([0] * 3)
        
        # Timesync
        if self.data['timesync_status']:
            row.append(self.data['timesync_status'].round_trip_time)
        else:
            row.append(0)
        
        # Failsafe Flags
        if self.data['failsafe_flags']:
            f = self.data['failsafe_flags']
            row.extend([
                int(f.mode_req_angular_velocity),
                int(f.mode_req_attitude),
                int(f.mode_req_local_alt),
                int(f.mode_req_local_position),
                int(f.mode_req_global_position),
                int(f.mode_req_mission)
            ])
        else:
            row.extend([0] * 6)
        
        # Target Detection
        if self.data['detected_target']:
            t = self.data['detected_target'].point
            row.extend([t.x, t.y, t.z])
        else:
            row.extend([0.0] * 3)
        
        # Bounding Box
        if self.data['target_bbox_info']:
            b = self.data['target_bbox_info']
            row.extend([b.x, b.y, b.z])
        else:
            row.extend([0.0] * 3)
        
        # Enhanced Target
        if self.data['enhanced_target_data']:
            e = self.data['enhanced_target_data']
            row.extend([e.x, e.y, e.z])
        else:
            row.extend([0.0] * 3)
        
        # Rover Command Velocity
        if self.data['cmd_vel']:
            c = self.data['cmd_vel']
            row.extend([c.linear.x, c.linear.y, c.angular.z])
        else:
            row.extend([0.0] * 3)
        
        # Rover Joints
        if self.data['joint_states']:
            j = self.data['joint_states']
            # Positions (up to 6)
            for i in range(6):
                row.append(j.position[i] if i < len(j.position) else 0.0)
            # Velocities (up to 6)
            for i in range(6):
                row.append(j.velocity[i] if i < len(j.velocity) else 0.0)
        else:
            row.extend([0.0] * 12)
        
        # Goal Pose
        if self.data['goal_pose']:
            g = self.data['goal_pose'].pose.position
            row.extend([g.x, g.y, g.z])
        else:
            row.extend([0.0] * 3)
        
        # Clicked Point
        if self.data['clicked_point']:
            c = self.data['clicked_point'].point
            row.extend([c.x, c.y, c.z])
        else:
            row.extend([0.0] * 3)
        
        # Write row
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.log_count += 1
    
    def _quat_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def print_status(self):
        """Print status"""
        self.get_logger().info(f"Logged {self.log_count} comprehensive entries")
    
    def __del__(self):
        """Close file"""
        try:
            self.csv_file.close()
            self.get_logger().info(f"Comprehensive logs saved: {self.log_file.absolute()}")
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    logger = ComprehensiveLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info("\nStopping comprehensive logger...")
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()