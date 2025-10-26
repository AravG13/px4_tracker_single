#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from geometry_msgs.msg import PointStamped, Vector3, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import csv
import time
from pathlib import Path

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Create log file with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_dir = Path.home() / 'drone_tracking_logs'
        log_dir.mkdir(exist_ok=True)
        self.csv_file = log_dir / f'tracking_data_{timestamp}.csv'
        
        # Initialize data storage
        self.data = {
            'timestamp': 0.0,
            'drone_x': 0.0, 'drone_y': 0.0, 'drone_z': 0.0,
            'drone_vx': 0.0, 'drone_vy': 0.0, 'drone_vz': 0.0,
            'target_norm_x': 0.0, 'target_norm_y': 0.0,
            'bbox_area': 0.0, 'boundary_violation': 0.0,
            'turtlebot_vx': 0.0, 'turtlebot_vz': 0.0,
            'armed': 0, 'nav_state': 0
        }
        
        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                self.position_callback, qos_profile)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                self.status_callback, qos_profile)
        self.create_subscription(PointStamped, '/detected_target',
                                self.target_callback, 10)
        self.create_subscription(Vector3, '/target_bbox_info',
                                self.bbox_callback, 10)
        self.create_subscription(Twist, '/cmd_vel',
                                self.turtlebot_callback, 10)
        
        # Write CSV header
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data.keys())
            writer.writeheader()
        
        # Timer to write data at 10Hz
        self.create_timer(0.1, self.write_data)
        
        self.get_logger().info(f'Data logger started. Writing to: {self.csv_file}')
    
    def position_callback(self, msg):
        self.data['timestamp'] = time.time()
        self.data['drone_x'] = msg.x
        self.data['drone_y'] = msg.y
        self.data['drone_z'] = msg.z
        self.data['drone_vx'] = msg.vx
        self.data['drone_vy'] = msg.vy
        self.data['drone_vz'] = msg.vz
    
    def status_callback(self, msg):
        self.data['armed'] = 1 if msg.arming_state == 2 else 0
        self.data['nav_state'] = msg.nav_state
    
    def target_callback(self, msg):
        self.data['target_norm_x'] = msg.point.x
        self.data['target_norm_y'] = msg.point.y
    
    def bbox_callback(self, msg):
        self.data['bbox_area'] = msg.x
        self.data['boundary_violation'] = msg.z
    
    def turtlebot_callback(self, msg):
        self.data['turtlebot_vx'] = msg.linear.x
        self.data['turtlebot_vz'] = msg.angular.z
    
    def write_data(self):
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data.keys())
            writer.writerow(self.data)

def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logger stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()