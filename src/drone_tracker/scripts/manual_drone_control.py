#!/usr/bin/env python3
# scripts/manual_drone_control.py - Updated with tracking controls

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import sys
import termios
import tty
import select
import time

class ManualDroneControl(Node):
    def __init__(self):
        super().__init__('manual_drone_control')
        
        # QoS for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Service clients
        self.arm_client = self.create_client(Empty, '/drone/arm')
        self.disarm_client = self.create_client(Empty, '/drone/disarm')
        self.takeoff_client = self.create_client(Empty, '/drone/takeoff')
        self.land_client = self.create_client(Empty, '/drone/land')
        self.emergency_client = self.create_client(Empty, '/drone/emergency')
        self.start_tracking_client = self.create_client(Empty, '/drone/start_tracking')
        self.stop_tracking_client = self.create_client(Empty, '/drone/stop_tracking')
        
        # Status subscriber
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_callback, qos_profile)
        
        self.running = True
        self.armed = False
        self.nav_state = 0
        self.tracking_active = False
        
    def status_callback(self, msg):
        old_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        
        if old_armed != self.armed:
            status = "ARMED" if self.armed else "DISARMED" 
            print(f"\n>>> Drone {status} <<<")
        
    def print_help(self):
        print("\n" + "="*60)
        print("      MANUAL DRONE CONTROL - ROS 2 Humble")
        print("="*60)
        print("Flight Commands:")
        print("  a - ARM drone")
        print("  d - DISARM drone")
        print("  t - TAKEOFF (auto-arms if needed)")
        print("  l - LAND")
        print("  e - EMERGENCY STOP")
        print()
        print("Tracking Commands:")
        print("  k - START TRACKING (after target selected)")
        print("  p - STOP TRACKING (pause tracking)")
        print()
        print("Info Commands:")
        print("  s - Show current status")
        print("  h - Show this help")
        print("  q - QUIT")
        print("-"*60)
        print("Current Status:")
        print(f"  Armed: {self.armed}")
        print(f"  Nav State: {self.nav_state}")
        print(f"  Tracking: {'ACTIVE' if self.tracking_active else 'INACTIVE'}")
        print("="*60)
        print("Press any key for command (no Enter needed)...")
        
    def call_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=2.0):
            print(f"X {service_name} service not available")
            return False
            
        print(f"Calling {service_name}...")
        request = Empty.Request()
        
        try:
            future = client.call_async(request)
            
            # Wait for result with timeout
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                print(f"OK {service_name} command sent")
                return True
            else:
                print(f"TIMEOUT {service_name} service timeout")
                return False
                
        except Exception as e:
            print(f"ERROR {service_name} failed: {e}")
            return False
    
    def get_key(self):
        """Get single keypress without Enter"""
        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
        
    def show_status(self):
        print(f"\nCurrent Status:")
        print(f"   Armed: {'YES' if self.armed else 'NO'}")
        print(f"   Nav State: {self.nav_state}")
        
        # Nav state meanings
        nav_states = {
            0: "MANUAL",
            1: "ALTCTL", 
            2: "POSCTL",
            3: "AUTO_MISSION",
            4: "AUTO_LOITER", 
            5: "AUTO_RTL",
            6: "OFFBOARD"
        }
        state_name = nav_states.get(self.nav_state, f"UNKNOWN({self.nav_state})")
        print(f"   Mode: {state_name}")
        print(f"   Tracking: {'ACTIVE' if self.tracking_active else 'INACTIVE'}")
        
    def run(self):
        self.print_help()
        
        # Set terminal to raw mode for single key input
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.running and rclpy.ok():
                # Process ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.0)
                
                # Check for keypress
                key = self.get_key()
                if key:
                    key = key.lower()
                    
                    if key == 'q':
                        print("\nQuitting manual control...")
                        self.running = False
                        
                    elif key == 'h':
                        self.print_help()
                        
                    elif key == 'a':
                        print("\nArming drone...")
                        self.call_service(self.arm_client, "ARM")
                        
                    elif key == 'd':
                        print("\nDisarming drone...")
                        self.call_service(self.disarm_client, "DISARM")
                        
                    elif key == 't':
                        print("\nInitiating takeoff sequence...")
                        print("   (Will auto-arm if needed)")
                        self.call_service(self.takeoff_client, "TAKEOFF")
                        
                    elif key == 'l':
                        print("\nLanding drone...")
                        self.call_service(self.land_client, "LAND")
                        self.tracking_active = False
                        
                    elif key == 'e':
                        print("\nEMERGENCY STOP!")
                        self.call_service(self.emergency_client, "EMERGENCY")
                        self.tracking_active = False
                        
                    elif key == 'k':
                        print("\nStarting object tracking...")
                       
                        if self.call_service(self.start_tracking_client, "START_TRACKING"):
                            self.tracking_active = True
                            print("   Tracking STARTED - Drone will follow selected target")
                        
                    elif key == 'p':
                        print("\nStopping tracking...")
                        if self.call_service(self.stop_tracking_client, "STOP_TRACKING"):
                            self.tracking_active = False
                            print("   Tracking STOPPED - Drone will hover in place")
                        
                    elif key == 's':
                        self.show_status()
                        
                    else:
                        print(f"\nUnknown command: '{key}' (press 'h' for help)")
                
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                        
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\nManual control terminated")

def main(args=None):
    rclpy.init(args=args)
    
    print("Starting Manual Drone Control Interface...")
    print("Waiting for drone connection...")
    
    control_node = ManualDroneControl()
    
    try:
        control_node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()