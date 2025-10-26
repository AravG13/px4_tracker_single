
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import math

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Parameters
        self.declare_parameter('max_altitude', 20.0)
        self.declare_parameter('max_distance_from_home', 100.0)
        self.declare_parameter('battery_warning_threshold', 20.0)
        
        self.max_altitude = self.get_parameter('max_altitude').value
        self.max_distance_from_home = self.get_parameter('max_distance_from_home').value
        self.battery_threshold = self.get_parameter('battery_warning_threshold').value
        
        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_callback, 10)
            
        # Service clients for emergency actions
        self.emergency_client = self.create_client(Empty, '/drone_control/emergency')
        self.land_client = self.create_client(Empty, '/drone_control/land')
        
        # Publishers
        self.safety_status_pub = self.create_publisher(Bool, '/safety/status_ok', 10)
        
        # State variables
        self.home_position = None
        self.current_position = None
        self.armed = False
        self.battery_percentage = 100.0
        self.safety_violations = 0
        
        # Timer for safety checks
        self.safety_timer = self.create_wall_timer(1.0, self.safety_check)
        
        self.get_logger().info("Safety Monitor started")
        
    def vehicle_status_callback(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        
    def position_callback(self, msg):
        self.current_position = Point()
        self.current_position.x = msg.x
        self.current_position.y = msg.y
        self.current_position.z = msg.z
        
        # Set home position on first armed state
        if self.home_position is None and self.armed:
            self.home_position = self.current_position
            self.get_logger().info(f"Home position set: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})")
            
    def battery_callback(self, msg):
        self.battery_percentage = msg.remaining * 100.0
        
    def safety_check(self):
        if not self.armed or self.current_position is None:
            return
            
        safety_ok = True
        
        # Check altitude (convert from NED)
        current_altitude = -self.current_position.z
        if current_altitude > self.max_altitude:
            self.get_logger().error(f"ALTITUDE VIOLATION: {current_altitude:.1f}m > {self.max_altitude}m")
            safety_ok = False
            
        # Check distance from home
        if self.home_position is not None:
            distance = math.sqrt(
                (self.current_position.x - self.home_position.x)**2 +
                (self.current_position.y - self.home_position.y)**2
            )
            if distance > self.max_distance_from_home:
                self.get_logger().error(f"DISTANCE VIOLATION: {distance:.1f}m from home")
                safety_ok = False
                
        # Check battery
        if self.battery_percentage < self.battery_threshold:
            self.get_logger().warn(f"LOW BATTERY: {self.battery_percentage:.1f}%")
            if self.battery_percentage < 10.0:
                self.get_logger().error("CRITICAL BATTERY - Initiating emergency landing")
                self.trigger_emergency_land()
                safety_ok = False
                
        # Handle safety violations
        if not safety_ok:
            self.safety_violations += 1
            if self.safety_violations > 3:
                self.get_logger().error("MULTIPLE SAFETY VIOLATIONS - Triggering emergency stop")
                self.trigger_emergency()
        else:
            self.safety_violations = 0
            
        # Publish safety status
        safety_msg = Bool()
        safety_msg.data = safety_ok
        self.safety_status_pub.publish(safety_msg)
        
    def trigger_emergency(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.emergency_client.call_async(req)
            self.get_logger().error("EMERGENCY STOP TRIGGERED")
            
    def trigger_emergency_land(self):
        if self.land_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            self.land_client.call_async(req)
            self.get_logger().warn("EMERGENCY LANDING TRIGGERED")

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()