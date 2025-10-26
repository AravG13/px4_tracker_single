#!/usr/bin/env python3
# launch/drone_tracking.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os

def generate_launch_description():
    
    # Package directory
    pkg_drone_tracker = FindPackageShare('drone_tracker')
    
    # Launch arguments
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable safety monitoring'
    )
    
    enable_manual_control_arg = DeclareLaunchArgument(
        'enable_manual_control',
        default_value='false',
        description='Start manual control interface'
    )
    
    camera_source_arg = DeclareLaunchArgument(
        'camera_source',
        default_value='auto',
        choices=['auto', 'gstreamer', 'usb', 'test'],
        description='Camera source: auto, gstreamer (PX4), usb (/dev/video0), or test pattern'
    )
    
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='5.0',
        description='Takeoff altitude in meters'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='2.0',
        description='Maximum drone velocity in m/s'
    )
    
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('drone_tracker'),
        'config',
        'tracking_params.yaml'
    ])
    
    return LaunchDescription([
        # Arguments
        enable_safety_arg,
        enable_manual_control_arg,
        camera_source_arg,
        takeoff_altitude_arg,
        max_velocity_arg,
        
        # Startup information
        LogInfo(msg="================================================="),
        LogInfo(msg="      DRONE TRACKING SYSTEM - ROS2 HUMBLE       "),
        LogInfo(msg="================================================="),
        LogInfo(msg="Prerequisites:"),
        LogInfo(msg="1. PX4 SITL running: cd ~/PX4-Autopilot && make px4_sitl gazebo"),
        LogInfo(msg="2. QGroundControl closed"),
        LogInfo(msg="3. Camera connected (if using real camera)"),
        LogInfo(msg="================================================="),
        
        # System check
        ExecuteProcess(
            cmd=['bash', '-c', '''
                echo "Checking system prerequisites..."
                
                # Check if PX4 is running
                if pgrep -f "px4" > /dev/null; then
                    echo "✓ PX4 process detected"
                else
                    echo "✗ PX4 not running - start with: make px4_sitl gazebo"
                fi
                
                # Check ROS2 environment
                if [ -z "$ROS_DISTRO" ]; then
                    echo "✗ ROS2 not sourced - run: source /opt/ros/humble/setup.bash"
                else
                    echo "✓ ROS2 $ROS_DISTRO environment active"
                fi
                
                # Check workspace
                if [ -f "$AMENT_PREFIX_PATH/share/drone_tracker/package.xml" ]; then
                    echo "✓ drone_tracker package found"
                else
                    echo "✗ drone_tracker package not found - build workspace first"
                fi
                
                sleep 2
            '''],
            output='screen',
            name='system_check'
        ),
        
        # Camera node with fallback options
        Node(
            package='drone_tracker',
            executable='gstreamer_camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'camera_source': LaunchConfiguration('camera_source'),
                'use_sim_time': False,
                'frame_width': 640,
                'frame_height': 480,
                'fps': 30
            }],
            respawn=True,
            respawn_delay=3.0
        ),
        
        # Wait for camera initialization
        TimerAction(
            period=4.0,
            actions=[
                LogInfo(msg="Starting drone control and tracking nodes..."),
                
                # Main drone control node
                Node(
                    package='drone_tracker',
                    executable='working_px4_control.py',
                    name='drone_controller',
                    output='screen',
                    parameters=[{
                        'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                        'max_velocity': LaunchConfiguration('max_velocity'),
                        'target_follow_distance': 3.5,
                        'distance_tolerance_m': 0.8,
                        'min_altitude_m': 3.0,
                        'safety_altitude_m': 4.0,
                        'safety_radius': 50.0,
                        'hover_altitude_tolerance': 0.5,
                        'tracking_enabled_on_start': False,
                        'use_sim_time': False
                    }],
                    emulate_tty=True
                ),
                
                # Object tracker node
                Node(
                    package='drone_tracker',
                    executable='tracker_node',
                    name='object_tracker',
                    output='screen',
                    parameters=[{
                        'tracking_confidence_threshold': 0.7,
                        'movement_scale_x': 1.5,
                        'movement_scale_y': 1.5,
                        'enable_prediction': True,
                        'use_sim_time': False
                    }],
                    emulate_tty=True,
                    # Ensure display is available for OpenCV windows
                    additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')}
                ),
            ]
        ),
        
        # Safety monitor
        Node(
            condition=IfCondition(LaunchConfiguration('enable_safety')),
            package='drone_tracker',
            executable='safety_monitor.py',
            name='safety_monitor',
            output='screen',
            parameters=[config_file, {
                'max_altitude': 25.0,
                'max_distance_from_home': 100.0,
                'battery_warning_threshold': 20.0,
                'enable_battery_monitor': False,  # Disabled in simulation
                'use_sim_time': False
            }]
        ),
        
        # Manual control interface (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('enable_manual_control')),
            package='drone_tracker',
            executable='manual_drone_control.py',
            name='manual_controller',
            output='screen',
            emulate_tty=True,
            prefix='gnome-terminal -- '  # Open in separate terminal
        ),
        
        # System status monitor
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', '''
                        echo ""
                        echo "=== DRONE TRACKING SYSTEM STATUS ==="
                        
                        # Check active nodes
                        echo "Active Nodes:"
                        nodes=$(ros2 node list 2>/dev/null | grep -E "(drone_controller|object_tracker|camera_node|safety_monitor)" || echo "  No tracking nodes found")
                        echo "$nodes"
                        
                        # Check topics
                        echo ""
                        echo "Key Topics:"
                        if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
                            echo "  ✓ Camera feed: /camera/image_raw"
                            hz_info=$(timeout 3 ros2 topic hz /camera/image_raw 2>/dev/null | head -1 | cut -d: -f2 || echo " No data")
                            echo "    Rate:$hz_info"
                        else
                            echo "  ✗ No camera feed"
                        fi
                        
                        if ros2 topic list 2>/dev/null | grep -q "/detected_target"; then
                            echo "  ✓ Target detection: /detected_target"
                        else
                            echo "  - Target detection: waiting for target selection"
                        fi
                        
                        if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status"; then
                            echo "  ✓ PX4 connection: /fmu/out/vehicle_status"
                        else
                            echo "  ✗ PX4 not connected"
                        fi
                        
                        echo ""
                        echo "=== CONTROL COMMANDS ==="
                        echo "Service Control:"
                        echo "  ros2 service call /drone/arm std_srvs/srv/Empty"
                        echo "  ros2 service call /drone/takeoff std_srvs/srv/Empty"
                        echo "  ros2 service call /drone/land std_srvs/srv/Empty"
                        echo "  ros2 service call /drone/emergency std_srvs/srv/Empty"
                        echo ""
                        echo "Manual Control Interface:"
                        echo "  ros2 run drone_tracker manual_drone_control.py"
                        echo ""
                        echo "Target Selection:"
                        echo "  - Look for OpenCV camera window"
                        echo "  - Click and drag to select target"
                        echo "  - Drone will track selected object"
                        echo ""
                        echo "Emergency Stop: Ctrl+C or emergency service call"
                        echo "================================="
                    '''],
                    output='screen',
                    name='status_monitor'
                )
            ]
        ),
        
        # Shutdown message
        ExecuteProcess(
            cmd=['bash', '-c', '''
                # This runs on shutdown
                trap 'echo ""; echo "=== SYSTEM SHUTDOWN ==="; echo "All nodes terminated safely"; exit 0' INT TERM
                sleep infinity
            '''],
            output='screen',
            name='shutdown_handler'
        )
    ])