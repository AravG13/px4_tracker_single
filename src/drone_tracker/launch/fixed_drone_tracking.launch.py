#!/usr/bin/env python3
# launch/fixed_drone_tracking.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    
    # Launch arguments
    enable_manual_control_arg = DeclareLaunchArgument(
        'enable_manual_control',
        default_value='false',
        description='Start manual control interface'
    )
    
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='5.0',
        description='Takeoff altitude in meters'
    )
    
    return LaunchDescription([
        # Arguments
        enable_manual_control_arg,
        takeoff_altitude_arg,
        
        # System information
        LogInfo(msg="=== FIXED DRONE TRACKING SYSTEM ==="),
        LogInfo(msg="Prerequisites:"),
        LogInfo(msg="1. PX4 SITL: cd ~/PX4-Autopilot && make px4_sitl gazebo"),
        LogInfo(msg="2. QGroundControl closed"),
        LogInfo(msg="3. Display available for OpenCV"),
        LogInfo(msg="====================================="),
        
        # System check
        ExecuteProcess(
            cmd=['bash', '-c', '''
                echo "Checking system..."
                
                if pgrep -f "px4" > /dev/null; then
                    echo "✓ PX4 process running"
                else
                    echo "✗ PX4 not running - start with: make px4_sitl gazebo"
                fi
                
                if [ -n "$DISPLAY" ]; then
                    echo "✓ Display available: $DISPLAY"
                else
                    echo "✗ No DISPLAY - OpenCV windows may fail"
                fi
                
                if [ -n "$ROS_DISTRO" ]; then
                    echo "✓ ROS2 $ROS_DISTRO active"
                else
                    echo "✗ ROS2 environment not sourced"
                fi
            '''],
            output='screen'
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                # Camera node (from src/)
                Node(
                    package='drone_tracker',
                    executable='gstreamer_camera_node',
                    name='camera_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False
                    }],
                    respawn=True,
                    respawn_delay=2.0
                ),
                
                # Tracker node (from src/)
                Node(
                    package='drone_tracker',
                    executable='tracker_node',
                    name='object_tracker',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False
                    }],
                    additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
                    emulate_tty=True
                ),
                
                # Working PX4 controller (use existing script)
                
            ]
        ),
        
        # Manual control (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('enable_manual_control')),
            package='drone_tracker',
            executable='manual_drone_control.py',
            name='manual_controller',
            output='screen',
            emulate_tty=True,
            prefix='gnome-terminal -- '
        ),
        
        # Status monitor
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c', '''
                        echo ""
                        echo "=== INTEGRATION STATUS ==="
                        
                        echo "Active Nodes:"
                        ros2 node list 2>/dev/null | grep -E "(camera|tracker|controller)" | head -5
                        
                        echo ""
                        echo "Key Topics (with rates):"
                        
                        if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
                            echo "✓ Camera: /camera/image_raw"
                            timeout 2 ros2 topic hz /camera/image_raw 2>/dev/null | head -1 || echo "  No data yet"
                        else
                            echo "✗ No camera feed"
                        fi
                        
                        if ros2 topic list 2>/dev/null | grep -q "/target_bbox_info"; then
                            echo "✓ Bbox data: /target_bbox_info" 
                            timeout 2 ros2 topic hz /target_bbox_info 2>/dev/null | head -1 || echo "  Waiting for target selection"
                        else
                            echo "- Bbox data: waiting for tracker"
                        fi
                        
                        if ros2 topic list 2>/dev/null | grep -q "/enhanced_target_data"; then
                            echo "✓ Enhanced tracking: /enhanced_target_data"
                        else
                            echo "- Enhanced tracking: waiting"
                        fi
                        
                        if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_status"; then
                            echo "✓ PX4 connection active"
                        else
                            echo "✗ PX4 not connected"
                        fi
                        
                        echo ""
                        echo "=== QUICK USAGE ==="
                        echo "1. Wait for camera window to appear"
                        echo "2. Click and drag to select target"
                        echo "3. Use service calls to control:"
                        echo "   ros2 service call /drone/arm std_srvs/srv/Empty"
                        echo "   ros2 service call /drone/takeoff std_srvs/srv/Empty"
                        echo "   ros2 service call /drone/start_tracking std_srvs/srv/Empty"
                        echo "4. Emergency stop: ros2 service call /drone/emergency std_srvs/srv/Empty"
                        echo "========================"
                    '''],
                    output='screen'
                )
            ]
        )
    ])