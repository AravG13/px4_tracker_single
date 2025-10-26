#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    return LaunchDescription([
        # Spawn TurtleBot3
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                         '-entity', 'turtlebot3',
                         '-database', 'turtlebot3_burger',
                         '-x', '5.0',
                         '-y', '0.0',
                         '-z', '0.0'],
                    output='screen'
                ),
            ]
        ),
        
        # Launch teleop keyboard in a new terminal
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'bash', '-c',
                         'source /opt/ros/humble/setup.bash && '
                         'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                         '--ros-args -r /cmd_vel:=/cmd_vel; exec bash'],
                    output='screen'
                )
            ]
        )
    ])