#!/bin/bash
# setup_drone_tracker.sh - Automated setup script for drone tracker package

set -e  # Exit on any error

echo "=================================================="
echo "  DRONE TRACKER PACKAGE SETUP - ROS2 HUMBLE"
echo "=================================================="

# Check if we're in the right location
if [[ ! -d "src" ]]; then
    echo "Creating ROS2 workspace..."
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
else
    echo "Found src directory, using current location"
fi

PACKAGE_NAME="drone_tracker"
PACKAGE_DIR="$PWD/$PACKAGE_NAME"

echo "Setting up package structure in: $PACKAGE_DIR"

# Create package directory structure
mkdir -p "$PACKAGE_DIR"/{src,include/drone_tracker,scripts,launch,config}

echo "✓ Created directory structure"

# Function to create file with content
create_file() {
    local file_path="$1"
    local content="$2"
    echo "$content" > "$file_path"
    echo "✓ Created: $file_path"
}

echo ""
echo "Creating package files..."

# Create CMakeLists.txt
create_file "$PACKAGE_DIR/CMakeLists.txt" 'cmake_minimum_required(VERSION 3.8)
project(drone_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# C++ executables
add_executable(tracker_node 
  src/tracker.cpp 
  src/object_tracker.cpp
)
ament_target_dependencies(tracker_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  cv_bridge 
  image_transport
  OpenCV
)

add_executable(gstreamer_camera_node 
  src/gstreamer_camera_node.cpp
)
ament_target_dependencies(gstreamer_camera_node 
  rclcpp 
  sensor_msgs 
  cv_bridge
  OpenCV
)

# Install C++ executables
install(TARGETS
  tracker_node
  gstreamer_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/working_px4_control.py
  scripts/safety_monitor.py
  scripts/manual_drone_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()'

# Create package.xml
create_file "$PACKAGE_DIR/package.xml" '<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drone_tracker</name>
  <version>1.0.0</version>
  <description>PX4 drone tracking system for ROS 2 Humble</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>px4_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>libopencv-dev</depend>
  <depend>rclpy</depend>
  
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>'

echo "✓ Created build files (CMakeLists.txt, package.xml)"

# Make scripts executable
chmod +x "$PACKAGE_DIR"/src/*.py 2>/dev/null || true
chmod +x "$PACKAGE_DIR"/scripts/*.py 2>/dev/null || true

echo ""
echo "=================================================="
echo "PACKAGE STRUCTURE CREATED SUCCESSFULLY!"
echo "=================================================="
echo ""
echo "Package location: $PACKAGE_DIR"
echo ""
echo "Next steps:"
echo "1. Copy the provided source files into their respective directories:"
echo ""
echo "   Source files (src/):"
echo "   ├── working_px4_control.py"
echo "   ├── tracker.cpp" 
echo "   ├── object_tracker.cpp"
echo "   └── gstreamer_camera_node.cpp"
echo ""
echo "   Header files (include/drone_tracker/):"
echo "   ├── tracker.hpp"
echo "   └── object_tracker.hpp"
echo ""
echo "   Scripts (scripts/):"
echo "   ├── safety_monitor.py"
echo "   └── manual_drone_control.py"
echo ""
echo "   Launch (launch/):"
echo "   └── drone_tracking.launch.py"
echo ""
echo "   Config (config/):"
echo "   └── tracking_params.yaml"
echo ""
echo "2. Build the package:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select drone_tracker"
echo "   source install/setup.bash"
echo ""
echo "3. Run the system:"
echo "   Terminal 1: cd ~/PX4-Autopilot && make px4_sitl gazebo"
echo "   Terminal 2: ros2 launch drone_tracker drone_tracking.launch.py"
echo ""
echo "=================================================="

# Create a simple README
create_file "$PACKAGE_DIR/README.md" "# Drone Tracker Package

ROS2 Humble package for PX4 drone tracking system.

## Quick Start

1. Build: \`colcon build --packages-select drone_tracker\`
2. Run PX4: \`make px4_sitl gazebo\` 
3. Launch: \`ros2 launch drone_tracker drone_tracking.launch.py\`
4. Control: \`ros2 run drone_tracker manual_drone_control.py\`

## Files Created by Setup Script

This directory structure was created. You need to add the source files from the artifacts provided.
"

echo "Setup complete! Package ready for source file installation."
