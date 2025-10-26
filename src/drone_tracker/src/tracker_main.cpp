// src/tracker_main.cpp - Main function for tracker_node
#include "drone_tracker/tracker.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TrackerNode>();
    
    RCLCPP_INFO(node->get_logger(), "Tracker Node started - waiting for camera");
    RCLCPP_INFO(node->get_logger(), "Select target in camera window by clicking and dragging");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}