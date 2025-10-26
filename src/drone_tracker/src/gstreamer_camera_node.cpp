// Save as: src/gstreamer_camera_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class GstreamerCameraNode : public rclcpp::Node
{
public:
    GstreamerCameraNode() : Node("gstreamer_camera_node")
    {
        // Create ROS2 publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        
        // Define GStreamer pipeline
         std::string pipeline = "udpsrc port=5600 ! "
                              "application/x-rtp,encoding-name=H264,payload=96 ! "
                              "rtph264depay ! h264parse ! avdec_h264 ! "
                              "videoconvert ! appsink";
        
        cap_.open(pipeline, cv::CAP_GSTREAMER);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer pipeline");
            RCLCPP_INFO(this->get_logger(), "Make sure PX4 is streaming video to UDP port 5600");
            RCLCPP_INFO(this->get_logger(), "Close QGroundControl before running this node");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "GStreamer camera node started - receiving from UDP:5600");
        
        // Timer to capture and publish frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // ~30 FPS
            std::bind(&GstreamerCameraNode::capture_and_publish, this));
    }

private:
    void capture_and_publish()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Failed to read frame from GStreamer");
            return;
        }
        
        if (frame.empty()) {
            return;
        }
        
        // Convert to ROS2 message
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_link";
        
        image_pub_->publish(*msg);
        
        static int count = 0;
        if (++count % 90 == 0) {  // Log every 3 seconds at 30fps
            RCLCPP_INFO(this->get_logger(), "Published %d frames", count);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GstreamerCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}