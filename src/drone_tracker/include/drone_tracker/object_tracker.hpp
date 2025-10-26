#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <deque>
#include <chrono>

class ObjectTracker {
public:
    ObjectTracker();
    
    bool initialize(cv::Mat &frame);
    void manual_init(cv::Ptr<cv::Tracker> tracker, const cv::Rect& bbox);
    bool update(cv::Mat &frame, cv::Point &center);
    
    cv::Rect get_bbox() const;
    double get_bbox_size() const;
    cv::Point get_smoothed_center() const;
    cv::Point2f get_object_velocity() const;
    double get_tracking_confidence() const;
    bool is_object_moving(double threshold_pixels_per_sec = 5.0) const;
    cv::Point get_predicted_position(double time_ahead_seconds = 0.1) const;
    
    void reset_tracking_history();
    void draw_enhanced_feedback(cv::Mat &frame, const cv::Point &center, bool show_velocity = true) const;
    
    // NEW: Scale adaptation methods from Gazebo
    void enable_scale_adaptation(bool enable = true) { scale_adaptation_enabled_ = enable; }
    void set_scale_adaptation_rate(double rate) { 
        scale_adaptation_rate_ = std::max(0.01, std::min(0.5, rate)); 
    }
    double get_scale_factor() const { return current_scale_factor_; }
    void set_scale_limits(double min_scale, double max_scale);
    
private:
    cv::Ptr<cv::Tracker> tracker_;
    cv::Rect bbox_;
    cv::Rect original_bbox_;  // Store original bounding box for reference
    bool initialized_;
    
    // Tracking history
    static const size_t MAX_HISTORY_SIZE = 10;
    std::deque<cv::Point> center_history_;
    std::deque<std::chrono::steady_clock::time_point> time_history_;
    std::deque<double> confidence_history_;
    
    // NEW: Scale adaptation members (from Gazebo)
    bool scale_adaptation_enabled_;
    double scale_adaptation_rate_;
    double current_scale_factor_;
    double min_scale_factor_;
    double max_scale_factor_;
    std::deque<double> scale_history_;
    cv::Ptr<cv::SIFT> sift_detector_;  // For feature-based scale detection
    std::vector<cv::KeyPoint> reference_keypoints_;
    cv::Mat reference_descriptors_;
    int scale_update_counter_;
    
    // Helper methods
    void update_tracking_history(const cv::Point &center, double confidence);
    cv::Point2f calculate_velocity() const;
    double calculate_smoothed_confidence() const;
    
    // NEW: Scale adaptation helper methods (from Gazebo)
    void update_scale_adaptation(const cv::Mat &frame, const cv::Rect &detected_bbox);
    double estimate_scale_change_feature_based(const cv::Mat &frame, const cv::Rect &current_bbox);
    double estimate_scale_change_template_based(const cv::Mat &frame, const cv::Rect &current_bbox);
    cv::Rect apply_scale_adaptation(const cv::Rect &original_rect, double scale_factor);
    void update_reference_features(const cv::Mat &frame, const cv::Rect &bbox);
    double calculate_smoothed_scale_factor() const;
};