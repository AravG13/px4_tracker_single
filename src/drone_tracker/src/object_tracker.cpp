// src/object_tracker.cpp - Enhanced with Gazebo-style scale adaptation
#include "drone_tracker/object_tracker.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>

ObjectTracker::ObjectTracker() 
    : tracker_(nullptr), 
      bbox_(), 
      original_bbox_(),
      initialized_(false),
      scale_adaptation_enabled_(true),
      scale_adaptation_rate_(0.2),
      current_scale_factor_(1.0),
      min_scale_factor_(0.3),
      max_scale_factor_(3.0),
      scale_update_counter_(0) {
    
    center_history_.clear();
    time_history_.clear();
    confidence_history_.clear();
    scale_history_.clear();
    
    // Initialize SIFT detector for feature-based scale detection (like Gazebo)
    try {
        sift_detector_ = cv::SIFT::create(100); // Limit to 100 features for performance
    } catch (...) {
        std::cerr << "SIFT not available, using simpler scale adaptation\n";
        sift_detector_ = nullptr;
    }
}

bool ObjectTracker::initialize(cv::Mat &frame) {
    auto sel = cv::selectROI("Select Target - Click and drag", frame, false, false);
    if (sel.width <= 0 || sel.height <= 0) return false;
    
    bbox_ = cv::Rect(static_cast<int>(sel.x), static_cast<int>(sel.y), 
                     static_cast<int>(sel.width), static_cast<int>(sel.height));
    original_bbox_ = bbox_;

    try {
        tracker_ = cv::TrackerCSRT::create();
    } catch (...) {
        try { 
            tracker_ = cv::TrackerKCF::create(); 
        } catch(...) { 
            tracker_ = nullptr; 
        }
    }

    if (!tracker_) {
        std::cerr << "Tracker creation failed: ensure opencv-contrib is installed\n";
        return false;
    }

    tracker_->init(frame, bbox_);
    initialized_ = true;
    
    reset_tracking_history();
    cv::Point initial_center(bbox_.x + bbox_.width/2, bbox_.y + bbox_.height/2);
    update_tracking_history(initial_center, 1.0);
    
    // Initialize reference features for scale tracking
    update_reference_features(frame, bbox_);
    scale_history_.clear();
    scale_history_.push_back(1.0);
    current_scale_factor_ = 1.0;
    
    return true;
}

void ObjectTracker::manual_init(cv::Ptr<cv::Tracker> tracker, const cv::Rect& bbox) {
    tracker_ = tracker;
    bbox_ = bbox;
    original_bbox_ = bbox;
    initialized_ = true;
    
    reset_tracking_history();
    cv::Point initial_center(bbox_.x + bbox_.width/2, bbox_.y + bbox_.height/2);
    update_tracking_history(initial_center, 1.0);
    
    scale_history_.clear();
    scale_history_.push_back(1.0);
    current_scale_factor_ = 1.0;
}

void ObjectTracker::update_reference_features(const cv::Mat &frame, const cv::Rect &bbox) {
    if (!sift_detector_) return;
    
    if (bbox.x < 0 || bbox.y < 0 || 
        bbox.x + bbox.width > frame.cols || 
        bbox.y + bbox.height > frame.rows) {
        return;
    }
    
    cv::Mat roi = frame(bbox);
    if (roi.empty()) return;
    
    try {
        reference_keypoints_.clear();
        reference_descriptors_ = cv::Mat();
        sift_detector_->detectAndCompute(roi, cv::noArray(), reference_keypoints_, reference_descriptors_);
    } catch (...) {
        // Feature detection failed
    }
}

double ObjectTracker::estimate_scale_change_feature_based(const cv::Mat &frame, const cv::Rect &current_bbox) {
    if (!sift_detector_ || reference_keypoints_.empty() || reference_descriptors_.empty()) {
        return 1.0;
    }
    
    if (current_bbox.x < 0 || current_bbox.y < 0 || 
        current_bbox.x + current_bbox.width > frame.cols || 
        current_bbox.y + current_bbox.height > frame.rows) {
        return 1.0;
    }
    
    cv::Mat roi = frame(current_bbox);
    if (roi.empty()) return 1.0;
    
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat current_descriptors;
    
    try {
        sift_detector_->detectAndCompute(roi, cv::noArray(), current_keypoints, current_descriptors);
    } catch (...) {
        return 1.0;
    }
    
    if (current_keypoints.size() < 5 || current_descriptors.empty()) {
        return 1.0;
    }
    
    // Match features using FLANN for SIFT
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    
    try {
        matcher.knnMatch(reference_descriptors_, current_descriptors, knn_matches, 2);
    } catch (...) {
        return 1.0;
    }
    
    // Apply ratio test (Lowe's ratio)
    std::vector<cv::DMatch> good_matches;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() >= 2 && match_pair[0].distance < 0.7 * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }
    
    if (good_matches.size() < 10) {
        return 1.0;
    }
    
    // Calculate average scale change from matched keypoint sizes
    double total_scale = 0.0;
    int valid_matches = 0;
    
    for (const auto& match : good_matches) {
        double ref_size = reference_keypoints_[match.queryIdx].size;
        double curr_size = current_keypoints[match.trainIdx].size;
        
        if (ref_size > 0) {
            total_scale += curr_size / ref_size;
            valid_matches++;
        }
    }
    
    if (valid_matches > 5) {
        double avg_scale = total_scale / valid_matches;
        return std::max(0.5, std::min(2.0, avg_scale));
    }
    
    return 1.0;
}

double ObjectTracker::estimate_scale_change_template_based(const cv::Mat &frame, const cv::Rect &current_bbox) {
    // Fallback method: compare bbox size to original
    if (original_bbox_.width <= 0 || original_bbox_.height <= 0) {
        return 1.0;
    }
    
    double width_ratio = static_cast<double>(current_bbox.width) / original_bbox_.width;
    double height_ratio = static_cast<double>(current_bbox.height) / original_bbox_.height;
    
    // Use geometric mean for more balanced scaling
    double scale = std::sqrt(width_ratio * height_ratio);
    
    return std::max(0.5, std::min(2.0, scale));
}

void ObjectTracker::update_scale_adaptation(const cv::Mat &frame, const cv::Rect &detected_bbox) {
    if (!scale_adaptation_enabled_) return;
    
    scale_update_counter_++;
    
    // Update scale every 5 frames to reduce computation
    if (scale_update_counter_ % 5 != 0) {
        return;
    }
    
    double scale_estimate = 1.0;
    
    // Try feature-based method first (more accurate like Gazebo)
    if (sift_detector_) {
        scale_estimate = estimate_scale_change_feature_based(frame, detected_bbox);
        
        // If feature-based fails, fallback to template-based
        if (std::abs(scale_estimate - 1.0) < 0.01) {
            scale_estimate = estimate_scale_change_template_based(frame, detected_bbox);
        }
    } else {
        scale_estimate = estimate_scale_change_template_based(frame, detected_bbox);
    }
    
    // Add to history for smoothing
    scale_history_.push_back(scale_estimate);
    if (scale_history_.size() > 10) {
        scale_history_.pop_front();
    }
    
    // Calculate smoothed scale factor
    double smoothed_scale = calculate_smoothed_scale_factor();
    
    // Apply adaptation rate (gradual change like Gazebo)
    current_scale_factor_ = current_scale_factor_ * (1.0 - scale_adaptation_rate_) + 
                           smoothed_scale * scale_adaptation_rate_;
    
    // Clamp to limits
    current_scale_factor_ = std::max(min_scale_factor_, 
                                    std::min(max_scale_factor_, current_scale_factor_));
}

cv::Rect ObjectTracker::apply_scale_adaptation(const cv::Rect &original_rect, double scale_factor) {
    int new_width = static_cast<int>(original_rect.width * scale_factor);
    int new_height = static_cast<int>(original_rect.height * scale_factor);
    
    // Keep center position fixed
    int new_x = original_rect.x + (original_rect.width - new_width) / 2;
    int new_y = original_rect.y + (original_rect.height - new_height) / 2;
    
    return cv::Rect(new_x, new_y, new_width, new_height);
}

double ObjectTracker::calculate_smoothed_scale_factor() const {
    if (scale_history_.empty()) return 1.0;
    
    // Weighted average with more weight on recent values
    double weighted_sum = 0.0;
    double weight_sum = 0.0;
    
    for (size_t i = 0; i < scale_history_.size(); ++i) {
        double weight = (i + 1.0) / scale_history_.size();
        weighted_sum += scale_history_[i] * weight;
        weight_sum += weight;
    }
    
    return weight_sum > 0 ? weighted_sum / weight_sum : 1.0;
}

void ObjectTracker::set_scale_limits(double min_scale, double max_scale) {
    min_scale_factor_ = std::max(0.1, min_scale);
    max_scale_factor_ = std::min(5.0, max_scale);
}

bool ObjectTracker::update(cv::Mat &frame, cv::Point &center) {
    if (!initialized_) {
        return false;
    }
    
    cv::Rect updated_bbox = bbox_;
    bool ok = tracker_->update(frame, updated_bbox);
    
    double confidence = 0.0;
    
    if (ok) {
        // Validate bounding box
        if (updated_bbox.x >= 0 && updated_bbox.y >= 0 && 
            updated_bbox.x + updated_bbox.width <= frame.cols &&
            updated_bbox.y + updated_bbox.height <= frame.rows &&
            updated_bbox.width > 5 && updated_bbox.height > 5) {
            
            // Apply scale adaptation (like Gazebo)
            if (scale_adaptation_enabled_) {
                update_scale_adaptation(frame, updated_bbox);
                
                // Apply smoothed scale factor
                if (std::abs(current_scale_factor_ - 1.0) > 0.05) {
                    cv::Rect scaled_bbox = apply_scale_adaptation(updated_bbox, current_scale_factor_);
                    
                    // Validate scaled bbox
                    if (scaled_bbox.x >= 0 && scaled_bbox.y >= 0 &&
                        scaled_bbox.x + scaled_bbox.width <= frame.cols &&
                        scaled_bbox.y + scaled_bbox.height <= frame.rows &&
                        scaled_bbox.width > 5 && scaled_bbox.height > 5) {
                        updated_bbox = scaled_bbox;
                    }
                }
                
                // Update reference features periodically
                if (scale_update_counter_ % 30 == 0) {
                    update_reference_features(frame, updated_bbox);
                }
            }
            
            bbox_ = updated_bbox;
            center = cv::Point(bbox_.x + bbox_.width/2, bbox_.y + bbox_.height/2);
            
            // Calculate confidence
            if (!center_history_.empty()) {
                cv::Point last_center = center_history_.back();
                double distance = cv::norm(center - last_center);
                double size_change = std::abs(get_bbox_size() - calculate_smoothed_scale_factor() * get_bbox_size());
                
                confidence = std::max(0.0, 1.0 - (distance / 50.0) - (size_change / 20.0));
                confidence = std::min(1.0, confidence);
            } else {
                confidence = 1.0;
            }
            
            update_tracking_history(center, confidence);
            
            // Enhanced visual feedback with scale info
            cv::Scalar bbox_color = confidence > 0.7 ? cv::Scalar(0,255,0) : 
                                   confidence > 0.4 ? cv::Scalar(0,255,255) : cv::Scalar(0,0,255);
            
            cv::rectangle(frame, bbox_, bbox_color, 3);
            cv::circle(frame, center, 5, cv::Scalar(0,0,255), -1);
            
            // Draw corner markers
            int corner_size = 10;
            cv::Scalar corner_color(255, 255, 0);
            
            cv::line(frame, cv::Point(bbox_.x, bbox_.y), 
                    cv::Point(bbox_.x + corner_size, bbox_.y), corner_color, 2);
            cv::line(frame, cv::Point(bbox_.x, bbox_.y), 
                    cv::Point(bbox_.x, bbox_.y + corner_size), corner_color, 2);
            
            cv::line(frame, cv::Point(bbox_.x + bbox_.width, bbox_.y), 
                    cv::Point(bbox_.x + bbox_.width - corner_size, bbox_.y), corner_color, 2);
            cv::line(frame, cv::Point(bbox_.x + bbox_.width, bbox_.y), 
                    cv::Point(bbox_.x + bbox_.width, bbox_.y + corner_size), corner_color, 2);
            
            cv::line(frame, cv::Point(bbox_.x, bbox_.y + bbox_.height), 
                    cv::Point(bbox_.x + corner_size, bbox_.y + bbox_.height), corner_color, 2);
            cv::line(frame, cv::Point(bbox_.x, bbox_.y + bbox_.height), 
                    cv::Point(bbox_.x, bbox_.y + bbox_.height - corner_size), corner_color, 2);
            
            cv::line(frame, cv::Point(bbox_.x + bbox_.width, bbox_.y + bbox_.height), 
                    cv::Point(bbox_.x + bbox_.width - corner_size, bbox_.y + bbox_.height), corner_color, 2);
            cv::line(frame, cv::Point(bbox_.x + bbox_.width, bbox_.y + bbox_.height), 
                    cv::Point(bbox_.x + bbox_.width, bbox_.y + bbox_.height - corner_size), corner_color, 2);
            
            // Show confidence, velocity, and scale
            cv::Point2f velocity = get_object_velocity();
            std::string conf_text = "Conf: " + std::to_string((int)(confidence * 100)) + "%";
            std::string vel_text = "Vel: " + std::to_string((int)cv::norm(velocity)) + "px/s";
            std::string scale_text = "Scale: " + std::to_string(current_scale_factor_).substr(0, 4) + "x";
            
            cv::putText(frame, conf_text, cv::Point(bbox_.x, bbox_.y - 10), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, bbox_color, 1);
            cv::putText(frame, vel_text, cv::Point(bbox_.x, bbox_.y - 25), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,0,255), 1);
            cv::putText(frame, scale_text, cv::Point(bbox_.x, bbox_.y - 40), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,200,0), 1);
            
            // Draw velocity arrow
            if (cv::norm(velocity) > 2.0) {
                cv::Point vel_end = center + cv::Point(static_cast<int>(velocity.x * 0.5), 
                                                      static_cast<int>(velocity.y * 0.5));
                cv::arrowedLine(frame, center, vel_end, cv::Scalar(255,0,255), 2);
            }
            
        } else {
            ok = false;
            confidence = 0.0;
            update_tracking_history(cv::Point(-1, -1), confidence);
        }
    } else {
        confidence = 0.0;
        update_tracking_history(cv::Point(-1, -1), confidence);
    }
    
    return ok;
}

// Keep all other methods the same as before
cv::Rect ObjectTracker::get_bbox() const {
    return bbox_;
}

double ObjectTracker::get_bbox_size() const {
    if (!initialized_) return 0.0;
    return std::sqrt(bbox_.width * bbox_.width + bbox_.height * bbox_.height);
}

cv::Point ObjectTracker::get_smoothed_center() const {
    if (center_history_.empty()) return cv::Point(0, 0);
    
    cv::Point2f smoothed(0, 0);
    double total_weight = 0.0;
    
    for (size_t i = 0; i < center_history_.size(); ++i) {
        double weight = (i + 1.0) / center_history_.size();
        if (center_history_[i].x >= 0 && center_history_[i].y >= 0) {
            smoothed.x += center_history_[i].x * weight;
            smoothed.y += center_history_[i].y * weight;
            total_weight += weight;
        }
    }
    
    if (total_weight > 0) {
        smoothed.x /= total_weight;
        smoothed.y /= total_weight;
        return cv::Point(static_cast<int>(smoothed.x), static_cast<int>(smoothed.y));
    }
    
    return center_history_.back();
}

cv::Point2f ObjectTracker::get_object_velocity() const {
    return calculate_velocity();
}

double ObjectTracker::get_tracking_confidence() const {
    return calculate_smoothed_confidence();
}

bool ObjectTracker::is_object_moving(double threshold_pixels_per_sec) const {
    cv::Point2f velocity = calculate_velocity();
    return cv::norm(velocity) > threshold_pixels_per_sec;
}

cv::Point ObjectTracker::get_predicted_position(double time_ahead_seconds) const {
    if (center_history_.empty()) return cv::Point(0, 0);
    
    cv::Point current_center = center_history_.back();
    cv::Point2f velocity = calculate_velocity();
    
    cv::Point predicted = current_center + cv::Point(
        static_cast<int>(velocity.x * time_ahead_seconds),
        static_cast<int>(velocity.y * time_ahead_seconds)
    );
    
    return predicted;
}

void ObjectTracker::reset_tracking_history() {
    center_history_.clear();
    time_history_.clear();
    confidence_history_.clear();
}

void ObjectTracker::draw_enhanced_feedback(cv::Mat &frame, const cv::Point & /*center*/, bool show_velocity) const {
    if (!initialized_) return;
    
    // Draw tracking trail
    if (center_history_.size() > 1) {
        for (size_t i = 1; i < center_history_.size(); ++i) {
            if (center_history_[i-1].x >= 0 && center_history_[i].x >= 0) {
                double alpha = static_cast<double>(i) / center_history_.size();
                cv::Scalar trail_color(0, static_cast<int>(255 * alpha), static_cast<int>(255 * (1-alpha)));
                cv::line(frame, center_history_[i-1], center_history_[i], trail_color, 2);
            }
        }
    }
    
    // Draw prediction
    if (show_velocity && is_object_moving()) {
        cv::Point predicted = get_predicted_position(0.2);
        cv::circle(frame, predicted, 3, cv::Scalar(255,255,0), 2);
        cv::putText(frame, "PRED", cv::Point(predicted.x + 5, predicted.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255,255,0), 1);
    }
}

void ObjectTracker::update_tracking_history(const cv::Point &center, double confidence) {
    auto current_time = std::chrono::steady_clock::now();
    
    center_history_.push_back(center);
    time_history_.push_back(current_time);
    confidence_history_.push_back(confidence);
    
    while (center_history_.size() > MAX_HISTORY_SIZE) {
        center_history_.pop_front();
        time_history_.pop_front();
        confidence_history_.pop_front();
    }
}

cv::Point2f ObjectTracker::calculate_velocity() const {
    if (center_history_.size() < 2 || time_history_.size() < 2) {
        return cv::Point2f(0, 0);
    }
    
    std::vector<cv::Point2f> valid_points;
    std::vector<double> times;
    
    auto base_time = time_history_.front();
    
    for (size_t i = 0; i < center_history_.size(); ++i) {
        if (center_history_[i].x >= 0 && center_history_[i].y >= 0) {
            valid_points.push_back(cv::Point2f(center_history_[i].x, center_history_[i].y));
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_history_[i] - base_time);
            times.push_back(duration.count() / 1000.0);
        }
    }
    
    if (valid_points.size() < 2) return cv::Point2f(0, 0);
    
    cv::Point2f velocity = (valid_points.back() - valid_points.front()) / std::max(0.001, times.back() - times.front());
    
    const double MAX_VELOCITY = 100.0;
    if (cv::norm(velocity) > MAX_VELOCITY) {
        velocity = velocity * (MAX_VELOCITY / cv::norm(velocity));
    }
    
    return velocity;
}

double ObjectTracker::calculate_smoothed_confidence() const {
    if (confidence_history_.empty()) return 0.0;
    
    double smoothed_confidence = 0.0;
    double total_weight = 0.0;
    
    for (size_t i = 0; i < confidence_history_.size(); ++i) {
        double weight = std::exp(static_cast<double>(i) / confidence_history_.size());
        smoothed_confidence += confidence_history_[i] * weight;
        total_weight += weight;
    }
    
    return total_weight > 0 ? smoothed_confidence / total_weight : 0.0;
}