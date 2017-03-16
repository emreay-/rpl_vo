#include "mono_odometer.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/tracking.hpp> // cv::calcOpticalFlowPyrLK
#include <opencv2/calib3d/calib3d.hpp> //cv::findEssentialMat

namespace rplvo_mono {

  MonoOdometer::MonoOdometer(const std::string node_namespace) :
    node_namespace_(node_namespace) {
    // Obtain mono odometer parameters from parameter server
    parameters_.Read(node_namespace_);
    // Create new camera object with parameters from the server assigned to the member camera pointer
    if (!vk::camera_loader::loadFromRosNs(node_namespace_, camera_ptr_)) {
        throw std::runtime_error("Missing or undefined camera model parameter.");
    }
    // Camera subscription with image_transport
    ros::NodeHandle nh;
    image_transport::ImageTransport img_transport(nh);
    image_sub_ = img_transport.subscribe(parameters_.image_topic, 1, &MonoOdometer::ImageCallback, this);
    first_run_ = true;
  } /* constructor MonoOdometer */

  MonoOdometer::~MonoOdometer() {
    delete camera_ptr_;
  }

  void MonoOdometer::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    // Converting the sensor_msgs::Image type to cv::Mat
    cv_bridge::CvImagePtr input_image_ptr;
    try {
      input_image_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Rectify the image
    const cv::Mat raw_image = input_image_ptr->image.clone();
    cv::Mat rectified_image;
    camera_ptr_->undistortImage(raw_image, rectified_image);
    current_image_ = rectified_image.clone();
  } /* function ImageCallback */

  void MonoOdometer::CalculateOdometry() {
//    TODO: Throw exception if the curr_img is empty ie. no messages received in callback
//    if (current_image_.empty()) {
//        throw
//      }
    if (first_run_) {
      previous_features_.clear();
      previous_features_ = DetectFeatures(current_image_, image_to_draw_features_);
      previous_image_ = current_image_.clone();
      first_run_ = false;
    } else {
      TrackFeatures();
      if (current_features_.size() < parameters_.min_number_of_features) {
        previous_features_ = DetectFeatures(previous_image_, image_to_draw_features_);
        TrackFeatures();
      }
      cv::Mat essential_matrix, rotation, translation;
      std::vector< uchar > mask;
      cv::Point2d principal_point(camera_ptr_->cx_, camera_ptr->cy);
      double focal_length = camera_ptr_->fx_;
      // TODO: essential matrix, I have to install opencv3! :(
      previous_image_ = current_image_.clone();
    }
  } /* function CalculateOdometry */

  PointVector MonoOdometer::DetectFeatures(cv::Mat input_image, cv::Mat output_image) {
    KeyPointVector features;
    cv::FAST(input_image, features, parameters_.feature_detector_threshold, true);
    ROS_DEBUG("Number of features: %d", features.size());
    cv::drawKeypoints(input_image, features, output_image,
                      cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    PointVector feature_points;
    cv::KeyPoint::convert(features, feature_points, std::vector< int >());
    return feature_points;
  }

  void MonoOdometer::TrackFeatures() {
    std::vector< uchar > status;
    std::vector< double > error;
    cv::Size window_size(parameters_.feature_tracker_window_size,
                         parameters_.feature_tracker_window_size);
    cv::TermCriteria criterion(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,
                               parameters_.feature_tracker_max_iterations,
                               parameters_.feature_tracker_epsilon);
    PointVector tracked_features;
    int flags = 0;
    cv::calcOpticalFlowPyrLK(previous_image_, current_image_,
                             previous_features_, tracked_features,
                             status, error, window_size,
                             parameters_.feature_tracker_max_pyramid_level,
                             criterion, flags, parameters_.feature_tracker_eigen_threshold);
    // Only pass the feature points that successfully tracked
    current_features_.clear();
    PointVector previous_features_temp;
    for (size_t i = 0; i < status.size(); i++) {
        if (status[i] == 1) {
          current_features_.push_back(tracked_features[i]);
          previous_features_temp.push_back(previous_features_[i]);
        }
    }
    previous_features_.clear();
    previous_features_ = previous_features_temp;
  }













} /* namespace rplvo_mono */
