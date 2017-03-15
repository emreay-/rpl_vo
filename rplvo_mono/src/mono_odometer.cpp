#include "mono_odometer.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/tracking.hpp> // cv::calcOpticalFlowPyrLK

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
      previous_features_ = DetectFeatures();
      previous_image_ = current_image_.clone();
      first_run_ = false;
    } else {
//    TODO

    }
  } /* function CalculateOdometry */

  PointVector MonoOdometer::DetectFeatures() {
    KeyPointVector features;
    cv::FAST(current_image_, features, parameters_.feature_detector_threshold, true);
    ROS_DEBUG("Number of features: %d", features.size());
    cv::drawKeypoints(current_image_, features, image_to_draw_features,
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
    current_features_.clear();
    int flags = 0;
    cv::calcOpticalFlowPyrLK(previous_image_, current_image_,
                             previous_features_, current_features_,
                             status, error, window_size,
                             parameters_.feature_tracker_max_pyramid_level,
                             criterion, flags, parameters_.feature_tracker_eigen_threshold);


  }













} /* namespace rplvo_mono */
