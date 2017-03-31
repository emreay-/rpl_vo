#include "rplvo_mono/rpl_mono_odometer.h"
#include <ros/ros.h>
#include <ros/exception.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vikit/camera_loader.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/video/tracking.hpp> // cv::calcOpticalFlowPyrLK
#include <opencv2/calib3d/calib3d.hpp> //cv::findEssentialMat
#include <iostream>

namespace rplvo {
  namespace mono {
    ///
    /// \fn ModoOdometer
    /// \brief Constructor for MonoOdometer class.
    /// \param node_namespace String to state the namespace of the node.
    ///
    /// Constructs the object, gathers all the necessary parameters from the parameter server, subscribes to the image topic
    ///
    MonoOdometer::MonoOdometer(std::string node_namespace) :
      node_namespace_(node_namespace),
      camera_ptr_(NULL),
      first_run_(true) {
      // Obtain mono odometer parameters from parameter server
      parameters_.Read(node_namespace_);
      // Create new camera object with parameters from the server assigned to the member camera pointer
      if (!vk::camera_loader::loadFromRosNs(node_namespace_, camera_ptr_)) {
          throw std::runtime_error("Missing or undefined camera model parameter.");
      }
      current_image_.zeros(1,1,CV_8U);
      previous_image_.zeros(1,1,CV_8U);
      current_features_.clear();
      previous_features_.clear();

      // Camera subscription with image_transport
      ros::NodeHandle nh;
      image_transport::ImageTransport img_transport(nh);
      image_sub_ = img_transport.subscribe(parameters_.image_topic, 2, &MonoOdometer::ImageCallback, this);

      // If visualizing is switched on, create a visualizer object
      if(parameters_.visualize) {
        visualizer_ptr_ = new visualizer::Visualizer(node_namespace_);
      }
    }// constructor MonoOdometer

    ///
    /// \fn ~MonoOdometer
    /// \brief Destructor of MonoOdometer class.
    ///
    /// Releases the camera object pointer.
    ///
    MonoOdometer::~MonoOdometer() {
      delete camera_ptr_;
      if (parameters_.visualize) {
        delete visualizer_ptr_;
      }
    }

    ///
    /// \fn ImageCallback
    /// \brief Converts the image from ros message type to opencv type, rectifies it using the camera calibration parameters and assings it to the current image.
    /// \param msg Incoming message from the image topic.
    ///
    void MonoOdometer::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
      current_image_header_ = msg->header;
      // Converting the sensor_msgs::Image type to cv::Mat
      cv_bridge::CvImagePtr input_image_ptr;
      try {
        input_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      const cv::Mat raw_image = input_image_ptr->image.clone();
      // Rectify the image if the parameter is passed
      if (parameters_.rectify_image) {
        cv::Mat rectified_image;
        // Casting pointers from AbstractCamera ptr to PinholeCamera ptr.
        // The latter is a child class of the first and we know we would only use pinhole model
        ((vk::PinholeCamera*)camera_ptr_)->undistortImage(raw_image, rectified_image);
        current_image_ = rectified_image.clone();
      } else {
        current_image_ = raw_image;
      }
      // Publish current image
      ros::NodeHandle nh("~");
      image_pub_ = nh.advertise<sensor_msgs::Image>(node_namespace_+"/image_rect",1,this);
      cv_bridge::CvImage current;
      current.image = current_image_;
      current.encoding = "bgr8";
      // Covert cv::Mat to sensor_msgs::Image and publish
      image_pub_.publish(current.toImageMsg());

    }// function ImageCallback

    ///
    /// \fn DetectFeatures
    /// \brief Detects features using FAST algorithm on the input image.
    /// \param input_image Image which features are to be detected.
    /// \return Vector of points corresponding to the detected point features.
    ///
    PointVector MonoOdometer::DetectFeatures(cv::Mat input_image) {
      KeyPointVector features;
      int no_feature_count = 0;
      while (features.empty()) {
        no_feature_count++;
        cv::FAST(input_image, features, parameters_.feature_detector_threshold, true);
        if (no_feature_count > 10) {
          throw std::runtime_error("Can't detect features on current scene!");
        }
      }
      ROS_DEBUG("Number of features: %zu", features.size());
      PointVector feature_points;
      cv::KeyPoint::convert(features, feature_points, std::vector< int >());
      return feature_points;
    }// function DetectFeatures

    ///
    /// \fn TrackFeatures
    /// \brief Tracks the previous features on the current image using Lucas-Kanade method with pyramids.
    ///
    void MonoOdometer::TrackFeatures() {
      std::vector< uchar > status;
      cv::Mat error;
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
    }// function TrackFeatures

    ///
    /// \fn CalculateOdometry
    /// \brief Performs visual odometry
    ///
    void MonoOdometer::CalculateOdometry() {
      //ROS_DEBUG("Entered Calculate Odometry");
      // Throw exception if the curr_img is empty ie. no messages received in callback
      if (current_image_.empty()) {
        throw ros::Exception("No image message received yet.");
      } else if (first_run_) {
        previous_features_.clear();
        previous_features_ = DetectFeatures(current_image_);
        previous_image_ = current_image_.clone();
        first_run_ = false;
      } else {
        TrackFeatures();
        if (current_features_.size() < parameters_.min_number_of_features) {
          ROS_ERROR("Injecting new features");
          previous_features_ = DetectFeatures(previous_image_);
          TrackFeatures();
        }

        if (parameters_.visualize) {
          visualizer_ptr_->ShowTracking(current_image_, current_features_);
        }

        // Estimation and publishing
//        cv::Mat essential_matrix, rotation, translation, mask;
//        cv::Point2d principal_point(((vk::PinholeCamera*)camera_ptr_)->cx(), ((vk::PinholeCamera*)camera_ptr_)->cy());
//        double focal_length = ((vk::PinholeCamera*)camera_ptr_)->fx();
//        essential_matrix = cv::findEssentialMat(current_features_, previous_features_, focal_length, principal_point,
//                                                cv::RANSAC, parameters_.ransac_confidence, parameters_.ransac_threshold,
//                                                mask);
//        cv::recoverPose(essential_matrix, current_features_, previous_features_, rotation, translation, focal_length, principal_point, mask);

//        ROS_INFO("Essential Matix:");
//        for (int i = 0; i < essential_matrix.rows; i++) {
//          for (int j = 0; j < essential_matrix.cols; j++) {
//            std::cout << essential_matrix.at<int>(i,j) << "\t";
//          }
//          std::cout << std::endl;
//        }

//        ROS_INFO("Rotation Matix:");
//        for (int i = 0; i < rotation.rows; i++) {
//          for (int j = 0; j < rotation.cols; j++) {
//            std::cout << rotation.at<int>(i,j) << "\t";
//          }
//          std::cout << std::endl;
//        }

//        ROS_INFO("Trans Matix:");
//        for (int i = 0; i < translation.rows; i++) {
//          for (int j = 0; j < translation.cols; j++) {
//            std::cout << translation.at<int>(i,j) << "\t";
//          }
//          std::cout << std::endl;
//        }

//        tf::Matrix3x3 R(
//          rotation.at<int>(0,0), rotation.at<int>(0,1), rotation.at<int>(0,2),
//          rotation.at<int>(1,0), rotation.at<int>(1,1), rotation.at<int>(1,2),
//          rotation.at<int>(2,0), rotation.at<int>(2,1), rotation.at<int>(2,2));
//        tf::Vector3 T(translation.at<int>(0,0), translation.at<int>(0,1), translation.at<int>(0,2));
//        tf::Transform delta_transform(R,T);
//        integrateAndPublish(delta_transform, current_image_header_.stamp);

        previous_image_ = current_image_.clone();
        previous_features_.clear();
        previous_features_ = current_features_;
        current_features_.clear();
      }
    }// function CalculateOdometry



  }// namespace mono
}// namespace rplvo
