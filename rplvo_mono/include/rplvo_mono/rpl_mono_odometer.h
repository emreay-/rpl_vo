#ifndef rpl_mono_odometer_h
#define rpl_mono_odometer_h

#include "odometer_base.h"
#include <vector>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <vikit/params_helper.h>
#include <vikit/abstract_camera.h>

namespace rplvo_mono {

  typedef std::vector< cv::KeyPoint > KeyPointVector;
  typedef std::vector< cv::Point2f > PointVector;

  ///
  /// \struct MonoOdometerParameters
  /// \brief The MonoOdometerParameters struct which contains the parameters needed
  /// to run the odometer for feature detection, feature tracking, RANSAC and so on.
  ///
  struct MonoOdometerParameters {
    int min_number_of_features;
    double ransac_threshold;
    double ransac_confidence;
    int feature_detector_threshold;
    int feature_tracker_window_size;
    int feature_tracker_max_pyramid_level;
    int feature_tracker_max_iterations;
    double feature_tracker_epsilon;
    double feature_tracker_eigen_threshold;
    std::string image_topic;
    ///
    /// \fn Read
    /// \brief Gathers all the values for members from ROS parameter server. Assings
    /// deafult values in case of failure.
    /// \param node_namespace String to state the namespace of the node which all the
    /// parameters would be gathered relatively.
    ///
    void Read(const std::string node_namespace) {
      min_number_of_features = vk::getParam<int>(node_namespace+"/min_number_of_features",100);
      ransac_threshold = vk::getParam<double>(node_namespace+"/ransac_threshold",1.0);
      ransac_confidence = vk::getParam<double>(node_namespace+"/ransac_confidence",0.99);
      image_topic = vk::getParam<std::string>(node_namespace+"/image_topic","/camera/image_raw");
      feature_detector_threshold = vk::getParam<int>(node_namespace+"/feature_detector_threshold",40);
      feature_tracker_window_size = vk::getParam<int>(node_namespace+"/feature_tracker_window_size",21);
      feature_tracker_max_pyramid_level = vk::getParam<int>(node_namespace+"/feature_tracker_max_pyramid_level",2);
      feature_tracker_max_iterations = vk::getParam<int>(node_namespace+"/feature_tracker_max_iterations",30);
      feature_tracker_epsilon = vk::getParam<double>(node_namespace+"/feature_tracker_epsilon",0.01);
      feature_tracker_eigen_threshold = vk::getParam<double>(node_namespace+"/feature_tracker_eigen_threshold",0.0001);
    }
  }; /* struct MonoOdometerParameters */

  ///
  /// \class MonoOdometer
  /// \brief Class to implement monocular visual odometry
  ///
  class MonoOdometer : public viso2_ros::OdometerBase {
  public:
    MonoOdometer(std::string node_namespace);
    ~MonoOdometer();
    vk::AbstractCamera* camera_ptr_;
    void CalculateOdometry();

  protected:
    std::string node_namespace_;
    MonoOdometerParameters parameters_;
    PointVector DetectFeatures(cv::Mat input_image, cv::Mat output_image);
    void TrackFeatures();
    void EstimateMotion();
    void Visualize();

  private:
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    image_transport::Subscriber image_sub_;
    ros::Publisher image_pub_;
    cv::Mat current_image_;
    cv::Mat previous_image_;
    cv::Mat image_to_draw_features_;
    PointVector previous_features_;
    PointVector current_features_;
    bool first_run_;

  }; /* class MonoOdometer */
} /* namespace rplvo_mono */

#endif
