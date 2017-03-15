#ifndef mono_odometer_h
#define mono_odometer_h

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

  typedef std::vector< cv::KeyPoint > FeatureVector;

  struct MonoOdometerParameters {
    size_t min_number_of_features;
    size_t ransac_threshold;
    std::string image_topic;
    void Read(const std::string node_namespace) {
      min_number_of_features = vk::getParam<size_t>(node_namepsace+"min_number_of_features",100);
      ransac_threshold = vk::getParam<size_t>(node_namespace+"ransac_threshold",100);
      image_topic = vk::getParam<size_t>(node_namespace+"image_topic",100);
    }
  }; /* struct MonoOdometerParameters */

  class MonoOdometer : public viso2_ros::OdometerBase {
  public:
    MonoOdometer(const std::string node_namespace);
    ~MonoOdometer();
    void CalculateOdometry(const sensor_msgs::ImageConstPtr& input_image);
    vk::AbstractCamera* camera_ptr_;

  protected:
    const std::string node_namespace_;
    MonoOdometerParameters parameters_;
    FeatureVector DetectFeatures(cv::Mat input_image, cv::Mat output_image, int threshold);
    void TrackFeatures();
    void EstimateMotion();
    void Visualize();

  private:
    image_transport::Subscriber image_sub_;
    cv::Mat current_image_;
    cv::Mat previous_image_;
    FeatureVector previous_features_;

  }; /* class MonoOdometer */
} /* namespace rplvo_mono */

#endif
