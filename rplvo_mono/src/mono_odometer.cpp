#include "mono_odometer.h"
#include <ros/ros.h>

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
    image_sub_ = img_transport.subscribe(parameters_.image_topic, 1, &MonoOdometer::CalculateOdometry, this);
  }

  MonoOdometer::~MonoOdometer() {
    delete camera_ptr_;
  }















} /* namespace rplvo_mono */
