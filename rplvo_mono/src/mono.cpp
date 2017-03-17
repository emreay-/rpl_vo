#include "rplvo_mono/rpl_mono_odometer.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mono_vo");
  std::string node_namespace = ros::this_node::getName();
  rplvo_mono::MonoOdometer mono(node_namespace);
//  ros::spin();
  while (true) {
    try {
      ros::spinOnce();
      mono.CalculateOdometry();
    } catch (ros::Exception& e) {
        ROS_WARN("Calculate Odometry exception: %s", e.what());
        ros::Rate rate(1);
        rate.sleep();
    }
  }
  return 0;
}
