#include "rplvo_mono/rpl_mono_odometer.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mono_vo");
  std::string node_namespace = ros::this_node::getName();
  rplvo::mono::MonoOdometer mono(node_namespace);

  while (true) {
    try {
      ros::spinOnce();
      ros::WallTime t = ros::WallTime::now();
      mono.CalculateOdometry();
      ROS_INFO("Duration: %f", (ros::WallTime::now()-t).toSec());
    } catch (ros::Exception& e) {
      ROS_WARN("Calculate Odometry exception: %s", e.what());
      ros::Duration(1).sleep();
    }
  }
  return 0;
}
