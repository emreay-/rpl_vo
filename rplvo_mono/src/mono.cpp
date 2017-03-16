#include "rplvo_mono/rpl_mono_odometer.h"
#include <ros/ros.h>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mono_vo");
  std::string node_namespace = ros::this_node::getName();
  rplvo_mono::MonoOdometer mono(node_namespace);
  ros::spin();
  return 0;
}
