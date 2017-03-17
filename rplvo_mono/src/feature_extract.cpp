#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<vector>

void camcb(const sensor_msgs::ImageConstPtr& img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  std::vector< cv::KeyPoint > features;
  cv::Mat input = cv_ptr->image;
  cv::FAST(input, features, 40, true);
  ROS_DEBUG("Number of features: %d", features.size());
  cv::drawKeypoints(input, features, input, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  cv::imshow("Image", input);
  cv::waitKey(3);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "feature_node");
  ros::NodeHandle nh;
  cv::namedWindow("Image");
  image_transport::ImageTransport img_transport(nh);
  image_transport::Subscriber image_sub = img_transport.subscribe("/mono/image_rect",10,camcb);
  ros::spin();
  return 0;
}
