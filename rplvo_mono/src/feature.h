#ifndef feature_h
#define feature_h

#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<vector>

namespace rplvo_mono {
  std::vector< cv::KeyPoint > DetectFeatures(cv::Mat input_image, cv::Mat output_image, int threshold) {
    std::vector< cv::KeyPoint > features;
    cv::FAST(input_image, features, threshold, true);
    ROS_DEBUG("Number of features: %d", features.size());
    cv::drawKeypoints(input_image, features, output_image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    return features;
  }

}

#endif
