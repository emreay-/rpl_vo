#include "rplvo_core/rplvo_visualizer.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <time.h>

namespace rplvo {
  namespace visualizer {
    Visualizer::Visualizer(std::string node_namespace) :
      node_namespace_(node_namespace),
      draw_accumulate_(0) {
      // Gather visualizer parameters from the param server
      parameters_.Read(node_namespace_);
      image_to_draw_.zeros(1,1,CV_8U);
      // Seed random and create the color vector with size given from the parameter
      srand(std::time(NULL));
      for (int i = 0; i < parameters_.visualize_color_variety; i++) {
        color_vector_.push_back(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
      }
    }// Constructor Visualizer

    void Visualizer::DrawCircles(cv::Mat image) {
      for (size_t i = 0; i < feature_container_.size(); i++) {
        for (size_t j = 0; j < feature_container_[i].size(); j++) {
          cv::circle(image, feature_container_[i][j], 2, color_vector_[j % parameters_.visualize_color_variety], 2, 4, 0);
        }
      }
    }

    void Visualizer::DrawLines(cv::Mat image) {
      if (feature_container_.size() <= 1) {
        throw ros::Exception("Invalid container size for DrawLine Function. Should be > 1");
      }
      for (size_t i = 1; i < feature_container_.size(); i++) {
        for (size_t j = 0; j < feature_container_[i].size(); j++) {
          cv::line(image, feature_container_[i][j], feature_container_[i-1][j], color_vector_[j % parameters_.visualize_color_variety], 2, 4, 0);
        }
      }
    }

    void Visualizer::ShowTracking(cv::Mat current_image, PointVector current_features) {
      // Visualize tracked features
      image_to_draw_ = current_image.clone();
      if (draw_accumulate_ == 0 || draw_accumulate_ >= parameters_.visualize_frame_tracking) {
        draw_accumulate_ = 1;
        feature_container_.clear();
        feature_container_.push_back(current_features);
        DrawCircles(image_to_draw_);
      } else if (draw_accumulate_ < parameters_.visualize_frame_tracking) {
        draw_accumulate_++;
        feature_container_.push_back(current_features);
        DrawCircles(image_to_draw_);
        DrawLines(image_to_draw_);
      }
      // Publishing image for visualization
      ros::NodeHandle nh;
      visual_pub_ = nh.advertise<sensor_msgs::Image>(node_namespace_+"/visualize",1,this);
      cv_bridge::CvImage img;
      img.image = image_to_draw_;
      img.encoding = "bgr8";
      visual_pub_.publish(img.toImageMsg());
    }


  }//namespace visualizer
}//namespace rplvo
