#ifndef rplvo_visualizer_h
#define rplvo_visualizer_h

#include "rplvo_core/rplvo_core.h"
#include <ros/ros.h>
#include <vikit/params_helper.h>
#include <string>

namespace rplvo {
  namespace visualizer {

    struct VisualizerParameters {
      int visualize_frame_tracking;
      int visualize_color_variety;

      ///
      /// \fn Read
      /// \brief Gathers all the values for members from ROS parameter server. Assings deafult values in case of failure.
      /// \param node_namespace String to state the namespace of the node which all the parameters would be gathered relatively.
      ///
      void Read(const std::string node_namespace) {
        visualize_frame_tracking = vk::getParam<int>(node_namespace+"/visualize_frame_tracking",10);
        visualize_color_variety = vk::getParam<int>(node_namespace+"/visualize_color_variety",50);
      }
    };

    class Visualizer {
    public:
      Visualizer(std::string node_namespace);
      void ShowTracking(cv::Mat current_image, PointVector current_features);
    protected:
      std::string node_namespace_;
      VisualizerParameters parameters_;
      void DrawCircles(cv::Mat image);
      void DrawLines(cv::Mat image);
      rplvo::ScalarVector color_vector_;
      ros::Publisher visual_pub_;
    private:
      int draw_accumulate_;
      rplvo::PointContainer feature_container_;
      cv::Mat image_to_draw_;
    };
  }// namespace visualizer
}// namespace rplvo

#endif
