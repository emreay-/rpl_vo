#ifndef rplvo_core_h
#define rplvo_core_h

#include <vector>
#include <opencv2/features2d/features2d.hpp>

namespace rplvo {

  typedef std::vector< cv::KeyPoint > KeyPointVector;
  typedef std::vector< cv::Point2f > PointVector;
  typedef std::vector < PointVector > PointContainer;
  typedef std::vector < cv::Scalar > ScalarVector;

}// namespace rplvo

#endif
