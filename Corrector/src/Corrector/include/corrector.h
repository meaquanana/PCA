#include <opencv2/opencv.hpp>
#include <numeric>
#include<vector>

namespace lzz{
struct LightParams {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };


struct Axis {
  cv::Point2f centroid;
  cv::Point2f direction;
  float mean_val; // Mean brightness
};

struct Light{
  Light() = default;
  void init(const std::vector<cv::Point> &contour);
  std::vector<cv::Point> contour_;
  cv::RotatedRect Rect_;
  cv::Point2f top, bottom, center;
  double length;
  double width;
  float tilt_angle;
  cv::Point2f p[4];
};

class Pca_Correct {
public:
  Pca_Correct() = default; 

  // Correct the corners of the armor's lights
  void correctCorners(Light &left_light,Light &right_light, const cv::Mat &gray_img,int target_color);

private:
  // Find the symmetry axis of the light
  Axis findAxis(const cv::Mat &gray_img, const Light &light);

  // Find the corner of the light
  cv::Point2f findCorner(const cv::Mat &gray_img,
                         const Light &light,
                         const Axis &axis,
                         std::string order,int target_color);
};

}