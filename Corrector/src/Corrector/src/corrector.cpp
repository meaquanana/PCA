#include "corrector.h"
#include <numeric>

namespace lzz{
    void Light::init(const std::vector<cv::Point> &contour){
  contour_ = contour;
  Rect_ = cv::RotatedRect(cv::minAreaRect(contour));
  center = std::accumulate(
      contour_.begin(),
      contour_.end(),
      cv::Point2f(0, 0),
      [n = static_cast<float>(contour_.size())](const cv::Point2f &a, const cv::Point &b) {
        return a + cv::Point2f(b.x, b.y) / n;
      });
  // std::cout<<center<<std::endl;
  Rect_.points(p);
  std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
  top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    
    
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
    Rect_.center = center;
}
void Pca_Correct::correctCorners(Light &left_light,Light &right_light, const cv::Mat &gray_img,int target_color) {
  
  int min_width = 3;

  if (left_light.width > min_width) {
    Axis left_axis = findAxis(gray_img, left_light);
    left_light.top = findCorner(gray_img, left_light, left_axis, "top",target_color);
    left_light.bottom = findCorner(gray_img, left_light, left_axis, "bottom",target_color);
    // std::cout<<left_light.top <<" "<<left_light.bottom<<std::endl;
    left_light.center = left_axis.centroid;
  }

  if (right_light.width > min_width) {
    Axis right_axis = findAxis(gray_img, right_light);
    right_light.top = findCorner(gray_img, right_light, right_axis, "top",target_color);
    right_light.bottom = findCorner(gray_img, right_light, right_axis, "bottom",target_color);
    right_light.center = right_axis.centroid;
  }
}
Axis Pca_Correct::findAxis(const cv::Mat &gray_img, const Light &light) {
  constexpr float MAX_BRIGHTNESS = 25;
  constexpr float SCALE = 0.08;

  
  cv::Rect light_box = light.Rect_.boundingRect();
  light_box.x -= light_box.width * SCALE;
  light_box.y -= light_box.height * SCALE;
  light_box.width += light_box.width * SCALE * 2;
  light_box.height += light_box.height * SCALE * 2;

  
  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  cv::Mat roi = gray_img(light_box);
  float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // 质心
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
                         cv::Point2f(light_box.x, light_box.y);

  // ToDo:目前points是boundingRect的集合，其实并不是lightbar，所以考虑把这部分替换lightbar的points集合
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // 中心
  cv::Point2f axis =
    cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

  axis = axis / cv::norm(axis);

  return Axis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}
cv::Point2f Pca_Correct::findCorner(const cv::Mat &gray_img,
                                             const Light &light,
                                             const Axis &axis,
                                             std::string order,
                                             int target_color) {
  float Scale;
  if(target_color == 1){
    Scale = 1.65; //red range(1.5-1.7)
  }else
    Scale = 2.3; //blue range(2,2.5)
  

  float START = 0.8 / Scale;
  float END = 1.2 / Scale;

  auto inImage = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
  };

  auto distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };

  int oper = order == "top" ? 1 : -1;
  float L = light.length;
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;
  
  
  int n = light.width - 2;
  int half_n = std::round(n / 2);
  for (int i = -half_n; i <= half_n; i++) {
    float x0 = axis.centroid.x + L * START * dx + i;
    float y0 = axis.centroid.y + L * START * dy;

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!inImage(cv::Point(cur))) {
        break;
      }

      float brightness_diff = gray_img.at<uchar>(cv::Point(prev)) - gray_img.at<uchar>(cur);
      if (brightness_diff > max_brightness_diff && gray_img.at<uchar>(prev) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
      }

      prev = cur;
    }

    candidates.emplace_back(corner);
  }
  cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0)) /
                       static_cast<float>(candidates.size());

  return result;
}
}