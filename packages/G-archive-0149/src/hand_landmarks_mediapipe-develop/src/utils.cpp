#include "include/utils.h"

builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec& time_spec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

// calc distance between two points
float Distance(const Point& a, const Point& b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// calc IOU between two boxes
double CalculateIOU(const cv::Rect& rect1, const cv::Rect& rect2)
{
  double intersection_area = (rect1 & rect2).area();

  double union_area = rect1.area() + rect2.area() - intersection_area;

  if (union_area == 0)
  {
    return 0.0;
  }
  return intersection_area / union_area;
}

// move palm bbox to the direction of hand, for get hand detection
cv::Rect MoveBox(const cv::Rect& ori_rect, float scale, float offset_y, Point direc)
{
  auto angle = std::atan2(direc.y, direc.x);  // calc angle from direc
  auto width = ori_rect.width;
  auto height = ori_rect.height;
  auto cx = ori_rect.x + width / 2.0;
  auto cy = ori_rect.y + height / 2.0;
  // move center
  auto moveDistance = offset_y * height;
  cx += moveDistance * std::cos(angle);
  cy += moveDistance * std::sin(angle);
  // extend bbox
  width = std::max(width, height) * scale;
  height = width;
  auto x1 = static_cast<int>(cx - width / 2.0);
  auto y1 = static_cast<int>(cy - height / 2.0);
  return { x1, y1, width, height };
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start, const builtin_interfaces::msg::Time& end)
{
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 - start.nanosec / 1000 / 1000;
}