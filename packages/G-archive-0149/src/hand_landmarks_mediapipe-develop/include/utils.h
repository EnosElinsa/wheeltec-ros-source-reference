#include "include/post_process/hand_lmk_parser.h"

builtin_interfaces::msg::Time ConvertToRosTime(const struct timespec& time_spec);

float Distance(const Point& a, const Point& b);

double CalculateIOU(const cv::Rect& rect1, const cv::Rect& rect2);

cv::Rect MoveBox(const cv::Rect& ori_rect, float scale, float offset_y, Point direc);

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start, const builtin_interfaces::msg::Time& end);