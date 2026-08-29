#include "include/post_process/gesture.h"

std::map<std::string, int> gesture_to_int = { { "", 0 },
                                              { "Finger Heart", 1 },
                                              { "Thumb Up", 2 },
                                              { "Victory", 3 },
                                              { "Mute", 4 },
                                              { "Palm", 5 },
                                              { "IndexFingerAntiClockwise", 6 },
                                              { "IndexFingerClockwise", 7 },
                                              { "Pinch", 8 },
                                              { "Palmpat", 9 },
                                              { "Palm Move", 10 },
                                              { "Okay", 11 },
                                              { "Thumb Left", 12 },
                                              { "Thumb Right", 13 },
                                              { "Awesome", 14 },
                                              { "PinchMove", 15 },
                                              { "PinchAntiClockwise", 16 },
                                              { "PinchClockwise", 17 } };

float dotProduct(const Point& a, const Point& b)
{
  return a.x * b.x + a.y * b.y;
}

float norm(const Point& a)
{
  return std::sqrt(a.x * a.x + a.y * a.y);
}

float clip(float value, float min_val, float max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

float distance(const Point& a, const Point& b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool is_finger_extended(const std::map<int, Point>& landmarks, int tip, int pip, int mcp, bool is_thumb)
{
  Point a = landmarks.at(mcp);
  Point b = landmarks.at(pip);
  Point c = landmarks.at(tip);

  Point ba = a - b;
  Point bc = c - b;

  if (is_thumb)
  {
    Point d = landmarks.at(1);
    Point da = a - d;
    float cos_angle = dotProduct(da, bc) / (norm(da) * norm(bc));
    cos_angle = clip(cos_angle, -1.0f, 1.0f);
    float angle = std::acos(cos_angle) * 180.0f / M_PI;
    return angle < 45.0f;
  }
  else
  {
    float cos_angle = dotProduct(ba, bc) / (norm(ba) * norm(bc));
    cos_angle = clip(cos_angle, -1.0f, 1.0f);
    float angle = std::acos(cos_angle) * 180.0f / M_PI;
    if (angle > 160.0f && distance(a, c) > distance(a, b))
      return true;
    return false;
  }
}

std::vector<bool> _check_fingers_extended(const std::map<int, Point>& landmarks)
{
  std::vector<bool> fingers_extended;
  std::vector<std::tuple<int, int, int>> finger_joints = {
    { 4, 3, 2 },     // Thumb
    { 8, 6, 5 },     // Index
    { 12, 10, 9 },   // Middle
    { 16, 14, 13 },  // Ring
    { 20, 18, 17 }   // Pinky
  };

  bool is_thumb = true;
  for (const auto& joint : finger_joints)
  {
    int tip = std::get<0>(joint);
    int pip = std::get<1>(joint);
    int mcp = std::get<2>(joint);
    bool extended = is_finger_extended(landmarks, tip, pip, mcp, is_thumb);
    fingers_extended.push_back(extended);
    is_thumb = false;
  }

  return fingers_extended;
}

int recognise_gesture(const Landmarks& landmark_list)
{
  std::string gesture_type;

  if (landmark_list.empty())
  {
    return 0;
  }

  std::map<int, Point> landmarks;
  for (size_t i = 0; i < landmark_list.size(); ++i)
  {
    landmarks[i] = landmark_list[i];
  }

  std::vector<bool> fingers_extended = _check_fingers_extended(landmarks);
  {
    std::stringstream ss;
    for (const auto& extended : fingers_extended)
    {
      if (extended)
        ss << "extended ";
      else
        ss << "closed ";
    }
    RCLCPP_INFO(rclcpp::get_logger("mono2d_hand_lmk"), "fingers extended: %s", ss.str().c_str());
  }

  // Check simple gestures
  if (fingers_extended == std::vector<bool>{ false, true, true, false, false })
  {
    gesture_type = "Victory";
  }
  else if (fingers_extended == std::vector<bool>{ false, true, false, false, false })
  {
    gesture_type = "Mute";
  }
  else if (fingers_extended == std::vector<bool>{ true, true, true, true, true })
  {
    gesture_type = "Palm";
  }
  else if (fingers_extended == std::vector<bool>{ false, false, true, true, true } ||
           fingers_extended == std::vector<bool>{ true, false, true, true, true })
  {
    if (distance(landmarks.at(4), landmarks.at(8)) < 30) // Index and Thumb is close
      gesture_type = "Okay";
  }
  else if (fingers_extended == std::vector<bool>{ true, false, false, false, true })
  {
    gesture_type = "Awesome";
  }
  else if (fingers_extended == std::vector<bool>{ true, false, false, false, false })
  {
    gesture_type = "Thumb ";
    Point a = landmarks.at(4);
    Point b = landmarks.at(1);
    Point ab = a - b;
    float angle_rad = std::atan2(ab.y, ab.x);
    float theta = -angle_rad * 180.0f / M_PI;

    if (0 < theta && theta < 45)
    {
      gesture_type += "Right";
    }
    else if (45 < theta && theta < 135)
    {
      gesture_type += "Up";
    }
    else if (135 < theta && theta < 215)
    {
      gesture_type += "Left";
    }
  }

  return gesture_to_int[gesture_type]; // change string to int
}