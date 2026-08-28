// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDE_HOBOT_FALLDOWN_DETECTION_HOBOT_FALLDOWN_DETECTION_H_
#define INCLUDE_HOBOT_FALLDOWN_DETECTION_HOBOT_FALLDOWN_DETECTION_H_

#include <string>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "ai_msgs/msg/target.hpp"
#include "ai_msgs/msg/point.hpp"
#include <std_msgs/msg/float32.hpp>
#include "wheeltec_bodymsg/msg/bodyposture.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define TargetTypePersion "person"
#define PointTypeBody_kps "body_kps"
#define BodyKpsSize (19)

#define Jaw 0
#define LEFT_HEAD 1 
#define RIGHT_HEAD 2 
#define LEFT_EAR 3 
#define RIGHT_EAR 4 
#define LEFT_SHOULDER 5 
#define RIGHT_SHOULDER 6 
#define LEFT_ELBOW 7 
#define RIGHT_ELBOW 8 
#define LEFT_WRIST 9 
#define RIGHT_WRIST 10 
#define LEFT_HIP 11 
#define RIGHT_HIP 12
#define LEFT_KNEE 13 
#define RIGHT_KNEE 14 
#define LEFT_FOOT 15 
#define RIGHT_FOOT 16 
#define LEFT_HAND 17 
#define RIGHT_HAND 18 
#define UNKNOWN 255

typedef enum {
    ExLow = 0,
    Low,
    Middle,
    High
}Sensivity;


using rclcpp::NodeOptions;
using ai_msgs::msg::PerceptionTargets;

class hobot_falldown_detection: public rclcpp::Node
{
 public:
    explicit hobot_falldown_detection(
        const NodeOptions &options = NodeOptions(),
        std::string node_name = "body_kps_node");
    ~hobot_falldown_detection();

 private:
    void topic_callback(
        const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

  void IsFallDown(const std::vector<geometry_msgs::msg::Point32> &body_kps,std::vector<float> confidence);
//    void IsFallDown(ai_msgs::msg::Point point_list);
    
    void PointDebugInfo(
        const std::vector<geometry_msgs::msg::Point32> &body_kps);

    void PublishFallDownEvent(
        const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg,
        ai_msgs::msg::PerceptionTargets::UniquePtr publish_data,
        ai_msgs::msg::Perf perf);

    float upper_body_low_ = 30.0f;
    float upper_body_high_ = 66.0f;
    float lower_body_low_ = 45.0f;
    float lower_body_high_ = 80.0f;
    float differ_ = 30.0f;

    const float PI = 3.14159265f;

    int paramSensivity = 3;
    std::string body_kps_topic_name = "hobot_mono2d_body_detection";

    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::ConstSharedPtr
                                        subscription_ = nullptr;

    std::string msg_falldown_topic_name = "falldown_event";
    std::string msg_bodyposture_name = "bodyposture_event";
    rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr
    falldown_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmdvel;

    rclcpp::Publisher<wheeltec_bodymsg::msg::Bodyposture>::SharedPtr bodyposture_publisher_;
};

#endif  // INCLUDE_HOBOT_FALLDOWN_DETECTION_HOBOT_FALLDOWN_DETECTION_H_

