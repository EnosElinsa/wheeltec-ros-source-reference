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

#include "include/wheeltec_bodyreader/hobot_falldown_detection.h"

#include "wheeltec_bodymsg/msg/bodyposture.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <vector>
#include <memory>
#include <cmath>

wheeltec_bodymsg::msg::Bodyposture bodyposture_msg;
int count_pub=0;
int count_cancel=0;
hobot_falldown_detection::hobot_falldown_detection(
    const NodeOptions &options, std::string node_name)
    : Node(node_name, options)
{
    this->declare_parameter<int>("paramSensivity", paramSensivity);
    this->declare_parameter<std::string>("body_kps_topic_name",
                                            body_kps_topic_name);
    this->declare_parameter<std::string>("pub_smart_topic_name",
                                            body_kps_topic_name);
    this->declare_parameter<std::string>("pub_body_topic_name",
                                            msg_bodyposture_name);


    this->get_parameter<int>("paramSensivity", paramSensivity);
    this->get_parameter<std::string>("body_kps_topic_name",
                                        body_kps_topic_name);
    this->get_parameter<std::string>("pub_smart_topic_name",
                                            msg_falldown_topic_name);
    this->get_parameter<std::string>("pub_body_topic_name",
                                            msg_bodyposture_name);
                                            
    subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        body_kps_topic_name, 10,
        std::bind(&hobot_falldown_detection::topic_callback,
        this, std::placeholders::_1));
    falldown_publisher_ =this->create_publisher<ai_msgs::msg::PerceptionTargets>(msg_falldown_topic_name, 10);

    pub_cmdvel =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    bodyposture_publisher_ =this->create_publisher<wheeltec_bodymsg::msg::Bodyposture>("/bodyposture_event", 10);
}

hobot_falldown_detection::~hobot_falldown_detection() {}

void hobot_falldown_detection::topic_callback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg)
{
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    ai_msgs::msg::Perf perf;
    perf.set__type("PostProcess");
    perf.stamp_start.sec = time_start.tv_sec;
    perf.stamp_start.nanosec = time_start.tv_nsec;

    auto targetList = msg->targets;
    ai_msgs::msg::PerceptionTargets::UniquePtr
            publish_data(new ai_msgs::msg::PerceptionTargets());

    for (auto &target : targetList)
    {
        auto targetType = target.type;
        auto pointList = target.points;
        auto track_id = target.track_id;
        bool isfalldown = false;
        bool has_body_kps = false;
        bodyposture_msg.bodyid = target.track_id;
        bodyposture_msg.left_foot_up =0;
        bodyposture_msg.right_foot_up=0;
        count_pub=0;
        count_cancel=0;
        for (auto &pointNode : pointList)
        {
            auto pointType = pointNode.type;
            if (PointTypeBody_kps != pointType)
            {
                isfalldown = false;
            } else {
                has_body_kps = true;
                auto point32List = pointNode.point;
                auto confidenceList = pointNode.confidence;
                IsFallDown(point32List,confidenceList);
            }
        }
    }

}

void hobot_falldown_detection::PublishFallDownEvent(
        const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg,
        ai_msgs::msg::PerceptionTargets::UniquePtr publish_data,
        ai_msgs::msg::Perf perf)
{
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    perf.stamp_end.sec = time_start.tv_sec;
    perf.stamp_end.nanosec = time_start.tv_nsec;

    publish_data->header.set__stamp(msg->header.stamp);
    publish_data->header.set__frame_id(msg->header.frame_id);
    publish_data->set__fps(msg->fps);
    publish_data->perfs.emplace_back(perf);
    falldown_publisher_->publish(std::move(publish_data));
}

void hobot_falldown_detection::IsFallDown(
    const std::vector<geometry_msgs::msg::Point32> &body_kps,
    std::vector<float> confidence )

{

    bool detection_confidence=false;
    bool point_confidence=false;
    geometry_msgs::msg::Twist twist;
    float sum=0;

    if (body_kps.size() != BodyKpsSize)
    {
        RCLCPP_WARN(rclcpp::get_logger("body_kps_Subscriber"),
                    "body_kps size: ", body_kps.size());
        detection_confidence=false;
    }
    else{

        for(int i=LEFT_HIP;i<(RIGHT_FOOT+1);i++)
        {
            sum+=confidence[i];
            if(confidence[i]>0.75){
                point_confidence=true;
                //continue;
            } 
            else 
            {
            point_confidence=false;
            continue;   
            };//break
        }
        if((sum/(LEFT_HAND-LEFT_HIP))>0.8 && point_confidence) {
                detection_confidence=true;
                sum=0;
        }
        else {
            detection_confidence=false;
            sum=0;
        }
    
    //RCLCPP_INFO(this->get_logger(), "LR FOOT arctanc : %d",detection_confidence);

    if(detection_confidence){  //确保下半身在监测范围内

    float tana=0;
    float tanb=0;
    float tanc=0;

    auto diff_1112_x = body_kps[LEFT_HIP].x - body_kps[RIGHT_HIP].x;
    auto diff_1112_y = body_kps[LEFT_HIP].y - body_kps[RIGHT_HIP].y;
    if(diff_1112_x!=0){
        tana=diff_1112_y/diff_1112_x;
        //auto arctana = std::atan(std::abs(tana)) * 180.0f / PI; 

    }

    auto diff_1314_x = body_kps[LEFT_KNEE].x - body_kps[RIGHT_KNEE].x;
    auto diff_1314_y = body_kps[LEFT_KNEE].y - body_kps[RIGHT_KNEE].y;
    if(diff_1314_x!=0){
        tanb=diff_1314_y/diff_1314_x;
        //auto arctanb = std::atan(std::abs(tanb)) * 180.0f / PI;  
           
    }

    auto diff_1516_x = body_kps[LEFT_FOOT].x - body_kps[RIGHT_FOOT].x;
    auto diff_1516_y = body_kps[LEFT_FOOT].y - body_kps[RIGHT_FOOT].y;
    if(diff_1516_x!=0){
        tanc=diff_1516_y/diff_1516_x;
        //auto arctanc = std::atan(std::abs(tanc)) * 180.0f / PI;  
   
    }

            RCLCPP_INFO(this->get_logger(), "LR HIP tana : %f",tana);
            RCLCPP_INFO(this->get_logger(), "LR KNEE tanb : %f",tanb);
            RCLCPP_INFO(this->get_logger(), "LR FOOT tanc : %f",tanc);
 

    if((tana<0.5)&&(tanb>0.7)&&(tanc>1))
      {
            bodyposture_msg.right_foot_up = 1;

            twist.linear.x=0.4;
            printf("Right foot up !\n");

    }

    else if((tana<0.2)&&(tanb<-0.7)&&(tanc<-1))
      {
            bodyposture_msg.left_foot_up = 1; 
            twist.linear.x=-0.2;
            printf("Left foot up !\n");
    }
    else{

        twist.linear.x=0;
    }
    pub_cmdvel->publish(twist);
    bodyposture_publisher_->publish(bodyposture_msg);
  }

    }
}
