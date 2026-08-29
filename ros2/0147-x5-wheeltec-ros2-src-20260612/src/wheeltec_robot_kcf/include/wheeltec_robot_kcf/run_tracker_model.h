#ifndef RUN_TRACKER_H_
#define RUN_TRACKER_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "kcftracker.h"
#include "PID.h"

class ImageConverter : public rclcpp::Node
{
public:
    /* 构造函数：接收初始 bbox 左上角(x1,y1) 右下角(x2,y2) */
    ImageConverter();

    ~ImageConverter();

    void Reset();      // 重置跟踪
    void Cancel();     // 退出清理
    void PIDcallback();

private:
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);
    void depthCb(const sensor_msgs::msg::Image::SharedPtr msg);

    /* ROS 接口 */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr     image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr  depth_sub_;

    /* 跟踪器与 PID */
    KCFTracker tracker{false, true, true, false};   // 固定参数
    PID* linear_PID_;
    PID* angular_PID_;

    /* 状态量 */
    cv::Rect selectRect_;           // 目标框
    cv::Rect result_;               // 跟踪结果
    cv::Mat  rgb_img_;              // 当前帧图像
    bool     bBeginKCF_      = false;
    bool     enable_depth_   = false;
    float    targetDist_     = 1.0f;
    float    linear_speed_   = 0.0f;
    float    rotation_speed_ = 0.0f;
    float    dist_val_[5]    = {0};

    /* 常量 */
    static constexpr int IMG_W = 640;
    static constexpr int IMG_H = 480;
    const char* RGB_WINDOW = "KCF_RGB";
    const char* DEPTH_WINDOW = "KCF_RGB";
};

#endif  // RUN_TRACKER_H_