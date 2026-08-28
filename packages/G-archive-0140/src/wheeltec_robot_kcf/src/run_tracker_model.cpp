#include "run_tracker_model.h"
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdexcept>

/* -------------------- 构造函数 -------------------- */
ImageConverter::ImageConverter()
: Node("kcf_tracker_model"),
  tracker(false, true, true, false)
{
    /* 参数声明与读取 */
    this->declare_parameter<float>("targetDist_", 0.6);
    this->get_parameter("targetDist_", targetDist_);

    linear_PID_  = new PID(1.5, 0.0, 1.0);
    angular_PID_ = new PID(0.5, 0.0, 2.0);

    /* 初始 bbox 合法性检查 */
    this->declare_parameter<int>("x1", 0);
    this->declare_parameter<int>("y1", 0);
    this->declare_parameter<int>("x2", 640);
    this->declare_parameter<int>("y2", 480);

    int x1 = this->get_parameter("x1").as_int();
    int y1 = this->get_parameter("y1").as_int();
    int x2 = this->get_parameter("x2").as_int();
    int y2 = this->get_parameter("y2").as_int();

    selectRect_ = cv::Rect(x1, y1, x2 - x1, y2 - y1);
    if (selectRect_.width <= 0 || selectRect_.height <= 0)
        throw std::runtime_error("Invalid bbox size!");

    /* ROS 接口 */
    rgb_sub_   = this->create_subscription<sensor_msgs::msg::Image>(
                    "/camera/color/image_raw", 10,
                    std::bind(&ImageConverter::imageCb, this, std::placeholders::_1));
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "/camera/depth/image_raw", 10,
                    std::bind(&ImageConverter::depthCb, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/KCF_image", 10);
    vel_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // cv::namedWindow(RGB_WINDOW);
    // cv::namedWindow(DEPTH_WINDOW);
    RCLCPP_INFO(this->get_logger(),
                "KCF tracker started with bbox: (%d,%d) -> (%d,%d)",
                x1, y1, x2, y2);
}

/* -------------------- 析构 -------------------- */
ImageConverter::~ImageConverter()
{
    cv::destroyWindow(RGB_WINDOW);
    cv::destroyWindow(DEPTH_WINDOW);
    delete linear_PID_;
    delete angular_PID_;
}

/* -------------------- 重置 -------------------- */
void ImageConverter::Reset()
{
    bBeginKCF_     = false;
    enable_depth_  = false;
    linear_speed_  = 0;
    rotation_speed_= 0;
    linear_PID_->reset();
    angular_PID_->reset();
    vel_pub_->publish(geometry_msgs::msg::Twist());
}

/* -------------------- 退出清理 -------------------- */
void ImageConverter::Cancel()
{
    Reset();
    cv::destroyWindow(RGB_WINDOW);
    cv::destroyWindow(DEPTH_WINDOW);
    rclcpp::shutdown();
}

/* -------------------- PID 参数重设 -------------------- */
void ImageConverter::PIDcallback()
{
    targetDist_ = 1.0;
    linear_PID_->Set_PID(3.0, 0.0, 1.0);
    angular_PID_->Set_PID(0.5, 0.0, 2.0);
    linear_PID_->reset();
    angular_PID_->reset();
}

/* -------------------- RGB 回调 -------------------- */
void ImageConverter::imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv_ptr->image.copyTo(rgb_img_);

    /* 首次帧初始化 KCF */
    if (!bBeginKCF_) {
        tracker.init(selectRect_, rgb_img_);
        bBeginKCF_    = true;
        enable_depth_ = false;
        RCLCPP_INFO(this->get_logger(), "KCF initialized.");
    }

    /* 跟踪 */
    result_ = tracker.update(rgb_img_);
    cv::rectangle(rgb_img_, result_, cv::Scalar(0, 255, 255), 2);
    cv::circle(rgb_img_,
               cv::Point(result_.x + result_.width / 2,
                         result_.y + result_.height / 2),
               3, cv::Scalar(0, 0, 255), -1);
    enable_depth_ = true;

    /* 发布调试图像 */
    sensor_msgs::msg::Image out_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(),
                       sensor_msgs::image_encodings::BGR8,
                       rgb_img_).toImageMsg(out_msg);
    image_pub_->publish(out_msg);

    // cv::imshow(RGB_WINDOW, rgb_img_);
    // if ((cv::waitKey(1) & 0xff) == 'q') Cancel();
}

/* -------------------- Depth 回调 -------------------- */
void ImageConverter::depthCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "depth convert error");
        return;
    }

    if (!enable_depth_ || result_.width <= 0) return;

    const cv::Mat& depth = cv_ptr->image;
    int cx = result_.x + result_.width / 2 + 15;
    int cy = result_.y + result_.height / 2 + 15;

    /* 5 点深度平均 */
    dist_val_[0] = depth.at<float>(cy - 5, cx - 5) / 1000.0f;
    dist_val_[1] = depth.at<float>(cy - 5, cx + 5) / 1000.0f;
    dist_val_[2] = depth.at<float>(cy + 5, cx + 5) / 1000.0f;
    dist_val_[3] = depth.at<float>(cy + 5, cx - 5) / 1000.0f;
    dist_val_[4] = depth.at<float>(cy, cx) / 1000.0f;

    float d_sum = 0;
    int   d_cnt = 0;
    for (int i = 0; i < 5; ++i)
        if (dist_val_[i] > 0.05f && dist_val_[i] < 10.0f) {
            d_sum += dist_val_[i];
            d_cnt++;
        }
    if (d_cnt == 0) return;
    float distance = d_sum / d_cnt;

    /* PID 计算 */
    if (std::abs(distance - targetDist_) < 0.1f) linear_speed_ = 0.0f;
    else linear_speed_ = -linear_PID_->compute(targetDist_, distance);

    rotation_speed_ = angular_PID_->compute(3.2f, cx / 100.0f);
    if (std::abs(rotation_speed_) < 0.1f) rotation_speed_ = 0.0f;

    /* 限速 */
    linear_speed_   = std::clamp(linear_speed_,   -0.35f, 0.35f);
    rotation_speed_ = std::clamp(rotation_speed_, -0.35f, 0.35f);

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear_speed_;
    twist.angular.z = rotation_speed_;
    vel_pub_->publish(twist);
    
    // RCLCPP_INFO(this->get_logger(), "Target distance: %.2f", targetDist_);
    // RCLCPP_INFO(this->get_logger(), "Measured distance: %.2f", distance);
    // RCLCPP_INFO(this->get_logger(), "Linear speed: %.2f", linear_speed_);
    // /* ---------- 显示深度图 ---------- */
    // cv::Mat depth_show;
    // cv::normalize(depth, depth_show, 0, 255, cv::NORM_MINMAX, CV_8UC1);   // 0-255 灰度
    // cv::applyColorMap(depth_show, depth_show, cv::COLORMAP_JET);          // 伪彩色

    // /* 在深度图中心画框 + 写距离 */
    // cv::rectangle(depth_show,
    //             cv::Point(result_.x, result_.y),
    //             cv::Point(result_.x + result_.width,
    //                         result_.y + result_.height),
    //             cv::Scalar(0, 0, 255), 2);
    // cv::putText(depth_show,
    //             cv::format("dist=%.2fm", distance),
    //             cv::Point(result_.x, result_.y - 5),
    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);

    // cv::imshow("KCF_Depth", depth_show);
    // if ((cv::waitKey(1) & 0xff) == 'q') Cancel();

}

/* -------------------- main -------------------- */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}
