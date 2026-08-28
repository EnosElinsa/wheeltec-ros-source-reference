#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mutex>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>

#include "stepper_motor_arm.cpp" 

using namespace std;
using namespace cv;
using std::placeholders::_1;

// ================= 常量定义 =================
#define RECOVERY_STEP_X 0.002 
#define ARC_CENTER_X  0.0f
#define ARC_CENTER_Y -0.16f
#define ARC_RADIUS    0.14f
#define FAR_THRESHOLD_PIXEL 15.0 
#define ZONE_THRESHOLD 15.0 
#define TRANSITION_HEIGHT -0.148f 
#define TRAJECTORY_STEP_X 0.0015f 
#define ALIGN_THRESHOLD 20.0 

static Arm_t arm;
static float gripper_val = 0.0f;   // 0~100

// 定义状态机状态
enum class RobotState {
    IDLE,           // 空闲/未开始
    SEARCHING,      // 寻找色块（旋转/前进）
    ALIGNING_BASE,  // 调整车体位置（对准色块）
    APPROACHING,
    GRASPING,       // 抓取动作（夹紧）
    RETRACT_TO_LOOK, // 1. 抓完先缩回到 Look 位置
    MOVE_TO_PUT,     // 2. 移动到放置点
    RELEASING,       // 3. 松开夹爪
    BACK_TO_LOOK,    // 4. 放完后回到 Look 位置
    WAIT_USER_RESET  // 5. 等待用户松开 pick_start
};

class AutoPickNode : public rclcpp::Node {
public:
    AutoPickNode() : Node("arm_2_auto_pick_colorBlock") {
        declareParameters();

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_ori", 10);
        arm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_cmd", 1);
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 20, std::bind(&AutoPickNode::odomCallback, this, _1));

       
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 1, std::bind(&AutoPickNode::imageCallback, this, _1));

        // 4. 参数回调句柄 (替代 dynamic_reconfigure callback)
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&AutoPickNode::parametersCallback, this, _1));

        // 5. 初始化机械臂
        changeArmState();

        // 预生成形态学核心
        morph_element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_erode_size_, dilate_erode_size_));
        
        currentState_ = RobotState::IDLE;
        RCLCPP_INFO(this->get_logger(), "AutoPickNode Initialized (ROS 2 Humble).");
    }

    void controlLoop() {
        std::lock_guard<std::mutex> lock(data_mutex_); 

        // 1. 处理状态机逻辑
        processStateMachine();

        // 2. 发布关节消息
        publishArmState();

        // 3. 发布速度消息
        publishCmdVel();
    }

private:
    // ROS 2 通信对象
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_pub_;
    
    // 参数回调句柄
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::mutex data_mutex_;

    // 图像处理相关
    cv::Mat morph_element_;
    double area_ = 0;
    double x_bias_ = 0, y_bias_ = 0;
    double distance_bias_ = 0;
    bool object_detected_ = false;
    
    // 控制参数
    RobotState currentState_;
    rclcpp::Time state_start_time_; 
    
    // 小车与机械臂控制量
    double car_foward_vel_ = 0, car_turn_vel_ = 0;
    double pid_yaw_kp_ = 0.0003;   // Yaw轴 旋转系数

    // 参数变量 (从 Parameter Server 获取)
    bool pick_start = false;
    int color_mode = 0;
    int hsv_params[6] = {0};
    int green_hsv[6]  = {60, 35,  0,   80,  180, 255};
    int blue_hsv[6]   = {80, 90,  100, 130, 200, 255};
    int yellow_hsv[6] = {15, 110, 30,  50,  255, 255};
    int red_hsv[6]    = {0,  100, 53,  18,  223, 255};
    
    int dilate_erode_size_ = 7;
    double target_area_size_ = 22000;
    double target_y_bias_ = 0;
    double pid_bias_kp_ = 0.00015; 
    
    double ratio_dominant_ = 1.0; 
    double ratio_secondary_ = 0.3; 
    double min_speed = 0.001;

    // 机械臂参数
    double look_jointA = -1.80796;
    double look_position_x = 0.062;
    double look_position_y = -0.008;

    double put_jointA = -1.570796;
    double put_position_x = 0.062;
    double put_position_y = 0.062;

    // PID 系数
    double car_foward_kp_ = 0.0;
    double car_search_foward_ = 0.1;
    rclcpp::Time last_pid_time_;       
    const double PID_INTERVAL = 0.05; 

    // 里程计
    double odom_foward = 0, car_search_dist_max = 3.0;
    bool car_stop = false;
    double last_odom_foward_ = 0;



    // --- 初始化参数声明 ---
    void declareParameters() {
        this->declare_parameter("Pick_start", false); // bool 不需要滑块，rqt会自动显示复选框
        declare_slider("car_search_distance_max", 3.0, -20.0, 20.0);

        declare_slider("look_jointA",     -1.80796, -2.0, 2.0);
        declare_slider("look_position_x",  0.062,    0.0, 0.27);
        declare_slider("look_position_y", -0.008,   -0.18, 0.24);

        declare_slider("put_jointA",       0.0,     -2.0, 2.0);
        declare_slider("put_position_x",   0.2,      0.0, 0.27);
        declare_slider("put_position_y",  -0.145,   -0.18, 0.24);

        declare_slider("gripper", 0.0, 0.0, 100.0);

        declare_slider("color", 0, 0, 4, "0:Dynamic, 1:Green, 2:Blue, 3:Yellow, 4:Red");

        declare_slider("HSV_H_MIN", 0, 0, 255);
        declare_slider("HSV_S_MIN", 0, 0, 255);
        declare_slider("HSV_V_MIN", 0, 0, 255);
        declare_slider("HSV_H_MAX", 0, 0, 255);
        declare_slider("HSV_S_MAX", 0, 0, 255);
        declare_slider("HSV_V_MAX", 0, 0, 255);

        declare_slider("dilate_erode_size", 7, 0, 25);
        
        declare_slider("car_search_foward", 0.25, -2.0, 2.0);
        declare_slider("target_areaSize", 6000.0, 0.0, 50000.0);
        declare_slider("target_y_bias", 60.0, -120.0, 120.0);
        declare_slider("Car_foward_KP", 0.1, -1.0, 1.0);
        
        // Sync initial values
        updateLocalParams();
    }

    // 辅助函数：声明带范围限制的浮点型参数（用于生成滑块）
    void declare_slider(const std::string& name, double default_val, double min_val, double max_val, const std::string& desc = "") {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = desc;
        
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = min_val;
        range.to_value = max_val;
        range.step = 0.0; // 0 表示连续
        param_desc.floating_point_range.push_back(range);

        this->declare_parameter(name, default_val, param_desc);
    }

    // 辅助函数：声明带范围限制的整型参数（用于生成滑块）
    void declare_slider(const std::string& name, int default_val, int min_val, int max_val, const std::string& desc = "") {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.description = desc;

        rcl_interfaces::msg::IntegerRange range;
        range.from_value = min_val;
        range.to_value = max_val;
        range.step = 1;
        param_desc.integer_range.push_back(range);

        this->declare_parameter(name, default_val, param_desc);
    }




    // 从 Parameter Server 读取所有值更新到本地变量
    void updateLocalParams() {
        pick_start = this->get_parameter("Pick_start").as_bool();
        color_mode = this->get_parameter("color").as_int();
        
        hsv_params[0] = this->get_parameter("HSV_H_MIN").as_int();
        hsv_params[1] = this->get_parameter("HSV_S_MIN").as_int();
        hsv_params[2] = this->get_parameter("HSV_V_MIN").as_int();
        hsv_params[3] = this->get_parameter("HSV_H_MAX").as_int();
        hsv_params[4] = this->get_parameter("HSV_S_MAX").as_int();
        hsv_params[5] = this->get_parameter("HSV_V_MAX").as_int();

        dilate_erode_size_ = this->get_parameter("dilate_erode_size").as_int();

        gripper_val = this->get_parameter("gripper").as_double();
        look_jointA = this->get_parameter("look_jointA").as_double();
        look_position_x = this->get_parameter("look_position_x").as_double();
        look_position_y = this->get_parameter("look_position_y").as_double();
        
        put_jointA = this->get_parameter("put_jointA").as_double();
        put_position_x = this->get_parameter("put_position_x").as_double();
        put_position_y = this->get_parameter("put_position_y").as_double();
        
        car_search_foward_ = this->get_parameter("car_search_foward").as_double();
        target_area_size_ = this->get_parameter("target_areaSize").as_double();
        target_y_bias_ = this->get_parameter("target_y_bias").as_double();
        car_foward_kp_ = this->get_parameter("Car_foward_KP").as_double();
        car_search_dist_max = this->get_parameter("car_search_distance_max").as_double();
        
        // 更新形态学算子
        if (dilate_erode_size_ > 0)
            morph_element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_erode_size_, dilate_erode_size_));
    }

    // 参数回调函数
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters) {
        
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        std::lock_guard<std::mutex> lock(data_mutex_);

        for (const auto &param : parameters) {
            if (param.get_name() == "Pick_start") pick_start = param.as_bool();
            else if (param.get_name() == "color") color_mode = param.as_int();
   
            
            if (param.get_name() == "gripper") gripper_val = param.as_double();
            if (param.get_name() == "look_jointA") look_jointA = param.as_double();
            if (param.get_name() == "look_position_x") look_position_x = param.as_double();
            if (param.get_name() == "look_position_y") look_position_y = param.as_double();
            
            if (param.get_name() == "put_jointA") put_jointA = param.as_double();
            if (param.get_name() == "put_position_x") put_position_x = param.as_double();
            if (param.get_name() == "put_position_y") put_position_y = param.as_double();
            
            if (param.get_name() == "HSV_H_MIN") hsv_params[0] = param.as_int();
            if (param.get_name() == "HSV_S_MIN") hsv_params[1] = param.as_int();
            if (param.get_name() == "HSV_V_MIN") hsv_params[2] = param.as_int();
            if (param.get_name() == "HSV_H_MAX") hsv_params[3] = param.as_int();
            if (param.get_name() == "HSV_S_MAX") hsv_params[4] = param.as_int();
            if (param.get_name() == "HSV_V_MAX") hsv_params[5] = param.as_int();
            
            if (param.get_name() == "dilate_erode_size") {
                dilate_erode_size_ = param.as_int();
                morph_element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilate_erode_size_, dilate_erode_size_));
            }
            
            if (param.get_name() == "car_search_foward") car_search_foward_ = param.as_double();
            if (param.get_name() == "target_areaSize") target_area_size_ = param.as_double();
            if (param.get_name() == "target_y_bias") target_y_bias_ = param.as_double();
            if (param.get_name() == "Car_foward_KP") car_foward_kp_ = param.as_double();
            if (param.get_name() == "car_search_distance_max") car_search_dist_max = param.as_double();
        }

        return result;
    }

    void changeArmState() {
        arm.joint_rad[0] = look_jointA;
        Arm2DOF_SetTargetEndEffector(&arm, look_position_x, look_position_y);
        car_foward_vel_ = 0; 
        car_turn_vel_ = 0;
    }

    void resetJobData() {
        car_stop = false;
        car_foward_vel_ = 0;
        odom_foward = 0;
        last_odom_foward_ = 0;
        object_detected_ = false; // 确保视觉状态也被重置
        area_ = 0;
        RCLCPP_INFO(this->get_logger(), "Job Data Reset Complete. Ready for new task.");
    }

    void processStateMachine() {
        // 全局复位逻辑
        if (!pick_start && currentState_ != RobotState::IDLE && currentState_ != RobotState::WAIT_USER_RESET) {
            currentState_ = RobotState::IDLE;
            changeArmState();
        }

        switch (currentState_) {
            case RobotState::IDLE:
                changeArmState();
                if (pick_start) {
                    resetJobData();
                    currentState_ = RobotState::SEARCHING;
                    RCLCPP_INFO(this->get_logger(), "Mission Start: Searching...");
                }
                break;

            case RobotState::SEARCHING:
                if (object_detected_) {
                    car_foward_vel_ = 0;
                    currentState_ = RobotState::ALIGNING_BASE;
                    state_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Object found, aligning base.");
                } else {
                    if (!car_stop) {
                        car_foward_vel_ = car_search_foward_;
                    } else {
                        car_foward_vel_ = 0;
                    }
                }
                break;

            case RobotState::ALIGNING_BASE:
                if (abs(x_bias_) > 70) {
                    car_foward_vel_ = (x_bias_ / 400.0) * car_foward_kp_;
                } else {
                    car_foward_vel_ = 0;
                    currentState_ = RobotState::APPROACHING;    
                    state_start_time_ = this->now();
                    last_pid_time_ = this->now() - rclcpp::Duration::from_seconds(PID_INTERVAL);
                    RCLCPP_INFO(this->get_logger(), "Base aligned, moving arm.");
                }
                break;

            case RobotState::APPROACHING:
                executeApproach();
                break;

            case RobotState::GRASPING:
                gripper_val = 100; // 闭合
                if ((this->now() - state_start_time_).seconds() > 1.5) {
                    arm.joint_rad[0] = look_jointA;
                    Arm2DOF_SetTargetEndEffector(&arm, look_position_x, look_position_y);
                    
                    currentState_ = RobotState::RETRACT_TO_LOOK;
                    state_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Grasped. Retracting to Look position...");
                }
                break;

            case RobotState::RETRACT_TO_LOOK:
                if ((this->now() - state_start_time_).seconds() > 3.0) {
                    arm.joint_rad[0] = put_jointA;
                    Arm2DOF_SetTargetEndEffector(&arm, put_position_x, put_position_y);
                    
                    currentState_ = RobotState::MOVE_TO_PUT;
                    state_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Moving to Put position...");
                }
                break;

            case RobotState::MOVE_TO_PUT:
                if ((this->now() - state_start_time_).seconds() > 5.0) {
                    currentState_ = RobotState::RELEASING;
                    state_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "In position. Releasing...");
                }
                break;

            case RobotState::RELEASING:
                gripper_val = 0; // 松开
                if ((this->now() - state_start_time_).seconds() > 1.0) {
                    changeArmState(); 
                    currentState_ = RobotState::BACK_TO_LOOK;
                    state_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Released. Returning to Look position...");
                }
                break;

            case RobotState::BACK_TO_LOOK:
                if ((this->now() - state_start_time_).seconds() > 5.0) {
                    currentState_ = RobotState::WAIT_USER_RESET;
                    RCLCPP_INFO(this->get_logger(), "Cycle Complete. Waiting for user to reset 'pick_start'...");
                }
                break;
            
            case RobotState::WAIT_USER_RESET:
                if (!pick_start) {
                    currentState_ = RobotState::IDLE;
                    RCLCPP_INFO(this->get_logger(), "User reset detected. Ready for next run.");
                }
                break;

            default:
                break;
        }
    }

    void executeApproach() {
        if (!object_detected_) return;

        double time_elapsed = (this->now() - last_pid_time_).seconds();
        if (time_elapsed < PID_INTERVAL) return;
        last_pid_time_ = this->now();

        if (area_ >= target_area_size_) {
            RCLCPP_INFO(this->get_logger(), "Target Reached. Grasping!");
            currentState_ = RobotState::GRASPING;
            state_start_time_ = this->now();
            return;
        }

        double ratio = (area_ / target_area_size_);
        if (ratio > 1.0) ratio = 1.0;

        // Yaw 控制
        if (ratio <= 0.85 && abs(x_bias_) > (5.0 + ratio * 10.0)) {
            double kp = std::max(pid_yaw_kp_ * (1.0 - ratio * 0.8), 0.00005);
            double step = x_bias_ * kp;
            
            if (step > 0.02) step = 0.02; else if (step < -0.02) step = -0.02;
            if (abs(step) > 1e-5) arm.joint_rad[0] -= step;
        }

        // X/Y 控制
        double step_x = 0.0;
        double step_down = 0.0;
        double raw_step = abs(y_bias_) * pid_bias_kp_;
        if (raw_step > 0.008) raw_step = 0.008;

        if (y_bias_ > 10.0) {
            step_x    = raw_step * ratio_dominant_;   
            step_down = raw_step * ratio_secondary_;  
        } else if (y_bias_ < -10.0) {
            step_down = raw_step * ratio_dominant_;   
            step_x    = raw_step * ratio_secondary_; 
        } else {
            step_x    = min_speed; 
            step_down = min_speed; 
        }

        double current_x = arm.ee_x;
        double current_y = arm.ee_y;
        double target_x = current_x + step_x;
        double target_y = current_y - step_down; 
        if (target_y < -0.168) target_y = -0.168;

        if (!Arm2DOF_SetTargetEndEffector(&arm, target_x, target_y)) {
            bool fallback_success = false;
            if (y_bias_ > 10.0) { 
                 if (abs(step_x) > 1e-5) 
                     fallback_success = Arm2DOF_SetTargetEndEffector(&arm, target_x, current_y);
                 if (!fallback_success && abs(step_down) > 1e-5)
                     Arm2DOF_SetTargetEndEffector(&arm, current_x, target_y);
            } else { 
                 if (abs(step_down) > 1e-5)
                     fallback_success = Arm2DOF_SetTargetEndEffector(&arm, current_x, target_y);
                 if (!fallback_success && abs(step_x) > 1e-5)
                     Arm2DOF_SetTargetEndEffector(&arm, target_x, current_y);
            }
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        Mat raw_image, hsv_image, mask;
        try {
            raw_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        resize(raw_image, raw_image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        cvtColor(raw_image, hsv_image, CV_BGR2HSV);
        
        // 可视化
        double target_x_img = raw_image.cols/2;
        double target_y_img = raw_image.rows/2 - target_y_bias_; 
        circle(raw_image, cv::Point(target_x_img, target_y_img), 10, cv::Scalar(0, 0, 255), 1);

        int* current_hsv = hsv_params;
        if (color_mode == 1) current_hsv = green_hsv;
        else if (color_mode == 2) current_hsv = blue_hsv;
        else if (color_mode == 3) current_hsv = yellow_hsv;
        else if (color_mode == 4) current_hsv = red_hsv;

        inRange(hsv_image, cv::Scalar(current_hsv[0], current_hsv[1], current_hsv[2]),
                           cv::Scalar(current_hsv[3], current_hsv[4], current_hsv[5]), mask);

        dilate(mask, mask, morph_element_);
        erode(mask, mask, morph_element_, cv::Point(-1,-1), 2);
        dilate(mask, mask, morph_element_);

        std::vector<std::vector<cv::Point>> contours;
        findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (contours.empty()) {
                object_detected_ = false;
                area_ = 0;
            } else {
                auto max_cnt = std::max_element(contours.begin(), contours.end(), 
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b){
                        return contourArea(a) < contourArea(b);
                    });
                
                area_ = contourArea(*max_cnt);
                cv::Point2f center;
                float radius;
                minEnclosingCircle(*max_cnt, center, radius);

                object_detected_ = true;
                x_bias_ = raw_image.cols/2 - center.x;
                y_bias_ = raw_image.rows/2 - center.y;
                distance_bias_ = target_area_size_ - area_;

                circle(raw_image, cv::Point(center.x, center.y), radius, cv::Scalar(0, 0, 255), 5); 
            }
        }
        resize(raw_image, raw_image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        cv::imshow("rgb_image", raw_image);
        cv::waitKey(3);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        double current_x = msg->pose.pose.position.x;
        
        if (!pick_start) {
            last_odom_foward_ = current_x;
        }

        odom_foward = current_x - last_odom_foward_;
        if (abs(odom_foward) > car_search_dist_max) {
            car_stop = true;
        }
    }

    void publishArmState() {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = arm.joint_rad[0];
        msg.data[1] = arm.joint_rad[1];
        msg.data[2] = arm.joint_rad[2];
        msg.data[3] = gripper_val;
        arm_pub_->publish(msg);
    }

    void publishCmdVel() {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = car_foward_vel_;
        vel.angular.z = car_turn_vel_;
        vel_pub_->publish(vel);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    Arm2DOF_Init(&arm);

    // 使用 MultiThreadedExecutor 确保图像回调和主循环能并行处理
    auto node = std::make_shared<AutoPickNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 在一个单独的线程中运行 controlLoop
    std::thread control_thread([node]() {
        rclcpp::Rate loop_rate(50);
        while (rclcpp::ok()) {
            node->controlLoop();
            loop_rate.sleep();
        }
    });

    // 主线程处理回调
    executor.spin();

    // 退出清理
    if (control_thread.joinable()) {
        control_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
