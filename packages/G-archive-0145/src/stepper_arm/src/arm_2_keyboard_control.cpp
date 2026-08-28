#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <string>

#include "stepper_motor_arm.cpp"

using namespace std;
using namespace cv;

// 全局变量
static Arm_t arm;
static float gripper_val = 0.0f;   // 0~100
static geometry_msgs::msg::Twist twist;

// 参数定义
static const float JOINT_STEP = 0.02f;   // ~1.1 度
static const float EE_STEP    = 0.01f;   // 1 cm
static const float GRIP_STEP  = 5.0f;    // 夹爪每次 5%
static const double LIN_VEL   = 0.2;     // m/s
static const double ANG_VEL   = 0.5;     // rad/s

class ArmCarTeleop : public rclcpp::Node
{
public:
    ArmCarTeleop() : Node("arm_car_keyboard_teleop")
    {
        // 初始化发布者
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        arm_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("arm_cmd", 10);

        // 初始化机械臂底层结构
        Arm2DOF_Init(&arm);
        gripper_val = 0.0f;
        
        // 初始化 GUI 窗口
        cv::namedWindow("Arm 2DOF Control", cv::WINDOW_AUTOSIZE);
        
        // 打印提示
        RCLCPP_INFO(this->get_logger(), "ROS 2 Teleop Node Started.");
        RCLCPP_INFO(this->get_logger(), "Focus on the GUI window to control.");
        RCLCPP_INFO(this->get_logger(), "Press ESC to exit.");

        // 设置定时器，用于主循环 (30Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ArmCarTeleop::timer_callback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        // 1. 绘制 GUI
        drawArmGUI();

        // 2. 处理键盘输入 (只在这一帧内有效)
        // waitKey(1) 会阻塞 1ms，刚好配合定时器
        int key = cv::waitKey(1);

        if (key >= 0)
        {
            if (key == 27) // ESC
            {
                rclcpp::shutdown();
                return;
            }

            // 大小写转换
            if (key >= 'A' && key <= 'Z') key = key - 'A' + 'a';

            handleKey(key);
        }
    }

    void publishArmState()
    {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(4);
        msg.data[0] = arm.joint_rad[0];   // 关节 A
        msg.data[1] = arm.joint_rad[1];   // 关节 B
        msg.data[2] = arm.joint_rad[2];   // 关节 C
        msg.data[3] = gripper_val;        // 夹爪

        arm_pub_->publish(msg);
    }

    void handleKey(int c)
    {
        bool arm_changed  = false;
        bool base_changed = false;

        switch (c)
        {
        // --- 机械臂控制 ---
        case 'q': // Joint A +
            { float a = arm.joint_rad[0] + JOINT_STEP; if (a <= arm.joint_max[0]) { arm.joint_rad[0] = a; arm_changed = true; } break; }
        case 'a': // Joint A -
            { float a = arm.joint_rad[0] - JOINT_STEP; if (a >= arm.joint_min[0]) { arm.joint_rad[0] = a; arm_changed = true; } break; }
        
        case 'w': // Joint B +
            { float b = arm.joint_rad[1] + JOINT_STEP; if (Arm2DOF_SetTargetJoint(&arm, b, arm.joint_rad[2])) arm_changed = true; break; }
        case 's': // Joint B -
            { float b = arm.joint_rad[1] - JOINT_STEP; if (Arm2DOF_SetTargetJoint(&arm, b, arm.joint_rad[2])) arm_changed = true; break; }

        case 'e': // Joint C +
            { float c2 = arm.joint_rad[2] + JOINT_STEP; if (Arm2DOF_SetTargetJoint(&arm, arm.joint_rad[1], c2)) arm_changed = true; break; }
        case 'd': // Joint C -
            { float c2 = arm.joint_rad[2] - JOINT_STEP; if (Arm2DOF_SetTargetJoint(&arm, arm.joint_rad[1], c2)) arm_changed = true; break; }

        // 末端控制
        case 'r': if (Arm2DOF_MoveEndEffectorRelative(&arm, MOVE_RIGHT, EE_STEP)) arm_changed = true; break;
        case 'f': if (Arm2DOF_MoveEndEffectorRelative(&arm, MOVE_LEFT,  EE_STEP)) arm_changed = true; break;
        case 't': if (Arm2DOF_MoveEndEffectorRelative(&arm, MOVE_UP,    EE_STEP)) arm_changed = true; break;
        case 'g': if (Arm2DOF_MoveEndEffectorRelative(&arm, MOVE_DOWN,  EE_STEP)) arm_changed = true; break;

        // 夹爪
        case 'y': gripper_val = std::min(100.0f, gripper_val + GRIP_STEP); arm.gripper = (uint8_t)gripper_val; arm_changed = true; break;
        case 'h': gripper_val = std::max(0.0f,   gripper_val - GRIP_STEP); arm.gripper = (uint8_t)gripper_val; arm_changed = true; break;

        // --- 小车底盘控制 ---
        case 'u': twist.linear.x =  LIN_VEL;  twist.angular.z =  ANG_VEL;  base_changed = true; break;
        case 'i': twist.linear.x =  LIN_VEL;  twist.angular.z =  0.0;      base_changed = true; break;
        case 'o': twist.linear.x =  LIN_VEL;  twist.angular.z = -ANG_VEL;  base_changed = true; break;
        case 'j': twist.linear.x =  0.0;      twist.angular.z =  ANG_VEL;  base_changed = true; break;
        case 'l': twist.linear.x =  0.0;      twist.angular.z = -ANG_VEL;  base_changed = true; break;
        case 'm': twist.linear.x = -LIN_VEL;  twist.angular.z =  ANG_VEL;  base_changed = true; break;
        case ',': twist.linear.x = -LIN_VEL;  twist.angular.z =  0.0;      base_changed = true; break;
        case '.': twist.linear.x = -LIN_VEL;  twist.angular.z = -ANG_VEL;  base_changed = true; break;
        case 'k': twist.linear.x =  0.0;      twist.angular.z =  0.0;      base_changed = true; break;

        default: break;
        }

        if (base_changed) cmd_pub_->publish(twist);
        if (arm_changed)  publishArmState();
    }

    // GUI 绘制函数 
    void drawArmGUI()
    {
        static const int width  = 600;
        static const int height = 600;
        static bool initialized = false;
        static cv::Mat canvas(height, width, CV_8UC3);

        if (!initialized) { initialized = true; }

        // 背景
        canvas.setTo(cv::Scalar(30, 30, 30));

        const int cx = width / 2;
        const int cy = height * 3 / 4;
        const float scale = 800.0f;

        auto toPixel = [&](float x, float y) -> cv::Point {
            int px = static_cast<int>(cx + x * scale);
            int py = static_cast<int>(cy - y * scale);
            return cv::Point(px, py);
        };

       
        cv::Scalar grid_color(60, 60, 60);
        cv::line(canvas, toPixel(-0.3, 0), toPixel(0.3, 0), cv::Scalar(80, 80, 80), 2); // X轴
        cv::line(canvas, toPixel(0, -0.3), toPixel(0, 0.3), cv::Scalar(80, 80, 80), 2); // Y轴

        // 画工作空间圆弧
        const int ARC_COUNT = static_cast<int>(sizeof(g_workspace_arcs) / sizeof(g_workspace_arcs[0]));
        for (int i = 0; i < ARC_COUNT; ++i) {
            const ArmWorkspaceArc_t &arc = g_workspace_arcs[i];
            const int segments = 40;
            cv::Point prev;
            bool has_prev = false;
            for (int k = 0; k <= segments; ++k) {
                float ratio = (float)k / segments;
                float theta = arc.theta_start + (arc.theta_end - arc.theta_start) * ratio;
                float x = arc.center_x + arc.radius * std::cos(theta);
                float y = arc.center_y + arc.radius * std::sin(theta);
                cv::Point p = toPixel(x, y);
                if (has_prev) cv::line(canvas, prev, p, cv::Scalar(0, 160, 0), 1);
                prev = p; has_prev = true;
            }
        }

        float b = arm.joint_rad[1];
        float c = arm.joint_rad[2];
        float x1 = Link_B * std::cos(b);
        float y1 = Link_B * std::sin(b);
        float x2 = x1 + Link_C * std::cos(c - PI + b);
        float y2 = y1 + Link_C * std::sin(c - PI + b);

        cv::Point base  = toPixel(0.0f, 0.0f);
        cv::Point joint = toPixel(x1, y1);
        cv::Point ee    = toPixel(x2, y2);

 
        bool inside_ws = isPointInsideArcs(arm.ee_x, arm.ee_y); 

        cv::Scalar link_color = inside_ws ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255);
        cv::line(canvas, base, joint, link_color, 2);
        cv::line(canvas, joint, ee, link_color, 2);
        cv::circle(canvas, base, 6, cv::Scalar(0, 255, 0), -1);
        cv::circle(canvas, joint, 5, cv::Scalar(0, 255, 255), -1);
        cv::circle(canvas, ee, 6, cv::Scalar(0, 0, 255), -1);

        // 显示文字信息
        string info1 = "Joint A: " + to_string(arm.joint_rad[0]);
        string info2 = "Joint B: " + to_string(arm.joint_rad[1]);
        string info3 = "Joint C: " + to_string(arm.joint_rad[2]);
        string info4 = "Gripper: " + to_string((int)gripper_val);
        string info5 = "EEX: " + to_string(arm.ee_x);
        string info6 = "EEY: " + to_string(arm.ee_y);

        cv::putText(canvas, info1, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
        cv::putText(canvas, info2, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
        cv::putText(canvas, info3, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
        cv::putText(canvas, info4, cv::Point(200, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
        cv::putText(canvas, info5, cv::Point(200, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));
        cv::putText(canvas, info6, cv::Point(200, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255));

       // --- 机械臂键位帮助 ---
        cv::putText(canvas, "Arm: q/a w/s e/d", cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200));
        cv::putText(canvas, "EE:  r/f t/g", cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200));
        cv::putText(canvas, "Grip: y/h", cv::Point(10, 140), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200));

        // --- 小车控制键位块 (还原详细布局) ---
        int base_block_x = width - 180;  // 右侧留出一块区域
        cv::putText(canvas, "car_control", cv::Point(base_block_x-10, 20),
                    cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvas, "u   i   o", cv::Point(base_block_x, 40),
                    cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvas, "j    k   l", cv::Point(base_block_x, 65),
                    cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvas, "m  ,   .", cv::Point(base_block_x, 90),
                    cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvas, "k: stop", cv::Point(base_block_x, 115),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(180, 180, 180), 1);
        cv::imshow("Arm 2DOF Control", canvas);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmCarTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}