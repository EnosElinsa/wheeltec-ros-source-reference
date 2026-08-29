#include <memory>
#include <cmath>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFixerNode : public rclcpp::Node
{
public:
    ScanFixerNode() : Node("scan_fixer_node")
    { 
        // 这里我们要设定两个关键参数
        // 1. target_range: 假墙距离（或者叫 range_max - epsilon）
        // 2. real_range_max: 雷达真实的物理极限（用于判断是否超量程）
        this->declare_parameter<float>("target_range", 5.0); // 假墙设为 5.0 (或者你的 29.99)
        this->declare_parameter<float>("real_range_max", 30.0); // 雷达物理极限

        this->declare_parameter<std::string>("input_topic", "/scan");
        this->declare_parameter<std::string>("output_topic", "/scan_clearing");

        target_range_ = this->get_parameter("target_range").as_double();
        real_range_max_ = this->get_parameter("real_range_max").as_double();
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        auto qos = rclcpp::SensorDataQoS();
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic, qos, 
            std::bind(&ScanFixerNode::scan_callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            output_topic, qos);

        // 随机数引擎（用于微抖动，解决静止残留）
        std::random_device rd;
        gen_ = std::mt19937(rd());
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto new_scan = std::make_unique<sensor_msgs::msg::LaserScan>(*msg);
        
        // 替换值：即 range_max - epsilon
        float replace_value = target_range_ - 0.001f; 

        // 加上微小的角度抖动（Jitter），解决光线梳子效应
        std::uniform_real_distribution<float> dis(-0.5f, 0.5f);
        float jitter = dis(gen_) * msg->angle_increment;
        new_scan->angle_min += jitter;
        new_scan->angle_max += jitter;

        // 遍历每一个点
        for (float &range : new_scan->ranges)
        {
            // ====================================================
            // 这里就是你在 ROS1 中想要的完整逻辑
            // ====================================================
            bool condition_1 = std::isinf(range);            // 情况1: inf
            bool condition_2 = std::isnan(range);            // 情况2: NaN
            bool condition_3 = (range == 0.0f);              // 情况3: 0.0 (国内部分雷达特性)
            bool condition_4 = (range >= real_range_max_);   // 情况4: 超过物理量程 (如 Hokuyo 返回 65.33)
            bool condition_5 = (range >= target_range_);     // 情况5: 超过我们设定的假墙距离

            if (condition_1 || condition_2 || condition_3 || condition_4 || condition_5)
            {
                // 强制设为 "有效" 的极大值
                range = replace_value;
            } 
        }

        pub_->publish(std::move(new_scan));
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    float target_range_;
    float real_range_max_;
    std::mt19937 gen_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanFixerNode>());
    rclcpp::shutdown();
    return 0;
}