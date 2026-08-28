#include "wheeltec_ultrasonic/supersonic_converter.hpp"

#include <limits>
#include <cmath>

using std::placeholders::_1;



static inline bool valid_distance(float d, double min_r, double max_r)
{
  return std::isfinite(d) && d >= min_r && d <= max_r;
}

SupersonicConverter::SupersonicConverter(const rclcpp::NodeOptions & options)
: rclcpp::Node("supersonic_converter", options)
{
  cfg_.robot_type = this->declare_parameter<std::string>("robot_type", "s300_pro");
  cfg_.input_topic = this->declare_parameter<std::string>("input_topic", "Distance");
  cfg_.base_frame = this->declare_parameter<std::string>("base_frame", "base_footprint");
  cfg_.publish_pointcloud = this->declare_parameter<bool>("publish_pointcloud", true);
  cfg_.min_range = this->declare_parameter<double>("min_range", 0.02);
  cfg_.max_range = this->declare_parameter<double>("max_range", 3.0);
  cfg_.field_of_view = this->declare_parameter<double>("field_of_view", 0.5);

  // Create publishers for A..F (mini将不发布F)
  const std::vector<std::string> labels = {"A","B","C","D","E","F"};
  for (const auto & lb : labels)
  {
    if (cfg_.robot_type == "s300_mini" && lb == "F") continue;
    const std::string topic = std::string("/ultrasonic/") + lb;
    range_pubs_[lb] = this->create_publisher<sensor_msgs::msg::Range>(topic, rclcpp::QoS(10));
  }

  if (cfg_.publish_pointcloud)
  {
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ultrasonic/points", rclcpp::QoS(10));
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = this->create_subscription<robot_interfaces::msg::Supersonic>(
    cfg_.input_topic, rclcpp::QoS(10), std::bind(&SupersonicConverter::supersonic_callback, this, _1));
}

void SupersonicConverter::publish_one_range(const std::string & frame_id,
                                            const std::string & topic_suffix,
                                            float distance,
                                            const rclcpp::Time & stamp)
{
  if (!range_pubs_.count(topic_suffix)) return;
  auto pub = range_pubs_.at(topic_suffix);

  sensor_msgs::msg::Range msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = static_cast<float>(cfg_.field_of_view);
  msg.min_range = static_cast<float>(cfg_.min_range);
  msg.max_range = static_cast<float>(cfg_.max_range);
  msg.range = valid_distance(distance, cfg_.min_range, cfg_.max_range) ? distance : std::numeric_limits<float>::infinity();
  pub->publish(msg);
}

void SupersonicConverter::publish_pointcloud( robot_interfaces::msg::Supersonic & in)
{
  if (!cloud_pub_ || !cfg_.publish_pointcloud) return;
  
  if(in.distance_a > 0.2) in.distance_a=0;
  if(in.distance_b > 0.7) in.distance_b=0;
  if(in.distance_c > 0.7) in.distance_c=0;
  if(in.distance_d > 0.7) in.distance_d=0;
  if(in.distance_e > 0.7) in.distance_e=0;
  if(in.distance_f > 0.2) in.distance_f=0;

  if (cfg_.robot_type == "s300_mini")
  {
    in.distance_a=0;
    in.distance_e=0;
  }
  // 需要把各超声束长变成各自坐标下 (d,0,0)，再通过 TF 变到 base_frame
  std::vector<std::pair<std::string, float>> vals = {
    {"A", in.distance_a}, {"B", in.distance_b}, {"C", in.distance_c},
    {"D", in.distance_d}, {"E", in.distance_e}, {"F", in.distance_f}
  };
  if (cfg_.robot_type == "s300_mini")
  {
    vals.pop_back(); // remove F
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = this->get_clock()->now();
  cloud.header.frame_id = cfg_.base_frame;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(0);

  auto append_point = [&](double x, double y, double z){
    const auto old_size = mod.size();
    mod.resize(old_size + 1);
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    ix += old_size; iy += old_size; iz += old_size;
    *ix = static_cast<float>(x);
    *iy = static_cast<float>(y);
    *iz = static_cast<float>(z);
  };

  for (const auto & p : vals)
  {
    const auto & label = p.first;
    const float d = p.second;
    if (!valid_distance(d, cfg_.min_range, cfg_.max_range)) continue;
    const std::string frame_id = std::string("ultrasonic_") + label;

    try {
      geometry_msgs::msg::TransformStamped T = tf_buffer_->lookupTransform(
        cfg_.base_frame, frame_id, tf2::TimePointZero);

      tf2::Transform tfb;
      tfb.setOrigin(tf2::Vector3(T.transform.translation.x,
                                 T.transform.translation.y,
                                 T.transform.translation.z));
      tf2::Quaternion q;
      tf2::fromMsg(T.transform.rotation, q);
      tfb.setRotation(q);

      tf2::Vector3 ps(d, 0.0, 0.0); // 传感器坐标下的点
      tf2::Vector3 pb = tfb * ps;   // 变换到 base_frame
      append_point(pb.x(), pb.y(), pb.z());
    }
    catch(const std::exception & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF lookup failed for %s -> %s: %s", frame_id.c_str(), cfg_.base_frame.c_str(), e.what());
    }
  }

  cloud_pub_->publish(cloud);
}

void SupersonicConverter::supersonic_callback(const robot_interfaces::msg::Supersonic::SharedPtr in)
{
  const auto stamp = in->header.stamp; // 由底盘发布节点填充
  if(in->distance_a > 0.2) in->distance_a=0;
  if(in->distance_b > 0.8) in->distance_b=0;
  if(in->distance_c > 0.8) in->distance_c=0;
  if(in->distance_d > 0.8) in->distance_d=0;
  if(in->distance_e > 0.8) in->distance_e=0;
  if(in->distance_f > 0.2) in->distance_f=0;
 

  publish_one_range("ultrasonic_A", "A", in->distance_a, stamp);
  publish_one_range("ultrasonic_B", "B", in->distance_b, stamp);
  publish_one_range("ultrasonic_C", "C", in->distance_c, stamp);
  publish_one_range("ultrasonic_D", "D", in->distance_d, stamp);
  publish_one_range("ultrasonic_E", "E", in->distance_e, stamp);

  if (cfg_.robot_type != "s300_mini")
    publish_one_range("ultrasonic_F", "F", in->distance_f, stamp);

  publish_pointcloud(*in);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupersonicConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

