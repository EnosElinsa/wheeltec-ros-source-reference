#include "double_lidar_fusion/lidar_fusion.hpp"
#include <cmath>
#include <limits>

using sensor_msgs::msg::LaserScan;

LidarFusion::LidarFusion(const rclcpp::NodeOptions & options)
: Node("double_lidar_fusion", options)
{
  declare_parameter("scan1_topic",          "/scan1");
  declare_parameter("scan2_topic",          "/scan2");
  declare_parameter("fused_topic",          "scan");
  declare_parameter("frame_id",             "laser");
  declare_parameter("lidar1_angle_deg",      45.0);
  declare_parameter("lidar1_x_offset_m",      0.3);
  declare_parameter("lidar1_y_offset_m",      0.235);
  declare_parameter("lidar2_angle_deg",     -135.0);
  declare_parameter("lidar2_x_offset_m",     -0.3);
  declare_parameter("lidar2_y_offset_m",     -0.235);

  get_parameter("scan1_topic",          scan1_topic_);
  get_parameter("scan2_topic",          scan2_topic_);
  get_parameter("fused_topic",          fused_topic_);
  get_parameter("frame_id",             frame_id_);
  get_parameter("lidar1_angle_deg",     lidar1_angle_deg_);
  get_parameter("lidar1_x_offset_m",    lidar1_x_offset_m_);
  get_parameter("lidar1_y_offset_m",    lidar1_y_offset_m_);
  get_parameter("lidar2_angle_deg",     lidar2_angle_deg_);
  get_parameter("lidar2_x_offset_m",    lidar2_x_offset_m_);
  get_parameter("lidar2_y_offset_m",    lidar2_y_offset_m_);

  scan1_sub_.subscribe(this, scan1_topic_);
  scan2_sub_.subscribe(this, scan2_topic_);

  scans_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), scan1_sub_, scan2_sub_);
  scans_sync_->registerCallback(
      std::bind(&LidarFusion::fuseScans, this, std::placeholders::_1, std::placeholders::_2));

  fused_scan_pub_ = create_publisher<LaserScan>(fused_topic_, 10);

}

/* ============================================================ */
void LidarFusion::fuseScans(const LaserScan::ConstSharedPtr & scan1,
                            const LaserScan::ConstSharedPtr & scan2)
{

  if (angle_increment_rad_ == 0.0) {
    angle_increment_rad_ = scan1->angle_increment;
    bucket_count_ = static_cast<size_t>(std::round(2 * M_PI / angle_increment_rad_));
  }

  constexpr double ANGLE_MIN = -M_PI;         // 输出激光从 -π 开始
  constexpr double ANGLE_MAX =  M_PI;
  const size_t     TOTAL_BUCKETS = bucket_count_;
  constexpr float  MERGE_THRESHOLD_M = 0.03f; // 3 cm 死区，抑制闪烁

  LaserScan fused_scan;
  fused_scan.header.stamp       = scan1->header.stamp;            // 取第一台时间戳
  fused_scan.header.frame_id    = frame_id_;
  fused_scan.angle_min          = ANGLE_MIN;
  fused_scan.angle_max          = ANGLE_MAX;
  fused_scan.angle_increment    = angle_increment_rad_;
  fused_scan.range_min          = std::min(scan1->range_min, scan2->range_min);
  fused_scan.range_max          = std::max(scan1->range_max, scan2->range_max);
  fused_scan.scan_time          = scan1->scan_time;
  fused_scan.time_increment     = fused_scan.scan_time /
                                  static_cast<double>(TOTAL_BUCKETS);
  fused_scan.ranges.assign(TOTAL_BUCKETS, std::numeric_limits<float>::infinity());
  fused_scan.intensities.assign(TOTAL_BUCKETS, 0.0f);

  /* ---------- 预计算 cos/sin ---------- */
  const double cos_lidar1 = std::cos(lidar1_angle_deg_ * M_PI / 180.0);
  const double sin_lidar1 = std::sin(lidar1_angle_deg_ * M_PI / 180.0);
  const double cos_lidar2 = std::cos(lidar2_angle_deg_ * M_PI / 180.0);
  const double sin_lidar2 = std::sin(lidar2_angle_deg_ * M_PI / 180.0);

  /* ---------- 定义融合 Lambda ---------- */
  auto integrateOneLidar = [&](const LaserScan::ConstSharedPtr & scan,
                               double cos_theta, double sin_theta,
                               double x_offset, double y_offset)
  {
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const float raw_range = scan->ranges[i];
      if (!std::isfinite(raw_range)) continue;

      const double local_angle = scan->angle_min + angle_increment_rad_ * static_cast<double>(i);
      const double local_x     = raw_range * std::cos(local_angle);
      const double local_y     = raw_range * std::sin(local_angle);

      /* --- 局部坐标 -> 机器人基坐标 --- */
      const double base_x = local_x * cos_theta - local_y * sin_theta + x_offset;
      const double base_y = local_x * sin_theta + local_y * cos_theta + y_offset;

      const float  fused_range  = std::hypot(base_x, base_y);
      const float  fused_angle  = std::atan2(base_y, base_x);

      /* 把角度包裹到 (-π, π]  */
      int bucket_index = static_cast<int>(std::floor((fused_angle - ANGLE_MIN) /
                                                     angle_increment_rad_));
      if (bucket_index < 0 || static_cast<size_t>(bucket_index) >= TOTAL_BUCKETS) continue;

      float & current_value = fused_scan.ranges[static_cast<size_t>(bucket_index)];
      if (std::isinf(current_value)) {
        current_value = fused_range;
      } else if (std::fabs(current_value - fused_range) < MERGE_THRESHOLD_M) {
        current_value = 0.5f * (current_value + fused_range);      
      } else if (fused_range < current_value) {
        current_value = fused_range;                               // 取最近者
      }
    }
  };

  /* ----------  融合两台雷达 ---------- */
  integrateOneLidar(scan1, cos_lidar1, sin_lidar1,
                    lidar1_x_offset_m_, lidar1_y_offset_m_);
  integrateOneLidar(scan2, cos_lidar2, sin_lidar2,
                    lidar2_x_offset_m_, lidar2_y_offset_m_);

  /* ---------- 发布结果 ---------- */
  fused_scan_pub_->publish(fused_scan);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFusion>());
  rclcpp::shutdown();
  return 0;
}