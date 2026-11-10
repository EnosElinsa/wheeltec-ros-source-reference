// Humble's Python subscriptions omit publisher GIDs; use C++ MessageInfo.
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TfAuthorityGuard : public rclcpp::Node
{
  using SteadyClock = std::chrono::steady_clock;
  struct Observation {
    std::string parent;
    SteadyClock::time_point received;
  };
  using Authors = std::map<std::string, Observation>;

public:
  TfAuthorityGuard() : Node("vision_tf_authority")
  {
    require_map_ = declare_parameter("require_map", true);
    author_timeout_s_ = declare_parameter("author_timeout_s", 5.0);
    if (!std::isfinite(author_timeout_s_) || author_timeout_s_ <= 0.0) {
      throw std::invalid_argument("author_timeout_s must be finite and positive");
    }
    output_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/vision/tf_authority", 2);
    input_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SensorDataQoS(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        const auto & gid = info.get_rmw_message_info().publisher_gid;
        std::ostringstream identifier;
        bool nonzero = false;
        for (auto byte : gid.data) {
          nonzero = nonzero || byte != 0;
          identifier << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
        }
        const std::string author = nonzero ? identifier.str() : "unknown";
        for (const auto & tf : msg->transforms) {
          if (tf.child_frame_id == "base_footprint" || tf.child_frame_id == "odom") {
            authors_[tf.child_frame_id][author] = {tf.header.frame_id, SteadyClock::now()};
          }
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { report(); });
  }

private:
  void report()
  {
    const auto current = SteadyClock::now();
    for (auto & child : authors_) {
      for (auto it = child.second.begin(); it != child.second.end();) {
        if (std::chrono::duration<double>(current - it->second.received).count() >= author_timeout_s_) {
          it = child.second.erase(it);
        } else {
          ++it;
        }
      }
    }
    const auto valid = [this](const std::string & child, const std::string & parent) {
        const auto & sources = authors_[child];
        return sources.size() == 1 && sources.begin()->first != "unknown" &&
               sources.begin()->second.parent == parent;
      };
    bool healthy = valid("base_footprint", "odom");
    if (require_map_ || !authors_["odom"].empty()) {
      healthy = healthy && valid("odom", "map");
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "vision/tf_authority";
    status.hardware_id = "tf_publisher_gid";
    status.level = healthy ? status.OK : status.ERROR;
    status.message = healthy ? "unique_expected_parents" : "missing_duplicate_or_wrong_parent";
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(status);
    output_->publish(message);
  }

  bool require_map_;
  double author_timeout_s_;
  std::map<std::string, Authors> authors_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr input_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr output_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfAuthorityGuard>());
  rclcpp::shutdown();
  return 0;
}
