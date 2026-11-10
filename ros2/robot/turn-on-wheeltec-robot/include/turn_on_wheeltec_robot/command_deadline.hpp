#ifndef WHEELTEC_COMMAND_DEADLINE_HPP_
#define WHEELTEC_COMMAND_DEADLINE_HPP_

#include <chrono>

namespace wheeltec
{
// Caller serializes access. This clock is independent of ROS time and sensor I/O.
class CommandDeadline
{
public:
  using Clock = std::chrono::steady_clock;
  explicit CommandDeadline(double timeout_s = 0.0) : timeout_s_(timeout_s) {}

  void note(Clock::time_point now) { last_ = now; seen_ = true; }

  bool expired(Clock::time_point now) const
  {
    return timeout_s_ > 0.0 && (!seen_ || now < last_ ||
      std::chrono::duration<double>(now - last_).count() >= timeout_s_);
  }

private:
  double timeout_s_;
  Clock::time_point last_{};
  bool seen_{false};
};
}  // namespace wheeltec

#endif  // WHEELTEC_COMMAND_DEADLINE_HPP_
