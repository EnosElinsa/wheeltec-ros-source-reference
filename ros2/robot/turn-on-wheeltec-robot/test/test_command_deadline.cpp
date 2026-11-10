#include "turn_on_wheeltec_robot/command_deadline.hpp"
#include <cassert>
#include <chrono>

int main()
{
  using Deadline = wheeltec::CommandDeadline;
  const auto start = Deadline::Clock::time_point{};
  Deadline deadline(0.3);
  assert(deadline.expired(start));
  deadline.note(start);
  assert(!deadline.expired(start + std::chrono::milliseconds(299)));
  assert(deadline.expired(start + std::chrono::milliseconds(300)));
  // A wall-clock/ROS-clock change cannot reset a steady-clock deadline.
  assert(deadline.expired(start - std::chrono::milliseconds(1)));
  deadline.note(start + std::chrono::seconds(1));
  assert(!deadline.expired(start + std::chrono::milliseconds(1200)));
  assert(deadline.expired(start + std::chrono::seconds(3)));
  Deadline disabled;
  assert(!disabled.expired(start));
  return 0;
}
