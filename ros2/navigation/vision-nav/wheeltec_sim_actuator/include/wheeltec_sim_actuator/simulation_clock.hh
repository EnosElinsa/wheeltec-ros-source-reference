#pragma once
#include <cstdint>

namespace wheeltec {
// Integer simulation time rate limit; never quantize or invent a clock stamp.
// Publish rewinds immediately, even while paused, to invalidate stale ROS epochs.
struct SimulationClock {
  std::int64_t periodNs{10000000}, previousNs{0}, publishedNs{0};
  bool initialized{false};
  bool Due(std::int64_t nowNs,bool paused) {
    if (nowNs<0) return false;
    const bool rewind=initialized && nowNs<previousNs;
    previousNs=nowNs;
    if (!initialized || rewind || (!paused && nowNs-publishedNs>=periodNs)) {
      initialized=true;publishedNs=nowNs;return true;
    }
    return false;
  }
};
}
