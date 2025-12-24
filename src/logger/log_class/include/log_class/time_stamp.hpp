#ifndef TIME_STAMP_HPP
#define TIME_STAMP_HPP

#include <chrono>

using time_point_t = std::chrono::system_clock::time_point;
using milliseconds_t = std::chrono::milliseconds;

// A time stamp with a well defined layout for serialization
class time_stamp {
 public:
  std::uint64_t ms_since_epoch;

  static time_stamp now() {
    return time_stamp(std::chrono::system_clock::now());
  }

  time_stamp() : ms_since_epoch(0) {}
  time_stamp(std::uint64_t ms) : ms_since_epoch(ms) {}
  time_stamp(time_point_t time)
      : ms_since_epoch(
            std::chrono::duration_cast<milliseconds_t>(time.time_since_epoch())
                .count()) {}

  time_point_t to_time_point() const {
    return time_point_t{milliseconds_t{ms_since_epoch}};
  }
};

#endif  // TIME_STAMP_HPP