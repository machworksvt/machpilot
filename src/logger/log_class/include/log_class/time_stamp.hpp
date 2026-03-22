#ifndef TIME_STAMP_HPP
#define TIME_STAMP_HPP

#include <chrono>

using time_point_t = std::chrono::system_clock::time_point;
using milliseconds_t = std::chrono::milliseconds;

/**
 * @brief A POD-like timestamp wrapper for stable binary serialization.
 * * This class provides a well-defined 64-bit layout (milliseconds since epoch) 
 * to ensure that time data remains consistent when written to or read from 
 * disk, regardless of the platform's @c std::chrono::time_point implementation.
 */
class time_stamp {
 public:
  /// Milliseconds elapsed since the Unix epoch (1970-01-01 00:00:00 UTC).
  std::uint64_t ms_since_epoch;

  /**
   * @brief Captures the current system time.
   * @return A time_stamp representing the current moment.
   */
  static time_stamp now() {
    return time_stamp(std::chrono::system_clock::now());
  }

  /// Constructs a zero-initialized timestamp.
  time_stamp() : ms_since_epoch(0) {}

  /// Constructs a timestamp from a raw millisecond count.
  explicit time_stamp(std::uint64_t ms) : ms_since_epoch(ms) {}

  /**
   * @brief Constructs a timestamp from a standard library time_point.
   * @param time The system clock time point to convert.
   */
  time_stamp(time_point_t time)
      : ms_since_epoch(
            std::chrono::duration_cast<milliseconds_t>(time.time_since_epoch())
                .count()) {}
  
  /**
   * @brief Converts the serialized millisecond count back to a system time_point.
   * @return A std::chrono::system_clock::time_point.
   */
  time_point_t to_time_point() const {
    return time_point_t{milliseconds_t{ms_since_epoch}};
  }
};

#endif  // TIME_STAMP_HPP