#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <chrono>
#include <iostream>
#include <variant>

#include "log_types.hpp"
#include "time_stamp.hpp"
#include "trivially_copyable_variant.hpp"
#include "source.hpp"
#include "severity.hpp"

/**
 * @brief The variant type representing the payload of a log entry.
 */
typedef TriviallyCopyableVariant<LoggerStartup, Heartbeat,
                                 InvaidDeserializationBadTag,
                                 InvalidDeserializationBadVariantSize,
                                 InvalidDeserializationBadSouce,
                                 InvalidDeserializationBadSeverity>
    LogInner;

/**
 * @brief A complete log entry including metadata and payload.
 */
class Log {
  /**
   * @brief Formats the log entry into a human-readable string.
   */
  friend std::ostream& operator<<(std::ostream& os, const Log& log);

 public:
  /// The time at which the event occurred.
  time_stamp time;

  /// The importance or urgency of the log entry.
  Severity severity;

  /// The subsystem or component that generated the log.
  Source source;

  /// The specific data payload of the log.
  LogInner sub_log;

  /**
   * @brief Constructs a log entry with a specific timestamp.
   * @param t  Timestamp for the log.
   * @param se Severity level.
   * @param so Source component.
   * @param l  The log payload.
   */
  Log(time_stamp t, Severity se, Source so, LogInner l)
      : time(t), severity(se), source(so), sub_log(l) {}

  /**
   * @brief Constructs a log entry using the current system time.
   * @param se Severity level.
   * @param so Source component.
   * @param l  The log payload.
   */
  Log(Severity se, Source so, LogInner l) : Log(time_stamp::now() ,se ,so, l) {}

  /**
   * @brief Default constructor creating a Heartbeat log at epoch 0.
   */
  Log() : Log(time_stamp{}, Severity::Log ,Source::Logger ,Heartbeat{}) {}
};

#endif  // MESSAGE_HPP