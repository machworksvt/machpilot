#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <chrono>
#include <iostream>
#include <variant>

#include "log_types.hpp"
#include "time_stamp.hpp"
#include "trivially_copyable_variant.hpp"

enum class Severity { Log, Warning, Error };

//defines what types we can log
typedef TriviallyCopyableVariant<LoggerStartup, Heartbeat,
                                 InvaidDeserializationBadTag,
                                 InvalidDeserializationBadVariantSize>
    LogInner;

class Log {
  friend std::ostream& operator<<(std::ostream& os, const Log& log);

 public:
  // the time of the log
  time_stamp time;

  // how severe the log
  Severity severity;
  // a variant that holds the log
  LogInner sub_log;

  Log(time_stamp t, Severity s, LogInner l)
      : time(t), severity(s), sub_log(l) {}

  Log(Severity s, LogInner l) : Log(time_stamp::now(), s, l) {}

  Log() : Log(0, Severity::Log, Heartbeat{}) {}
};

#endif  // MESSAGE_HPP