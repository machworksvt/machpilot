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


//defines what types we can log
typedef TriviallyCopyableVariant<LoggerStartup, Heartbeat,
                                 InvaidDeserializationBadTag,
                                 InvalidDeserializationBadVariantSize,
                                 InvalidDeserializationBadSouce,
                                 InvalidDeserializationBadSeverity>
    LogInner;

class Log {
  friend std::ostream& operator<<(std::ostream& os, const Log& log);

 public:
  // the time of the log
  time_stamp time;

  // how severe the log
  Severity severity;

  // the source of the log
  Source source;

  // a variant that holds the log
  LogInner sub_log;

  Log(time_stamp t, Severity se, Source so, LogInner l)
      : time(t), severity(se), source(so), sub_log(l) {}

  Log(Severity se, Source so, LogInner l) : Log(time_stamp::now() ,se ,so, l) {}

  Log() : Log(time_stamp{}, Severity::Log ,Source::Logger ,Heartbeat{}) {}
};

#endif  // MESSAGE_HPP