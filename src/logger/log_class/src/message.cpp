#include "log_class/message.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

std::string_view severityToString(Severity s) {
  switch (s) {
    case Severity::Log:
      return "LOG";
    case Severity::Warning:
      return "WARN";
    case Severity::Error:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

void print_time_with_ms(std::ostream& os,
                        const std::chrono::system_clock::time_point& tp) {
  // Convert time_point to time_t for standard date/time components
  std::time_t t_c = std::chrono::system_clock::to_time_t(tp);

  // Convert to local time structure
  std::tm* local_tm = std::localtime(&t_c);

  // Extract the millisecond part using duration_cast
  auto duration = tp.time_since_epoch();
  auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() %
      1000;

  // %F is equivalent to %Y-%m-%d, %T is equivalent to %H:%M:%S
  os << std::put_time(local_tm, "%F %T") << '.' << std::setfill('0')
     << std::setw(3) << millis;
}

std::ostream& operator<<(std::ostream& os, const Log& log) {
  os << "Log{";
  os << "time: ";
  print_time_with_ms(os, log.time.to_time_point());
  os << " severity: " << severityToString(log.severity) << ", ";
  os << "data: ";
  log.sub_log.visit([&os](const auto& inner_log) { os << inner_log; });
  os << "}";
  return os;
}