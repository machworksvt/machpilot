#include "log_class/severity.hpp"


std::string_view severity_to_string(Severity s) {
  switch (s) {
    case Severity::Log:
      return "LOG";
    case Severity::Warning:
      return "WARN";
    case Severity::Error:
      return "ERROR";
  }
  return "UNKNOWN";
}