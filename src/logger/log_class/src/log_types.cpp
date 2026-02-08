#include "log_class/log_types.hpp"

std::ostream& operator<<(std::ostream& os, const Heartbeat&) {
  return os << "Heartbeat";
}

std::ostream& operator<<(std::ostream& os, const LoggerStartup&) {
  return os << "LoggerStartup";
}

std::ostream& operator<<(std::ostream& os,
                         const InvaidDeserializationBadTag& log) {
  return os << "InvaidDeserializationBadTag{tag: " << log.tag << "}";
}

std::ostream& operator<<(std::ostream& os,
                         const InvalidDeserializationBadVariantSize& log) {
  return os << "InvalidDeserializationBadVariantSize{tag: " << log.tag
            << ", correct_size: " << log.correct_size
            << ", given size: " << log.given_size << "}";
}