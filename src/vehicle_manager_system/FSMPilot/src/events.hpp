#ifndef EVENTSHPP
#define EVENTSHPP

#include <string_view>

#include "subsystems.hpp"

struct Event {};

struct ArmMotor : Event {
  static constexpr std::string_view NAME = "ArmMotor";
};
struct ManualControl : Event {
  static constexpr std::string_view NAME = "ManualControl";
};
struct RelinquishManualControl : Event {
  static constexpr std::string_view NAME = "RelinquishManualControl";
};
struct DisarmEvent : Event {
  static constexpr std::string_view NAME = "DisarmEvent";
};
struct ShutdownEvent : Event {
  static constexpr std::string_view NAME = "ShutdownEvent";
};
struct InitializeSubsystem : Event {
  SubSystems subsystem_id;
  InitializeSubsystem(SubSystems id) : subsystem_id(id) {}

  static constexpr std::string_view NAME = "InitializeSubsystem";
};

#endif  // EVENTSHPP