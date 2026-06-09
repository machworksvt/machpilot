#ifndef EVENTSHPP
#define EVENTSHPP

#include <string_view>

#include "sub_systems.hpp"


/**
 * Base Class representing a event that the state machine can react to
 *
 * All instences will have a static constexpr std::string_view NAME varable that holds the states name for printing
 */
struct Event {};

/**
 * Event to arm the motor.
 */
struct ArmMotor : Event {
  static constexpr std::string_view NAME = "ArmMotor";
};

/**
 * Event to take control of the aircraft
 */
struct ManualControl : Event {
  static constexpr std::string_view NAME = "ManualControl";
};

/**
 * Event to take relinquish control of the aircraft
 */
struct RelinquishManualControl : Event {
  static constexpr std::string_view NAME = "RelinquishManualControl";
};

/**
 * event to turn off the motor
 */
struct DisarmEvent : Event {
  static constexpr std::string_view NAME = "DisarmEvent";
};

/**
 * event to shut down the craft
 */
struct ShutdownEvent : Event {
  static constexpr std::string_view NAME = "ShutdownEvent";
};

/**
 * event to report that one of the subsystems is ready
 */
struct InitializeSubsystem : Event {

  /// The id of the subsystem that is now ready
  std::uint8_t subsystem_id;

  /**
   * Creates a new Event saying that a subsystem is Initialised
   * @param id the subsystem that has been Initialised
   */
  InitializeSubsystem(std::uint8_t id) : subsystem_id(id) {}

  static constexpr std::string_view NAME = "InitializeSubsystem";
};

#endif  // EVENTSHPP