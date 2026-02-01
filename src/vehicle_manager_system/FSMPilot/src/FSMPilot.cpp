#include "FSMPilot.hpp"

#include <iostream>
#include <type_traits>
#include <utility>

// FlightStateMachine
FlightStateMachine::FlightStateMachine()
    : current_state(substates{Uninitialized{}}) {
  std::get<Uninitialized>(current_state).enter(*this);
}

// Uninitialized
Uninitialized::Uninitialized() : subsystems_ready{false}, init_count{0} {}
void Uninitialized::react(FlightStateMachine& machine,
                          InitializeSubsystem event) {
  std::size_t ready_index = static_cast<std::size_t>(event.subsystem_id);
  std::string_view subsystem_name = sub_system_to_string(event.subsystem_id);

  if (!subsystems_ready[ready_index]) {
    subsystems_ready[ready_index] = true;
    ++init_count;

    std::cout << "Subsystem " << subsystem_name << " initialized ("
              << init_count << "/" << static_cast<int>(SUBSYSTEM_COUNT) << ")"
              << std::endl;

    if (init_count == SUBSYSTEM_COUNT) {
      machine.transition(*this, Initialized{});
    }
  } else {
    std::cout << "Subsystem " << subsystem_name
              << " has already been initialized" << std::endl;
  }
}
void Uninitialized::react(FlightStateMachine& machine, ShutdownEvent event) {
  machine.transition(*this, Shutdown{});
}
void Uninitialized::enter(FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
  if constexpr (SUBSYSTEM_COUNT == 0) {
    machine.transition(*this, Initialized{});
  }
}
void Uninitialized::exit(FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// Initialized
void Initialized::react(FlightStateMachine& machine, ArmMotor event) {
  machine.transition(*this, Armed{});
}
void Initialized::react(FlightStateMachine& machine, ShutdownEvent event) {
  machine.transition(*this, Shutdown{});
}
void Initialized::enter(FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void Initialized::exit(FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// Armed
void Armed::react(FlightStateMachine& machine, ManualControl event) {
  machine.transition(*this, ManualFlight{});
}
void Armed::react(FlightStateMachine& machine, DisarmEvent event) {
  machine.transition(*this, Initialized{});
}
void Armed::react(FlightStateMachine& machine, ShutdownEvent event) {
  machine.transition(*this, Shutdown{});
}
void Armed::enter(FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void Armed::exit(FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// ManualFlight
void ManualFlight::react(FlightStateMachine& machine,
                         RelinquishManualControl event) {
  machine.transition(*this, Armed{});
}
void ManualFlight::enter(FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void ManualFlight::exit(FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// Shutdown
void Shutdown::enter(FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void Shutdown::exit(FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}
