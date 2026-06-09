#include "flight_state_machine.hpp"

#include <iostream>
#include <type_traits>
#include <utility>

// FlightStateMachine
FlightStateMachine::FlightStateMachine(TransitionVerifier transition_verifier, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher)
    : current_state(substates{Uninitialized{}})
    , transition_verifier(transition_verifier) 
    , state_publisher(state_publisher){
  std::get<Uninitialized>(current_state).enter(*this);
}

// Uninitialized
Uninitialized::Uninitialized() : subsystems_ready{false}, init_count{0} {}
void Uninitialized::react([[maybe_unused]] FlightStateMachine& machine,
                          [[maybe_unused]] InitializeSubsystem event) {
  
  std::uint8_t initialized_subsystem_index = event.subsystem_id;
  
  if (initialized_subsystem_index>=SUBSYSTEM_COUNT){
    std::cerr<<"Invalid Subsystem Id "<< static_cast<int>(initialized_subsystem_index) <<std::endl;
    return;
  }

  SubSystems initialized_subsystem=static_cast<SubSystems>(event.subsystem_id);
  std::string_view subsystem_name = sub_system_to_string(initialized_subsystem);

  if (!subsystems_ready[initialized_subsystem_index]) {
    subsystems_ready[initialized_subsystem_index] = true;
    ++init_count;

    std::cout << "Subsystem " << subsystem_name << " initialized ("
              << init_count << "/" << static_cast<int>(SUBSYSTEM_COUNT) << ")"
              << std::endl;

    if (init_count == SUBSYSTEM_COUNT) {
      if (!machine.transition(*this, Initialized{})){
        machine.force_transition(*this,Shutdown{});
      }
    }
  } else {
    std::cout << "Subsystem " << subsystem_name
              << " has already been initialized" << std::endl;
  }
}
void Uninitialized::react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ShutdownEvent event) {
  machine.transition(*this, Shutdown{});
}
void Uninitialized::enter([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
  if constexpr (SUBSYSTEM_COUNT == 0) {
    machine.transition(*this, Initialized{});
  }
}
void Uninitialized::exit([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// Initialized
void Initialized::react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ArmMotor event) {
  machine.transition(*this, Armed{});
}
void Initialized::react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ShutdownEvent event) {
  machine.transition(*this, Shutdown{});
}
void Initialized::enter([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void Initialized::exit([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// Armed
void Armed::react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ManualControl event) {
  machine.transition(*this, ManualFlight{});
}
void Armed::react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] DisarmEvent event) {
  machine.transition(*this, Initialized{});
}
void Armed::react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ShutdownEvent event) {
  machine.transition(*this, Shutdown{});
}
void Armed::enter([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void Armed::exit([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// ManualFlight
void ManualFlight::react([[maybe_unused]] FlightStateMachine& machine,
                         [[maybe_unused]] RelinquishManualControl event) {
  machine.transition(*this, Armed{});
}
void ManualFlight::enter([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void ManualFlight::exit([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}

// Shutdown
void Shutdown::enter([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "entering " << NAME << std::endl;
}
void Shutdown::exit([[maybe_unused]] FlightStateMachine& machine) {
  std::cerr << "exiting " << NAME << std::endl;
}
