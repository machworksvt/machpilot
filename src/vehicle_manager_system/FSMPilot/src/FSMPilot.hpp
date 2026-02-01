#ifndef FSMPILOTHPP
#define FSMPILOTHPP

#include <array>
#include <cstddef>
#include <iostream>
#include <variant>

#include "events.hpp"
#include "has_react.hpp"
#include "subsystems.hpp"

class State {};
class FlightStateMachine;
class Uninitialized;
class Initialized;
class Armed;
class ManualFlight;
class Shutdown;

using substates =
    std::variant<Uninitialized, Initialized, Armed, ManualFlight, Shutdown>;

class Uninitialized : State {
 private:
  std::array<bool, SUBSYSTEM_COUNT> subsystems_ready;
  size_t init_count;

 public:
  Uninitialized();
  void react(FlightStateMachine& machine, InitializeSubsystem event);
  void react(FlightStateMachine& machine, ShutdownEvent event);
  void enter(FlightStateMachine& machine);
  void exit(FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Uninitialized";
};

class Initialized : State {
 public:
  void react(FlightStateMachine& machine, ArmMotor event);
  void react(FlightStateMachine& machine, ShutdownEvent event);
  void enter(FlightStateMachine& machine);
  void exit(FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Initialized";
};

class Armed : State {
 public:
  void react(FlightStateMachine& machine, ManualControl event);
  void react(FlightStateMachine& machine, DisarmEvent event);
  void react(FlightStateMachine& machine, ShutdownEvent event);
  void enter(FlightStateMachine& machine);
  void exit(FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Armed";
};

class ManualFlight : State {
 public:
  void react(FlightStateMachine& machine, RelinquishManualControl event);
  void enter(FlightStateMachine& machine);
  void exit(FlightStateMachine& machine);

  static constexpr std::string_view NAME = "ManualFlight";
};

class Shutdown : State {
 public:
  void enter(FlightStateMachine& machine);
  void exit(FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Shutdown";
};

class FlightStateMachine {
 private:
  substates current_state;

 public:
  FlightStateMachine();
  template <typename T>
  void react(T event) {
    static_assert(std::is_base_of<Event, T>::value,
                  "input to react must be an event");

    std::visit([this, event](auto& state) { react_inner(state, event); },
               current_state);
  }
  template <typename StartT, typename EndT>
  void transition(StartT& start_state, EndT end_state) {
    static_assert(std::is_base_of<State, StartT>::value,
                  "we must transition from a state type");
    static_assert(std::is_base_of<State, EndT>::value,
                  "we must transition to a state type");
    start_state.exit(*this);
    end_state.enter(*this);
    current_state.emplace<EndT>(std::move(end_state));
  }

 private:
  template <typename StateT, typename EventT>
  void react_inner(StateT& state, EventT& event) {
    if constexpr (has_react_v<StateT, EventT>) {
      // only runs is react exist for the given types
      state.react(*this, event);
    } else {
      // Fallback behavior
      std::cerr << StateT::NAME << " recived an unexpected event "
                << EventT::NAME << std::endl;
    }
  }
};

#endif  // FSMPILOTHPPF