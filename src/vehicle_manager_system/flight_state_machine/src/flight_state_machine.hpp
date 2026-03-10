#ifndef FSMPILOTHPP
#define FSMPILOTHPP

#include <array>
#include <cstddef>
#include <iostream>
#include <variant>

#include "events.hpp"
#include "has_react.hpp"
#include "sub_systems.hpp"
#include "transition_verifier.hpp"
#include "states_enum.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

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
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] InitializeSubsystem event);
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ShutdownEvent event);
  void enter([[maybe_unused]] FlightStateMachine& machine);
  void exit ([[maybe_unused]] FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Uninitialized";
  static constexpr StateEnum AS_ENUM=StateEnum::Uninitialized;
};

class Initialized : State {
 public:
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ArmMotor event);
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ShutdownEvent event);
  void enter([[maybe_unused]] FlightStateMachine& machine);
  void exit ([[maybe_unused]] FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Initialized";
  static constexpr StateEnum AS_ENUM=StateEnum::Initialized;
};

class Armed : State {
 public:
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ManualControl event);
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] DisarmEvent event);
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] ShutdownEvent event);
  void enter([[maybe_unused]] FlightStateMachine& machine);
  void exit ([[maybe_unused]] FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Armed";
  static constexpr StateEnum AS_ENUM=StateEnum::Armed;
};

class ManualFlight : State {
 public:
  void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] RelinquishManualControl event);
  void enter([[maybe_unused]] FlightStateMachine& machine);
  void exit ([[maybe_unused]] FlightStateMachine& machine);

  static constexpr std::string_view NAME = "ManualFlight";
  static constexpr StateEnum AS_ENUM=StateEnum::ManualFlight;
};

class Shutdown : State {
 public:
  void enter([[maybe_unused]] FlightStateMachine& machine);
  void exit ([[maybe_unused]] FlightStateMachine& machine);

  static constexpr std::string_view NAME = "Shutdown";
  static constexpr StateEnum AS_ENUM=StateEnum::Shutdown;
};


class FlightStateMachine {
 private:
  substates current_state;
  TransitionVerifier transition_verifier;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher;

 public:
  FlightStateMachine(TransitionVerifier transition_verifier, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher);
  template <typename T>
  void react(T event) {
    static_assert(std::is_base_of<Event, T>::value,
                  "input to react must be an event");

    std::visit([this, event](auto& state) { react_inner(state, event); },
               current_state);
  }

  /** 
   * @brief attempts to transition the state machine from StartT to EndT
   * 
   * @param [in] start_state the current state the machine is in
   * 
   * @param [in] end_state the state you want the machine to be in
   * 
   * @return whether the transition was successful
   */
  template <typename StartT, typename EndT>
  bool transition(StartT& start_state, EndT end_state) {
    bool output=this->transition_verifier.request_transition(std::chrono::milliseconds(500),EndT::AS_ENUM);

    if (output){
      force_transition(start_state,end_state);
    }

    return output;
  }

  /** 
   * @brief transitions state machine from StartT to EndT without asking the transition verifier
   * 
   * @param [in] start_state the current state the machine is in
   * 
   * @param [in] end_state the state you want the machine to be in
   */
  template <typename StartT, typename EndT>
  void force_transition(StartT& start_state, EndT end_state) {
    static_assert(std::is_base_of_v<State, StartT>,
                  "we must transition from a state type");
    static_assert(std::is_base_of_v<State, EndT>,
                  "we must transition to a state type");
    
    start_state.exit(*this);
    end_state.enter(*this);
    current_state.emplace<EndT>(std::move(end_state));

    auto message = std_msgs::msg::UInt8();
    message.data=static_cast<std::uint8_t>(EndT::AS_ENUM);
    this->state_publisher->publish(message);
  }

 private:
  template <typename StateT, typename EventT>
  void react_inner(StateT& state, EventT& event) {
    if constexpr (has_react_v<StateT, EventT>) {
      // only runs if react exist for the given types
      state.react(*this, event);
    } else {
      // Fallback behavior
      std::cerr << StateT::NAME << " recived an unexpected event "
                << EventT::NAME << std::endl;
    }
  }
};

#endif  // FSMPILOTHPPF