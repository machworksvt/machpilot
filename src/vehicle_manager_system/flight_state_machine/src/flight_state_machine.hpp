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

/**
 * Base class representing a state that the craft can be in
 *
 * All Subclasses will have the variables
 * static constexpr std::string_view NAME holding the name of the class
 * static constexpr StateEnum AS_ENUM holding a StateEnum that holds the classes type
 *
 * All Subclasses will have the Methods
 * void enter([[maybe_unused]] FlightStateMachine& machine); That will be run whenever the state machine enters the state
 * void exit ([[maybe_unused]] FlightStateMachine& machine); That will be run whenever the state machine exits the state
 *
 * Any Methods of the form
 * void react([[maybe_unused]] FlightStateMachine& machine, [[maybe_unused]] SomeEventType event);
 * will be called when in the current state the state machine receives an event of the type SomeEventType
 * Within these methods calls to FlightStateMachine::transition will bring the statemachine into a differents state
 * These should be the last code run within react (excluding destructors)
 */

class State {};


//List of states the machine can be in
class FlightStateMachine;
class Uninitialized;
class Initialized;
class Armed;
class ManualFlight;
class Shutdown;


/**
 * Type holding the current state of the aircraft
 */
using substates =
    std::variant<Uninitialized, Initialized, Armed, ManualFlight, Shutdown>;

class Uninitialized : State {
 private:
  
  /**
    subsystems_ready[i] represents if the ith subsystem is read
  */
  std::array<bool, SUBSYSTEM_COUNT> subsystems_ready;
  /**
   * how many subsystems are ready
   */
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
  /// The current state the machine is
  substates current_state;

  /// A compoent that allows other nodes to block transitions
  TransitionVerifier transition_verifier;

  /// A publisher to publish when the state changes
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher;

 public:
  FlightStateMachine(TransitionVerifier transition_verifier, rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher);


  /**
   * @brief Sends an event to the state machine.
   *
   * @tparam T Type of the event. Must derive from Event.
   * @param event The event to process.
   */
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

  /**
   * Reacts to a Event
   * 
   * @param state the current state the state machine is in
   * @param event the event that is being reacted to
   */
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