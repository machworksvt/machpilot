#include "FSMPilot.hpp"
#include <iostream>
#include <array>
#include "ErrorManagerList.hpp"

// --- Define Extern Constants ---
std::string sub_system_to_String(SubSystems id) {
  switch (id) {
      case SUBSYSTEM_0: return "Subsystem 0";
      case SUBSYSTEM_1: return "Subsystem 1";
      default: return "Unknown Subsystem";
  }
};

// --- Base State Machine Method Implementations ---

// Default empty implementations for react methods in the base class
void StateMachine::react(InitializeSubsystem const & e)     {}
void StateMachine::react(ArmMotor const & e)                {}
void StateMachine::react(ManualControl const & e)           {}
void StateMachine::react(RelinquishManualControl const & e) {}
void StateMachine::react(DisarmEvent const & e)             {}
void StateMachine::react(ShutdownEvent const & e)           {}
void StateMachine::react(OnFireEvent const & e)             {
    std::cout <<"\t"<<get_name() << " received fire event\n";
}
void StateMachine::react(FireSuppressedEvent const & e)     {
    std::cout <<"\t"<<get_name() << " received fire suppressed event\n";
}

// Default implementation for get_motor_state
MotorSound StateMachine::get_motor_state() {
    return MotorSound::Silent;
}


void StateMachine::entry_start() {
    std::cout << "Entering state: " << get_name() << "\n";
}
void StateMachine::entry_end()   {}


void StateMachine::entry() {
    entry_start();

    for (ErrorManager* error_handler : error_handlers) {
        error_handler->message_state_machine_if_error();
    }


    set_screen_state(get_screen_state());
    set_motor_state(get_motor_state());

    entry_end();
}


// --- Uninitialized State Implementation ---
Uninitialized::Uninitialized() : subsystems{}, init_count(0) {}

std::string Uninitialized::get_name() const{
    return "Uninitialized";
}

void Uninitialized::entry_end() {
    if (SUBSYSTEM_COUNT == 0) {
        std::cout << "No subsystems need be initialized transitioning to initialized state directly\n";
        transit<Initialized>();
    }
}

void Uninitialized::react(InitializeSubsystem const & e) {
    if (e.subsystem_id < SUBSYSTEM_COUNT) {
        std::string subsystem_name = sub_system_to_String(e.subsystem_id);

        if (!subsystems[e.subsystem_id]) {
            subsystems[e.subsystem_id] = true;
            ++init_count;
            std::cout << "Subsystem " << subsystem_name << " initialized ("
                      << init_count << "/" << SUBSYSTEM_COUNT << ")\n";
            if (init_count == SUBSYSTEM_COUNT) {
                transit<Initialized>();
            }
        } else {
            std::cout << "Subsystem " << subsystem_name << " has already been initialized\n";
        }
    } else {
        std::cout << "Invalid Subsystem id: " << e.subsystem_id << "\n";
    }
}

ScreenState Uninitialized::get_screen_state() {
    return ScreenState::ScreenOn;
}

// --- Initialized State Implementation ---

std::string Initialized::get_name() const{
    return "Initialized";
}

void Initialized::react(ArmMotor const &) {
    transit<Armed>();
}

ScreenState Initialized::get_screen_state() {
    return ScreenState::ScreenOn;
}

MotorSound Initialized::get_motor_state() {
    return MotorSound::Quiet;
}

// --- Armed State Implementation ---


std::string Armed::get_name() const{
    return "Armed";
}

void Armed::react(ManualControl const &) {
    transit<ManualFlight>();
}

void Armed::react(DisarmEvent const &) {
    transit<Disarm>();
}

ScreenState Armed::get_screen_state() {
    return ScreenState::ScreenOff;
}

MotorSound Armed::get_motor_state() {
    return MotorSound::Loud;
}

// --- ManualFlight State Implementation ---

std::string ManualFlight::get_name() const{
    return "ManualFlight";
}

void ManualFlight::react(RelinquishManualControl const &) {
    transit<Armed>();
}

ScreenState ManualFlight::get_screen_state() {
    return ScreenState::ScreenOff;
}

// --- Disarm State Implementation ---
std::string Disarm::get_name() const{
    return "Disarm";
}

void Disarm::react(ShutdownEvent const &) {
    transit<Shutdown>();
}

ScreenState Disarm::get_screen_state() {
    return ScreenState::ScreenOn;
}

MotorSound Disarm::get_motor_state() {
    return MotorSound::Quiet;
}

// --- Shutdown State Implementation ---
std::string Shutdown::get_name() const{
    return "Shutdown";
}

ScreenState Shutdown::get_screen_state() {
    return ScreenState::ScreenOff;
}

// --- Define the Initial State ---
FSM_INITIAL_STATE(StateMachine, Uninitialized)