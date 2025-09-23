#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <tinyfsm.hpp>
#include <array>
#include <string>
#include <memory>
#include "ScreenManager.hpp"
#include "motorManager.hpp"
#include "FSMPilotStates.hpp"
#include "state_machine_node.hpp"

////imu, gps, peto, engine, landing gear, pressure and temp,
// --- Subsystem Names ---
enum SubSystems: size_t {
    SUBSYSTEM_0,
    SUBSYSTEM_1,
};
static const size_t SUBSYSTEM_COUNT = 2;
extern std::shared_ptr<StateMachineNode> output_node;

std::string sub_system_to_String(SubSystems id);

// --- Event Declarations ---
struct InitializeSubsystem : tinyfsm::Event {
    SubSystems subsystem_id;
    InitializeSubsystem(SubSystems id) : subsystem_id(id) {}
};

struct ArmMotor                : tinyfsm::Event { };
struct ManualControl           : tinyfsm::Event { };
struct RelinquishManualControl : tinyfsm::Event { };
struct DisarmEvent             : tinyfsm::Event { };
struct ShutdownEvent           : tinyfsm::Event { };
struct OnFireEvent             : tinyfsm::Event { };
struct FireSuppressedEvent     : tinyfsm::Event { };

// --- Forward Declarations for States ---
class Uninitialized;
class Initialized;
class Armed;
class ManualFlight;
class Disarm;
class Shutdown;

// --- Base State Machine Class Declaration ---
class StateMachine : public tinyfsm::Fsm<StateMachine> {
public:
    virtual void react(InitializeSubsystem     const & e);
    virtual void react(ArmMotor                const & e);
    virtual void react(ManualControl           const & e);
    virtual void react(RelinquishManualControl const & e);
    virtual void react(DisarmEvent             const & e);
    virtual void react(ShutdownEvent           const & e);
    virtual void react(OnFireEvent             const & e);
    virtual void react(FireSuppressedEvent     const & e);

    virtual ScreenState get_screen_state() = 0;
    virtual MotorSound get_motor_state();

    virtual std::string get_name() const=0;
    virtual FSMPilotStates get_state() const=0;
    virtual void entry_start();
    virtual void entry_end();

    void entry();
    virtual void exit() {}
};

// --- State Class Declarations ---
class Uninitialized : public StateMachine {
private:
    std::array<bool, SUBSYSTEM_COUNT> subsystems;
    size_t init_count;
public:
    Uninitialized();
private:
    FSMPilotStates get_state() const override;
    std::string get_name() const override;
    void entry_end() override;
    void react(InitializeSubsystem const & e) override;
    ScreenState get_screen_state() override;
};

class Initialized : public StateMachine {
    FSMPilotStates get_state() const override;
    std::string get_name() const override;
    void react(ArmMotor const &) override;
    ScreenState get_screen_state() override;
    MotorSound get_motor_state() override;
};

class Armed : public StateMachine {
    FSMPilotStates get_state() const override;
    std::string get_name() const override;
    void react(ManualControl const &) override;
    void react(DisarmEvent const &) override;
    ScreenState get_screen_state() override;
    MotorSound get_motor_state() override;
};

class ManualFlight : public StateMachine {
    FSMPilotStates get_state() const override;
    std::string get_name() const override;
    void react(RelinquishManualControl const &) override;
    ScreenState get_screen_state() override;
};

class Disarm : public StateMachine {
    FSMPilotStates get_state() const override;
    std::string get_name() const override;
    void react(ShutdownEvent const &) override;
    ScreenState get_screen_state() override;
    MotorSound get_motor_state() override;
};

class Shutdown : public StateMachine {
    FSMPilotStates get_state() const override;
    std::string get_name() const override;
    ScreenState get_screen_state() override;
};


// --- Global Event Dispatch Function (Template) ---
template<typename E>
void send_event(E const & event)
{
    StateMachine::template dispatch<E>(event);
}

#endif // STATEMACHINE_HPP