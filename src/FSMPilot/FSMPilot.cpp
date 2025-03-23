#include <tinyfsm.hpp>
#include "ScreenManager.cpp"
#include <iostream>
#include <array>
#include "motorManager.cpp"




// Event declarations
struct InitializeSubsystem : tinyfsm::Event {
  size_t subsystem_id;
  InitializeSubsystem(size_t id) : subsystem_id(id) {}
};

struct ArmMotor                : tinyfsm::Event { };
struct ManualControl           : tinyfsm::Event { };
struct RelinquishManualControl : tinyfsm::Event { };
struct DisarmEvent             : tinyfsm::Event { };
struct ShutdownEvent           : tinyfsm::Event { };

class Uninitialized;
class Initialized;
class Armed;
class ManualFlight;
class Disarm;
class Shutdown;

// Base state machine class
class StateMachine : public tinyfsm::Fsm<StateMachine> {
public:
    virtual void react(InitializeSubsystem     const & e) {};
    virtual void react(ArmMotor                const & e) {};
    virtual void react(ManualControl           const & e) {};
    virtual void react(RelinquishManualControl const & e) {};
    virtual void react(DisarmEvent             const & e) {};
    virtual void react(ShutdownEvent           const & e) {};
    
    virtual ScreenState get_screen_state()=0;
    virtual MotorSound get_motor_state(){
      return MotorSound::Silent;
    };

    virtual void entry_start() {}
    virtual void entry_end() {}

    void entry() {

      entry_start();

      set_screen_state(get_screen_state());
      set_motor_state(get_motor_state());

      entry_end();
    };
    void exit() {}
};


static const size_t SUBSYSTEM_COUNT = 2;
static const std::array<std::string,SUBSYSTEM_COUNT> SUBSYSTEM_NAMES={"subsystems1","subsystems2"};

// Uninitialized state:
// Holds an array to track what systems are initializated
// use InitializeSubsystem event to initailize a subsystem
// once all subsystems are initalized transitions to Initialized
class Uninitialized : public StateMachine {
private:
  std::array<bool, SUBSYSTEM_COUNT> subsystems;
  size_t init_count;
public:
  Uninitialized() : subsystems{}, init_count(0) {}
private:
  void entry_start() override {
    std::cout << "Entering Uninitialized state\n";
  }
  void entry_end() override{
    if (SUBSYSTEM_COUNT==0){
      std::cout << "No subsystems need be initialized transitioning to initalizied state directly\n";
      transit<Initialized>();
    }
  }

  void react(InitializeSubsystem const & e) override {
    if (e.subsystem_id < SUBSYSTEM_COUNT) {
      std::string subsystem_name=SUBSYSTEM_NAMES[e.subsystem_id];

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

  ScreenState get_screen_state() override{
    return ScreenState::ScreenOn;
  }
};

// Initialized state: on receiving ArmMotor, transition to Armed.
class Initialized : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Initialized state\n";
  }
  void react(ArmMotor const &) override {
    transit<Armed>();
  }

  ScreenState get_screen_state() override{
    return ScreenState::ScreenOn;
  }

  MotorSound get_motor_state() override{
    return MotorSound::Quiet;
  }
};

// Armed state: on receiving ManualControl, transition to ManualFlight. on reciving DisarmEvent, transition to Disarm
class Armed : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Armed state\n";
  }
  void react(ManualControl const &) override {
    transit<ManualFlight>();
  }
  void react(DisarmEvent const &) override {
    transit<Disarm>();
  }

  ScreenState get_screen_state() override{
    return ScreenState::ScreenOff;
  }

  MotorSound get_motor_state() override{
    return MotorSound::Loud;
  }

};

// ManualFlight state: on receiving RelinquishManualControl, transition to Armed.
class ManualFlight : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Flight state\n";
  }
  void react(RelinquishManualControl const &) override {
    transit<Armed>();
  }

  ScreenState get_screen_state() override{
    return ScreenState::ScreenOff;
  }
};

// Disarm state: on receiving Shutdown, transition to Shutdown.
class Disarm : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Disarm state\n";
  }
  void react(ShutdownEvent const &) override {
    transit<Shutdown>();
  }

  ScreenState get_screen_state() override{
    return ScreenState::ScreenOn;
  }

  MotorSound get_motor_state() override{
    return MotorSound::Quiet;
  }
};

// Shutdown state: final state.
class Shutdown : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Shutdown state\n";
  }
  ScreenState get_screen_state() override{
    return ScreenState::ScreenOff;
  }
};

// Define the initial state
FSM_INITIAL_STATE(StateMachine, Uninitialized)


template<typename E>
void send_event(E const & event)
{
    StateMachine::template dispatch<E>(event);
}


int main(){
    // Dispatch events in sequence to simulate state transitions.
    // The FSM starts in Uninitialized state.
    
    StateMachine::start();

    // Initialize subsystem 0.
    send_event(InitializeSubsystem(0));

    // Initialize sensor 0 twice
    send_event(InitializeSubsystem(0));

     // Initialize subsystem 1.
    send_event(InitializeSubsystem(1));
    
    // Transition from Initialized to Armed.
    send_event(ArmMotor());
    
    // Transition from Armed to Manual Flight.
    send_event(ManualControl());
    
    // Transition from Manual Flight to Landing.
    send_event(RelinquishManualControl());
    
    // Transition from Landing to Disarm.
    send_event(DisarmEvent());
    
    // Transition from Disarm to Shutdown.
    send_event(ShutdownEvent());
    return 0;
}
