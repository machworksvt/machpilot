#include <tinyfsm.hpp>
#include "ScreenManager.hpp"
#include <iostream>
#include <array>


// Event declarations
struct InitializeSubsystem : tinyfsm::Event {
  size_t subsystem_id;
  InitializeSubsystem(size_t id) : subsystem_id(id) {}
};

struct ARMMotor      : tinyfsm::Event { };
struct TakeOff       : tinyfsm::Event { };
struct Landing       : tinyfsm::Event { };
struct DisarmEvent   : tinyfsm::Event { };
struct ShutdownEvent : tinyfsm::Event { };

class Uninitialized;
class Initialized;
class Armed;
class Flight;
class Disarm;
class Shutdown;

// Base state machine class
class StateMachine : public tinyfsm::Fsm<StateMachine> {
public:
    virtual void react(InitializeSubsystem const & e) {};
    virtual void react(ARMMotor            const & e) {};
    virtual void react(TakeOff             const & e) {};
    virtual void react(Landing             const & e) {};
    virtual void react(DisarmEvent         const & e) {};
    virtual void react(ShutdownEvent       const & e) {};
    
    virtual bool screen_on()=0;

    virtual void entry_start() {}
    virtual void entry_end() {}

    void entry() {
      entry_start();

      if (screen_on()){
        ScreenManager::dispatch(ScreenOnEvent());
      }else{
        ScreenManager::dispatch(ScreenOffEvent());
      }
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

  bool screen_on() override{
    return true;
  }
};

// Initialized state: on receiving ARMMotor, transition to Armed.
class Initialized : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Initialized state\n";
  }
  void react(ARMMotor const &) override {
    transit<Armed>();
  }

  bool screen_on() override{
    return true;
  }
};

// Armed state: on receiving TakeOff, transition to Flight. on reciving DisarmEvent, transition to Disarm
class Armed : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Armed state\n";
  }
  void react(TakeOff const &) override {
    transit<Flight>();
  }
  void react(DisarmEvent const &) override {
    transit<Disarm>();
  }

  bool screen_on() override{
    return true;
  }
};

// Flight state: on receiving Landing, transition to Armed.
class Flight : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Flight state\n";
  }
  void react(Landing const &) override {
    transit<Armed>();
  }

  bool screen_on() override{
    return false;
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

  bool screen_on() override{
    return true;
  }
};

// Shutdown state: final state.
class Shutdown : public StateMachine {
  void entry_start() override {
    std::cout << "Transitioned to Shutdown state\n";
  }
  bool screen_on() override{
    return false;
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
    
    ScreenManager::start();
    StateMachine::start();

    // Initialize two sensors.
    send_event(InitializeSubsystem(0));

    // Initialize sensor 0 twice
    send_event(InitializeSubsystem(0));

    send_event(InitializeSubsystem(1));
    
    // Transition from Initialized to Armed.
    send_event(ARMMotor());
    
    // Transition from Armed to Flight.
    send_event(TakeOff());
    
    // Transition from Flight to Landing.
    send_event(Landing());
    
    // Transition from Landing to Disarm.
    send_event(DisarmEvent());
    
    // Transition from Disarm to Shutdown.
    send_event(ShutdownEvent());
    return 0;
}
