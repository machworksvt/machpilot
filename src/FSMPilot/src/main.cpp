#include "FSMPilot.hpp"
#include "FireManager.hpp"



int optimal_event_loop(){
    StateMachine::start();

    // Initialize subsystem 0.
    send_event(InitializeSubsystem(SubSystems::SUBSYSTEM_0));

    // Initialize sensor 0 twice
    send_event(InitializeSubsystem(SubSystems::SUBSYSTEM_0));

     // Initialize subsystem 1.
    send_event(InitializeSubsystem(SubSystems::SUBSYSTEM_1));
    
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
int fire_event_loop(){

    StateMachine::start();

    // Initialize subsystem 0.
    send_event(InitializeSubsystem(SubSystems::SUBSYSTEM_0));

     // Initialize subsystem 1.
    send_event(InitializeSubsystem(SubSystems::SUBSYSTEM_1));
    
    // Transition from Initialized to Armed.
    send_event(ArmMotor());
    
    // Transition from Armed to Manual Flight.
    send_event(ManualControl());
    
    // First started mid flight
    FireManager::fire_detected();

    // Transition from Manual Flight to Landing.
    send_event(RelinquishManualControl());
    
    // Transition from Landing to Disarm.
    send_event(DisarmEvent());

    FireManager::fire_suppressed();
    
    // Transition from Disarm to Shutdown.
    send_event(ShutdownEvent());
    return 0;
}

int main(){
    fire_event_loop();
}