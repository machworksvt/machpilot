#include "FSMPilot.hpp"
#include "events.hpp"

int main() {
  FlightStateMachine state;

  state.react(InitializeSubsystem{SubSystems::SUBSYSTEM_0});
  state.react(InitializeSubsystem{SubSystems::SUBSYSTEM_1});
  state.react(ArmMotor{});
  state.react(ManualControl{});
  state.react(RelinquishManualControl{});
  state.react(DisarmEvent{});
  state.react(ShutdownEvent{});

  
}