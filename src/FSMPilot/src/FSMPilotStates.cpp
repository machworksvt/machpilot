#include "FSMPilotStates.hpp"


std::string_view get_name(FSMPilotStates state) {
    switch (state) {
        case FSMPilotStates::UNINITIALIZED:
            return "UNINITIALIZED";
        case FSMPilotStates::INITIALIZED:
            return "INITIALIZED";
        case FSMPilotStates::ARMED:
            return "ARMED";
        case FSMPilotStates::MANUALFLIGHT:
            return "MANUALFLIGHT";
        case FSMPilotStates::DISARM:
            return "DISARM";
        case FSMPilotStates::SHUTDOWN:
            return "SHUTDOWN";
        default:
            return "UNKNOWNSTATE";
    }
}