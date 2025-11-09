#ifndef FSMPILOTSTATES_HPP
#define FSMPILOTSTATES_HPP

#include <string_view>

enum class FSMPilotStates: unsigned char{
    UNINITIALIZED,
    INITIALIZED,
    ARMED,
    MANUALFLIGHT,
    DISARM,
    SHUTDOWN,
};

std::string_view get_state_name(FSMPilotStates state);


#endif // FSMPILOTSTATES_HPP