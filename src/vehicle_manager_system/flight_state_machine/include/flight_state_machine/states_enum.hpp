#ifndef STATES_ENUM_HPP
#define STATES_ENUM_HPP

#include <cstdint>
#include <string_view>


/**
 * A list of states the state machine can be in. Used to represent the current state to the logger.
 */
enum class StateEnum: std::uint8_t{
    Uninitialized,
    Initialized,
    Armed,
    ManualFlight,
    Shutdown,
};

/**
 * @brief converts a StateEnum to string veiw with a static lifetime
*/
std::string_view state_to_string(StateEnum s);


#endif