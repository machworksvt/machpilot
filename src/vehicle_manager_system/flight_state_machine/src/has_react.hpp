
#include <type_traits>

class FlightStateMachine;

//  Primary template: assumes the method does not exist.
template <typename StateT, typename EventT, typename = void>
struct has_react : std::false_type {};
// Specialization: valid only if StateT::react(FlightStateMachine*, EventT) is a valid expression.
template <typename StateT, typename EventT>
struct has_react<
    StateT, EventT,
    std::void_t<decltype(std::declval<StateT>().react(
        std::declval<FlightStateMachine&>(), 
        std::declval<EventT>()
    ))>>
    : std::true_type {};

// Helper constant to lookup if StateT::react(FlightStateMachine*, EventT) is a valid expression.
template <typename StateT, typename EventT>
inline constexpr bool has_react_v = has_react<StateT, EventT>::value;