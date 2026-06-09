#include <states_enum.hpp>




std::string_view states_to_string(StateEnum s){
    switch (s){
        case StateEnum::Uninitialized:
            return "Uninitialized";
        case StateEnum::Initialized:
            return "Initialized";
        case StateEnum::Armed:
            return "Armed";
        case StateEnum::ManualFlight:
            return "ManualFlight";
        case StateEnum::Shutdown:
            return "Shutdown";
        default:
            return "Unknown";
    }
}