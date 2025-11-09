#include <tinyfsm.hpp>
#include <iostream>
#include "ErrorManager.hpp"

void ErrorManager::message_state_machine_if_error(){
    if (is_error()){
        message_state_machine_error();
    }
};
