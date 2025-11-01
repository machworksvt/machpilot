#ifndef ERRORMANAGER_HPP
#define ERRORMANAGER_HPP

#include <tinyfsm.hpp>
#include <iostream>
#include "FSMPilot.hpp"


class ErrorManager {
private:

    friend void StateMachine::entry();
    //Messange FSM that there is a error
    //This will should only called when is_error()==true
    virtual void message_state_machine_error()=0;
    
    //Messange FSM that there is no longer a error
    //This will should only called when is_error()==false
    virtual void message_state_error_ok()=0;

    void message_state_machine_if_error();
public:
    //this returns wether the system is in a error state
    virtual bool is_error()=0;
};

#endif // ERRORMANAGER_HPP