#include <tinyfsm.hpp>
#include "ScreenManager.hpp"
#include <iostream>

class ScreenOn;
class ScreenOff;

class ScreenOn: public ScreenManager{
    void entry() override{
        std::cout<<"Screen On\n";
    }
    void react(ScreenOffEvent const & e) override{
        transit<ScreenOff>();
    }
};


class ScreenOff: public ScreenManager{
    void entry() override{
        std::cout<<"Screen Off\n";
    }
    void react(ScreenOnEvent const & e) override{
        transit<ScreenOn>();
    }
};

FSM_INITIAL_STATE(ScreenManager,ScreenOn);