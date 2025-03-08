#include <tinyfsm.hpp>


struct ScreenOnEvent : tinyfsm::Event{};
struct ScreenOffEvent: tinyfsm::Event{};


class ScreenManager: public tinyfsm::Fsm<ScreenManager>{
public:
    virtual void react(ScreenOnEvent  const &){};
    virtual void react(ScreenOffEvent const &){};

    virtual void entry(void)=0;
    void exit(void){};
};

