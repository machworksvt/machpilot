#include <iostream>

enum ScreenState{
    ScreenOn,
    ScreenOff,
};


ScreenState current_screen_state=ScreenOff;

void set_screen_state(ScreenState new_state){
    if (new_state==current_screen_state){
        return;
    };
    current_screen_state=new_state;
    switch (new_state){
        case ScreenState::ScreenOn:
            std::cout << "\tScreen Set To On\n";
            break;
        case ScreenState::ScreenOff:
            std::cout << "\tScreen Set To Off\n";
            break;
    };
}