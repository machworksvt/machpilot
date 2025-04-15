enum ScreenState{
    ScreenOn,
    ScreenOff,
};


extern ScreenState current_screen_state;

void set_screen_state(ScreenState new_state);