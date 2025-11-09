#include "ErrorManager.hpp"


class FireManager : public ErrorManager{

private:
    bool on_fire=false;

public:
    FireManager() ;
    bool is_error() override;

    void message_state_machine_error() override;
    void message_state_error_ok() override;
    static void fire_detected();
    static void fire_suppressed();
};


extern FireManager fire_manager;