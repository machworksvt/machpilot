#include "FireManager.hpp"



FireManager::FireManager(){}

bool FireManager::is_error() {
    return on_fire;
}

void FireManager::message_state_machine_error() {
    send_event(OnFireEvent());
};

void FireManager::message_state_error_ok() {
    send_event(FireSuppressedEvent());
};

void FireManager::fire_detected() { 
    fire_manager.on_fire = true;
    std::cout << "Fire detected!\n";
    fire_manager.message_state_machine_error();
}
void FireManager::fire_suppressed() {
    fire_manager.on_fire = false;
    std::cout << "Fire suppressed!\n";
    fire_manager.message_state_error_ok();
}

FireManager fire_manager=FireManager();