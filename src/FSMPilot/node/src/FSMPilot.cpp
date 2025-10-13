#include "FSMPilot.hpp"
#include <iostream>
#include <array>
#include "ErrorManagerList.hpp"
#include "state_machine_node.hpp"
#include "publisher.hpp"
#include <fstream>
#include <string>


std::shared_ptr<StateMachineNode> output_node=nullptr;
// --- Base State Machine Method Implementations ---

// Default empty implementations for react methods in the base class
void StateMachine::react(InitializeSubsystem const & e)     {}
void StateMachine::react(ArmMotor const & e)                {}
void StateMachine::react(ManualControl const & e)           {}
void StateMachine::react(RelinquishManualControl const & e) {}
void StateMachine::react(DisarmEvent const & e)             {}
void StateMachine::react(ShutdownEvent const & e)           {}
void StateMachine::react(OnFireEvent const & e)             {
    std::cout <<"\t"<<get_name() << " received fire event\n";
}
void StateMachine::react(FireSuppressedEvent const & e)     {
    std::cout <<"\t"<<get_name() << " received fire suppressed event\n";
}

// Default implementation for get_motor_state
MotorSound StateMachine::get_motor_state() {
    return MotorSound::Silent;
}


void StateMachine::entry_start() {
    std::cout << "Entering state: " << get_name() << "\n";
}
void StateMachine::entry_end()   {}


void StateMachine::entry() {
    entry_start();

    pubish_state(output_node,get_state());

    for (ErrorManager* error_handler : error_handlers) {
        error_handler->message_state_machine_if_error();
    }


    set_screen_state(get_screen_state());
    set_motor_state(get_motor_state());

    entry_end();
}


static std::array<sub_system_active, SUBSYSTEM_COUNT> subsystems_start;

// --- Uninitialized State Implementation ---
Uninitialized::Uninitialized() : subsystems{subsystems_start}, init_count(0) {}


FSMPilotStates Uninitialized::get_state() const{
    return FSMPilotStates::UNINITIALIZED;
}

std::string Uninitialized::get_name() const{
    return "Uninitialized";
}

void Uninitialized::entry_end() {
    if (SUBSYSTEM_COUNT == 0) {
        std::cout << "No subsystems need be initialized transitioning to initialized state directly\n";
        transit<Initialized>();
    }
}

void Uninitialized::react(InitializeSubsystem const & e) {
    if (e.subsystem_id < SUBSYSTEM_COUNT) {
        std::string_view subsystem_name = SubSystemToString(e.subsystem_id);
        
        switch (subsystems[e.subsystem_id]){
            case sub_system_active::UNINITIALIZED:
                subsystems[e.subsystem_id] = sub_system_active::INITIALIZED;    
                ++init_count;
                std::cerr << "Subsystem " << subsystem_name << " initialized ("<< init_count << "/" << SUBSYSTEM_COUNT << ")\n";
                if (init_count == SUBSYSTEM_COUNT) {
                    transit<Initialized>();
                }
                break;
            case sub_system_active::INITIALIZED:
                std::cerr << "Subsystem " << subsystem_name << " has already been initialized\n";
                break;
            case sub_system_active::UNUSED:
                std::cerr << "Subsystem " << subsystem_name << " is not being used\n";
                break;
        }
    } else {
        std::cerr << "Invalid Subsystem id: " << e.subsystem_id << "\n";
    }
}

ScreenState Uninitialized::get_screen_state() {
    return ScreenState::ScreenOn;
}

// --- Initialized State Implementation ---

FSMPilotStates Initialized::get_state() const{
    return FSMPilotStates::INITIALIZED;
}

std::string Initialized::get_name() const{
    return "Initialized";
}

void Initialized::react(ArmMotor const &) {
    transit<Armed>();
}

ScreenState Initialized::get_screen_state() {
    return ScreenState::ScreenOn;
}

MotorSound Initialized::get_motor_state() {
    return MotorSound::Quiet;
}

// --- Armed State Implementation ---

FSMPilotStates Armed::get_state() const{
    return FSMPilotStates::ARMED;
}

std::string Armed::get_name() const{
    return "Armed";
}

void Armed::react(ManualControl const &) {
    transit<ManualFlight>();
}

void Armed::react(DisarmEvent const &) {
    transit<Disarm>();
}

ScreenState Armed::get_screen_state() {
    return ScreenState::ScreenOff;
}

MotorSound Armed::get_motor_state() {
    return MotorSound::Loud;
}

// --- ManualFlight State Implementation ---

FSMPilotStates ManualFlight::get_state() const{
    return FSMPilotStates::MANUALFLIGHT;
}

std::string ManualFlight::get_name() const{
    return "ManualFlight";
}

void ManualFlight::react(RelinquishManualControl const &) {
    transit<Armed>();
}

ScreenState ManualFlight::get_screen_state() {
    return ScreenState::ScreenOff;
}

// --- Disarm State Implementation ---

FSMPilotStates Disarm::get_state() const{
    return FSMPilotStates::DISARM;
}

std::string Disarm::get_name() const{
    return "Disarm";
}

void Disarm::react(ShutdownEvent const &) {
    transit<Shutdown>();
}

ScreenState Disarm::get_screen_state() {
    return ScreenState::ScreenOn;
}

MotorSound Disarm::get_motor_state() {
    return MotorSound::Quiet;
}

// --- Shutdown State Implementation ---

FSMPilotStates Shutdown::get_state() const{
    return FSMPilotStates::SHUTDOWN;
}

std::string Shutdown::get_name() const{
    return "Shutdown";
}

ScreenState Shutdown::get_screen_state() {
    return ScreenState::ScreenOff;
}

int set_subsystems(char* file){

    //for the purpose of this function INITIALIZED means we have read in a value for this subsystem yet
    subsystems_start.fill(sub_system_active::INITIALIZED);

    std::unordered_map<std::string_view, uint8_t> mapping;

    for (u_int8_t s_id=0;s_id<SUBSYSTEM_COUNT;s_id++){
        SubSystems s=static_cast<SubSystems>(s_id);

        std::string_view name=SubSystemToString(s);

        if (!mapping.emplace(name, s_id).second) {
            std::cerr << "Error: Duplicate subsystem name generated: " << name << std::endl;
            return 1;
        }
    }

    std::ifstream inputFile(file);
    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open file: " << file << std::endl;
        return 1;
    }
    
    std::string line;
    if (!std::getline(inputFile, line)) {
        std::cerr << "Error: Invalid CSV file." << std::endl;
        return 1;
    }

    if (line.rfind("Subsystem Name,Active", 0) != 0) {
        std::cerr << "Error: Invalid CSV header. Expected 'Subsystem Name,Active'." << std::endl;
        return 1;
    }

    while (std::getline(inputFile, line)) {
        size_t sep_index = line.find(',');
        if (sep_index == std::string::npos) {
            std::cerr << "Error: Malformed line (no comma found)" << line << std::endl;
            return 1;
        }

        std::string_view subsystem_name{line.data(),sep_index};
        std::string_view active_str{line.data()+sep_index+1,line.length()-sep_index-1};
        
        auto it = mapping.find(std::string(subsystem_name));
        if (it == mapping.end()) {
            std::cerr << "Error: Unrecognized subsystem '" << subsystem_name << "' in CSV." << std::endl;
            return 1;
        }
        uint8_t subsystem_id = it->second;
        
        sub_system_active new_value;
        if (active_str == "TRUE") {
            new_value = sub_system_active::UNINITIALIZED;
        } else if (active_str == "FALSE") {
            new_value = sub_system_active::UNUSED;
        } else {
            std::cerr << "Error: Subsystem '" << subsystem_name << "' has unrecognized state: '" << active_str << "'." << std::endl;
            return 1;
        }

        switch (subsystems_start[subsystem_id]){
            case sub_system_active::INITIALIZED:
                subsystems_start[subsystem_id]=new_value;
                break;
            case sub_system_active::UNINITIALIZED:
            case sub_system_active::UNUSED:
                std::cerr << "subsystem " << subsystem_name << "defined twice in csv" << std::endl;
                return 1;
        }
    }


    for (u_int8_t s_id=0;s_id<SUBSYSTEM_COUNT;s_id++){

        if (subsystems_start[s_id]==sub_system_active::INITIALIZED){
            SubSystems s=(SubSystems)s_id;

            std::string_view name = SubSystemToString(static_cast<SubSystems>(s_id));
            std::cerr << "Error: Subsystem '" << name << "' was not defined in the CSV." << std::endl;
            return 1;
        }
    }

    return 0;
    
}

// --- Define the Initial State ---
FSM_INITIAL_STATE(StateMachine, Uninitialized)