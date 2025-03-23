

#include <iostream>

enum MotorSound{
    Loud,
    Quiet,
    Silent,
};


MotorSound current_motor_state=Silent;

void set_motor_state(MotorSound new_state){
    if (new_state==current_motor_state){
        return;
    };
    current_motor_state=new_state;
    switch (new_state){
        case MotorSound::Loud:
            std::cout << "\tMotor Set To Loud\n";
            break;
        case MotorSound::Quiet:
            std::cout << "\tMotor Set To Quiet\n";
            break;
        case MotorSound::Silent:
            std::cout << "\tMotor Set To Silent\n";
            break;
    };
}