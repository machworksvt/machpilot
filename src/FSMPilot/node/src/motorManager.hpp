enum MotorSound{
    Loud,
    Quiet,
    Silent,
};

extern MotorSound current_motor_state;

void set_motor_state(MotorSound new_state);
