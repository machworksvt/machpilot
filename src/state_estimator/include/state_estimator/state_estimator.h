

// a store for raw data incoming from sensors
typedef struct raw_state_t {
    // pitot tube data
    double pitot_time_1r = 0.0;
    double pitot_time_2r = 0.0;
    double pitot_temp_1r = 0.0;
    double pitot_temp_2r = 0.0;
    double pitot_pressure_1r = 0.0;
    double pitot_pressure_2r = 0.0;
    double pitot_var_t_1r = 0.0;
    double pitot_var_p_1r = 0.0;
    double pitot_var_t_2r = 0.0;
    double pitot_var_p_2r = 0.0;

    // bmp data
    double bmp_time_1r = 0.0;
    double bmp_time_2r = 0.0;
    double bmp_temp_1r = 0.0;
    double bmp_temp_2r = 0.0;
    double bmp_pressure_1r = 0.0;
    double bmp_pressure_2r = 0.0;
    double bmp_var_t_1r = 0.0;
    double bmp_var_p_1r = 0.0;
    double bmp_var_t_2r = 0.0;
    double bmp_var_p_2r = 0.0;

    // imu data
} raw_state_t;

// a store for filtered data incoming from sensors and current control outputs
typedef struct state_estimate_t {
    double pitot_time_1f = 0.0;
} state_estimate_t;