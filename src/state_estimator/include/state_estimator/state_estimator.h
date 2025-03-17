#include "controller.hpp"
#include "rc/math/vector.h"
#include "rc/math/matrix.h"

// a store for raw data incoming from sensors
typedef struct {
    // pitot tube data
    double pitot_time_1;
    double pitot_time_2;
    double pitot_temp_1;
    double pitot_temp_2;
    double pitot_pressure_1;
    double pitot_pressure_2;
    double pitot_var_t_1;
    double pitot_var_p_1;
    double pitot_var_t_2;
    double pitot_var_p_2;

    // bmp data
    double bmp_time_1;
    double bmp_time_2;
    double bmp_temp_1;
    double bmp_temp_2;
    double bmp_pressure_1;
    double bmp_pressure_2;
    double bmp_var_t_1;
    double bmp_var_p_1;
    double bmp_var_t_2;
    double bmp_var_p_2;

    // imu data
    double imu_time_1;
    double imu_time_2;
    rc_vector_t imu_omega_1;
    rc_vector_t imu_omega_2;
    rc_matrix_t imu_var_o_1;
    rc_matrix_t imu_var_o_2;
    rc_vector_t imu_accel_1;
    rc_vector_t imu_accel_2;
    rc_matrix_t imu_var_a_1;
    rc_matrix_t imu_var_a_2;
    rc_vector_t imu_quat_1;
    rc_vector_t imu_quat_2;
    rc_matrix_t imu_var_t_1;
    rc_matrix_t imu_var_t_2;
} raw_state_t;

// a store for filtered data incoming from sensors and current control outputs
typedef struct {
    double pitot_time_1f;
} state_estimate_t;