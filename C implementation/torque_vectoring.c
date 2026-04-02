#include "torque_vectoring.h"
#include <math.h>

// Numero di punti nella nostra LUT
#define LUT_SIZE 30

// Array delle velocità (Vx in m/s) - Da 1 a 30 in ordine crescente
static const float Vx_LUT[LUT_SIZE] = {
    1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,  9.0f,  10.0f,
    11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f,
    21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f, 28.0f, 29.0f, 30.0f
};

// Array dei Ki calcolati da MATLAB (Gain Scheduling Analitico)
static const float Ki_LUT[LUT_SIZE] = {
    779061.4f, 388307.3f, 257523.4f, 191744.0f, 151979.2f, 
    125234.3f, 105941.2f, 91316.7f,  79814.6f,  70507.4f,
    62804.3f,  56311.3f,  50755.2f,  45940.5f,  41723.3f, 
    37995.6f,  34674.1f,  31694.1f,  29004.0f,  26562.3f,
    24335.5f,  22295.6f,  20419.6f,  18688.1f,  17084.8f, 
    15595.7f,  14208.8f,  12913.8f,  11701.9f,  10565.0f
};

void  PID_Init(PID_State *pid, float Kp, float Ki, float Kd, float dt){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_acc = 0.0f; // Initialize integral accumulator to zero
    pid->dt = dt;
}

float interpolate_KI(float Vx){
    // 1. Protection against out-of-bounds
    if (Vx <= Vx_LUT[0]) {
        return Ki_LUT[0];
    }
    if (Vx >= Vx_LUT[LUT_SIZE - 1]) {
        return Ki_LUT[LUT_SIZE - 1];
    }

    for (int i = 0; i < LUT_SIZE - 1; i++) {
        if (Vx >= Vx_LUT[i] && Vx <= Vx_LUT[i + 1]) {
            // Linear interpolation
            float slope = (Ki_LUT[i + 1] - Ki_LUT[i]) / (Vx_LUT[i + 1] - Vx_LUT[i]);
            return Ki_LUT[i] + slope * (Vx - Vx_LUT[i]);
        }
    }

    return Ki_LUT[0]; // Default return (should never reach here due to bounds check)
}

float LPF(float input, float* prev_output, float tau, float dt) {

    float alpha = dt / (tau + dt); // Calculate the filter coefficient
    
    float output = *prev_output + alpha * (input - *prev_output); // Apply the low-pass filter formula

    *prev_output = output; // Update the previous output for the next iteration
    return output; // Return the filtered output
}

float TV_PID(float yaw_rate_ref, float yaw_rate_actual, PID_State *pid) {
    // Calculate error
    float error = yaw_rate_ref - yaw_rate_actual;

    // Proportional term
    float P = pid->Kp * error;

    // Anti - wind up for integral term

    float Mz_test = P + pid->integral_acc + (pid->Ki * error * pid->dt);

    if ( (Mz_test >= MZ_MAX && error > 0.0f) || (Mz_test <= -MZ_MAX && error < 0.0f) ) {
        // Do not update integral accumulator to prevent wind-up
    } 
    else {
        // Update integral accumulator
        pid->integral_acc += pid->Ki * error * pid->dt;
    }

    // Derivative term

    float D = 0.0f; // Derivative term is set to zero

    // Mz control output

    float Mz_out = P + pid->integral_acc + D;

    if (Mz_out > MZ_MAX) { Mz_out = MZ_MAX; }
    if (Mz_out < -MZ_MAX) { Mz_out = -MZ_MAX; }

    return Mz_out;
}

Wheel_Torques torque_allocator(float T_req_pilot, float Mz_ctrl){

    Wheel_Torques torques_out;

    float deltaT = ( Mz_ctrl * WHEEL_RADIUS ) / TRACK_WIDTH;

    float T_L_raw = (T_req_pilot / 2.0f) - deltaT;
    float T_R_raw = (T_req_pilot / 2.0f) + deltaT;

    if ( T_L_raw > T_DRIVE_MAX ) { torques_out.T_left = T_DRIVE_MAX; }
    else if ( T_L_raw < -T_REGEN_MAX ) { torques_out.T_left = -T_REGEN_MAX; }
    else { torques_out.T_left = T_L_raw; }

    if ( T_R_raw > T_DRIVE_MAX ) { torques_out.T_right = T_DRIVE_MAX; }
    else if ( T_R_raw < -T_REGEN_MAX ) { torques_out.T_right = -T_REGEN_MAX; }
    else { torques_out.T_right = T_R_raw; }

    return torques_out;
}

float reference_generator(float Vx, float steering_wheel_angle){

    float delta_steering = steering_wheel_angle * STEER_RATIO; // Convert steering wheel angle to actual wheel steering angle
    float delta = steering_wheel_angle * STEER_RATIO; // Convert steering wheel angle to actual wheel steering angle

    float vx_safe = fmaxf(fabsf(Vx), 1.0f); // Prevent division by zero or very low speeds
    
    float r_linear = (Vx / (WHEELBASE * (1 + K_US * Vx * Vx))) * delta;

    float ay_max = MU * G_GRAVITY; // Maximum lateral acceleration based on friction

    float r_max = ay_max / vx_safe; // Maximum yaw rate based on maximum lateral acceleration
    int r_sign = (r_linear > 0.0f) - (r_linear < 0.0f); // Sign of the linear yaw rate

    if (fabsf(r_linear) > fabsf(r_max)) {
        return r_sign * r_max; // Limit the reference yaw rate to the maximum
    } else {
        return r_linear; // Return the linear reference yaw rate if it's within limits
    }
}

Wheel_Torques ASC(Wheel_Torques torques_in, float omega_left, float omega_right, float u){


    Wheel_Torques torques_out = torques_in; // Initialize output torques with input values
    float target_slip_abs = 0.11f; // Target slip ratio for maximum traction
    float Kp = 500.0f; // Proportional gain for slip control

    float slip_RL = 0.0f;
    float slip_RR = 0.0f;
    float target_slip = 0.0f;

    float V_wheel_RL = omega_left * WHEEL_RADIUS;

    if (fabsf(V_wheel_RL) < 0.1f){
        slip_RL = 0.0f; // Prevent division by zero at very low speeds
    }
    else {
        slip_RL = (u - V_wheel_RL) / V_wheel_RL; // Slip ratio for rear left wheel
    }


    if (fabsf(slip_RL) > target_slip_abs) {

        if (slip_RL < 0.0f){
            target_slip = -target_slip_abs;
        }
        else{
            target_slip = target_slip_abs;
        }

        float error_slip = target_slip - slip_RL; // Slip error
        float T_cut = Kp * error_slip; // Torque reduction based on slip error
        torques_out.T_left -= T_cut; // Reduce torque on rear left wheel

    }

    if (torques_out.T_left < -T_REGEN_MAX) { torques_out.T_left = -T_REGEN_MAX; } // Limit regenerative braking torque

    float V_wheel_RR = omega_right * WHEEL_RADIUS;

    if (fabsf(V_wheel_RR) < 0.1f){
        slip_RR = 0.0f; // Prevent division by zero at very low speeds
    }
    else {
        slip_RR = (u - V_wheel_RR) / V_wheel_RR; // Slip ratio for rear right wheel
    }

    if (fabsf(slip_RR) > target_slip_abs) {

        if (slip_RR < 0.0f){
            target_slip = -target_slip_abs;
        }
        else{
            target_slip = target_slip_abs;
        }

        float error_slip = target_slip - slip_RR; // Slip error
        float T_cut = Kp * error_slip; // Torque reduction based on slip error
        torques_out.T_right -= T_cut; // Reduce torque on rear right wheel

    }

    if (torques_out.T_right < -T_REGEN_MAX) { torques_out.T_right = -T_REGEN_MAX; } // Limit regenerative braking torque

    return torques_out;
}
