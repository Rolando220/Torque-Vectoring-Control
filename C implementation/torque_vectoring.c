#include "torque_vectoring.h"

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

    float vx_safe = max(abs(Vx), 1.0f); // Prevent division by zero or very low speeds
    
    float r_linear = (Vx / (WHEELBASE * (1 + K_US * Vx * Vx))) * delta;

    float ay_max = MU * G_GRAVITY; // Maximum lateral acceleration based on friction

    float r_max = ay_max / vx_safe; // Maximum yaw rate based on maximum lateral acceleration
    int r_sign = (r_linear > 0.0f) - (r_linear < 0.0f); // Sign of the linear yaw rate

    if (abs(r_linear) > abs(r_max)) {
        return r_sign * r_max; // Limit the reference yaw rate to the maximum
    } else {
        return r_linear; // Return the linear reference yaw rate if it's within limits
    }
}

Wheel_Torques ASC(Wheel_Torques torques_in, float omega_left, float omega_right, float u){


    Wheel_Torques torques_out = torques_in; // Initialize output torques with input values
    float target_slip_abs = 0.11f; // Target slip ratio for maximum traction
    int Kp = 500; // Proportional gain for slip control

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
