#include "torque_vectoring.h"
#include <math.h>

// Number of points in our LUT
#define LUT_SIZE 30

// Number of points in Battery MAP

#define TEMP_POINTS 11
#define SOC_POINTS 21

// Speed array (Vx in m/s) - From 1 to 30 in ascending order
static const float Vx_LUT[LUT_SIZE] = {
    1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f,  9.0f,  10.0f,
    11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 19.0f, 20.0f,
    21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f, 28.0f, 29.0f, 30.0f
};

// Ki array calculated by MATLAB (analytic gain scheduling)
static const float Ki_LUT[LUT_SIZE] = {
    779061.4f, 388307.3f, 257523.4f, 191744.0f, 151979.2f, 
    125234.3f, 105941.2f, 91316.7f,  79814.6f,  70507.4f,
    62804.3f,  56311.3f,  50755.2f,  45940.5f,  41723.3f, 
    37995.6f,  34674.1f,  31694.1f,  29004.0f,  26562.3f,
    24335.5f,  22295.6f,  20419.6f,  18688.1f,  17084.8f, 
    15595.7f,  14208.8f,  12913.8f,  11701.9f,  10565.0f
};

// Battery power map (kW) for 1 cell, indexed by [TEMP][SOC]

static const float Temp_Axis[TEMP_POINTS] = {
    10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 35.0f, 40.0f, 45.0f, 50.0f, 55.0f, 60.0f
};

static const float SoC_Axis[SOC_POINTS] = {
    100.0f, 95.0f, 90.0f, 85.0f, 80.0f, 75.0f, 70.0f, 65.0f, 60.0f, 55.0f, 50.0f,
    45.0f, 40.0f, 35.0f, 30.0f, 25.0f, 20.0f, 15.0f, 10.0f, 5.0f, 0.0f
};

static const float Cell_Power_Map[TEMP_POINTS][SOC_POINTS] = {

    // SoC 100% .. 0%
    // Temp 10°C .. 60°C on y axis
    {0.0f, 1.597450948f, 3.183515958f, 4.765967328f, 9.472614403f, 18.66921504f, 32.02727709f, 43.76692041f, 42.44803845f, 62.08892152f, 68.2734455f, 103.9493149f, 104.8685861f, 115.6951388f, 117.7027441f, 125.6575222f, 135.4996279f, 157.2712114f, 148.704496f, 131.3154471f, 116.1417737f},
    {0.0f, 1.597747378f, 3.97975977f, 6.355462621f, 11.05707699f, 21.03159216f, 37.40867148f, 49.84862937f, 48.92721426f, 60.65909561f, 82.18709357f, 100.5002075f, 109.8987964f, 114.453922f, 117.2204722f, 127.9354334f, 140.4357142f, 161.550676f, 156.1017461f, 147.9166633f, 135.7051686f},
    {0.0f, 1.598043807f, 4.776003583f, 7.944957915f, 12.64153959f, 23.39396928f, 42.79006588f, 55.93033833f, 55.40639008f, 59.2292697f, 96.10074163f, 97.05110018f, 114.9290068f, 113.2127053f, 116.7382004f, 130.2133447f, 145.3718004f, 165.8301406f, 163.4989961f, 164.5178795f, 155.2685635f},
    {0.0f, 1.598589397f, 7.166191612f, 10.33028909f, 16.60421004f, 31.22616819f, 58.13938915f, 74.90976221f, 80.23748206f, 90.49519516f, 121.762285f, 126.553698f, 144.6165716f, 138.9945878f, 144.9018732f, 156.5373975f, 167.1755145f, 181.4096934f, 176.8465172f, 168.6104985f, 154.7911699f},
    {0.0f, 1.599134986f, 9.556379641f, 12.71562026f, 20.56688048f, 39.05836709f, 73.48871242f, 93.88918608f, 105.068574f, 121.7611206f, 147.4238285f, 156.0562958f, 174.3041365f, 164.7764702f, 173.0655461f, 182.8614503f, 188.9792285f, 196.9892461f, 190.1940382f, 172.7031175f, 154.3137763f},
    {0.0f, 1.599858198f, 11.15603898f, 15.90331262f, 26.91455055f, 49.26858906f, 102.6994126f, 131.9156097f, 142.8104163f, 180.6320935f, 203.2523926f, 214.4783177f, 177.7426512f, 172.5186823f, 184.6127239f, 202.4017795f, 211.6044126f, 258.0427413f, 332.8924357f, 380.195229f, 378.0864022f},
    {0.0f, 1.60058141f, 12.75569833f, 19.09100499f, 33.26222062f, 59.47881103f, 131.9101129f, 169.9420332f, 180.5522585f, 239.5030663f, 259.0809567f, 272.9003395f, 181.181166f, 180.2608943f, 196.1599017f, 221.9421088f, 234.2295967f, 319.0962365f, 475.5908331f, 587.6873406f, 601.8590281f},
    {0.0f, 1.600290705f, 12.76489125f, 20.69624955f, 45.18005182f, 71.30097552f, 199.2418388f, 289.8047709f, 224.5046406f, 299.431233f, 469.8683836f, 453.6606286f, 418.4214498f, 240.6475594f, 263.1654988f, 323.8398984f, 417.9650672f, 462.0853535f, 536.9023705f, 583.6658982f, 597.0847731f},
    {0.0f, 1.600290705f, 12.77408417f, 22.30149412f, 57.09788302f, 83.12314f, 266.5735648f, 409.6675085f, 268.4570226f, 359.3593997f, 680.6558104f, 634.4209177f, 655.6617335f, 301.0342245f, 330.1710959f, 425.737688f, 601.7005377f, 605.0744705f, 598.2139078f, 579.6444557f, 592.3105182f},
    {0.0f, 1.600290705f, 12.78327709f, 23.90673869f, 68.91571421f, 95.04531f, 333.9052908f, 489.5302464f, 312.4094046f, 419.2517768f, 781.4422818f, 738.1865655f, 776.9025593f, 361.4208896f, 396.257761f, 491.8243531f, 687.7872028f, 690.5611356f, 683.7005729f, 664.5311208f, 677.1971833f},
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

void Kalman_Init(Kalman_State *kf, float radius) {
    kf->u_est = 0.0f;
    kf->bias = 0.0f;
    
    // Inizializzazione Matrice P (Identità)
    kf->P00 = 1.0f; kf->P01 = 0.0f;
    kf->P10 = 0.0f; kf->P11 = 1.0f;
    
    // Tuning del tuo script MATLAB corazzato
    kf->Q_vel = 0.01f;      
    kf->Q_bias = 0.0001f;   
    kf->R = 0.1f;  // Alzato come da tua telemetria   
    
    kf->wheel_radius = radius;
}

float Kalman_Update(Kalman_State *kf, float w_fl, float w_fr, float imu_ax, float dt) {
    if (dt <= 0.0f) return kf->u_est;

    // INIZIALIZZAZIONE SMART: Se il filtro è a zero ma le ruote girano forte, allinea la stima
    // if (kf->u_est == 0.0f && (fabsf(w_fl) > 1.0f || fabsf(w_fr) > 1.0f)) {
    //     kf->u_est = ((w_fl + w_fr) / 2.0f) * kf->wheel_radius;
    // }

    // ==========================================
    // 1. PREDIZIONE (Modello Cinematico con Bias)
    // ==========================================
    // X_pred = A * X + B * u (Sottraiamo il bias dall'accelerazione)
    kf->u_est += (imu_ax - kf->bias) * dt;
    // kf->bias rimane invariato nella predizione (derivata nulla)

    // Aggiornamento Matrice di Covarianza P_pred = A * P * A^T + Q
    // (Srotolamento algebrico della moltiplicazione di matrici 2x2)
    float P00_pred = kf->P00 - dt * (kf->P10 + kf->P01) + (dt * dt * kf->P11) + kf->Q_vel;
    float P01_pred = kf->P01 - dt * kf->P11;
    float P10_pred = kf->P10 - dt * kf->P11;
    float P11_pred = kf->P11 + kf->Q_bias;

    // ==========================================
    // 2. ELABORAZIONE MISURA E SLIP REJECTION
    // ==========================================
    float v_fl = w_fl * kf->wheel_radius;
    float v_fr = w_fr * kf->wheel_radius;
    float v_meas;

    // Logica Smart Average: scarta un sensore se sballa rispetto all'altro
    if (fabsf(v_fl - v_fr) > 2.0f) {
        if (fabsf(v_fl - kf->u_est) < fabsf(v_fr - kf->u_est)) {
            v_meas = v_fl;
        } else {
            v_meas = v_fr;
        }
    } else {
        v_meas = (v_fl + v_fr) / 2.0f;
    }

    // Slip Rejection (Se la ruota sana comunque slitta/blocca, ignorala)
    float current_R = kf->R;
    if (fabsf(v_meas - kf->u_est) > 2.0f) {
        current_R *= 50.0f; 
    }

    // ==========================================
    // 3. AGGIORNAMENTO (Kalman Gain)
    // ==========================================
    // Innovazione (Errore di misura)
    float y = v_meas - kf->u_est;

    // Fattore di scala (S = H * P_pred * H^T + R)
    float S = P00_pred + current_R;

    // Kalman Gain (K = P_pred * H^T * 1/S)
    float K0 = P00_pred / S; // Gain per la velocità
    float K1 = P10_pred / S; // Gain per il bias

    // Correzione degli stati
    kf->u_est += K0 * y;
    kf->bias  += K1 * y;

    // Aggiornamento Matrice P (P = (I - K*H) * P_pred)
    kf->P00 = P00_pred - K0 * P00_pred;
    kf->P01 = P01_pred - K0 * P01_pred;
    kf->P10 = P10_pred - K1 * P00_pred;
    kf->P11 = P11_pred - K1 * P01_pred;

    return kf->u_est;
}

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

Wheel_Targets get_wheel_targets(float u, float yaw_rate_actual) {
    Wheel_Targets targets;
    if (fabsf(u) < 1.0f) {
        targets.v_target_L = u;
        targets.v_target_R = u;
        return targets;
    }
    float delta_v = yaw_rate_actual * (TRACK_WIDTH / 2.0f);
    targets.v_target_L = u - delta_v;
    targets.v_target_R = u + delta_v;
    return targets;
}

float calculate_slip_factor(float v_wheel, float v_target, float slip_threshold_abs) {
    
    // Protection against division by zero at very low wheel speeds
    if (fabsf(v_wheel) < 0.1f) {
        return 0.0f;
    }

    // Negative slip -> acceleration
    float slip = (v_target - v_wheel) / v_wheel;

    // We control wheel spin (the wheel spins faster than the vehicle -> slip is negative)
    if (slip < -slip_threshold_abs) {
        
        // Compute the excess (e.g. threshold 0.11, current slip -0.20 -> excess = 0.09)
        float excess_slip = fabsf(slip) - slip_threshold_abs;
        
        // Safety saturation to never reverse the torque
        if (excess_slip > 1.0f) {
            excess_slip = 1.0f;
        }
        
        return excess_slip; // Percentage torque cut factor
    }

    return 0.0f; // No cut if there is no wheel spin
}

Wheel_Torques ASC_Advanced(Wheel_Torques torques_in, float omega_left, float omega_right, float u, float yaw_rate_actual){


    Wheel_Torques torques_out = torques_in;
    
    if (u < 1.0f) return torques_out; // No ASC if the vehicle is almost stopped

    float slip_threshold = 0.11f; 

    Wheel_Targets targets = get_wheel_targets(u, yaw_rate_actual);

    // Left wheel (RL)
    float v_wheel_L = omega_left * WHEEL_RADIUS;
    float slip_factor_L = calculate_slip_factor(v_wheel_L, targets.v_target_L, slip_threshold);
    
    if (torques_out.T_left > 0.0f) {
        torques_out.T_left *= (1.0f - slip_factor_L);
    }

    // Right wheel (RR)
    float v_wheel_R = omega_right * WHEEL_RADIUS;
    float slip_factor_R = calculate_slip_factor(v_wheel_R, targets.v_target_R, slip_threshold);
    
    if (torques_out.T_right > 0.0f) {
        torques_out.T_right *= (1.0f - slip_factor_R);
    }

    return torques_out;
}

// Brake_Blending_Output brake_blending(float brake_pedal_front, float bb_desired_pilot, float bb_actual, float SoC){

//     Brake_Blending_Output out = {0.0f, 0.0f, 0.0f, bb_desired_pilot};

//     if(brake_pedal_front <= BRAKE_DEADZONE){
//         return out;                                 // No braking if pedal is in the deadzone
//     }

//     // Calculate mechanical actual braking torques

//     float T_mech_front = - (brake_pedal_front * K_BRAKE); // Mechanical braking torque from front brake pedal input

//     float T_mech_total = T_mech_front / bb_actual; // Total mechanical braking torque based on actual bias
//     float T_mech_rear_actual = T_mech_total - T_mech_front; // Mechanical braking torque needed at the rear to achieve the actual bias

//     // Calculate desired braking torques based on the desired bias by the pilot

//     float T_total_target = T_mech_total / bb_desired_pilot; // Total braking torque needed to achieve the desired bias
//     float T_rear_target = T_total_target - T_mech_front; // Desired rear braking torque to achieve the desired bias

//     // Maximum regenerative braking torque based on SoC

//     float I_q_max = get_max_regen_current(SoC);
//     float T_regen_max = I_q_max * MOTOR_KT * GEAR_RATIO * 2.0f; // Convert max regen current to max regen torque

//     // Actual regenerative braking torque needed to achieve the desired bias

//     float T_regen_needed = T_rear_target - T_mech_rear_actual; 

//     if (T_regen_needed > 0.0f) {
//         T_regen_needed = 0.0f;
//     }

//     // Calculate actual regenerative braking torque (limited by max regen torque)

//     out.T_regen_TV = fmaxf(T_regen_max, T_regen_needed);

//     // Calculate mechanical braking

//     float T_mech_rear_target = T_rear_target - T_regen_max;
    
//     if (T_mech_rear_target > 0.0f) { 
//         T_mech_rear_target = 0.0f; // I freni meccanici non possono spingere
//     }

//     float total_mech_target = T_mech_front + T_mech_rear_target;

//     if (total_mech_target < -1.0f) { // Protezione divisione per zero
//         out.mech_bias_requested = T_mech_front / total_mech_target;
//     } else {
//         out.mech_bias_requested = bb_desired_pilot;
//     }

//     // Limiti fisici di sicurezza per la richiesta al motorino meccanico
//     if (out.mech_bias_requested > 0.80f) out.mech_bias_requested = 0.84f;
//     if (out.mech_bias_requested < 0.40f) out.mech_bias_requested = 0.60f;

//     // Output per logging o altri calcoli
//     out.T_brake_front = T_mech_front;
//     out.T_brake_rear = T_mech_rear_actual;

//     return out;

// }

float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float interpolate_cells_power(float current_SoC, float current_temp){

    current_SoC = clamp(current_SoC, SoC_Axis[0], SoC_Axis[SOC_POINTS - 1]);
    current_temp = clamp(current_temp, Temp_Axis[0], Temp_Axis[TEMP_POINTS - 1]);

    // Find the indices for SoC 

    int i = 0;
    while (i < SOC_POINTS - 2 && current_SoC >= SoC_Axis[i + 1]) {
        i++;
    }

    // Find the indices for Temperature
    int j = 0;
    while (j < TEMP_POINTS - 2 && current_temp >= Temp_Axis[j + 1]) {
        j++;
    }

    float tx = (current_SoC - SoC_Axis[i]) / (SoC_Axis[i + 1] - SoC_Axis[i]);
    float ty = (current_temp - Temp_Axis[j]) / (Temp_Axis[j + 1] - Temp_Axis[j]);

    // Bilinear interpolation

    float Q11 = Cell_Power_Map[j][i];       
    float Q21 = Cell_Power_Map[j][i + 1];   
    float Q12 = Cell_Power_Map[j + 1][i];   
    float Q22 = Cell_Power_Map[j + 1][i + 1];

    float R1 = Q11 * (1.0f - tx) + Q21 * tx;
    float R2 = Q12 * (1.0f - tx) + Q22 * tx;

    float final_P_cell = R1 * (1.0f - ty) + R2 * ty;

    if (final_P_cell < 0.0f) return 0.0f; 
    
    return final_P_cell;

}

float get_max_regen_current(float SoC, float Temp_battery, float Temp_inverter, float omega_motor_rads, float V_batt, float Kt) {

    if ((V_batt > (V_BATT_MAX - 5.0f)) || (omega_motor_rads < 0.0f) || (Temp_inverter >= TEMP_MAX_INVERTER)) {
        return 0.0f;
    }

    float P_cell_max_W = interpolate_cells_power(SoC, Temp_battery);

    if (P_cell_max_W <= 0.1f) { 
        return 0.0f;
    }

    float P_pack_ideal = P_cell_max_W * CELLS_IN_SERIES * CELLS_IN_PARALLEL;

    float V_batt_safe = (V_batt > 1.0f) ? V_batt : 1.0f; // Prevent division by zero
    float I_batt_est = P_pack_ideal / V_batt_safe;

    float P_loss_welds = R_PACK_WELDS * (I_batt_est * I_batt_est);

    float P_pack_net_per_motor = (P_pack_ideal - P_loss_welds) / NUM_MOTORS;

    float Iq_target = 0.0f;

    // Convert mechanical power constraint into electrical current (Iq) constraint
    if (fabsf(omega_motor_rads) > LOW_SPEED_THRS) {
        Iq_target = P_pack_net_per_motor / (Kt * fabsf(omega_motor_rads) * ETA_TOTAL);
    } else {
        // Allow high current at very low speeds where back-EMF is minimal
        Iq_target = IQ_MAX_INVERTER - 10.0f;
    }

    Iq_target *= GAIN_REGEN;

    if (Iq_target > IQ_MAX_INVERTER) Iq_target = IQ_MAX_INVERTER;
    if (Iq_target > IQ_MAX_PILOT) Iq_target = IQ_MAX_PILOT;

    return -fabsf(Iq_target);
}

TV_Output TVC_Main(float throttle, float brake_pedal, float steering_wheel_angle, 
                   float raw_gyro_z, float imu_ax, float omega_FL, float omega_FR, float omega_RL,
                   float omega_RR, float SoC, float Temp_battery, float Temp_inverter, float V_batt, 
                   float dt, PID_State *pid, Kalman_State *kf, float *prev_yaw_ref_filtered, 
                   float *prev_gyro_filtered) {

    TV_Output out;
    float T_req_pilot = 0.0f;

    // =========================================================
    // 1. ELABORAZIONE SEGNALI E STIMA STATO
    // =========================================================
    
    // Stima della Velocità Longitudinale (Vx) tramite Filtro di Kalman
    float Vx = Kalman_Update(kf, omega_FL, omega_FR, imu_ax, dt);

    // Filtraggio e correzione dello Yaw Rate dall'IMU
    float tau_gyro = 0.01f; 
    float gyro_z_corrected = raw_gyro_z - GYRO_Z_BIAS; 
    float yaw_rate_actual = LPF(gyro_z_corrected, prev_gyro_filtered, tau_gyro, dt);

    // =========================================================
    // 2. DINAMICA LONGITUDINALE (Trazione & Lift-off Regen)
    // =========================================================
    
    // Il Brake Blending è temporaneamente in pausa e permettiamo la 
    // sovrapposizione dei pedali (nessun taglio di sicurezza sul freno).
    
    if (throttle > 0.01f) {
        // Trazione normale: dipendente ESCLUSIVAMENTE dall'acceleratore
        T_req_pilot = throttle * K_THROTTLE; 
    } 
    else {
        float omega_wheels_avg = (omega_RL + omega_RR) / 2.0f;
        float omega_motor_rads = omega_wheels_avg * GEAR_RATIO;

        float I_q_max = get_max_regen_current(SoC, Temp_battery, Temp_inverter, omega_motor_rads, V_batt, MOTOR_KT);

        float T_regen_max = I_q_max * MOTOR_KT * GEAR_RATIO * NUM_MOTORS;

        T_req_pilot = T_regen_max;
    }

    // =========================================================
    // 3. DINAMICA LATERALE (Torque Vectoring)
    // =========================================================
    
    // Generazione riferimento di imbardata e filtraggio
    float raw_yaw_ref = reference_generator(Vx, steering_wheel_angle);
    float tau_ref = 0.02f;
    float yaw_rate_ref = LPF(raw_yaw_ref, prev_yaw_ref_filtered, tau_ref, dt);

    // Gain scheduling
    pid->Ki = interpolate_KI(Vx);

    // Fade-out a basse velocità
    float fade_multiplier = 1.0f;
    if (Vx < 3.0f) {
        fade_multiplier = 0.0f;
        pid->integral_acc = 0.0f;
    } else if (Vx < 5.0f) {
        fade_multiplier = (Vx - 3.0f) / (5.0f - 3.0f);
    }

    // Calcolo momento di imbardata correttivo
    float Mz_ctrl = TV_PID(yaw_rate_ref, yaw_rate_actual, pid);
    Mz_ctrl *= fade_multiplier;

    // =========================================================
    // 4. ALLOCAZIONE E CONTROLLO DI TRAZIONE (ASC)
    // =========================================================
    
    // Suddivisione della coppia totale tra le due ruote
    Wheel_Torques torques_allocated = torque_allocator(T_req_pilot, Mz_ctrl);

    // Taglio della coppia in caso di slittamento
    Wheel_Torques torques_final = ASC_Advanced(torques_allocated, omega_RL, omega_RR, Vx, yaw_rate_actual);

    // Wheel_Torques torques_final = torques_allocated;

    // =========================================================
    // 5. ATTUAZIONE (Conversione in Correnti)
    // =========================================================
    
    out.currents.current_left = torques_final.T_left / (MOTOR_KT * GEAR_RATIO);
    out.currents.current_right = torques_final.T_right / (MOTOR_KT * GEAR_RATIO);

    // =========================================================
    // 6. TELEMETRIA DI DEBUG
    // =========================================================
    out.debug.Vx_kf = Vx;
    out.debug.yaw_rate_ref = yaw_rate_ref;
    out.debug.yaw_rate_actual = yaw_rate_actual;
    out.debug.yaw_rate_error = yaw_rate_ref - yaw_rate_actual;
    out.debug.Mz_ctrl_pid = Mz_ctrl;
    out.debug.fade_multiplier = fade_multiplier;
    out.debug.Ki_current = pid->Ki;
    out.debug.T_req_pilot = T_req_pilot;

    return out;
}