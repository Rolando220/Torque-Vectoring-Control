#include "torque_vectoring.h"
#include <math.h>

// Number of points in our LUT
#define LUT_SIZE 30

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

void Kalman_Init(Kalman_State *kf, float radius) {
    kf->u_est = 0.0f;
    kf->bias = 0.0f;
    
    // Inizializzazione Matrice P (Identità)
    kf->P00 = 1.0f; kf->P01 = 0.0f;
    kf->P10 = 0.0f; kf->P11 = 1.0f;
    
    // Tuning del tuo script MATLAB corazzato
    kf->Q_vel = 0.05f;      
    kf->Q_bias = 0.0001f;   
    kf->R = 50.0f;  // Alzato come da tua telemetria       
    
    kf->wheel_radius = radius;
}

float Kalman_Update(Kalman_State *kf, float w_fl, float w_fr, float imu_ax, float dt) {
    if (dt <= 0.0f) return kf->u_est;

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

float get_max_regen_current(float SoC){

    if (SoC >= 90.0f) return 0.0f;       
    if (SoC >= 70.0f) return 50.0f;
    if (SoC >= 40.0f) return 100.0f;
    if (SoC >= 20.0f) return 150.0f;
    return 200.0f;                       

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

TV_Output TVC_Main(float throttle, float brake_pedal, float steering_wheel_angle, 
                   float raw_gyro_z, float imu_ax, float omega_left, float omega_right, 
                   float SoC, float dt, 
                   PID_State *pid, Kalman_State *kf, 
                   float *prev_yaw_ref_filtered, float *prev_gyro_filtered) {

    TV_Output out;
    float T_req_pilot = 0.0f;

    // =========================================================
    // 1. ELABORAZIONE SEGNALI E STIMA STATO
    // =========================================================
    
    // Stima della Velocità Longitudinale (Vx) tramite Filtro di Kalman
    float Vx = Kalman_Update(kf, omega_left, omega_right, imu_ax, dt);

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
        // Lift-off Regen: L'acceleratore è completamente rilasciato. 
        // 1. Leggiamo la corrente massima assorbibile dalla batteria in base al SoC
        float I_q_max = get_max_regen_current(SoC);
        
        // 2. Convertiamo gli Ampere in Coppia totale massima rigenerabile (per 2 motori)
        float T_regen_max = I_q_max * MOTOR_KT * GEAR_RATIO * 2.0f; 
        
        // 3. Applichiamo il 70% di questa coppia in frenata (segno negativo)
        T_req_pilot = - (T_regen_max * 0.70f);
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
    Wheel_Torques torques_final = ASC_Advanced(torques_allocated, omega_left, omega_right, Vx, yaw_rate_actual);

    // =========================================================
    // 5. ATTUAZIONE (Conversione in Correnti)
    // =========================================================
    
    out.currents.current_left = torques_final.T_left / (MOTOR_KT * GEAR_RATIO);
    out.currents.current_right = torques_final.T_right / (MOTOR_KT * GEAR_RATIO);

    return out;
}