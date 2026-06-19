#include "torque_vectoring.h"
#include <stdbool.h>

// Variabili "static" per mantenere la memoria tra un passo di simulazione e l'altro
static Kalman_State kf;
static PID_State pid;
static float prev_yaw_ref = 0.0f;
static float prev_gyro = 0.0f;
static bool is_initialized = false;

// Questa è la funzione che Simulink chiamerà
TV_Output TVC_Simulink_Wrapper(float throttle, float brake_pedal, float steering_wheel_angle, 
                               float raw_gyro_z, float imu_ax, float omega_FL, float omega_FR, float omega_RL, float omega_RR,
                               float SoC, float Temp_battery, float Temp_inverter, float V_batt, float dt){
    
    // Inizializzazione eseguita SOLO al primo istante (t=0)
    if (!is_initialized) {
        Kalman_Init(&kf, WHEEL_RADIUS);
        // NOTA: Inserisci qui i valori Kp e Kd che usavi nel tuo modello MATLAB originale!
        PID_Init(&pid, 2000.0f, 0.0f, 0.0f, dt); 
        is_initialized = true;
    }

    // Chiamata alla tua vera funzione Black Box
    return TVC_Main(throttle, brake_pedal, steering_wheel_angle, 
                    raw_gyro_z, imu_ax, omega_FL, omega_FR, omega_RL, omega_RR,
                    SoC, Temp_battery, Temp_inverter, V_batt, dt, 
                    &pid, &kf, &prev_yaw_ref, &prev_gyro);
}