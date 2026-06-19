#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

// Vehicle parameters

#define T_DRIVE_MAX     400.0f  // Maximum drive torque (Nm)
#define T_REGEN_MAX     400.0f  // Maximum regenerative torque (Nm)
#define TRACK_WIDTH     1.136f  // Track width vp.t2 (m)
#define WHEEL_RADIUS    0.2395f // Wheel radius vp.Rr (m)
#define WHEELBASE       1.527f  // Wheelbase (m)
#define STEER_RATIO     0.4273f // Steering ratio
#define MU              1.6f    // Coefficient of friction
#define G_GRAVITY       9.81f   // Gravitational acceleration (m/s^2)
#define VEHICLE_MASS    300.0f  // Vehicle mass (kg)
#define K_US            -0.0008f // Understeering coefficient

// Control limits

#define MZ_MAX          1897.0f // Maximum yaw moment (Nm)

// IMU signal processing

#define GYRO_Z_BIAS       0.0f  // Gyro bias for yaw rate (deg/s)

// Powertrain costants

#define GEAR_RATIO          5.0f    // Gear ratio
#define MOTOR_KT            0.54f   // Motor torque constant (Nm/A)
#define NUM_MOTORS          2
#define CELLS_IN_SERIES     144
#define CELLS_IN_PARALLEL   3
#define R_PACK_WELDS        0.005f  // Estimated total welding resistance [Ohms]
#define V_BATT_MAX          600.0f  // Maximum battery voltage [V]
#define TEMP_MAX_INVERTER   85.0f   // Maximum inverter temperature [C]
#define IQ_MAX_INVERTER     150.0f  // Hardware maximum quadrature current [A]
#define IQ_MAX_PILOT        25.0f   // Maximum regen current allowed by pilot [A]
#define GAIN_REGEN          0.70f   // Regen multiplier (70% of max capability)
#define LOW_SPEED_THRS      15.0f   // Low speed threshold [rad/s] (approx 140 rpm)

// Powertrain Efficiencies
#define ETA_INVERTER        0.95f
#define ETA_MOTOR           0.92f
#define ETA_TRANSMISSION    0.98f
#define ETA_TOTAL           (ETA_INVERTER * ETA_MOTOR * ETA_TRANSMISSION)

// Braking costants

#define IDEAL_FRONT_BIAS       0.6f    // 60 Front - 40 Rear
#define BRAKE_DEADZONE         0.05f   // Minimum brake pedal input to avoid noise
#define K_BRAKE                107700.0f // Gain to convert brake pedal input to torque

// Throttle costants

#define K_THROTTLE             400.0f // Gain to convert throttle input to total torque request

// Control structs

typedef struct {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain
    float integral_acc; // Integral accumulator
    float dt; // Sample time (s)
} PID_State;

typedef struct {
    float T_left;   // Torque command for left wheel (Nm)
    float T_right;  // Torque command for right wheel (Nm)
} Wheel_Torques;

// Struct for ASC wheel velocities targets

typedef struct {
    float v_target_L;
    float v_target_R;
} Wheel_Targets;

typedef struct {
    float current_left;   // Current command for left wheel (A)
    float current_right;  // Current command for right wheel (A)
} Inverter_Currents;

// Struttura di telemetria per il debugging in Simulink
typedef struct {
    float Vx_kf;              // Velocità longitudinale stimata dal Kalman
    float yaw_rate_ref;       // Riferimento desiderato (post-filtro)
    float yaw_rate_actual;    // Imbardata reale letta e filtrata dal giroscopio
    float yaw_rate_error;     // L'errore in pasto al PID
    float Mz_ctrl_pid;        // L'uscita del PID
    float fade_multiplier;    // Lo stato di accensione del TVC (0.0 a 1.0)
    float Ki_current;         // Il guadagno integrale che sta usando in quell'istante
    float T_req_pilot;        // La richiesta di coppia (per capire se entra in regen)
} TV_Debug_Telemetry;

typedef struct {
    Inverter_Currents currents;
    TV_Debug_Telemetry debug;
    // Brake_Blending_Output brake_cmds;
} TV_Output;

// Struct for brake blending controller

// typedef struct {
//     float T_regen_TV;
//     float T_brake_front;
//     float T_brake_rear;
//     float mech_bias_requested;
// } Brake_Blending_Output;

// Struttura per lo stato del Filtro di Kalman (2D)
typedef struct {
    float u_est;        // Stato 1: Velocità stimata (m/s)
    float bias;         // Stato 2: Bias stimato (m/s^2)
    
    // Matrice di covarianza dell'errore P (2x2)
    float P00, P01, P10, P11; 
    
    // Parametri di tuning
    float Q_vel;        // Rumore processo velocità
    float Q_bias;       // Rumore processo bias
    float R;            // Rumore misura ruote
    
    float wheel_radius;
} Kalman_State;


// Function prototypes


void Kalman_Init(Kalman_State *kf, float radius);
float Kalman_Update(Kalman_State *kf, float w_fl, float w_fr, float imu_ax, float dt);

void  PID_Init(PID_State *pid, float Kp, float Ki, float Kd, float dt);
float interpolate_KI(float Vx);
float LPF(float input, float* prev_output, float tau, float dt);
float TV_PID(float yaw_rate_ref, float yaw_rate_actual, PID_State *pid);
Wheel_Torques torque_allocator(float T_req_pilot, float Mz_ctrl);
float reference_generator(float Vx, float steering_wheel_angle);
Wheel_Targets get_wheel_targets(float u, float yaw_rate_actual);
float calculate_slip_factor(float v_wheel, float v_target, float slip_threshold_abs);
Wheel_Torques ASC_Advanced(Wheel_Torques torques_in, float omega_left, float omega_right, float u, float yaw_rate_actual);
// Brake_Blending_Output brake_blending(float brake_pedal_front, float bb_desired_pilot, float bb_actual, float SoC);

// Iq_max functions for regen

float clamp(float value, float min, float max);
float interpolate_cells_power(float current_SoC, float current_temp);
float get_max_regen_current(float SoC, float Temp_battery, float Temp_inverter, float omega_motor_rads, float V_batt, float Kt);


// TVC Main function

TV_Output TVC_Main(float throttle, float brake_pedal, float steering_wheel_angle, 
                   float raw_gyro_z, float imu_ax, float omega_FL, float omega_FR, float omega_RL,
                   float omega_RR, float SoC, float Temp_battery, float Temp_inverter, float V_batt, 
                   float dt, PID_State *pid, Kalman_State *kf, float *prev_yaw_ref_filtered, 
                   float *prev_gyro_filtered);
                   

// Simulink Wrapper Prototype
TV_Output TVC_Simulink_Wrapper(float throttle, float brake_pedal, float steering_wheel_angle, 
                               float raw_gyro_z, float imu_ax, float omega_FL, float omega_FR, float omega_RL, float omega_RR,
                               float SoC, float Temp_battery, float Temp_inverter, float V_batt, float dt);

#endif