#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

// Vehicle parameters

#define T_DRIVE_MAX     400.0f  // Maximum drive torque (Nm)
#define T_REGEN_MAX     400.0f  // Maximum regenerative torque (Nm)
#define TRACK_WIDTH     1.136f  // Track width vp.t2 (m)
#define WHEEL_RADIUS    0.2395f // Wheel radius vp.Rr (m)
#define WHEELBASE       1.527f  // Wheelbase (m)
#define STEER_RATIO     0.4273f // Steering ratio
#define MU              1.0f    // Coefficient of friction
#define G_GRAVITY       9.81f   // Gravitational acceleration (m/s^2)
#define VEHICLE_MASS    300.0f  // Vehicle mass (kg)
#define K_US            0.0008f // Understeering coefficient

// Control limits

#define MZ_MAX          1897.0f // Maximum yaw moment (Nm)

// IMU signal processing

#define GYRO_Z_BIAS       0.0f  // Gyro bias for yaw rate (deg/s)

// Powertrain costants

#define GEAR_RATIO       5.0f    // Gear ratio
#define MOTOR_KT         0.54f   // Motor torque constant (Nm/A)

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

typedef struct {
    Inverter_Currents currents;
    // Brake_Blending_Output brake_cmds;
} TV_Output;

// Struct for brake blending controller

// typedef struct {
//     float T_regen_TV;
//     float T_brake_front;
//     float T_brake_rear;
//     float mech_bias_requested;
// } Brake_Blending_Output;


// Function prototypes

void  PID_Init(PID_State *pid, float Kp, float Ki, float Kd, float dt);
float interpolate_KI(float Vx);
float LPF(float input, float* prev_output, float tau, float dt);
float TV_PID(float yaw_rate_ref, float yaw_rate_actual, PID_State *pid);
Wheel_Torques torque_allocator(float T_req_pilot, float Mz_ctrl);
float reference_generator(float Vx, float steering_wheel_angle);
Wheel_Targets get_wheel_targets(float u, float yaw_rate_actual);
float calculate_slip_factor(float v_wheel, float v_target, float slip_threshold_abs);
Wheel_Torques ASC_Advanced(Wheel_Torques torques_in, float omega_left, float omega_right, float u, float yaw_rate_actual);
float get_max_regen_current(float SoC);
// Brake_Blending_Output brake_blending(float brake_pedal_front, float bb_desired_pilot, float bb_actual, float SoC);
TV_Output TVC_Main(float throttle, float brake_pedal, float Vx, float steering_wheel_angle, float raw_gyro_z, float omega_left,
                        float omega_right, float dt, PID_State *pid, float *prev_yaw_ref_filtered, float *prev_gyro_filtered);

#endif