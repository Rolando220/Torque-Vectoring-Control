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

#define MZ_MAX          1897.0f  // Maximum yaw moment (Nm)

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

// Function prototypes

void  PID_Init(PID_State *pid, float Kp, float Ki, float Kd, float dt);
float interpolate_KI(float Vx);
float TV_PID(float yaw_rate_ref, float yaw_rate_actual, PID_State *pid);
Wheel_Torques torque_allocator(float T_req_pilot, float Mz_ctrl);
float reference_generator(float Vx, float steering_wheel_angle);
Wheel_Torques ASC(Wheel_Torques torques_in, float omega_left, float omega_right, float u);

#endif