/*
 * Cascaded PID controller for quadcopter attitude stabilization.
 * Outer loop: Angle setpoint -> Target rotation rate.
 * Inner loop: Target rotation rate -> Motor mixer.
 */
#include "pid_cascade.h"

pid_t pid_roll_angle  = { .kp = 4.5f, .ki = 0.0f, .kd = 0.0f };
pid_t pid_pitch_angle = { .kp = 4.5f, .ki = 0.0f, .kd = 0.0f };

pid_t pid_roll_rate  = { .kp = 0.15f, .ki = 0.05f, .kd = 0.01f };
pid_t pid_pitch_rate = { .kp = 0.15f, .ki = 0.05f, .kd = 0.01f };
pid_t pid_yaw_rate   = { .kp = 0.45f, .ki = 0.10f, .kd = 0.00f };

float update_pid(pid_t* pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    
    // Anti-windup
    if (pid->integral > 400.0f) pid->integral = 400.0f;
    if (pid->integral < -400.0f) pid->integral = -400.0f;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

void compute_attitude_control(float roll_sp, float pitch_sp, float yaw_rate_sp, 
                              float curr_roll, float curr_pitch, 
                              float gyro_roll, float gyro_pitch, float gyro_yaw, 
                              float dt, float* motor_out) 
{
    // Outer loop (Angle)
    float target_roll_rate = update_pid(&pid_roll_angle, roll_sp, curr_roll, dt);
    float target_pitch_rate = update_pid(&pid_pitch_angle, pitch_sp, curr_pitch, dt);

    // Inner loop (Rate)
    float roll_output = update_pid(&pid_roll_rate, target_roll_rate, gyro_roll, dt);
    float pitch_output = update_pid(&pid_pitch_rate, target_pitch_rate, gyro_pitch, dt);
    float yaw_output = update_pid(&pid_yaw_rate, yaw_rate_sp, gyro_yaw, dt);

    motor_out[0] = 1000.0f + roll_output + pitch_output - yaw_output; // Front Left
    motor_out[1] = 1000.0f - roll_output + pitch_output + yaw_output; // Front Right
    motor_out[2] = 1000.0f + roll_output - pitch_output + yaw_output; // Rear Left
    motor_out[3] = 1000.0f - roll_output - pitch_output - yaw_output; // Rear Right
}
