#include "pid.h"

PIDController initPID(float kp, float ki, float kd)
{
    PIDController pid;
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.previous_error = 0.0f;
    pid.integral = 0.0f;
    return pid;
}

double computePID(PIDController *pid, float setpoint, float measured_value, float dt)
{
    double error = setpoint - measured_value;
    pid->integral += error * dt;
    double derivative = (error - pid->previous_error) / dt;

    double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    pid->previous_error = error;

    return output;
}
