#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0;
  pid->previous_error = 0;
}

float PID_Update(PID_t *pid, float target, float actual, float dt)
{
  if (dt < 0.0001f) dt = 0.0001f;

  float error = target - actual;
  float derivative = (error - pid->previous_error) / dt;
  float integral_candidate = pid->integral + error * dt;

  float output = pid->kp * error
               + pid->ki * integral_candidate
               + pid->kd * derivative;

  // anti-windup: only accumulate integral if not driving further into saturation
  if (output > 1000)
  {
    output = 1000;
    if (error < 0) pid->integral = integral_candidate;
  }
  else if (output < -1000)
  {
    output = -1000;
    if (error > 0) pid->integral = integral_candidate;
  }
  else
  {
    pid->integral = integral_candidate;
  }

  pid->previous_error = error;
  return output;
}
