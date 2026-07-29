#include "pid.h"


void PID_Init(
PID_t *pid,
float kp,
float ki,
float kd)
{

pid->kp=kp;
pid->ki=ki;
pid->kd=kd;

pid->integral=0;

pid->previous_error=0;

}


/* NOTE ON GAINS: kp/ki/kd here operate on an error measured in encoder
 * counts/sec and produce an output in PWM units (-1000..1000). The
 * kp=2.0, ki=0.1, kd=0.01 starting values in main.c are a reasonable
 * starting point but will very likely need tuning on the real robot -
 * increase kp until you get a fast response with a bit of oscillation,
 * then add just enough kd to damp it, then add ki only if there's
 * steady-state error left over. */
float PID_Update(
PID_t *pid,
float target,
float actual,
float dt)
{

/* Guard against dt==0 (would cause a divide-by-zero in the derivative
 * term) if this is ever called back-to-back with no time elapsed. */
if(dt < 0.0001f)
{
    dt = 0.0001f;
}


float error =
target-actual;



float derivative =
(error-pid->previous_error)/dt;


/* Compute what the integral *would* become this step, but don't commit
 * it yet - see anti-windup logic below. */
float integral_candidate =
pid->integral + error*dt;



float output =
pid->kp*error+
pid->ki*integral_candidate+
pid->kd*derivative;



/* Anti-windup (conditional integration): if the output is already
 * saturated and this step's error would push it further into
 * saturation, don't accumulate the integral term. Without this, the
 * integral can grow huge while the motor is already maxed out, causing
 * bad overshoot once the error finally reverses sign. */
if(output>1000)
{
    output=1000;
    if(error < 0)
    {
        pid->integral = integral_candidate;
    }
}
else if(output<-1000)
{
    output=-1000;
    if(error > 0)
    {
        pid->integral = integral_candidate;
    }
}
else
{
    pid->integral = integral_candidate;
}



pid->previous_error=error;



return output;


}
