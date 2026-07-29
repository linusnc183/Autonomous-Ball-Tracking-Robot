#ifndef PID_H
#define PID_H


typedef struct
{

float kp;
float ki;
float kd;


float integral;

float previous_error;


}PID_t;



void PID_Init(
PID_t *pid,
float kp,
float ki,
float kd);



float PID_Update(
PID_t *pid,
float target,
float actual,
float dt);



#endif
