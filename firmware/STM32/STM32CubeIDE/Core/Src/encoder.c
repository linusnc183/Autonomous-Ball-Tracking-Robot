#include "encoder.h"


extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;



static int16_t left_last=0;
static int16_t right_last=0;


static float left_speed=0;
static float right_speed=0;



void Encoder_Update(float dt)
{


int16_t left_now;
int16_t right_now;



left_now =
(int16_t)__HAL_TIM_GET_COUNTER(&htim4);



right_now =
(int16_t)__HAL_TIM_GET_COUNTER(&htim3);


/* Signed 16-bit subtraction correctly handles counter wraparound as
 * long as the true movement between calls is less than 32768 counts,
 * which is fine at a ~50Hz control loop rate. */
int16_t left_delta =
(int16_t)(left_now-left_last);



int16_t right_delta =
(int16_t)(right_now-right_last);



if(dt > 0.0001f)
{
    left_speed = (float)left_delta / dt;
    right_speed = (float)right_delta / dt;
}



left_last=left_now;

right_last=right_now;


}



float Encoder_GetLeftSpeed(void)
{
return left_speed;
}



float Encoder_GetRightSpeed(void)
{
return right_speed;
}
