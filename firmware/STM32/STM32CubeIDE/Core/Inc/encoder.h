#ifndef ENCODER_H
#define ENCODER_H


#include "main.h"


/* dt = seconds elapsed since the last call, used to convert the raw
 * counter delta into counts/sec so speed is meaningful even if the
 * calling loop's timing isn't perfectly constant. */
void Encoder_Update(float dt);


float Encoder_GetLeftSpeed(void);   /* counts per second */

float Encoder_GetRightSpeed(void);  /* counts per second */


#endif
