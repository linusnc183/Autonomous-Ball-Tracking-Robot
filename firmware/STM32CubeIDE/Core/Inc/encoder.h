#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

void Encoder_Update(float dt);      // dt in seconds
float Encoder_GetLeftSpeed(void);   // counts/sec
float Encoder_GetRightSpeed(void);

#endif
