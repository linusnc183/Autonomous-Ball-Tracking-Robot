#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

void Ultrasonic_Init(void);
void Ultrasonic_Update(void);          // blocking, ~30ms worst case
float Ultrasonic_GetDistanceCm(void);  // -1 if no reading yet

#endif
