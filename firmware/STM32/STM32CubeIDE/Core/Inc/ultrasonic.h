#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#include <stdint.h>


/* Call once at startup (enables the cycle counter used for
 * microsecond-accurate pulse timing). */
void Ultrasonic_Init(void);


/* Blocking: triggers the HC-SR04 and waits for the echo, up to ~30ms
 * worst case. Call this from a slower loop (e.g. 10Hz), not the same
 * tight loop as your motor PID, or it will disturb your control dt. */
void Ultrasonic_Update(void);


/* Returns the most recent successful distance reading in cm, or -1.0f
 * if no valid echo has been received yet / the last one timed out. */
float Ultrasonic_GetDistanceCm(void);


#endif
