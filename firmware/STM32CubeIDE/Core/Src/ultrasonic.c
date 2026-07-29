#include "ultrasonic.h"
#include "main.h"


/* -1 means "no valid reading yet" so callers can tell that apart from
 * a real distance of 0. */
static float last_distance_cm = -1.0f;


/* ECHO (PA1) is a plain GPIO pin, not on a timer input-capture channel,
 * so we time the pulse in software using the Cortex-M4 cycle counter
 * (DWT->CYCCNT), which gives microsecond-level resolution at 170MHz. */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


/* Converts an already-computed cycle *delta* into microseconds. The
 * subtraction that produces the delta must happen on raw CYCCNT values
 * BEFORE this division - subtracting two independently-divided
 * micros() readings would break at the ~25 second CYCCNT overflow point,
 * since integer division doesn't preserve modular wraparound the way
 * plain unsigned subtraction does. */
static uint32_t cycles_to_us(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000U);
}


void Ultrasonic_Init(void)
{
    DWT_Init();
}


void Ultrasonic_Update(void)
{
    uint32_t t0;

    /* 10us trigger pulse, per HC-SR04 datasheet */
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);
    t0 = DWT->CYCCNT;
    while (cycles_to_us(DWT->CYCCNT - t0) < 2) { }

    HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_SET);
    t0 = DWT->CYCCNT;
    while (cycles_to_us(DWT->CYCCNT - t0) < 10) { }
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_GPIO_Port, ULTRASONIC_TRIG_Pin, GPIO_PIN_RESET);

    /* Wait for ECHO to go high (start of return pulse). Timeout ~30ms
     * covers "nothing in range" so we never hang here forever. */
    uint32_t wait_start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_RESET)
    {
        if (cycles_to_us(DWT->CYCCNT - wait_start) > 30000)
        {
            return; /* no echo - keep the last known-good reading */
        }
    }

    uint32_t echo_start = DWT->CYCCNT;

    /* Wait for ECHO to go low again (end of return pulse) */
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_GPIO_Port, ULTRASONIC_ECHO_Pin) == GPIO_PIN_SET)
    {
        if (cycles_to_us(DWT->CYCCNT - echo_start) > 30000)
        {
            return;
        }
    }

    uint32_t pulse_us = cycles_to_us(DWT->CYCCNT - echo_start);

    /* Speed of sound ~343 m/s at room temp -> ~58.2us round-trip per cm */
    last_distance_cm = (float)pulse_us / 58.2f;
}


float Ultrasonic_GetDistanceCm(void)
{
    return last_distance_cm;
}
