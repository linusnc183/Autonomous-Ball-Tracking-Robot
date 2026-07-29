/*
 * Motor Driver Test
 *
 * Tests:
 * - TB6612FNG enable
 * - Direction pins
 * - PWM output
 */


#include "main.h"
#include "motor.h"


void Motor_Test(void)
{

    /*
     * Enable motor driver
     */

    Motor_Enable();


    /*
     * Forward
     */

    Motor_Left(500);
    Motor_Right(500);

    HAL_Delay(2000);



    /*
     * Stop
     */

    Motor_Left(0);
    Motor_Right(0);

    HAL_Delay(1000);



    /*
     * Reverse
     */

    Motor_Left(-500);
    Motor_Right(-500);

    HAL_Delay(2000);



    /*
     * Stop
     */

    Motor_Left(0);
    Motor_Right(0);

}