/*
 * Encoder Test
 *
 * Prints wheel encoder counts
 * over UART.
 */


#include "main.h"
#include "encoder.h"
#include <stdio.h>


extern UART_HandleTypeDef huart2;


void Encoder_Test(void)
{

    char buffer[100];


    while(1)
    {

        int32_t left =
            Encoder_Left_Count();


        int32_t right =
            Encoder_Right_Count();



        sprintf(
            buffer,
            "Left: %ld  Right: %ld\r\n",
            left,
            right
        );


        HAL_UART_Transmit(
            &huart2,
            (uint8_t*)buffer,
            strlen(buffer),
            100
        );


        HAL_Delay(100);

    }

}