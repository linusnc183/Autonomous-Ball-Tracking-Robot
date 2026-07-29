#include "uart_control.h"
#include "main.h"



extern UART_HandleTypeDef huart2;


extern float target_left;
extern float target_right;


static uint8_t rx;

/* Tracks the last time a byte was received, so we can stop the robot
 * if the Raspberry Pi link goes quiet (crash, disconnect, etc). */
static volatile uint32_t last_rx_tick=0;

/* Stop the motors if no command has arrived in this many ms. */
#define UART_TIMEOUT_MS 500



void UART_Init(void)
{

last_rx_tick = HAL_GetTick();

HAL_UART_Receive_IT(
&huart2,
&rx,
1);

}



void UART_Update(void)
{

if((HAL_GetTick() - last_rx_tick) > UART_TIMEOUT_MS)
{
    target_left = 0;
    target_right = 0;
}

}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


if(huart->Instance==USART2)
{

last_rx_tick = HAL_GetTick();


switch(rx)
{


case 'F':

target_left=300;
target_right=300;

break;



case 'B':

target_left=-300;
target_right=-300;

break;



case 'L':

target_left=-200;
target_right=200;

break;



case 'R':

target_left=200;
target_right=-200;

break;



case 'S':

target_left=0;
target_right=0;

break;



}



HAL_UART_Receive_IT(
&huart2,
&rx,
1);


}


}
