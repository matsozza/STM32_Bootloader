/**
  ******************************************************************************
  * @file           : comm_uart.c
  * @brief          : UART communication API file
  ******************************************************************************
  * @attention
  *
  *  Created on: Feb 09, 2024
  *  Author: Matheus Sozza
  *
  ******************************************************************************
  */

/**
  ******************************************************************************
  * @file           : bootloader.c
  * @brief          : Bootloader main file
  ******************************************************************************
  * @attention
  *
  *  Created on: Feb 09, 2024
  *  Author: Matheus Sozza
  *
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "comm_uart.h"

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef UART_DEVICE_HANDLER;

/* Internal variables includes -----------------------------------------------*/


/* Internal functions prototypes ---------------------------------------------*/


/* Functions implementation --------------------------------------------------*/
void COMM_UART_ReceiveString(char* str)
{
    uint8_t idx;
    for(idx = 0; idx<0xFF; idx++)
    {
      HAL_UART_Receive(&UART_DEVICE_HANDLER , (uint8_t *) (str + idx), 1, HAL_MAX_DELAY);
      if( (char) *(str + idx) == '\0') break;
    }
}


// Transmit string via UART in blocking mode
void COMM_UART_SendString(char* str)
{
    uint8_t idx;
    for(idx = 0; idx<0xFF; idx++)
    {
	    HAL_UART_Transmit(&UART_DEVICE_HANDLER, (uint8_t *) (str + idx), 1, HAL_MAX_DELAY);
      if( (char) *(str + idx) == '\0') break;
    }
}
