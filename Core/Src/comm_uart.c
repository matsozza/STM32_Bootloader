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

/* Private includes ----------------------------------------------------------*/
#include "comm_uart.h"

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef UART_DEVICE_HANDLER;

/* Internal variables includes -----------------------------------------------*/


/* Internal functions prototypes ---------------------------------------------*/


/* Functions implementation --------------------------------------------------*/

HAL_StatusTypeDef COMM_UART_ReceiveData(uint8_t* dataPtr, uint8_t size, uint32_t delay)
{
	HAL_StatusTypeDef halStatus;
  halStatus = HAL_UART_Receive(&UART_DEVICE_HANDLER , dataPtr, size, delay);
  return halStatus;
}

// Transmit string via UART in blocking mode
HAL_StatusTypeDef COMM_UART_SendData(uint8_t* dataPtr, uint8_t size, uint32_t delay)
{
  // Start data transmission (force transmission)
  HAL_StatusTypeDef halStatus;
  do{
    halStatus = HAL_UART_Transmit(&UART_DEVICE_HANDLER, dataPtr , size, delay);
  }
  while(halStatus == HAL_BUSY);
  return halStatus;
}