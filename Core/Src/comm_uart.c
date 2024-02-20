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
static uint32_t _ticToc(void);

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
  // Ensure constant minimum latency between transmissions
  uint32_t dly = _ticToc();
  if(dly<100) HAL_Delay(100 - dly);
  
  // Start data transmission (force transmission)
  HAL_StatusTypeDef halStatus;
  do{
    halStatus = HAL_UART_Transmit(&UART_DEVICE_HANDLER, dataPtr , size, delay);
  }
  while(halStatus == HAL_BUSY);

  _ticToc(); // Counter for latency for next run

  return halStatus;
}

static uint32_t _ticToc()
{
	static uint8_t tickRunning=0;
	static uint32_t lastTick;

	if(!tickRunning) // First trigger
	{
		tickRunning=1;
		lastTick = HAL_GetTick();
		return 0;
	}
	else // Count the delta
	{
		uint32_t currTick = HAL_GetTick();
		uint32_t delta;
		if(currTick >= lastTick)
		{	
			delta =  currTick - lastTick;
		}
		else
		{
			delta = currTick + (0xFFFFFFFF - lastTick);
		}

		lastTick=currTick;
		return delta;
	}

}
