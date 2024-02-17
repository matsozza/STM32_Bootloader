/**
  ******************************************************************************
  * @file           : comm_uart.h
  * @brief          : UART communication API header file
  ******************************************************************************
  * @attention
  *
  *  Created on: Feb 09, 2024
  *  Author: Matheus Sozza
  *
  ******************************************************************************
  */

#ifndef EF22B106_379E_4678_B839_82EDA23F8A4B
#define EF22B106_379E_4678_B839_82EDA23F8A4B

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"

/* Defines -------------------------------------------------------------------*/
#define UART_DEVICE_HANDLER huart2

/* Function prototypes -------------------------------------------------------*/
HAL_StatusTypeDef COMM_UART_ReceiveData(uint8_t* dataPtr, uint8_t size, uint32_t delay);
HAL_StatusTypeDef COMM_UART_SendData(uint8_t* dataPtr, uint8_t size, uint32_t delay);

/* Inline Functions  ---------------------------------------------------------*/

#endif /* EF22B106_379E_4678_B839_82EDA23F8A4B */
