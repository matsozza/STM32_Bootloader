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
void COMM_UART_SendString(char[]);

#endif /* EF22B106_379E_4678_B839_82EDA23F8A4B */
