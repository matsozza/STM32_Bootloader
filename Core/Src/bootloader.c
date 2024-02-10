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
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "bootloader.h"
#include "comm_uart.h" // TODO remove later or encapsulate in submodules dependencies

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Internal variables includes -----------------------------------------------*/
const uint8_t PROG_VER[2] = {MAJOR, MINOR};

/* Internal functions prototypes ---------------------------------------------*/
static void bootloader_loadApplication(void);

/* Functions implementation --------------------------------------------------*/
void bootloader_init()
{
	//printf("\n\rBootloader - Start - Ver.(%d.%d) \r\n\0", PROG_VER[0], PROG_VER[1]);

	// Turn board LEDs off
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);

	// Send communication open command to UART interface
	//printf("\n\rBootloader - Check for OTA communication\r\n\0");
	COMM_UART_SendString("blc_0000");// Open communication with OTA interface

	char *str_tmp = malloc(sizeof(char)*64);
	COMM_UART_ReceiveString(str_tmp);

	//printf("\n\r\0");

	bootloader_loadApplication(); // Load application file
}

static void bootloader_loadApplication()
{
	//printf("Bootloader - Jumping to application \r\n\0");

	// Function pointer to application's reset handler
	void (*app_Reset_Handler)(void) =	(void*)(*(volatile uint32_t*)(APP_FLASH_ADDRESS + 0x04));

	// Turn-on board LED0
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

	app_Reset_Handler(); // Application starting point
}
