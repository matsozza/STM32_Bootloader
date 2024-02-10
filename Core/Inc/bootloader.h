/**
  ******************************************************************************
  * @file           : bootloader.h
  * @brief          : Bootloader main header file
  ******************************************************************************
  * @attention
  *
  *  Created on: Feb 09, 2024
  *  Author: Matheus Sozza
  *
  ******************************************************************************
  */

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

/* Includes ------------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
// Bootloader version
#define MAJOR 0
#define MINOR 1

// Application start address
#define APP_FLASH_ADDRESS         0x08040000  /*Sectors 6 to 11*/
#define APP_FLASH_SIZE            0x0BFFFF    /*768KB*/

// Configuration start address
#define CONFIG_FLASH_ADDRESS      0x08008000  /*Sectors 0 to 1*/
#define CONFIG_FLASH_SIZE         0x3FFFF     /*16KB*/

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Function prototypes -------------------------------------------------------*/
void bootloader_init(void);

#endif
