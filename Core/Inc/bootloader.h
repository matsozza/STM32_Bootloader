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
#define APP_FLASH_ADDRESS         0x08040000            /*Sectors 6 to 11*/
#define APP_FLASH_SIZE            0x0BFFFF              /*768KB*/
#define APP_FLASH_SECINI          6                     /*Init Sector for App. SW*/
#define APP_FLASH_SECFIN          11                    /*Last Sector for App. SW*/

// Flash memory sector architecture
#define FLASH_SECTOR_ADDR       {             \
                                  0x08000000, \
                                  0x08004000, \
                                  0x08008000, \
                                  0x0800C000, \
                                  0x08010000, \
                                  0x08020000, \
                                  0x08040000, \
                                  0x08060000, \
                                  0x08080000, \
                                  0x080A0000, \
                                  0x080C0000, \
                                  0x080E0000  \
                                } /* Init. address of flash sectors*/

#define FLASH_SECTOR_SIZE       {          \
                                  0x04000, \
                                  0x04000, \
                                  0x04000, \
                                  0x04000, \
                                  0x10000, \
                                  0x20000, \
                                  0x20000, \
                                  0x20000, \
                                  0x20000, \
                                  0x20000, \
                                  0x20000, \
                                  0x20000  \
                                } /* Size of flash sectors*/

// Configuration NVM parameters start address
#define CONFIG_FLASH_ADDRESS      0x08008000  /*Sectors 2*/
#define CONFIG_FLASH_SIZE         0x3FFFF     /*16KB*/

// Communication definitions
#define SERIAL_TIMEOUT				    1000      /* 1000ms */

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Function prototypes -------------------------------------------------------*/
void bootloader_init(void);

#endif
