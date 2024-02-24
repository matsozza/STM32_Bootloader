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

// ---------- Flash Memory standard parameters ----------

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

// ---------- Application Memory parameters ----------

// Application general addresses
#define APP_MEMORY_SECINI          6                     /*Init Sector for App. SW*/
#define APP_MEMORY_SECFIN          10                    /*Last Sector for App. SW*/

// Application memory specific offsets
#define APP_MEMORY_SW_MAJOR        0x0200                /*Memory rel. address containing SW Major version*/
#define APP_MEMORY_SW_MINOR        0x0204                /*Memory rel. address containing SW Minor version*/

// ---------- Config NVM Memory parameters ----------

// Configuration flash memory (NVM) general addresses
#define CONFIG_MEMORY_SECINI       2                     /*Init Sector for Config*/
#define CONFIG_MEMORY_SECFIN       2                     /*Last Sector for Config*/

// Configuration flash memory (NVM) specific offsets
#define CONFIG_MEMORY_USED_SIZE    0x02                  /*Define used memory for Heap optimization - words (4-bytes)*/
#define CONFIG_MEMORY_SW_MAJOR     0x0000                /*Offset - SW Major version*/ 
#define CONFIG_MEMORY_SW_MINOR     0x0004                /*Offset - SW Minor version*/

// ---------- Serial / Communication definitions ----------
#define SERIAL_TIMEOUT				    1000      /* 1000ms */

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Function prototypes -------------------------------------------------------*/
void bootloader_init(void);

#endif
