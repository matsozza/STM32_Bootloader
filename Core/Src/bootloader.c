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
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "bootloader.h"
#include "comm_uart.h" // TODO remove later or encapsulate in submodules dependencies

/* External variables includes -----------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Internal variables includes -----------------------------------------------*/
const uint8_t PROG_VER[2] = {MAJOR, MINOR};

/* Internal functions prototypes ---------------------------------------------*/
static void bootloader_loadApplication(void);
static void _setPacket(uint8_t* packetAddress, uint8_t packetValue[]);
static uint8_t _checkPacket(uint8_t* packet, uint8_t expectedPacket[4],  uint8_t significance);

/* Functions implementation --------------------------------------------------*/
void bootloader_init()
{
	//printf("\n\rBootloader - Start - Ver.(%d.%d) \r\n\0", PROG_VER[0], PROG_VER[1]);

	// Turn board LEDs off
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);

	// Send communication open command to UART interface
	//printf("\n\rBootloader - Check for OTA communication\r\n\0");

	uint8_t rx[4]; // Buffer for holding RX Packet information
	volatile HAL_StatusTypeDef halStatus;

	// Try to open communication with OTA serial interface + receive response if valid
	COMM_UART_SendData((uint8_t[4]) {0x80,0xFF,0xFF,0xFF}, 4, SERIAL_TIMEOUT);
	COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT);

	if(_checkPacket(rx, (uint8_t[4]){0xC0,0xFF,0xFF,0xFF}, (uint8_t) 0b1000))
	{
		// --------------- a_ Unlock memory + clear error flags ---------------
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
		halStatus = HAL_FLASH_Unlock();

		// --------------- b_ Erase all memory sectors dedicated to application software ---------------
		for(uint8_t idxS=APP_FLASH_SECINI; idxS <= APP_FLASH_SECFIN; idxS++ )
		{
			FLASH_Erase_Sector(idxS,FLASH_VOLTAGE_RANGE_3);
			COMM_UART_SendData((uint8_t[4]){0x81,idxS,0xFF,0xFF}, 4, SERIAL_TIMEOUT);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
		}
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);

		// --------------- c_ Confirm MCU is ready to receive the provided number of packets ---------------
		uint32_t nOfPackets = (uint32_t) (rx[3] + (rx[2]<< 8) + (rx[1] << 16));
		COMM_UART_SendData(
			(uint8_t[4]) {0x82,
			(uint8_t) ((nOfPackets & 0x00FF0000) >> 16),
			(uint8_t) ((nOfPackets & 0x0000FF00) >> 8),
			(uint8_t) ((nOfPackets & 0x000000FF) >> 0)}, 4, SERIAL_TIMEOUT);

		// --------------- d_ Start receiving data + flashing MCU sectors (dummy flashing) ---------------
		const uint32_t sectorAddr[] = FLASH_SECTOR_ADDR;
		const uint32_t sectorSize[] = FLASH_SECTOR_SIZE;
		uint32_t nOfFlashed = 0;

		for(uint8_t nSector = APP_FLASH_SECINI; nSector<= APP_FLASH_SECFIN; nSector++)
		{
			// Write all the addresses contained in the current flash sector
			for(uint32_t addr=sectorAddr[nSector]; addr<(sectorAddr[nSector]+sectorSize[nSector]); addr+=0x04)
			{
				// Receive a SW packet via serial + parse it (little-endian) + write it to flash memory
				COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT);
				uint32_t packetData = (rx[3] << 24) + (rx[2] << 16) + (rx[1] << 8) + (rx[0] << 0);
				halStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t) packetData);

				if(halStatus != HAL_OK) // If there was any error programming the memory
				{
					COMM_UART_SendData((uint8_t[4]) {0x8F, 0x01, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT);
					break;
				}

				HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
			}
			COMM_UART_SendData((uint8_t[4]) {0x83, nSector, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT);
		}
		HAL_FLASH_Lock();
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	}

	// Transmit feedback about flash process (whether successful or not)
	COMM_UART_SendData((uint8_t[4]) {0x84, 0x00, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT);

	//printf("\n\r\0");

	bootloader_loadApplication(); // Load application file
}

static void bootloader_loadApplication()
{
	//printf("Bootloader - Jumping to application \r\n\0");

	// Function pointer to application's reset handler
	const uint32_t sectorAddr[] = FLASH_SECTOR_ADDR;
	void (*app_Reset_Handler)(void) =	(void*)(*(volatile uint32_t*)(sectorAddr[APP_FLASH_SECINI] + 0x04));

	// Turn-on board LED0
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

	app_Reset_Handler(); // Application starting point
}

// ********** Internal Auxiliary methods **********

static void _setPacket(uint8_t* packetAddress, uint8_t packetValue[4])
{
	for(uint8_t idx = 0; idx<4; idx++)
	{
		*(packetAddress + idx) = packetValue[idx];
	}
}

/*
 * Method to check if the contents of a packet meet the required content according to the significance.
 */
static uint8_t _checkPacket(uint8_t* packet, uint8_t expectedPacket[4],  uint8_t significance)
{
	uint8_t result = 0;

	if(*packet	   == expectedPacket[0] || ((significance & 0b1000) == 0)) result |= (result | 0b1000);
	if(*(packet+1) == expectedPacket[1] || ((significance & 0b0100) == 0)) result |= (result | 0b0100);
	if(*(packet+2) == expectedPacket[2] || ((significance & 0b0010) == 0)) result |= (result | 0b0010);
	if(*(packet+3) == expectedPacket[3] || ((significance & 0b0001) == 0)) result |= (result | 0b0001);

	return (result == 0b1111);
}
