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
const uint8_t PROG_VER[2] 		= {MAJOR, MINOR};		// Bootloader version
const uint32_t sectorAddr[] 	= FLASH_SECTOR_ADDR; 	// Memory layout - sector addresses
const uint32_t sectorSize[] 	= FLASH_SECTOR_SIZE; 	// Memory layout - sector sizes
uint8_t rx[4] 					= {0,0,0,0}; 			// Buffer for holding RX Packet information

/* Internal functions prototypes ---------------------------------------------*/
inline static void _bootloader_eraseApplicationSectors(void);
inline static uint32_t _bootloader_acknowledgePackets(void);
inline static uint32_t _bootloader_receiveAndFlashPackets(uint32_t);
inline static void _bootloader_loadApplication(void);
inline static uint8_t _checkPacket(uint8_t* packet, uint8_t expectedPacket[4],  uint8_t significance);

/* Functions implementation --------------------------------------------------*/
void bootloader_init()
{
	// Turn board LEDs off - Visual feedback about data transmission
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);

	// 1_(0x80) Try to open communication with OTA serial interface 
	COMM_UART_SendData((uint8_t[4]) {0x80,0xFF,0xFF,0xFF}, 4, SERIAL_TIMEOUT);
	
	// 2_(0xC0) Check if MCU answered via OTA serial interface 
	COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT);
	if(_checkPacket(rx, (uint8_t[4]){0xC0,0xFF,0xFF,0xFF}, (uint8_t) 0b1000))
	{
		// --------------- a_ Unlock memory + clear error flags ---------------
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
		HAL_FLASH_Unlock();

		// --------------- b_(0x81) Erase memory dedicated to appl. sw + send feedback to OTA interface  ---------------
		_bootloader_eraseApplicationSectors();

		// --------------- c_(0x82) Send data to OTA serial interface to confirm it's ready to receive 'nOfPackets' of data  ---------------
		uint32_t nOfPackets = _bootloader_acknowledgePackets();

		// --------------- d_ Start receiving data + flashing MCU sectors (dummy flashing) ---------------
		uint32_t nOfFlashed = _bootloader_receiveAndFlashPackets(nOfPackets);

		// --------------- e_ Transmit feedback about flash process ---------------
		if(nOfFlashed == nOfPackets)
		{
			COMM_UART_SendData((uint8_t[4]) {0x84, 0xFF, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT); // OK
		}
		else
		{
			COMM_UART_SendData((uint8_t[4]) {0x8F, 0xFF, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT); // NOK
		}
	}
	
	COMM_UART_SendData((uint8_t[4]){0x85,0xFF,0xFF,0xFF}, 4, SERIAL_TIMEOUT); // Jump to application
	_bootloader_loadApplication(); // Load application file
}

inline static void _bootloader_eraseApplicationSectors()
{
	for(uint8_t idxS=APP_FLASH_SECINI; idxS <= APP_FLASH_SECFIN; idxS++ )
	{
		FLASH_Erase_Sector(idxS,FLASH_VOLTAGE_RANGE_3);
		COMM_UART_SendData((uint8_t[4]){0x81,idxS,0xFF,0xFF}, 4, SERIAL_TIMEOUT);
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
	}
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // Turn-off LED
}

inline static uint32_t _bootloader_acknowledgePackets()
{
	COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT); // Receive packets data

	uint32_t nOfPackets = (uint32_t) ((rx[3]<<0) + (rx[2]<<8) + (rx[1]<<16)); // Parse packets data
	COMM_UART_SendData((uint8_t[4]) {0x82,
		(uint8_t) ((nOfPackets & 0x00FF0000) >> 16),
		(uint8_t) ((nOfPackets & 0x0000FF00) >> 8),
		(uint8_t) ((nOfPackets & 0x000000FF) >> 0)},
		 4, SERIAL_TIMEOUT); // Answer back acknowledging the received 'nOfPackets' value

	return nOfPackets;
}

inline static uint32_t _bootloader_receiveAndFlashPackets(uint32_t nOfPackets)
{
	HAL_StatusTypeDef halStatus;
	uint32_t nOfFlashed = 0; // Counter to track progress of flashed packets
	for(uint8_t nSector = APP_FLASH_SECINI; nSector<= APP_FLASH_SECFIN; nSector++)
	{
		// Write all the addresses contained in the current flash sector
		for(uint32_t addr=sectorAddr[nSector]; addr<(sectorAddr[nSector]+sectorSize[nSector]); addr+=0x04)
		{
			// Receive a SW packet via serial + parse it (little-endian) + write it to flash memory
			COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT);
			uint32_t packetData = (rx[3] << 24) + (rx[2] << 16) + (rx[1] << 8) + (rx[0] << 0);
			halStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t) packetData);

			// TODO improve check robustness
			uint32_t* dataFlashed;
			dataFlashed = addr;

			if((*dataFlashed) != packetData)
			{
				goto END_PROGRAMMING; // Check for unflashed data
			}
			if(halStatus != HAL_OK || ++nOfFlashed == nOfPackets) goto END_PROGRAMMING; // Check for errors or end of binary flashing
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
		}
		//COMM_UART_SendData((uint8_t[4]) {0x83, nSector, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT); // TODO check if needed
	}

	END_PROGRAMMING:

	return nOfFlashed;
}

/**
 * @brief This function ensures a 'clean-state' jump to the application code
 */
inline static void _bootloader_loadApplication()
{
	// Disable all interrupts to perform all steps before loading application code
	__disable_irq();

	// Function pointer to application's reset handler
	void (*app_Reset_Handler)(void) =	(void*)(*(volatile uint32_t*)(sectorAddr[APP_FLASH_SECINI] + 0x04));

	// Reset SCB parameters
    SCB -> ICSR = 0x00000000;   // reset value;
    SCB -> SCR = 0;
    SCB -> CCR = 0x00000200;    // reset value
    SCB -> SHP[0] = 0;
    SCB -> SHCSR = 0;
    SCB -> CFSR = (SCB_CFSR_DIVBYZERO_Msk | SCB_CFSR_UNALIGNED_Msk | SCB_CFSR_UNDEFINSTR_Msk | SCB_CFSR_NOCP_Msk | SCB_CFSR_INVPC_Msk | SCB_CFSR_INVSTATE_Msk);
    SCB -> HFSR = (SCB_HFSR_DEBUGEVT_Msk | SCB_HFSR_FORCED_Msk | SCB_HFSR_VECTTBL_Msk);

	// Set the Vector Table Offset Register (VTOR) to the application vector table
	SCB -> VTOR = sectorAddr[APP_FLASH_SECINI];

    // Set the Main Stack Pointer (MSP) to the value at the beginning of the application's flash section
	__set_MSP(*(volatile uint32_t *) sectorAddr[APP_FLASH_SECINI]);

	// Ensure the VTOR and SP operations are complete
	__DSB(); // Ensure the VTOR and SP operations are complete
	__ISB(); // Flush the pipeline because of SP change

	// Re-enable all interrupts
	__enable_irq();

    // Disable / Reset SysTick before jumping to application
    SysTick -> CTRL = 0;
    SysTick -> LOAD = 0;
    SysTick -> VAL  = 0;

	// Call application's reset handler (starting point)
	app_Reset_Handler();
}

// ********** Internal Auxiliary methods **********

/*
 * @brief: Method to check if the contents of a packet meet the required content according to the significance.
 */
inline static uint8_t _checkPacket(uint8_t* packet, uint8_t expectedPacket[4],  uint8_t significance)
{
	uint8_t result = 0;

	if(*packet	   == expectedPacket[0] || ((significance & 0b1000) == 0)) result |= (result | 0b1000);
	if(*(packet+1) == expectedPacket[1] || ((significance & 0b0100) == 0)) result |= (result | 0b0100);
	if(*(packet+2) == expectedPacket[2] || ((significance & 0b0010) == 0)) result |= (result | 0b0010);
	if(*(packet+3) == expectedPacket[3] || ((significance & 0b0001) == 0)) result |= (result | 0b0001);

	return (result == 0b1111);
}
