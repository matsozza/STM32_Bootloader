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
#include "bootloader.h"

/* External variables includes -----------------------------------------------*/


/* Internal variables includes -----------------------------------------------*/
const uint8_t PROG_VER[2] 		= {MAJOR, MINOR};		// Bootloader version
const uint32_t sectorAddr[] 	= FLASH_SECTOR_ADDR; 	// Memory layout - sector addresses
const uint32_t sectorSize[] 	= FLASH_SECTOR_SIZE; 	// Memory layout - sector sizes
uint8_t rx[4] 					= {0,0,0,0}; 			// Buffer for holding RX Packet information
uint32_t swConfigShadow;								// SW Config memory - shadow copy address

/* Internal functions prototypes ---------------------------------------------*/
inline static uint32_t* _bootloader_configMemory_initShadow(void);
inline static uint8_t _bootloader_configMemory_deinitShadow(void);
inline static uint32_t _bootloader_configMemory_getParameter(uint32_t);
inline static uint32_t _bootloader_configMemory_setParameter(uint32_t , uint32_t);

inline static uint8_t _bootloader_appMemory_eraseSectors(void);
inline static uint32_t _bootloader_appMemory_acknowledgePackets(void);
inline static uint32_t _bootloader_appMemory_receiveAndFlash(uint32_t);
inline static void _bootloader_appMemory_loadApplication(void);

inline static void _bootloader_LCD_printSentence(char *sentence, uint8_t nCycles);
inline static void _bootloader_LCD_printSentence2(char *sentence, char *sentence2, uint8_t nCycles);
inline static void _bootloader_LCD_printSentence3(char *sentence, char *sentence2, char* sentence3, uint8_t nCycles);

uint16_t _gen_crc16(const uint8_t* , uint32_t );
inline static uint8_t _checkPacket(uint8_t* packet, uint8_t expectedPacket[4],  uint8_t significance);

/* Functions implementation --------------------------------------------------*/
void bootloader_init()
{
	_bootloader_LCD_printSentence("Booting", 4); // User feedback only via LCD
	
	_bootloader_configMemory_initShadow(); // Init. shadow copy of config. memory
	uint32_t curr_SW_MAJOR = _bootloader_configMemory_getParameter(CONFIG_MEMORY_SW_MAJOR); 
	uint32_t curr_SW_MINOR = _bootloader_configMemory_getParameter(CONFIG_MEMORY_SW_MINOR);

	// 0_ Unlock memory + clear error flags 
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
	if(HAL_FLASH_Unlock() != HAL_OK) goto FLASH_ERROR; // Error in step 'a'

	// 1_(0x80) Try to open communication with OTA serial interface 
	COMM_UART_SendData((uint8_t[4]) {0x80,0xFF,0xFF,0xFF}, 4, SERIAL_TIMEOUT_TX);
	
	// 2_(0xC0) Check if MCU answered via OTA serial interface 
	if(COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT_RX)!= HAL_OK) goto JUMP_TO_APPLICATION;

	if(_checkPacket(rx, (uint8_t[4]){0xC0,0xFF,0xFF,0xFF}, (uint8_t) 0b1000))
	{
		// Check if the SW to be flashed is a different version from current MCU image
		if(REPEATED_FLASH || !_checkPacket(rx, (uint8_t[4]){0xFF, (uint8_t)curr_SW_MAJOR, (uint8_t)curr_SW_MINOR, 0xFF}, (uint8_t) 0b0110))
		{
			// --------------- a_(0x81) Erase memory dedicated to appl. sw + inform serial interface  ---------------
			if(!_bootloader_appMemory_eraseSectors()) goto FLASH_ERROR; // Error in step 'b'

			// --------------- b_(0x82) Send to serial interface -> ready to receive 'nOfPackets' of data  ---------------
			uint32_t nOfPackets = _bootloader_appMemory_acknowledgePackets();
			if(nOfPackets == 0) goto FLASH_ERROR; // Error in step 'c'

			// --------------- c_Start receiving data + flashing MCU sectors (dummy flashing) ---------------
			uint32_t nOfFlashed = _bootloader_appMemory_receiveAndFlash(nOfPackets);

			// --------------- d_Transmit feedback about flash process ---------------
			HAL_Delay(50); // Wait last transmission
			uint16_t memCRC;
			if(nOfFlashed == nOfPackets)
			{
    			// Calculate CRC + send transmission complete feedback
				memCRC = _gen_crc16((uint8_t*) sectorAddr[APP_MEMORY_SECINI], (uint32_t)nOfPackets*4);
				COMM_UART_SendData((uint8_t[4]) {0x84, 0x00, (memCRC >> 8) & 0xFF, (memCRC >> 0) & 0xFF}, 4, SERIAL_TIMEOUT_TX); // OK
				_bootloader_LCD_printSentence3("  Done!", "Loading", "App...",  3); // User feedback only via LCD
			}
			else
			{
				FLASH_ERROR:
				COMM_UART_SendData((uint8_t[4]) {0x8F, 0xFF, 0xFF, 0xFF}, 4, SERIAL_TIMEOUT_TX); // NOK
				while(1)
				{
					_bootloader_LCD_printSentence2(" Flash", " Error!",  2);
					_bootloader_LCD_printSentence2(" Try", " Again!",  2);
				}
			}

			// --------------- e_Update config. memory area (shadow copy) ---------------
			_bootloader_configMemory_setParameter(CONFIG_MEMORY_SW_MAJOR, (uint32_t) *(uint32_t*)(sectorAddr[APP_MEMORY_SECINI] + APP_MEMORY_SW_MAJOR));
			_bootloader_configMemory_setParameter(CONFIG_MEMORY_SW_MINOR, (uint32_t) *(uint32_t*)(sectorAddr[APP_MEMORY_SECINI] + APP_MEMORY_SW_MINOR));
			_bootloader_configMemory_setParameter(CONFIG_MEMORY_SW_SIZE, (uint32_t) nOfPackets);
			_bootloader_configMemory_setParameter(CONFIG_MEMORY_SW_CRC, (uint32_t) memCRC);
		}
		else
		{
			// --------------- a_(0x8E) Attempt to re-flash the same SW - Abort
			COMM_UART_SendData((uint8_t[4]) {0x8E,0xFF,0xFF,0xFF}, 4, SERIAL_TIMEOUT_TX);
			_bootloader_LCD_printSentence2("   SW", "Up2Date!",  3);
		}
	}
	
	JUMP_TO_APPLICATION:
	_bootloader_configMemory_deinitShadow();
	HAL_FLASH_Lock();

	// 3_(0x85) Jump to application
	HAL_Delay(50); // Wait last transmission
	COMM_UART_SendData((uint8_t[4]){0x85,0xFF,0xFF,0xFF}, 4, SERIAL_TIMEOUT_TX);
	_bootloader_appMemory_loadApplication(); // Load application file
}

// ********** SW Config management methods **********

// Create a shadow of the Flash memory (SW configuration sector) in the heap for faster access and manipulation
inline static uint32_t* _bootloader_configMemory_initShadow()
{
	// Allocate shadow memory
	swConfigShadow =(uint32_t) malloc(CONFIG_MEMORY_USED_SIZE);
	
	// Create shadow copy of parameters
	for(uint32_t idx=0; idx < CONFIG_MEMORY_USED_SIZE; idx+=0x04)
	{
		*(uint32_t*)(swConfigShadow + idx) = (uint32_t) *(uint32_t*)(sectorAddr[CONFIG_MEMORY_SECINI] + idx);
	}
	return (uint32_t*)swConfigShadow;
}

inline static uint8_t _bootloader_configMemory_deinitShadow()
{
	HAL_StatusTypeDef halStatus;

	// Erase config. memory sectors
	for(uint8_t idxS=CONFIG_MEMORY_SECINI; idxS <= CONFIG_MEMORY_SECFIN; idxS++ )
	{
		FLASH_Erase_Sector(idxS,FLASH_VOLTAGE_RANGE_3);
	}

	// Write config. memory sectors with shadow copy
	for(uint32_t idx=0; idx < CONFIG_MEMORY_USED_SIZE; idx+=0x04)
	{
		halStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
			sectorAddr[CONFIG_MEMORY_SECINI] + idx, 
			(uint64_t) *(uint32_t*)(swConfigShadow + idx));

		if(halStatus != HAL_OK) return 0;
	}
	free((uint32_t*)swConfigShadow);
	swConfigShadow = (uint32_t) NULL;
	return 1;
}

inline static uint32_t _bootloader_configMemory_getParameter(uint32_t memOffset)
{
	return (uint32_t) *(uint32_t*)(swConfigShadow + memOffset);
}

inline static uint32_t _bootloader_configMemory_setParameter(uint32_t memOffset, uint32_t paramValue)
{
	*(uint32_t*)(swConfigShadow + memOffset) = (uint32_t) paramValue;
	return (uint32_t) *(uint32_t*)(swConfigShadow + memOffset);
}

// ********** Flashing procedure & SW Update methods **********
inline static uint8_t _bootloader_appMemory_eraseSectors()
{	
	HAL_StatusTypeDef halStatus;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // Turn-off LED0
	for(uint8_t idxS=APP_MEMORY_SECINI; idxS <= APP_MEMORY_SECFIN; idxS++ )
	{
		// Update LCD
		char p[8];
		sprintf(p, " %d/%d", idxS, APP_MEMORY_SECFIN);
		_bootloader_LCD_printSentence3("Erasing", "sector:", p, 2);

		// Erase sectors
		FLASH_Erase_Sector(idxS,FLASH_VOLTAGE_RANGE_3);
		HAL_Delay(50); // Wait last transmission
		halStatus = COMM_UART_SendData((uint8_t[4]){0x81,
										idxS,
										idxS == APP_MEMORY_SECFIN,
										0xFF}, 
										4, SERIAL_TIMEOUT_RX); // Send (0x81 + sector no. + isLastSector) to confirm erased sector
		if(halStatus!= HAL_OK) return 0; // Error in TX
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9); // Toggle LED0
	}
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET); // Turn-off LED0
	return 1; // OK
}

inline static uint32_t _bootloader_appMemory_acknowledgePackets()
{
	HAL_StatusTypeDef halStatus;

	// Receive packets data in RX
	if(COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT_RX) != HAL_OK) return 0; // Error in RX procedure
	
	// Check if a '0xC1' was received
	if(_checkPacket(rx, (uint8_t[4]){0xC1,0xFF,0xFF,0xFF}, (uint8_t) 0b1000))
	{
		uint32_t nOfPackets = (uint32_t) ((rx[3]<<0) + (rx[2]<<8) + (rx[1]<<16)); // Parse packets data

		// Update LCD
		char p[8];
		sprintf(p, " %d", nOfPackets);
		_bootloader_LCD_printSentence3("SW size:", p, "packets", 2);
		_bootloader_LCD_printSentence("Flashing", 0);

		// Acknowledge received command to host machine
		halStatus = COMM_UART_SendData((uint8_t[4]) {0x82,
			(uint8_t) ((nOfPackets & 0x00FF0000) >> 16),
			(uint8_t) ((nOfPackets & 0x0000FF00) >> 8),
			(uint8_t) ((nOfPackets & 0x000000FF) >> 0)},
			4, SERIAL_TIMEOUT_RX); // Answer back acknowledging the received 'nOfPackets' value
		if(halStatus != HAL_OK) return 0; // Error in TX procedure
		return nOfPackets;
	}
	else
	{
		return 0; //Error - Unexpected command from serial interface
	}
}

inline static uint32_t _bootloader_appMemory_receiveAndFlash(uint32_t nOfPackets)
{
	HAL_StatusTypeDef halStatus;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET); // Turn-off LED1
	uint32_t nOfFlashed = 0; // Counter to track progress of flashed packets
	for(uint8_t nSector = APP_MEMORY_SECINI; nSector<= APP_MEMORY_SECFIN; nSector++)
	{
		// FIXME this command cause all the memory to be shifted by 1 byte - Timing Critical
		//COMM_UART_SendData((uint8_t[4]) {0x83, nSector, 0x00, 0xFF}, 4, SERIAL_TIMEOUT_RX); // Feedback when a sector is starting to be flashed

		// Write all the addresses contained in the current flash sector
		for(uint32_t addr=sectorAddr[nSector]; addr<(sectorAddr[nSector]+sectorSize[nSector]); addr+=0x04)
		{
			// Receive a SW packet via serial + parse it (little-endian) + write it to flash memory
			halStatus = COMM_UART_ReceiveData(rx, 4, SERIAL_TIMEOUT_RX);
			if(halStatus != HAL_OK) goto END_PROGRAMMING;

			uint32_t packetData = (rx[0] << 24) + (rx[1] << 16) + (rx[2] << 8) + (rx[3] << 0);
			halStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t) packetData);

			// Check if programmed memory position == received packet OR
			// HAL error during programming OR all packets were flashed 
			if(*((uint32_t*)addr) != packetData
				|| halStatus != HAL_OK 
				|| ++nOfFlashed == nOfPackets)	goto END_PROGRAMMING;
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
		}
		COMM_UART_SendData((uint8_t[4]) {0x83, nSector, 0x01, 0xFF}, 4, SERIAL_TIMEOUT_RX); // Feedback when a sector is full
	}

	END_PROGRAMMING:
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET); // Turn-off LED1
	return nOfFlashed;
}

/**
 * @brief This function ensures a 'clean-state' jump to the application code
 */
inline static void _bootloader_appMemory_loadApplication()
{
	// --- 1_ Sanity check ---- Check is flashed image is valid / not empty
	
	// Get data from config. memory area
	uint32_t curr_SW_CRC = (uint32_t) *((uint32_t*)(sectorAddr[CONFIG_MEMORY_SECINI] + CONFIG_MEMORY_SW_CRC));
	uint32_t curr_SW_SIZE = (uint32_t) *((uint32_t*)(sectorAddr[CONFIG_MEMORY_SECINI] + CONFIG_MEMORY_SW_SIZE));

	// Recalculate CRC from app. memory area
	uint32_t memCRC = (uint32_t) _gen_crc16((uint8_t*) sectorAddr[APP_MEMORY_SECINI], (uint32_t)curr_SW_SIZE*4);

	// Plausibilize values - is 'config area CRC' equal to 'memory CRC'?
	// If not, app. memory might be empty or tampered - Halt program here
	if(memCRC!=curr_SW_CRC) while(1) _bootloader_LCD_printSentence3("Empty or", "corrupt", "binary!",  4);

	// --- 2_ Jump to application --- Perform all preliminary steps and jump to application address

	// Disable all interrupts to perform all steps before loading application code
	__disable_irq();

	//Function pointer to application's reset handler
	void (*app_Reset_Handler)(void) =	(void*)(*(volatile uint32_t*)(sectorAddr[APP_MEMORY_SECINI] + 0x04));

	// Reset SCB parameters
    SCB -> ICSR = 0x00000000;   // reset value;
    SCB -> SCR = 0;
    SCB -> CCR = 0x00000200;    // reset value
    SCB -> SHP[0] = 0;
    SCB -> SHCSR = 0;
    SCB -> CFSR = (SCB_CFSR_DIVBYZERO_Msk | SCB_CFSR_UNALIGNED_Msk | SCB_CFSR_UNDEFINSTR_Msk | SCB_CFSR_NOCP_Msk | SCB_CFSR_INVPC_Msk | SCB_CFSR_INVSTATE_Msk);
    SCB -> HFSR = (SCB_HFSR_DEBUGEVT_Msk | SCB_HFSR_FORCED_Msk | SCB_HFSR_VECTTBL_Msk);

	// Set the Vector Table Offset Register (VTOR) to the application vector table
	SCB -> VTOR = sectorAddr[APP_MEMORY_SECINI];

    // Set the Main Stack Pointer (MSP) to the value at the beginning of the application's flash section
	__set_MSP(*(volatile uint32_t *) sectorAddr[APP_MEMORY_SECINI]);

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

// ********** LCD & User feedback methods **********

inline static void _bootloader_LCD_printSentence(char *sentence, uint8_t nCycles)
{
  extern LCD_displayBuffer_t *LCD_displayBuffer01;
  extern const uint8_t bootloader_startScreen_01_map[];
  extern const uint8_t bootloader_startScreen_02_map[];

  uint8_t skipCyc = nCycles > 0 ? 0:1;
  for(uint8_t idx = 0; idx<nCycles + skipCyc; idx++)
  {
	if(idx%2==0)
	{
	  LCD_Buffer_writeMap(LCD_displayBuffer01, bootloader_startScreen_01_map);
	  LCD_Buffer_writeSentence(LCD_displayBuffer01," ... ", 26, 2);
	}
	else
	{
	  LCD_Buffer_writeMap(LCD_displayBuffer01, bootloader_startScreen_02_map);
	  LCD_Buffer_writeSentence(LCD_displayBuffer01,"     ", 26, 2);
	}

	LCD_Buffer_writeSentence(LCD_displayBuffer01,sentence, 18, 2);
	LCD_Buffer_sendToSPI(LCD_displayBuffer01);
	if(skipCyc==0) HAL_Delay(500);
  }
}

inline static void _bootloader_LCD_printSentence2(char *sentence, char *sentence2, uint8_t nCycles)
{
  extern LCD_displayBuffer_t *LCD_displayBuffer01;
  extern const uint8_t bootloader_startScreen_01_map[];
  extern const uint8_t bootloader_startScreen_02_map[];

  uint8_t skipCyc = nCycles > 0 ? 0:1;
  for(uint8_t idx = 0; idx<nCycles + skipCyc; idx++)
  {
	if(idx%2==0)
	{
	  LCD_Buffer_writeMap(LCD_displayBuffer01, bootloader_startScreen_01_map);
	}
	else
	{
	  LCD_Buffer_writeMap(LCD_displayBuffer01, bootloader_startScreen_02_map);
	}

	LCD_Buffer_writeSentence(LCD_displayBuffer01, sentence, 18, 2);
	LCD_Buffer_writeSentence(LCD_displayBuffer01, sentence2, 26, 2);
	LCD_Buffer_sendToSPI(LCD_displayBuffer01);
	if(skipCyc==0) HAL_Delay(500);
  }
}

inline static void _bootloader_LCD_printSentence3(char *sentence, char *sentence2, char *sentence3, uint8_t nCycles)
{
  extern LCD_displayBuffer_t *LCD_displayBuffer01;
  extern const uint8_t bootloader_startScreen_01_map[];
  extern const uint8_t bootloader_startScreen_02_map[];

  uint8_t skipCyc = nCycles > 0 ? 0:1;
  for(uint8_t idx = 0; idx<nCycles + skipCyc; idx++)
  {
	if(idx%2==0)
	{
	  LCD_Buffer_writeMap(LCD_displayBuffer01, bootloader_startScreen_01_map);
	}
	else
	{
	  LCD_Buffer_writeMap(LCD_displayBuffer01, bootloader_startScreen_02_map);
	}

	LCD_Buffer_writeSentence(LCD_displayBuffer01, sentence, 14, 2);
	LCD_Buffer_writeSentence(LCD_displayBuffer01, sentence2, 22, 2);
	LCD_Buffer_writeSentence(LCD_displayBuffer01, sentence3, 30, 2);
	LCD_Buffer_sendToSPI(LCD_displayBuffer01);
	if(skipCyc==0) HAL_Delay(500);
  }
}


// ********** Internal Auxiliary methods **********

uint16_t _gen_crc16(const uint8_t* data_p, uint32_t length){
    uint8_t x;
    uint16_t crc = 0xFFFF;

    //while (length--)
    for(uint32_t idx = 0; idx < length ; idx++)
    {
        if(idx == 8 || idx==512 || idx==256)
        {
            uint8_t xx = 1, xxt = 2;
            xx = xx+xxt;
        }

    	x = (crc >> 8) ^ (*(data_p+idx));
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x); // 0x1021
    }
    return crc;
}


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
