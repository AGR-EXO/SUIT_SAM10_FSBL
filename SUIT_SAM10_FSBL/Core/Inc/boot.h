/*
 * boot.h
 *
 *  Created on: Jul 8, 2024
 *      Author: Angelrobotics
 */

#ifndef INC_BOOT_H_
#define INC_BOOT_H_


#include "main.h"
#include "module.h"
#include "version.h"
#include "crc16.h"
#include "stdbool.h"
#include "BOOT_FTP_Dictionaries.h"

/* HW Peripherals */
#include "fatfs.h"
#include "IOIF_QSPI_Flash.h"			// QSPI Flash
#include "ioif_pca9957hnmp.h"			// LED Driver
#include "ioif_flash_common.h"			// Flash
#include "ioif_sai_common.h"			// Audio Beep
#include "ioif_fdcan_common.h"			// FD-CAN
#include "ioif_gpio_common.h"			// GPIO


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


#define BTN_HOLD_TIME					500000
#define SUIT_APP_FW_INFO_SIZE			0x400
#define SUIT_APP_FW_BLANK_SIZE			0x20000

#define SUIT_APP_FW_ADDRESS				IOIF_FLASH_SECTOR_1_BANK1_ADDR
#define SUIT_MD_FW_ADDRESS				IOIF_FLASH_SECTOR_5_BANK1_ADDR
#define STM32_MEMMAP_BASE_ADDRESS 		0x90000000
#define APP_FW_SIZE_MAX					0x080FFFFF - 0x080A0000			// App FW Max Size for 1 Bank
#define STM32H743_IFLASH_SECTOR_SIZE	0x00020000
#define BINARY_FILE_SIGN				{0xFF, 0x53, 0x43, 0x43, 0xFF, 0x4D, 0x41, 0x50}


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */


typedef enum _BootUpdateState
{
	BOOT_NORMAL,
	BOOT_BUTTON_WAIT,
	BOOT_USB_UPDATE,			//usb update mode
	BOOT_FLASH,					//copy file only to flash
	BOOT_MD_UPDATE,
	BOOT_UPDATE_WAIT,

	BOOT_ERROR = 99,
} BootUpdateState;

typedef enum _BootUpdateSubState
{
	BOOT_STX,
	BOOT_INFO,
	BOOT_DATA,			//usb update mode
	BOOT_EOT,					//copy file only to flash
	BOOT_NONE,

//	BOOT_ERROR = 99,
} BootUpdateSubState;


typedef enum _BootUpdateError
{
	BOOT_UPDATE_OK,
	BOOT_UPDATE_ERROR_FILE_SIZE,
	BOOT_UPDATE_ERROR_FILE_OPEN,
	BOOT_UPDATE_ERROR_FILE_READ,
	BOOT_UPDATE_ERROR_FLASH_WRITE,
	BOOT_UPDATE_ERROR_FLASH_READ,
	BOOT_UPDATE_ERROR_FLASH_ERASE,
	BOOT_UPDATE_ERROR_CRC,
	BOOT_UPDATE_ERROR_FILE_SIGN,
	BOOT_UPDATE_ERROR_FILE_OVERSIZE,
	BOOT_UPDATE_ERROR_VERIFY,
	BOOT_UPDATE_ERROR_INVALID_FW,
} BootUpdateError;


typedef enum _BootFlashDevice
{
	BOOT_INTERNAL_FLASH,
	BOOT_QSPI_FLASH,
	BOOT_SPI_FLASH,
} BootFlashDevice;

typedef struct _fw_version_t
{
	uint8_t  major;
	uint8_t  minor;
	uint8_t  patch;
//	uint16_t debug;
} fw_version_t;


typedef struct _fw_info_t
{
//	uint8_t  	 file_sign[8];
	uint32_t 	 fw_size;
	uint32_t 	 fw_crc;
	uint32_t 	 fw_startAddr;
	fw_version_t app_fw_ver;
} fw_info_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern uint8_t MD_STX_ACK_Flag;
extern uint8_t MD_STX_NACK_Flag;
extern uint16_t MD_Info_ACK_Flag;
extern uint16_t MD_Info_NACK_Flag;
extern uint16_t MD_Data_ACK_Flag;
extern uint16_t MD_Data_NACK_Flag;
extern uint8_t MD_EOT_ACK_Flag;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Boot_SetMDUpdateFlag(uint32_t flag);
bool			Boot_HWInit(void);
BootUpdateState Boot_CheckUpdateMode(void);																						// Check Update Mode
BootUpdateError	Boot_JumpToApp(uint32_t flashAddr);																							// Jump to Application
bool 			Boot_AllDev_DeInit(void);// QSPI to Internal Flash
BootUpdateError Boot_UpdateVerify(uint32_t flashAddr);
BootUpdateError Boot_EraseCurrentMDFW(uint32_t flashAddr);
BootUpdateError Boot_SaveNewMDFW(uint32_t flashAddr);
int Send_STX();
int Send_NACK(uint16_t reqframe_idx, uint8_t retrial);

#endif /* INC_BOOT_H_ */
