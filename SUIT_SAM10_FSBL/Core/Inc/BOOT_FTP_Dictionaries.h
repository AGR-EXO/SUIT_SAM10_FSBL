/*
 * BOOT_FTP_Dictionaries.h
 *
 *  Created on: Oct 29, 2024
 *      Author: Angelrobotics
 */

#ifndef INC_BOOT_FTP_DICTIONARIES_H_
#define INC_BOOT_FTP_DICTIONARIES_H_


#include "stdint.h"
#include "string.h"

/* Node ID definition (Same as DOP) */
typedef enum _FTP_NodeID_t{
	NODE_ID_ALL		=	(uint8_t)0x0,		// switch 0000
	NODE_ID_CM	    =	(uint8_t)0x1,		// 0001
	NODE_ID_RH_COR	=	(uint8_t)0x2,		// 0010
	NODE_ID_LH_COR	=	(uint8_t)0x3,		// 0011
	NODE_ID_RH_TRA	=	(uint8_t)0x4,		// 0100
	NODE_ID_LH_TRA	=	(uint8_t)0x5,		// 0101
	NODE_ID_RH_SAG	=	(uint8_t)0x6,		// 0110	Angel Suit&L30 RH
	NODE_ID_LH_SAG	=	(uint8_t)0x7,		// 0111 Angel Suit&L30 LH
	NODE_ID_RK    	=	(uint8_t)0x8,		// 1000 Angel Suit&L30 RK
	NODE_ID_LK  	=	(uint8_t)0x9,		// 1001 Angel Suit&L30 LK
	NODE_ID_RA_MED	=	(uint8_t)0xA,		// 1010 Angel Suit& RA
	NODE_ID_LA_MED	=	(uint8_t)0xB,		// 1011 Angel Suit& LA
	NODE_ID_RA_LAT	=	(uint8_t)0xC,		// 1100
	NODE_ID_LA_LAT	=	(uint8_t)0xD,		// 1101
	NODE_ID_WIDM_R	=	(uint8_t)0xE,		// 1110 WIDM Right
	NODE_ID_WIDM_L	=	(uint8_t)0xF,		// 1111 WIDM Left
} FTP_NodeID_t;


/* FNC Code */
typedef enum _FTP_FNCCode_t{
	STX   			= 	((uint16_t)0x000U),
	Info_MSG   		=	((uint16_t)0x100U),
	Data_MSG    	= 	((uint16_t)0x200U),
	ACK    			= 	((uint16_t)0x300U),
	NACK			= 	((uint16_t)0x400U),
	EOT				= 	((uint16_t)0x500U),
	TRIGGER			= ((uint16_t)0x600U),
	FW_UPDATE		= 	((uint16_t)0x700U),

} FTP_FNCCode_t;


#endif /* INC_BOOT_FTP_DICTIONARIES_H_ */
