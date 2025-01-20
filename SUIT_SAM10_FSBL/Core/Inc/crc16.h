/*
 * crc16.h
 *
 *  Created on: Oct 15, 2024
 *      Author: Angelrobotics
 */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_

#include "stdint.h"

void 	 Update_CRC16   (uint16_t *pCRC_current, uint8_t data);						// CRC 갱신 (CRC16-UMTS(BUYPASS))
uint16_t Calculate_CRC16(uint16_t crc_current, uint8_t *pData, uint32_t start, uint32_t length);	// 현재 Data 의 CRC 를 계산


#endif /* INC_CRC16_H_ */
