/*
 * boot.c
 *
 *  Created on: Jul 8, 2024
 *      Author: Angelrobotics
 */


#include "boot.h"



/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

#define FLASH_BUFFER_SIZE 1024
#define FLASH_WORD_SIZE 32
#define MD_UPDATE_FLAG ((uint32_t*)0x38000000)


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */
//Node ID
uint8_t cm_node_id = 1;
uint8_t MD_nodeID = 0;
uint8_t  ori_node;
uint32_t fnc_code;
uint8_t fdcan_rx_test_buf[64] = {0,};
int cb_cnt=0;

//FW Info Object
fw_version_t BL_FW_VER;
fw_info_t MD_FWInfoObj={0,};

//MD Update Flag
//uint32_t MD_Update_Flag=0;

//BOOT STATE
BootUpdateSubState MD_boot_state=BOOT_NONE;

//ACK/NACK Flag
uint8_t MD_STX_ACK_Flag = 0;
uint8_t MD_STX_NACK_Flag = 0;
uint16_t MD_Info_ACK_Flag = 0;
uint16_t MD_Info_NACK_Flag = 0;
uint16_t MD_Data_ACK_Flag = 0;
uint16_t MD_Data_NACK_Flag = 0;
uint8_t MD_EOT_ACK_Flag = 0;
uint8_t MD_EOT_NACK_Flag = 0;
uint8_t MD_Recovery_ACK_Flag = 0;
//STX
uint8_t stx_cnt=0;
uint8_t STX_Txbuf[64]={0,};
//INFO
uint32_t INFO_filesize;
uint32_t INFO_startaddroffset;
uint16_t INFO_filecrc;// (total)
uint16_t INFO_totaldataindex;// (number of data frames)
fw_version_t INFO_fwversion;
//nothing for 45 byte
uint16_t INFO_msgcrc;// (current)
uint16_t INFO_msgcrc_compare;// (current)
uint8_t INFO_Txbuf[64]={0,};
uint8_t INFO_Rxbuf[64]={0,};
//DATA
uint16_t DATA_indexnumber;//Current
uint8_t DATA_Rxbuf[60]={0,};
uint8_t DATA_Txbuf[64]={0,};
uint16_t DATA_msgcrc;// (current)
uint16_t DATA_msgcrc_compare;
uint32_t wr_size;
uint32_t fw_bin_size =0;
uint32_t f_index = 0;
uint8_t DATA_triggerWrite=0;
static uint16_t expected_index = 0; // Keep track of the expected index (starting at 0)
uint8_t DATA_WriteDone=0;
//EOT
uint8_t EOT_Txbuf[64]={0,};

uint32_t MDFWBinSize=0;

uint8_t MD_Update_Flag = 0;
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
static uint8_t Read_Node_ID();
static int  FDCAN_RX_CB(uint16_t id, uint8_t* rx_pData);
static void ProcessReceivedData(const uint8_t* buffer, uint8_t* output_buffer, uint32_t size);
static int Unpack_InfoMsg(uint32_t t_fnccode, uint8_t* t_buff);
static int Unpack_DataMsg(uint32_t t_fnccode, uint8_t* t_buff);
static int Unpack_EOT(uint32_t t_fnccode, uint8_t* t_buf);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */


//void Boot_SetMDUpdateFlag(uint32_t flag){
//	MD_Update_Flag=flag;
//}

bool Boot_HWInit(void)
{
	bool ret = true;

	/* BootLoader FW version update */
	BL_FW_VER.major = FW_VER_MAJOR;
	BL_FW_VER.minor = FW_VER_MINOR;
	BL_FW_VER.patch = FW_VER_PATCH;
//	BL_FW_VER.debug = FW_VER_DEBUG;

	MD_nodeID=Read_Node_ID();
	IOIF_InitFlash();											// Internal Flash Init.
	/* FD CAN Init. */
	IOIF_InitFDCAN1(MD_nodeID);
	IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, FDCAN_RX_CB);		// RX Callback Registration

	MD_Update_Flag=0;
	return ret;
}


BootUpdateState Boot_CheckUpdateMode(void)
{
	BootUpdateState ret = BOOT_UPDATE_WAIT;//BOOT_NORMAL;

	//No1
	if(MD_Update_Flag==1){
		return ret = BOOT_MD_UPDATE;
	}

	//End
	if(DATA_WriteDone==1){
		DATA_WriteDone = 0;
		return ret = BOOT_NORMAL;
	}
	return ret;
}


BootUpdateError Boot_JumpToApp(uint32_t flashAddr)
{
	BootUpdateError ret = BOOT_UPDATE_OK;

	/* 1. Verifying Application FW before jump */
//	ret = Boot_UpdateVerify((uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR);

	/* 2. Jump to App. FW */
	if(ret == BOOT_UPDATE_OK)
	{
		void (*SysMemBootJump)(void);
		SysMemBootJump = (void (*)(void)) (*((uint32_t *) ((flashAddr + SUIT_APP_FW_INFO_SIZE+  4))));//+4bytes worth of interrupt vector at first //+ SUIT_APP_FW_INFO_SIZE + 4))));

		Boot_AllDev_DeInit();

		__set_MSP(*(uint32_t*)flashAddr+ SUIT_APP_FW_INFO_SIZE);
		SCB->VTOR = flashAddr + SUIT_APP_FW_INFO_SIZE;

		SysMemBootJump();
	}
	else{
		ret = BOOT_UPDATE_ERROR_INVALID_FW;
	}
	return ret;
}


bool Boot_AllDev_DeInit(void)
{
	/* Disable all interrupts */

	__disable_irq();

	/* Set the clock to the default state */
	HAL_RCC_DeInit();

	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	for (uint8_t i=0;i<8;i++)
	{
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}
	/* Disable Systick timer */
	SysTick->CTRL = 0;

	return true;

}

BootUpdateError Boot_UpdateVerify(uint32_t flashAddr)
{
	BootUpdateError ret = BOOT_UPDATE_OK;

	uint16_t crc = 0;
	uint32_t addr = 0;
	uint32_t length = 0;

	uint8_t read_buf[64] = {0,};
	uint32_t read_buf_idx = 0;
	uint32_t read_buf_len = 0;

	fw_info_t *pInfo = (fw_info_t*)(flashAddr);

	do
	{
		/* 2. fw max size 초과 여부 */
		if(pInfo->fw_size >= APP_FW_SIZE_MAX)
		{
			ret = BOOT_UPDATE_ERROR_FILE_OVERSIZE;
			break;
		}

		/* 3. crc update & check*/
		addr = flashAddr + pInfo->fw_startAddr;
		length = pInfo->fw_size;

		while (read_buf_idx < length)
		{
			read_buf_len = length - read_buf_idx;
			if (read_buf_len > 64)
				read_buf_len = 64;

			if(IOIF_ReadFlash(addr + read_buf_idx, read_buf, read_buf_len) != IOIF_FLASH_STATUS_OK)
			{
				ret = BOOT_UPDATE_ERROR_FLASH_READ;
				break;
			}

			read_buf_idx += read_buf_len;

			for (int j=0; j<read_buf_len; j++)
			{
				Update_CRC16(&crc, read_buf[j]);
			}
		}

		if(ret == BOOT_UPDATE_OK)
		{
			if(pInfo->fw_crc != crc)
			{
				ret = BOOT_UPDATE_ERROR_CRC;
			}
		}

	}while(0);


	return ret;
}


BootUpdateError Boot_EraseCurrentMDFW(uint32_t flashAddr, uint32_t MD_size)
{
	BootUpdateError ret = BOOT_UPDATE_OK;

	//Erase flash sector of new fw
	uint8_t sector_idx = 0; // the sector where i want to write the new firmware
	uint8_t erase_sector = (MD_size + SUIT_APP_FW_INFO_SIZE)/ (uint32_t) STM32H743_IFLASH_SECTOR_SIZE + 1;//(MDFWBinSize + SUIT_APP_FW_INFO_SIZE) / (uint32_t) STM32H743_IFLASH_SECTOR_SIZE + 1;

	while(sector_idx < erase_sector)
	{
		if(IOIF_EraseFlash(flashAddr + (sector_idx * STM32H743_IFLASH_SECTOR_SIZE), false) != IOIF_FLASH_STATUS_OK)
		{
			return ret = BOOT_UPDATE_ERROR_FLASH_ERASE;
		}
		IOIF_SetCurrFlashAddr(0);

		f_index = 0;
		sector_idx++;
	}

	return ret;
}


BootUpdateError Boot_SaveNewMDFW(uint32_t origin_flashAddr, uint32_t destination_flashAddr, uint32_t MD_size)
{
	BootUpdateError ret = BOOT_UPDATE_OK;
    //uint32_t srcAddr = IOIF_FLASH_SECTOR_5_BANK1_ADDR; // Flash Bank 1 Sector 5 시작 주소
    uint32_t srcAddr = origin_flashAddr;//IOIF_FLASH_SECTOR_5_BANK1_ADDR; // Flash Bank 1 Sector 5 시작 주소

    fw_bin_size = MD_size+SUIT_APP_FW_INFO_SIZE; //MDFWBinSize+SUIT_APP_FW_INFO_SIZE;
    uint8_t buffer[1024]; // 읽기/쓰기 버퍼
    uint32_t f_index = 0;

    while (f_index < fw_bin_size)
    {
        uint32_t wr_size = (fw_bin_size - f_index > 60) ? 60 : (fw_bin_size - f_index);
        uint32_t wr_addr = destination_flashAddr + f_index;//SUIT_APP_FW_INFO_SIZE + f_index;

        // Flash에서 데이터를 읽어오기
        if (IOIF_ReadFlash(srcAddr + f_index, buffer, wr_size) != IOIF_FLASH_STATUS_OK)
        {
            return BOOT_UPDATE_ERROR_FLASH_READ;
        }

        // 마지막 chunk인지 확인
        uint8_t triggerWrite = (f_index + wr_size >= fw_bin_size);
//        DATA_triggerWrite = triggerWrite;

        // Flash에 데이터 쓰기
        if (IOIF_WriteFlashMassBuffered(wr_addr, buffer, wr_size, triggerWrite) != IOIF_FLASH_STATUS_OK)
        {
            return BOOT_UPDATE_ERROR_FLASH_WRITE;
        }

        f_index += wr_size;
    }

	return ret;
}

int Send_STX(){
	int ret=0;
	MD_boot_state=BOOT_STX;

	//send Start transmission
	uint16_t t_id = STX | (MD_nodeID << 4) | (cm_node_id) ;
	uint8_t array[8]={0,};
	if(IOIF_TransmitFDCAN1(t_id, array , 8) != 0){
		ret = 100;			// tx error
	}

	MD_STX_ACK_Flag ++;

	return ret;
}


int Send_EOT(uint8_t index){
	int ret=0;
	MD_boot_state=BOOT_EOT;

	//send Start transmission
	uint16_t t_id = EOT | (MD_nodeID << 4) | (cm_node_id) ;
	uint8_t array[8]={0,};
	array[0]=index;
	if(IOIF_TransmitFDCAN1(t_id, array , 8) != 0){
		ret = 100;			// tx error
	}

	MD_EOT_ACK_Flag ++;

	return ret;
}


int Send_Recovery(uint8_t index){
	int ret=0;
	MD_boot_state=BOOT_EOT;

	//send Start transmission
	uint16_t t_id = RECOVERY | (MD_nodeID << 4) | (cm_node_id) ;
	uint8_t array[8]={0,};
	array[0]=index;
	if(IOIF_TransmitFDCAN1(t_id, array , 8) != 0){
		ret = 100;			// tx error
	}

	MD_Recovery_ACK_Flag ++;

	return ret;
}


int Send_NACK(uint16_t reqframe_idx, uint8_t retrial){
	int ret = 0;
	//Send NACK
	int cursor2=0;
	int t_fnccode = NACK;
	memcpy(&STX_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
	cursor2+=sizeof(t_fnccode);

	memcpy(&STX_Txbuf[cursor2], &reqframe_idx, sizeof(reqframe_idx));
	cursor2+=sizeof(reqframe_idx);

	memcpy(&STX_Txbuf[cursor2], &retrial, sizeof(retrial));
	cursor2+=sizeof(retrial);

	retrial++;

	int idx=64-cursor2;
	memset(&STX_Txbuf[cursor2], 0, idx);
	cursor2+=idx;

	uint16_t t_id = NACK | (MD_nodeID << 4)|(cm_node_id) ;

	if(IOIF_TransmitFDCAN1(t_id, STX_Txbuf, 64) != 0)
		ret = 100;			// tx error

	return ret;

}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


/* ------------------- READ NODE ID ------------------- */
static uint8_t Read_Node_ID()
{
    uint8_t temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;
    temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
    temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
    temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
    temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
    return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
}

static int FDCAN_RX_CB(uint16_t id, uint8_t* rx_pData)
{
	cb_cnt++;
	// Extract origin node (upper byte)
	ori_node = (id >> 4) & 0x0F; // Shift right by 4 bits and mask with 0x0F
    // Extract destination node (lower nibble)
	fnc_code = id & 0x700;   // Mask with 0x0F to get the lower nibble
	memcpy(fdcan_rx_test_buf, rx_pData, 64);

	switch (fnc_code){
		case FW_UPDATE:
			//App1 copy from sector 1 to sector 5
			if(Boot_EraseCurrentMDFW((uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR, MDFWBinSize)==BOOT_UPDATE_OK){
				if(Boot_SaveNewMDFW((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR,(uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR, MDFWBinSize)==BOOT_UPDATE_OK){
					uint32_t MD_Update, MD_Backup=1;
					uint32_t writeAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;
					//erase
					IOIF_EraseFlash(writeAddr, IOIF_ERASE_ONE_SECTOR);
	        		for(int i=0; i<7000;i++){}

					//write
					IOIF_WriteFlash(writeAddr, &MD_Update);
					writeAddr+=32;
	        		for(int i=0; i<7000;i++){}

					IOIF_WriteFlash(writeAddr, &MD_Backup);
					writeAddr+=32;
	        		for(int i=0; i<7000;i++){}

					IOIF_WriteFlash(writeAddr, &MDFWBinSize);
	        		for(int i=0; i<7000;i++){}

					Send_STX();
					//					MD_Update_Flag=1;
				}
			}

			break;

		case NACK:
			if(stx_cnt<3){
				//App1 copy from sector 1 to sector 5
				if(Boot_EraseCurrentMDFW((uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR, MDFWBinSize)==BOOT_UPDATE_OK){
					if(Boot_SaveNewMDFW((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR,(uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR,MDFWBinSize)==BOOT_UPDATE_OK){
						uint32_t MD_Update, MD_Backup=1;
						uint32_t writeAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;
						//erase
						IOIF_EraseFlash(writeAddr, IOIF_ERASE_ONE_SECTOR);
		        		for(int i=0; i<7000;i++){}

						//write
						IOIF_WriteFlash(writeAddr, &MD_Update);
						writeAddr+=32;
		        		for(int i=0; i<7000;i++){}

						IOIF_WriteFlash(writeAddr, &MD_Backup);
						writeAddr+=32;
		        		for(int i=0; i<7000;i++){}

						IOIF_WriteFlash(writeAddr, &MDFWBinSize);
		        		for(int i=0; i<7000;i++){}

						Send_STX();

						stx_cnt++;
					}
				}

			}
			else{
				Send_NACK(0, stx_cnt);//STX
				stx_cnt=0;
			}
			break;

		case Info_MSG:
			if (Unpack_InfoMsg(fnc_code, fdcan_rx_test_buf) == 0) {
			} else{
				//Send NACK to CM
			}
			break;

		case Data_MSG:
			if (Unpack_DataMsg(fnc_code, fdcan_rx_test_buf) == 0) {
			} else{
				//Send NACK to CM
			}
			break;

//		case EOT:
//			//receive EOT, do CRC on whole file
//			if(Unpack_EOT(fnc_code, fdcan_rx_test_buf)==0){
//			} else{
//				//Send NACK to CM
//			}
//			break;

		default: break;
	}

	return 0;
}


static void ProcessReceivedData(const uint8_t* buffer, uint8_t* output_buffer, uint32_t size) {
    if (size != 64) {
        // Ensure the buffer is exactly 64 bytes
        return;
    }

    // Copy the original buffer into the output buffer
    memcpy(output_buffer, buffer, size);

    uint8_t temp;

    // 1. Swap the first 4 bytes
    temp = output_buffer[0];
    output_buffer[0] = output_buffer[3];
    output_buffer[3] = temp;

    temp = output_buffer[1];
    output_buffer[1] = output_buffer[2];
    output_buffer[2] = temp;

    // 2. Swap the next 4 bytes (index 4 to 7)
    temp = output_buffer[4];
    output_buffer[4] = output_buffer[7];
    output_buffer[7] = temp;

    temp = output_buffer[5];
    output_buffer[5] = output_buffer[6];
    output_buffer[6] = temp;

    // 3. Swap the next 2 bytes (index 8 and 9)
    temp = output_buffer[8];
    output_buffer[8] = output_buffer[9];
    output_buffer[9] = temp;

    // 4. Swap the next 2 bytes (index 10 and 11)
    temp = output_buffer[10];
    output_buffer[10] = output_buffer[11];
    output_buffer[11] = temp;

    // 5. Swap the last 2 bytes (index 62 and 63)
    temp = output_buffer[62];
    output_buffer[62] = output_buffer[63];
    output_buffer[63] = temp;
}

static int Unpack_InfoMsg(uint32_t t_fnccode, uint8_t* t_buff){
    int ret=0;
    int t_cursor = 0;

    MD_boot_state=BOOT_INFO;
	uint32_t t_file_size;
	uint32_t t_start_addr_offset;
	uint16_t t_file_crc;// (total)
	uint16_t t_total_data_index;// (number of data frames)
	fw_version_t t_fwversion;
	//nothing for 47 byte
	uint16_t t_infomsgcrc;// (current)
	uint16_t t_infomsgcrc_compare=0;// (current)
	uint8_t INFO_outputBuff[64]={0,};

	ProcessReceivedData(t_buff, INFO_outputBuff, 64);

	memcpy(&t_file_size, &INFO_outputBuff[t_cursor],sizeof(t_file_size));
	t_cursor += sizeof(t_file_size);
	INFO_filesize =t_file_size;
	MD_FWInfoObj.fw_size=t_file_size;

	memcpy(&t_start_addr_offset, &INFO_outputBuff[t_cursor],sizeof(t_start_addr_offset));
	t_cursor += sizeof(t_start_addr_offset);
	INFO_startaddroffset = t_start_addr_offset;
	MD_FWInfoObj.fw_startAddr=t_start_addr_offset;

	memcpy(&t_file_crc, &INFO_outputBuff[t_cursor],sizeof(t_file_crc));
	t_cursor += sizeof(t_file_crc);
	INFO_filecrc = t_file_crc;
	MD_FWInfoObj.fw_crc=t_file_crc;

	memcpy(&t_total_data_index, &INFO_outputBuff[t_cursor],sizeof(t_total_data_index));
	t_cursor += sizeof(t_total_data_index);
	INFO_totaldataindex = t_total_data_index;

	memcpy(&t_fwversion, &INFO_outputBuff[t_cursor],sizeof(t_total_data_index));
	t_cursor += sizeof(t_fwversion);
	INFO_fwversion = t_fwversion;
	MD_FWInfoObj.app_fw_ver = t_fwversion;

	t_cursor += 47;
	memcpy(&t_infomsgcrc, &INFO_outputBuff[t_cursor],sizeof(t_infomsgcrc));
	t_cursor += sizeof(t_infomsgcrc);
	INFO_msgcrc = t_infomsgcrc;

	//get CRC
	t_infomsgcrc_compare = Calculate_CRC16(t_infomsgcrc_compare, t_buff, 0, 62);
	INFO_msgcrc_compare= t_infomsgcrc_compare;

	//if CRC ok ACK, else NACK send
	uint8_t retrial=0;
	uint32_t wr_addr =0;

	if(INFO_msgcrc == INFO_msgcrc_compare){
		//Erase flash sector of new fw
		uint8_t sector_idx = 0; // the sector where i want to write the new firmware
		uint8_t erase_sector = (MD_FWInfoObj.fw_size + SUIT_APP_FW_INFO_SIZE) / (uint32_t) STM32H743_IFLASH_SECTOR_SIZE + 1;
		while(sector_idx < erase_sector)
		{
			if(IOIF_EraseFlash(IOIF_FLASH_SECTOR_1_BANK1_ADDR + (sector_idx * STM32H743_IFLASH_SECTOR_SIZE), false) != IOIF_FLASH_STATUS_OK)
			{
				return ret = BOOT_UPDATE_ERROR_FLASH_ERASE;
			}
			IOIF_SetCurrFlashAddr(0);

			f_index = 0;


			sector_idx++;
		}

		/* Write Addr : F/W App. Address + Info Address + SOME OTHER SECTOR*/
		wr_addr = IOIF_FLASH_SECTOR_1_BANK1_ADDR;//IOIF_FLASH_SECTOR_5_BANK1_ADDR + SUIT_APP_FW_INFO_SIZE + f_index + SUIT_APP_FW_BLANK_SIZE;

		if (IOIF_WriteFlashMassBuffered(wr_addr, &MD_FWInfoObj, SUIT_APP_FW_INFO_SIZE, 1) != IOIF_FLASH_STATUS_OK)
		{
			return BOOT_UPDATE_ERROR_FLASH_WRITE;
		}

		int cursor2=0;
		//Send ACK
		uint16_t next_idx=1;//DATA_FRAME_IDX_1
		memcpy(&INFO_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
		cursor2+=sizeof(t_fnccode);

		memcpy(&INFO_Txbuf[cursor2], &next_idx, sizeof(next_idx));
		cursor2+=sizeof(next_idx);

		int idx=64-cursor2;

		memset(&INFO_Txbuf[cursor2], 0, idx);
		cursor2+=idx;

		uint16_t t_id = ACK | (MD_nodeID << 4)| (cm_node_id) ;

		if(IOIF_TransmitFDCAN1(t_id, INFO_Txbuf, 64) != 0)
			ret = 100;			// tx error

		MD_Info_ACK_Flag++;
		return ret;
	}
	else{
		//Send NACK
		int cursor2=0;
		uint16_t curr_idx=0; //INFO_FRAME_IDX_0
		memcpy(&INFO_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
		cursor2+=sizeof(t_fnccode);

		memcpy(&INFO_Txbuf[cursor2], &curr_idx, sizeof(curr_idx));
		cursor2+=sizeof(curr_idx);

		memcpy(&INFO_Txbuf[cursor2], &retrial, sizeof(retrial));
		cursor2+=sizeof(retrial);

		retrial++;

		int idx=64-cursor2;
		memset(&INFO_Txbuf[cursor2], 0, idx);//60
		cursor2+=idx;//60

		uint16_t t_id = NACK | (MD_nodeID << 4)| (cm_node_id) ;

		if(IOIF_TransmitFDCAN1(t_id, INFO_Txbuf, 64) != 0)
			ret = 100;			// tx error

		MD_Info_NACK_Flag++;
		return ret;
	}
	return ret;
}

static int Unpack_DataMsg(uint32_t t_fnccode, uint8_t* t_buff){
	int ret=0;
	int t_cursor = 0;
	MD_boot_state=BOOT_DATA;

	uint16_t t_dataindexnumber=0;
	uint16_t t_datamsgcrc=0;
	uint16_t t_datamsgcrc_compare=0;// (current)

	memcpy(&t_dataindexnumber, &t_buff[t_cursor],sizeof(t_dataindexnumber));
	t_cursor += sizeof(t_dataindexnumber);
	DATA_indexnumber=t_dataindexnumber;

    // Check if the received index is not the expected index
    if (t_dataindexnumber != expected_index) {
        // Send NACK if the index is out of order
        int cursor2 = 0;
        uint8_t retrial = 0;

        uint16_t curr_idx = expected_index; // Report the expected index in the NACK
        memcpy(&DATA_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
        cursor2 += sizeof(t_fnccode);

        memcpy(&DATA_Txbuf[cursor2], &curr_idx, sizeof(curr_idx));
        cursor2 += sizeof(curr_idx);

        memcpy(&DATA_Txbuf[cursor2], &retrial, sizeof(retrial));
        cursor2 += sizeof(retrial);

        retrial++;

        int idx = 64 - cursor2;
        memset(&DATA_Txbuf[cursor2], 0, idx); // Pad with zeros
        cursor2 += idx;

        uint16_t t_id = NACK | (MD_nodeID << 4) | (cm_node_id);

        if (IOIF_TransmitFDCAN1(t_id, DATA_Txbuf, 64) != 0) {
            ret = 100; // TX error
        }

        MD_Data_NACK_Flag++;
        return ret; // Exit the function after sending NACK
    }

    // Update the expected index for the next iteration
    expected_index = t_dataindexnumber + 1;

    memcpy(&DATA_Rxbuf, &t_buff[t_cursor],sizeof(DATA_Rxbuf));
	t_cursor += sizeof(DATA_Rxbuf);

	t_datamsgcrc= ((uint16_t)t_buff[t_cursor] << 8) | t_buff[t_cursor+1];
	t_cursor += sizeof(t_datamsgcrc);
	DATA_msgcrc=t_datamsgcrc;

	//get CRC
	t_datamsgcrc_compare = Calculate_CRC16(t_datamsgcrc_compare, DATA_Rxbuf, 0, sizeof(DATA_Rxbuf));//sizeof(t_buff));
	DATA_msgcrc_compare=t_datamsgcrc_compare;
	//if CRC ok ACK, else NACK send
	uint8_t retrial=0;

	if(DATA_msgcrc == DATA_msgcrc_compare){
		t_datamsgcrc_compare=0;
		//Write in flash sector for new fw
		fw_bin_size = INFO_filesize;

		while(f_index < fw_bin_size)
		{
			wr_size = 0;
			uint32_t wr_addr = 0;

//			wr_size = ((fw_bin_size-f_index) < 0) ? 0 : ((fw_bin_size-f_index > 60) ? 60 : fw_bin_size-f_index);

			if (fw_bin_size - f_index < 0) {
			    wr_size = 0;
			} else if (fw_bin_size - f_index > 60) {
			    wr_size = 60;
			} else {
			    wr_size = fw_bin_size - f_index;
			}

			/* Write Addr : F/W App. Address + Info Address + SOME OTHER SECTOR*/
			wr_addr = IOIF_FLASH_SECTOR_1_BANK1_ADDR + SUIT_APP_FW_INFO_SIZE + f_index; //+ SUIT_APP_FW_INFO_SIZE + f_index + SUIT_APP_FW_BLANK_SIZE;

		    uint8_t triggerWrite = (f_index + wr_size >= fw_bin_size); // Trigger if last chunk////0//for overwrite and only last chunk //1;//for padded //
		    DATA_triggerWrite=triggerWrite;
		    if (IOIF_WriteFlashMassBuffered(wr_addr, &DATA_Rxbuf[f_index % sizeof(DATA_Rxbuf)], wr_size, triggerWrite) != IOIF_FLASH_STATUS_OK)
		       {
		           return BOOT_UPDATE_ERROR_FLASH_WRITE;
		       }
			f_index += wr_size;
			break;
		}
		//Send ACK
		int cursor2=0;
		uint16_t next_idx=DATA_indexnumber+1;//DATA_FRAME_IDX_1
		memcpy(&DATA_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
		cursor2+=sizeof(t_fnccode);

		memcpy(&DATA_Txbuf[cursor2], &next_idx, sizeof(next_idx));
		cursor2+=sizeof(next_idx);

		int idx=64-cursor2;

		memset(&DATA_Txbuf[cursor2], 0, idx);
		cursor2+=idx;

		uint16_t t_id = ACK | (MD_nodeID << 4) | (cm_node_id);

		if(IOIF_TransmitFDCAN1(t_id, DATA_Txbuf, 64) != 0)
			ret = 100;			// tx error

		MD_Data_ACK_Flag++;

		if(f_index==fw_bin_size){
			//end of write
			f_index = 0;
			MD_boot_state=BOOT_EOT;


		}
	}
	else{
		t_datamsgcrc_compare=0;
		//Send NACK
		int cursor2=0;
		uint16_t curr_idx=DATA_indexnumber; //INFO_FRAME_IDX_0
		memcpy(&DATA_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
		cursor2+=sizeof(t_fnccode);

		memcpy(&DATA_Txbuf[cursor2], &curr_idx, sizeof(curr_idx));
		cursor2+=sizeof(curr_idx);

		memcpy(&DATA_Txbuf[cursor2], &retrial, sizeof(retrial));
		cursor2+=sizeof(retrial);

		retrial++;

		int idx=64-cursor2;
		memset(&DATA_Txbuf[cursor2], 0, idx);//60
		cursor2+=idx;//60

		uint16_t t_id = NACK | (MD_nodeID << 4) | (cm_node_id) ;

		if(IOIF_TransmitFDCAN1(t_id, DATA_Txbuf, 64) != 0)
			ret = 100;			// tx error

		MD_Data_NACK_Flag ++;

	}
	return ret;
}

static int Unpack_EOT(uint32_t t_fnccode, uint8_t* t_buf){
	int ret=0;
	MD_boot_state=BOOT_EOT;
	//if CRC ok ACK, else NACK send
	uint8_t retrial=0;

	if(Boot_UpdateVerify((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR)==BOOT_UPDATE_OK){
//		int cursor2=0;
//		//Send ACK
//		uint16_t next_idx=1;//DATA_FRAME_IDX_1
//		memcpy(&EOT_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
//		cursor2+=sizeof(t_fnccode);
//
//		memcpy(&EOT_Txbuf[cursor2], &next_idx, sizeof(next_idx));
//		cursor2+=sizeof(next_idx);
//
//		int idx=64-cursor2;
//
//		memset(&EOT_Txbuf[cursor2],0, idx);//61
//		cursor2+=idx;//61;
//
//		uint16_t t_id = ACK | (MD_nodeID << 4) | (cm_node_id) ;
//
//		if(IOIF_TransmitFDCAN1(t_id, EOT_Txbuf, 64) != 0)
//			ret = 100;			// tx error
//
//		MD_EOT_ACK_Flag ++;//= 1;

		*MD_UPDATE_FLAG = 2;

		for(int i=0; i<70000; i++){

		}

		MD_Update_Flag = 0;
		DATA_WriteDone=1;
	}
	else{
		if(Boot_EraseCurrentMDFW((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR,MD_FWInfoObj.fw_size)==BOOT_UPDATE_OK){
			if(Boot_SaveNewMDFW((uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR,(uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR,MD_FWInfoObj.fw_size)==BOOT_UPDATE_OK){
				{
					//Send NACK
					int cursor2=0;
					uint16_t curr_idx=0; //INFO_FRAME_IDX_0
					memcpy(&EOT_Txbuf[cursor2], &t_fnccode, sizeof(t_fnccode));
					cursor2+=sizeof(t_fnccode);

					memcpy(&EOT_Txbuf[cursor2], &curr_idx, sizeof(curr_idx));
					cursor2+=sizeof(curr_idx);

					memcpy(&EOT_Txbuf[cursor2], &retrial, sizeof(retrial));
					cursor2+=sizeof(retrial);

					retrial++;

					int idx=64-cursor2;
					memset(&EOT_Txbuf[cursor2], 0, idx);
					cursor2+=idx;

					uint16_t t_id = NACK | (MD_nodeID << 4) | (cm_node_id) ;

					if(IOIF_TransmitFDCAN1(t_id, EOT_Txbuf, 64) != 0)
						ret = 100;			// tx error

					MD_EOT_NACK_Flag++;
				}
			}
		}
	}
	return ret;
}

