/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "quadspi.h"
#include "sai.h"
#include "spi.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "boot.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Define the same flash address where the variable is stored */
#define MDUPDATEFLAG_SWITCH  ((uint32_t*)0x38000000)
#define MDFWBINSIZE  ((uint32_t*)0x38000004)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


BootUpdateState		boot_state = BOOT_MD_UPDATE;
bool				boot_is_Upgrade = false;
bool				boot_is_jump = false;

uint32_t time_difference = 0;
uint32_t MDUpdateFlag=0;

uint32_t FW_Update_Flag = 0;
uint32_t FW_Backup_Flag = 0;
//uint32_t FW_EOT_Flag = 0;

uint32_t FW_Copy_Flag = 0;
uint32_t MD_Update = 0;
uint32_t MD_Backup = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint32_t ReadMDUpdateFlag();
uint32_t ReadMDFWBinSize();
void BL_LED_Blinking();
void MD_ReadFlags();
void MD_EraseWriteFW();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	uint32_t upgrade_pretime = 0;
	uint32_t upgrade_current_time=0;

	uint32_t main_start_time=0;

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_FDCAN1_Init();
	/* USER CODE BEGIN 2 */

	__enable_irq();
	main_start_time = HAL_GetTick();

	/* HW Init. */
	if(Boot_HWInit() != true)
		boot_state = BOOT_ERROR;

	uint8_t cnt=0;

	MD_ReadFlags();

    if((FW_Update_Flag == 0xFFFFFFFF)&&(FW_Backup_Flag == 0xFFFFFFFF)){//0x0)&&(FW_Backup_Flag ==0x0)){
    	//jump to App1
		Boot_JumpToApp(IOIF_FLASH_SECTOR_1_BANK1_ADDR);
    }
    else if((FW_Update_Flag == 1)&&(FW_Backup_Flag == 1)){
    	//Send trigger to CM to stop FW Update or restart, MD entering recovery mode
    	Send_Recovery(0);
    	MD_EraseWriteFW();
   	}

    if(FW_Update_Flag == 1){
    	MD_Update_Flag=1;
    	boot_state = BOOT_MD_UPDATE;
    }

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		BL_LED_Blinking();
		/* 1. Check Boot Mode */
		if(boot_state != BOOT_ERROR){
			boot_state = Boot_CheckUpdateMode();
		}

		if(boot_state == BOOT_MD_UPDATE){
			boot_is_Upgrade = true;

			//App1 copy from sector 1 to sector 5
			if(Boot_EraseCurrentMDFW((uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR,MDFWBinSize)==BOOT_UPDATE_OK){
				if(Boot_SaveNewMDFW((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR,(uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR,MDFWBinSize)==BOOT_UPDATE_OK){
		    		uint32_t writeAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;
		    		//erase
		    		IOIF_EraseFlash(writeAddr, IOIF_ERASE_ONE_SECTOR);
		    		for(int i=0; i<7000;i++){}

		    		//write
					MD_Update = 0x1;
		    		IOIF_WriteFlash(writeAddr, &MD_Update);
		    		for(int i=0; i<7000;i++){}

		    		writeAddr+=32;
					MD_Backup = 0x1;//0000000000000000000000000000000000000000000000000000000000000001;
		    		IOIF_WriteFlash(writeAddr, &MD_Backup);
		    		for(int i=0; i<7000;i++){}

		    		writeAddr+=32;
		    		IOIF_WriteFlash(writeAddr, &MDFWBinSize);
		    		for(int i=0; i<7000;i++){}

					Send_STX();

					MD_Update_Flag = 0;
				}
			}
			else{
				Send_NACK(0, cnt);
				cnt++;
			}
		}

		/* Error Handler */
		else if(boot_state == BOOT_ERROR)					//BOOT_ERROR
		{
			while(1)
			{
				HAL_Delay(2000);
			}
		}

		uint8_t retrial =0;

		if(MD_boot_state==BOOT_EOT){
			if(Boot_UpdateVerify((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR)==BOOT_UPDATE_OK){
				if(Boot_JumpToApp(IOIF_FLASH_SECTOR_1_BANK1_ADDR) != BOOT_UPDATE_OK)
					boot_state = BOOT_ERROR;
			}
			else{
				//Send NACK
				int cursor2=0;
				uint16_t curr_idx=0; //INFO_FRAME_IDX_0
				uint8_t EOT_Txbuf[64]={0,};
				uint16_t t_fnccode = (uint16_t)0x500U;
				memcpy(&EOT_Txbuf[cursor2], &t_fnccode , sizeof(t_fnccode));
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

				if(IOIF_TransmitFDCAN1(t_id, EOT_Txbuf, 64) != 0){}
				//								ret = 100;			// tx error

				MD_EOT_NACK_Flag++;
				MD_boot_state = 0;
			}
		}
		/* USER CODE END WHILE */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t ReadMDUpdateFlag()
{
    return *MDUPDATEFLAG_SWITCH;
}

uint32_t ReadMDFWBinSize()
{
    return *MDFWBINSIZE;
}

void BL_LED_Blinking() {
    static uint32_t previous_time = 0;
    const uint32_t blink_interval = 250;

    uint32_t current_time = HAL_GetTick();  // Get current time

    if ((current_time - previous_time) >= blink_interval) {
        IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13);  // Toggle LED
        previous_time = current_time;  // Update last toggle time
    }
}


void MD_ReadFlags(){
    uint32_t readAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;

    IOIF_ReadFlash(readAddr, &FW_Update_Flag,       	IOIF_FLASH_READ_SIZE_4B);
    readAddr += IOIF_FLASH_READ_SIZE_32B;
    IOIF_ReadFlash(readAddr, &FW_Backup_Flag,           IOIF_FLASH_READ_SIZE_4B);
    readAddr += IOIF_FLASH_READ_SIZE_32B;
    IOIF_ReadFlash(readAddr, &MDFWBinSize,      	    IOIF_FLASH_READ_SIZE_4B);

}

void MD_EraseWriteFW(){
	//copy App1 to sector 1
	if(Boot_EraseCurrentMDFW((uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR, MDFWBinSize)==BOOT_UPDATE_OK){
		if(Boot_SaveNewMDFW((uint32_t)IOIF_FLASH_SECTOR_5_BANK1_ADDR,(uint32_t)IOIF_FLASH_SECTOR_1_BANK1_ADDR,MDFWBinSize)==BOOT_UPDATE_OK){
    		uint32_t MD_Update=1;
    		uint32_t MD_Backup=1;
//        		uint32_t MD_EOT=2;
    		uint32_t writeAddr = IOIF_FLASH_SECTOR_3_BANK2_ADDR;
    		//erase
    		IOIF_EraseFlash(writeAddr, IOIF_ERASE_ONE_SECTOR);
    		for(int i=0; i<7000;i++){}

    		//write
    		IOIF_WriteFlash(writeAddr, &MD_Update);
    		writeAddr += 32;
    		for(int i=0; i<7000;i++){}

    		//write
    		IOIF_WriteFlash(writeAddr, &MD_Backup);
    		writeAddr += 32;
    		for(int i=0; i<7000;i++){}

    		//write
    		IOIF_WriteFlash(writeAddr, &MDFWBinSize);
    		for(int i=0; i<7000;i++){}

			Boot_JumpToApp(IOIF_FLASH_SECTOR_1_BANK1_ADDR);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
