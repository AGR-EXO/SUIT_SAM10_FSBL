/**
 *-----------------------------------------------------------
 *                 PCA9957HNMP LED DRIVER
 *-----------------------------------------------------------
 * @file ioif_pca9957hnmp.c
 * @date Created on: Sep 14, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the PCA9957HNMP LED DRIVER.
 *
 * Todo: Add Annotation
 *
 * Refer to the PCA9957HNMP datasheet and related documents for more information.
 *
 * @ref PCA9957HNMP Datasheet
 */

/* CAUTION!!
 * "IOIF_InitLEDDriver" function must be used.
 */


#include "ioif_pca9957hnmp.h"

/** @defgroup I2C I2C
  * @brief I2C ICM20608G module driver
  * @{
  */

#ifdef IOIF_PCA9957HNMP_ENABLED
/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

IOIF_SPI_t spiHandle = IOIF_SPI1;
PCA9957HNMP_CallbackStruct LED_Driver_Callback;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static uint8_t LedCmdTxData[2] = {0,0};
static uint8_t LedCmdRxData[2] = {0,0};
//static uint8_t LedCmdTxData[2] __attribute__((section(".spi1TxBuff"))) = {0};
//static uint8_t LedCmdRxData[2] __attribute__((section(".spi1RxBuff"))) = {0};


#ifdef _USE_DEBUG_CLI
static IOIF_LEDState_t moduleinit_res = IOIF_LED_STATUS_ERROR;
#endif /*_USE_DEBUG_CLI*/
/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
/*Callback Regist*/
static void SpiLedTxCB(void* params);
static void SpiLedTxRxCB(void* params);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

//Test Tagging Branch

bool IOIF_LED24chSPIStart(uint8_t reg, uint8_t tdata, uint8_t* rdata, uint8_t timeout, PCA9957HNMP_SPIOP_t operation)
{

	bool ret = true;

	uint8_t packet_size = 2;
	//uint8_t tx_data[2] = {reg, tdata};
	LedCmdTxData[0] = reg; LedCmdTxData[1] = tdata;
	//uint8_t rx_data[2] = {0,};

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET); 		 //CS is low,

	switch(operation)
	{
	case BSP_SPI_TRANSMIT :
		if(BSP_RunSPIBlock((BSP_SPI_t)spiHandle, LedCmdTxData, rdata, packet_size, 1000, (BSP_SPIOP_t)operation) != BSP_OK)
			ret = false;
		break;
	case BSP_SPI_RECEIVE :
	case BSP_SPI_TRANSMIT_RECEIVE :
		//if(BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, rdata, packet_size, (BSP_SPIOP_t)operation) != BSP_OK)
		if(BSP_RunSPIBlock((BSP_SPI_t)spiHandle, LedCmdTxData, rdata, packet_size, 1000, (BSP_SPIOP_t)operation) != BSP_OK)
			ret = false;
		break;
	case BSP_SPI_TRANSMIT_IT :
	case BSP_SPI_RECEIVE_IT :
	case BSP_SPI_TRANSMIT_RECEIVE_IT :
	case BSP_SPI_TRANSMIT_DMA :
	case BSP_SPI_RECEIVE_DMA :
	case BSP_SPI_TRANSMIT_RECEIVE_DMA :
		if(BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, rdata, packet_size, operation) != BSP_OK)
			ret = false;
		break;
	default:
		ret = false;
		break;
	}

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_SET); 		//CS becomes high after transmission complete

	return ret;
}

bool IOIF_LED24chnOECtrl(bool state)
{
	bool ret = true;
	/* Active Low, Output Enable Port : GPIOD, GPIO 9 */
	if(state == true)
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nOE_Pin, IOIF_GPIO_PIN_SET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}
	else
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nOE_Pin, IOIF_GPIO_PIN_RESET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}

	return ret;
}

bool IOIF_LED24chnRSTCtrl(bool state)
{
	bool ret = true;
	/* Active Low, Reset Port : GPIOD, GPIO 9 */
	if(state == true)
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nRESET_Pin, IOIF_GPIO_PIN_SET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}
	else
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nRESET_Pin, IOIF_GPIO_PIN_RESET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}

	return ret;
}

IOIF_LEDState_t IOIF_LED24chSPIDMAStart(IOIF_SPI_t spiChannel)
{
	IOIF_LEDState_t ret = IOIF_OK;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);		// cs low

	if(BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, LedCmdRxData, sizeof(LedCmdRxData), BSP_SPI_TRANSMIT_RECEIVE_DMA) != BSP_OK)
		return ret = IOIF_LED_STATUS_ERROR;

	return ret;
}

IOIF_LEDState_t IOIF_LED24chDriverInit(IOIF_SPI_t spiChannel)
{
	IOIF_LEDState_t init_res = IOIF_LED_STATUS_OK;

	/* SPI DMA Mode Callback Driver Registration */
//	BSP_SetSPICB(spiHandle, BSP_SPI_TX_CPLT_CALLBACK, SpiLedTxCB, NULL);		// Register TX Complete Callback
//	BSP_SetSPICB(spiHandle, BSP_SPI_TXRX_CPLT_CALLBACK, SpiLedTxRxCB, NULL);	// Register TX-RX Complete Callback

//	if(IOIF_LED24chSPIDMAStart(spiHandle) != IOIF_LED_STATUS_OK)
//		return IOIF_LED_STATUS_ERROR;

	//Todo : 함수인자 맞는지 확인

	/* LED Driver Init. */
	LED_Driver_Callback.pca9957hnmp_spi_op 	  = IOIF_LED24chSPIStart;
	LED_Driver_Callback.pca9957hnmp_nOE_ctrl  = IOIF_LED24chnOECtrl;
	LED_Driver_Callback.pca9957hnmp_nRST_ctrl = IOIF_LED24chnRSTCtrl;
	LED_Driver_Callback.pca9957hnmp_delay     = IOIF_LED24chDeviceDelay;

	PCA9957HNMP_RegisterCallback(&LED_Driver_Callback);

	if(PCA9957HNMP_InitDrv() != true)
		init_res = IOIF_LED_STATUS_ERROR;

#ifdef _USE_DEBUG_CLI
	moduleinit_res = init_res;
#endif /*_USE_DEBUG_CLI*/

	return init_res;
}


void IOIF_LED24chDeviceDelay(uint32_t ms_delay)
{
#ifdef _USE_OS_RTOS
	osDelay(ms_delay);
#else
	HAL_Delay(ms_delay);
#endif
}


//void IOIF_LED24chErrorDetect(){ //should be tested
//
//	static uint8_t led_error_flag[2] = {0};
//	static bool Is_led_output_error = false;
//	static bool Is_led_overtemp_error =false;
//
//	PCA9957HNMP_ReadReg(MODE2, 0xff, led_error_flag);
//
//	//if error detected,
//		if (led_error_flag[1] == 0b01001001) {
//			Is_led_output_error = true;	//breakpoint
//			PCA9957HNMP_ReadReg(EFLAG0, 0xff, ledg0_error_flag);
//			PCA9957HNMP_ReadReg(EFLAG1, 0xff, ledg1_error_flag);
//			PCA9957HNMP_ReadReg(EFLAG2, 0xff, ledg2_error_flag);
//			PCA9957HNMP_ReadReg(EFLAG3, 0xff, ledg3_error_flag);
//			PCA9957HNMP_ReadReg(EFLAG4, 0xff, ledg4_error_flag);
//		} else if (led_error_flag[1] == 0b10001001) {
//			Is_led_overtemp_error = true; //breakpoint
//		}
//}


/*LED Control*/
void IOIF_LED24chBattery(uint8_t nth, IOIF_LED24chColor_t color, uint8_t brightness) //Todo: 함수인자 구조체로 받기
{
	uint8_t pwm  ;
	uint8_t iref ;

	/*if (p_color != color) {
		for (int i = 0; i < 5; i++) {
			pwm  = i + 16;
			iref = i + 40;
			PCA9957HNMP_WriteReg(pwm,  0xFF);
			PCA9957HNMP_WriteReg(iref, brightness);
		}
	}*/
	for (int i = 0; i < 5; i++) {
		pwm  = i + 16;
		iref = i + 40;
		PCA9957HNMP_WriteReg(pwm,  0x00);
		PCA9957HNMP_WriteReg(iref, 0x00);
	}


	switch(color){
	case GREEN:
		for (int i = 1; i < nth+1; i++) {
			pwm  = i * 2 + 14;
			iref = i * 2 + 38;
			PCA9957HNMP_WriteReg(pwm,  0xFF);
			PCA9957HNMP_WriteReg(iref, brightness);
		}
		break;
	case RED:
		for (int i = 1; i < nth+1; i++) {
			pwm  = i * 2 + 14 + 1;
			iref = i * 2 + 38 + 1;
			PCA9957HNMP_WriteReg(pwm,  0xFF);
			PCA9957HNMP_WriteReg(iref, brightness);
		}

		break;
	default:
		break;
	}
}

void IOIF_LED24chError(IOIF_LED24chColor_t color, uint8_t brightness){  //Todo: 함수인자 구조체로 받기

	static uint8_t p_color;
		if (p_color != color) {
			PCA9957HNMP_WriteReg((uint8_t)IREF6, 0x00);
			PCA9957HNMP_WriteReg((uint8_t)IREF7, 0x00);
			PCA9957HNMP_WriteReg((uint8_t)PWM6,  0x00);
			PCA9957HNMP_WriteReg((uint8_t)PWM7,  0x00);
		}

	switch(color){
	case GREEN:
		PCA9957HNMP_WriteReg((uint8_t)IREF6, brightness);
		PCA9957HNMP_WriteReg((uint8_t)PWM6,  0xFF);

		break;
	case RED:
		PCA9957HNMP_WriteReg((uint8_t)IREF7, brightness);
		PCA9957HNMP_WriteReg((uint8_t)PWM7,  0xFF);

		break;

	default:
			break;
	}

	p_color = color;

}

void IOIF_LED24chAssist(uint8_t nth, uint8_t brightness)
{
	/*ON*/
	for (int i = 1; i < nth+1; i++) {
		uint8_t pwm  = i + 23;
		uint8_t iref = i + 47;
		PCA9957HNMP_WriteReg(pwm,  0xFF);
		PCA9957HNMP_WriteReg(iref, brightness);
	}

	/*OFF*/
	for (int j = nth + 1 ; j < 10 +1 ; j++) { //10:LED num
		uint8_t off_pwm = j + 23;
		uint8_t off_iref = j + 47;
		PCA9957HNMP_WriteReg(off_pwm,  0x00);
		PCA9957HNMP_WriteReg(off_iref, 0x00);
	}
}


void IOIF_LED24chBluetooth(uint8_t brightness)
{
	PCA9957HNMP_WriteReg((uint8_t)IREF18, brightness);
	PCA9957HNMP_WriteReg((uint8_t)PWM18,  0xFF);
}

void IOIF_LED24chMode(IOIF_LED24chColor_t color, uint8_t brightness){

	static uint8_t p_color;
	if (p_color != color) {
		PCA9957HNMP_WriteReg((uint8_t)IREF19, 0x00);
		PCA9957HNMP_WriteReg((uint8_t)IREF20, 0x00);
		PCA9957HNMP_WriteReg((uint8_t)IREF21, 0x00);
		PCA9957HNMP_WriteReg((uint8_t)IREF22, 0x00);
		PCA9957HNMP_WriteReg((uint8_t)PWM19,  0x00);
		PCA9957HNMP_WriteReg((uint8_t)PWM20,  0x00);
		PCA9957HNMP_WriteReg((uint8_t)PWM21,  0x00);
		PCA9957HNMP_WriteReg((uint8_t)PWM22,  0x00);
	}

	switch(color){
	case WHITE:
		PCA9957HNMP_WriteReg((uint8_t)IREF19, brightness);
		PCA9957HNMP_WriteReg((uint8_t)PWM19,  0xFF);

		break;
	case BLUE:
		PCA9957HNMP_WriteReg((uint8_t)IREF20, brightness);
		PCA9957HNMP_WriteReg((uint8_t)PWM20,  0xFF);

		break;
	case RED:
		PCA9957HNMP_WriteReg((uint8_t)IREF21, brightness);
		PCA9957HNMP_WriteReg((uint8_t)PWM21,  0xFF);

		break;
	case GREEN:
		PCA9957HNMP_WriteReg((uint8_t)IREF22, brightness);
		PCA9957HNMP_WriteReg((uint8_t)PWM22,  0xFF);

		break;
	default:
			break;
	}

	p_color = color;
}

/* test needed
 void IOIF_LED24ch4Blink(uint16_t interval_time, FunctionPointer led_drive_func, uint8_t color, uint8_t brightness ){

	static uint16_t cnt = 0;

	if (cnt < interval_time) {
		led_drive_func(color,brightness);
	} else if (cnt < interval_time * 2) {
		led_drive_func(color,0x00);
	} else {
		cnt = 0;
	}

	cnt++;
}

void IOIF_LED24ch4BlinkAssist(uint16_t interval_time, uint8_t nth, uint8_t brightness ){

	static uint16_t cnt = 0;

	if (cnt < interval_time) {
		Drive_LED_Auxiliary(nth,brightness);
	} else if (cnt < interval_time * 2) {
		Drive_LED_Auxiliary(nth,0x00);
	} else {
		cnt = 0;
	}

	cnt++;
}

*/

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/*Callback Function*/
static void SpiLedTxCB(void* params)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_SET);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);
	/* Re-start DMA */
	BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, LedCmdRxData, sizeof(LedCmdRxData), BSP_SPI_TRANSMIT_DMA);

}

static void SpiLedTxRxCB(void* params)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_SET);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRV_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);	// cs becomes low if transmit is starting,
	/* Re-start DMA */
	BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, LedCmdRxData, sizeof(LedCmdRxData), BSP_SPI_TRANSMIT_RECEIVE_DMA);
}


#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

void CLI_RunPca9957hnmp(cli_args_t *args)
{
	bool ret = false;

	uint8_t brigthness = 0;

	if(args->cmpStr(0, "isinit") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		IOIF_StatusTypeDef_t state = moduleinit_res;

		const char* ledStateStrings[] = {
				"IOIF_LED_STATUS_OK",
				"IOIF_LED_STATUS_ERROR",
				"IOIF_LED_STATUS_BUSY",
				"IOIF_LED_STATUS_TIMEOUT"
		};

		CLI_Printf(" * PCA9957HNMP init state : %s * \r\n", ledStateStrings[state]);

	} else if (args->cmpStr(0, "battery") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;

		if 		(args->cmpStr(1, "green") == true) IOIF_LED24chBattery(3, GREEN, brigthness);
		else if (args->cmpStr(1, "red") == true) IOIF_LED24chBattery(3, RED, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	} else if (args->cmpStr(0, "error") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;

		if 		(args->cmpStr(1, "green") == true) IOIF_LED24chError(GREEN, brigthness);
		else if (args->cmpStr(1, "red") == true) IOIF_LED24chError(RED, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	} else if (args->cmpStr(0, "mode") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;

		if 		(args->cmpStr(1, "white") == true) IOIF_LED24chMode(WHITE, brigthness);
		else if (args->cmpStr(1, "blue") == true) IOIF_LED24chMode(BLUE, brigthness);
		else if (args->cmpStr(1, "red") == true) IOIF_LED24chMode(RED, brigthness);
		else if (args->cmpStr(1, "green") == true) IOIF_LED24chMode(GREEN, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	}else if (args->cmpStr(0, "assist") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		uint8_t stage = 0;

		stage = (uint8_t) args->getData(2);
		if(stage > 10) stage = 10;
		brigthness = (uint8_t) args->getData(3);
		if (brigthness > 255) brigthness = 255;

		IOIF_LED24chAssist(stage, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	} else if (args->cmpStr(0, "ble") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(1);
		if (brigthness > 255) brigthness = 255;

		IOIF_LED24chBluetooth(brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	}  else if (args->cmpStr(0, "all") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		if (args->cmpStr(1, "on") == true)
		{
			brigthness = (uint8_t) args->getData(2);
			if (brigthness > 255) brigthness = 255;
			PCA9957HNMP_WriteReg(PWMALL, 0xFF);
			PCA9957HNMP_WriteReg(IREFALL, brigthness);

		} else if (args->cmpStr(1, "off") == true)
		{
			PCA9957HNMP_WriteReg(PWMALL, 0x00);
			PCA9957HNMP_WriteReg(IREFALL, 0x00);
		}
		CLI_Printf(" * Check the LED on board * \r\n");
	}

	  else if (ret == false) //help
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		CLI_Printf(" * LED Display * \r\n");
		CLI_Printf("   Range of data : brightness (0~255), stage(0~10) \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   pca9957hnmp isinit \r\n");
		CLI_Printf("   pca9957hnmp [battery/error] [green/red] [brightness] \r\n");
		CLI_Printf("   pca9957hnmp mode [green/red/white/blue] [brightness]\r\n");
		CLI_Printf("   pca9957hnmp assist led [stage] [brightness]\r\n");
		CLI_Printf("   pca9957hnmp ble [brightness] \r\n");
		CLI_Printf("   pca9957hnmp all on [brightness] \r\n");
		CLI_Printf("   pca9957hnmp all off \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   pca9957hnmp error red 50 \r\n");
		CLI_Printf("   pca9957hnmp mode white 50 \r\n");
		CLI_Printf("   pca9957hnmp assist led 5 10 \r\n");
	}

}

#endif /*_USE_DEBUG_CLI*/
#endif /* IOIF_PCA9957HNMP_ENABLED */
