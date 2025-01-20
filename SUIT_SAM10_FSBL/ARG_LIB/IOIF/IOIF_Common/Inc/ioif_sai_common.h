

#ifndef IOIF_COMMON_INC_IOIF_SAI_COMMON_H_
#define IOIF_COMMON_INC_IOIF_SAI_COMMON_H_

#include "module.h"
#include "bsp_sai.h"
#include "string.h"
#include "ioif_gpio_common.h"

/** @defgroup SAI SAI
  * @brief SAI BSP module driver
  * @{
  */

#ifdef BSP_SAI_MODULE_ENABLED

#ifdef IOIF_SAI_COMMON_ENABLED
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */
#define IOIF_SAI_BUF_LEN   		(1024*4)	// SAI Buffer length : 1024 * 4 bytes
#define IOIF_SAI_BUF_MS    		(10)		// SAI sampling duration : 10ms
#define IOIF_SAI_SAMPLE_FREQ	16000

#define IOIF_SAI_MACROENUMTOSTR(s)	#s

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef enum _IOIF_SAINote{
	//It deviates from the naming convention;
	//However, it is allowed as an exception for readability and CLI terminal display.
	NOTE_C = 1,
	NOTE_Csharp,
	NOTE_D,
	NOTE_Dsharp,
	NOTE_E,
	NOTE_F,
	NOTE_Fsharp,
	NOTE_G,
	NOTE_Gsharp,
	NOTE_A,
	NOTE_Asharp,
	NOTE_B
} IOIF_SAINote_t;

typedef enum _IOIF_SAISoundState_t {
    IOIF_SAI_SOUND_STATUS_OK = 0,
    IOIF_SAI_SOUND_STATUS_ERROR
} IOIF_SAISoundState_t;

#ifdef SUIT_MINICM_ENABLED
typedef enum _IOIF_SAIMAX98357_GPIO_t { //audio popsound ctrl
	IOIF_MAX98357_SDMODE_GPIO_PORT = BSP_GPIO_PORT_E,
	IOIF_MAX98357_SDMODE_GPIO_PIN  = BSP_GPIO_PIN_6,
} IOIF_SAIMAX98357_GPIO_t;
#endif


#ifdef SUIT_CM_BL_ENABLED
typedef enum _IOIF_SAIMAX98357_GPIO_t { //audio popsound ctrl
	IOIF_MAX98357_SDMODE_GPIO_PORT = BSP_GPIO_PORT_E,
	IOIF_MAX98357_SDMODE_GPIO_PIN  = BSP_GPIO_PIN_6,
} IOIF_SAIMAX98357_GPIO_t;
#endif


typedef struct _IOIF_SAIChannel_t{
  int16_t left;
  int16_t right;
} IOIF_SAIChannel_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* Player Init. & Deinit. */
IOIF_SAISoundState_t IOIF_SAI_PlaySoundInit(void);

/* Play Beep & Note */
IOIF_SAISoundState_t IOIF_SAI_PlayNote(int8_t octave, int8_t note, uint16_t volume, uint32_t time_ms);
IOIF_SAISoundState_t IOIF_SAI_PlayBeep(uint32_t freq_hz, uint16_t volume, uint32_t time_ms);

/* Basic Play Functions */
IOIF_SAISoundState_t IOIF_SAI_StartBeep(uint32_t volume);
IOIF_SAISoundState_t IOIF_SAI_ErrorBeep(uint32_t volume);
IOIF_SAISoundState_t IOIF_SAI_OffBeep(uint32_t volume);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunSAI(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /*IOIF_SAI_COMMON_ENABLED*/
#endif /* BSP_SAI_MODULE_ENABLED */

#endif /* IOIF_COMMON_INC_IOIF_SAI_COMMON_H_ */
