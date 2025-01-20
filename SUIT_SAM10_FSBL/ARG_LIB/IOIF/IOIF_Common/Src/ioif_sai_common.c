/*
 * ioif_sai_common.c
 *
 *  Created on: May 2, 2024
 *      Author: Angelrobotics
 */

#include "ioif_sai_common.h"

/** @defgroup SAI SAI
  * @brief SAI BSP module driver
  * @{
  */
#ifdef BSP_SAI_MODULE_ENABLED

#ifdef IOIF_SAI_COMMON_ENABLED

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




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

volatile static uint32_t sai_sample_rate = IOIF_SAI_SAMPLE_FREQ;		// SAI sample rate (can be changed by audio sample rate)
volatile static uint32_t sai_qbuf_in = 0;				// queue buffer index : head
volatile static uint32_t sai_qbuf_out = 0;				// queue buffer index : tail
volatile static uint32_t sai_q_buf_len = 0;				// sampling data queue buffer length calculation
volatile static uint32_t sai_qbuf_len = 0; 				// total size of queue buffer

/* print note name */
static char *NoteStr[] = {
		IOIF_SAI_MACROENUMTOSTR(NOTE_C),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Csharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_D),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Dsharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_E),
		IOIF_SAI_MACROENUMTOSTR(NOTE_F),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Fsharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_G),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Gsharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_A),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Asharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_B),
};

static int16_t sai_q_buf_zero[IOIF_SAI_BUF_LEN*2] __attribute__((section(".Sai1_RxBuff")));		// queue buffer for SAI
static IOIF_SAIChannel_t sai_q_buf[IOIF_SAI_BUF_LEN]__attribute__((section(".Sai1_RxBuff")));		// 2-channel queue buffer for SAI

static bool sai_stop = false;							// SAI DMA transmit start/stop (no use in DMA circular, only normal mode)
static uint8_t sai_r_buf_dummy = 0;						// SAI DMA dummy byte

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
/* Make Beep using SAI */
static float SaiGetNoteHz(int8_t octave, int8_t note);
static float MakeSineWave(float x);

/* SAI TX Complete Callback */
static void SaiAudioDMATxCB(void* param);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_SAISoundState_t IOIF_SAI_PlaySoundInit(void)
{
	IOIF_SAISoundState_t res = IOIF_SAI_SOUND_STATUS_ERROR;

	memset(sai_q_buf_zero, 0, sizeof(sai_q_buf_zero));												//SAI buffer Init.
	memset(sai_q_buf,0, sizeof(sai_q_buf));															//SAI buffer Init.

	sai_q_buf_len = (sai_sample_rate * 1) / (1000/IOIF_SAI_BUF_MS);      								//SAI buffer length calculation
	sai_qbuf_len = IOIF_SAI_BUF_LEN / sai_q_buf_len;														//SAI buffer length calculation

	BSP_SetSAICB(BSP_SAI1, BSP_SAI_TX_CPLT_CALLBACK, SaiAudioDMATxCB, NULL);						//SAI Transmit Complete Callback Register

	if(BSP_RunSAIDMA(BSP_SAI1, (uint8_t*) sai_q_buf_zero, &sai_r_buf_dummy, sai_q_buf_len*2, BSP_SAI_TRANSMIT_DMA) == BSP_OK)
		res = IOIF_SAI_SOUND_STATUS_OK;

	return res;
}


IOIF_SAISoundState_t IOIF_SAI_PlayNote(int8_t octave, int8_t note, uint16_t volume, uint32_t time_ms)
{
  uint32_t pre_time;
  int32_t sample_rate = sai_sample_rate;
  int32_t num_samples = sai_q_buf_len;
  float sample_point;
  int16_t sample_index = 0;
  float div_freq;
  int32_t volume_out;
  uint32_t buf_len;

  volume = (volume < 0) ? 0 : ((volume > 100) ? 100 : volume);
  volume_out = (INT16_MAX / 40) * volume / 100;

  sai_qbuf_in=0; sai_qbuf_out=0;				//sai queue buffer in/out init.

  /* SAI Transmit DMA Start */
	if(BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA) == BSP_OK)
		sai_stop = false;

  div_freq = (float)sample_rate/(float)SaiGetNoteHz(octave, note);
  //pre_time = BSP_GetTick();
  pre_time = HAL_GetTick();

  /* Mute off */
  IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_SET);


  //while(BSP_GetTick() - pre_time <= time_ms)
  while(HAL_GetTick() - pre_time <= time_ms)
  {
    buf_len = ((sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len);
    buf_len = (sai_qbuf_len - buf_len) - 1;

    if (buf_len > 0)
    {
      for (int i=0; i<num_samples; i++)
      {
        sample_point = MakeSineWave(2 * M_PI * (float)(sample_index) / ((float)div_freq));
#ifdef _USE_SOUND_MONO
                sai_q_buf[sai_q_in * sai_q_buf_len + i] = (int16_t)(sample_point * volume_out);
#else
                sai_q_buf[sai_qbuf_in*sai_q_buf_len + i].left  = (int16_t)(sample_point * volume_out);
                sai_q_buf[sai_qbuf_in*sai_q_buf_len + i].right = (int16_t)(sample_point * volume_out);
#endif
        sample_index = (sample_index + 1) % (int)div_freq;
      }

      if (((sai_qbuf_in + 1) % sai_qbuf_len) != sai_qbuf_out)
      {
        sai_qbuf_in = (sai_qbuf_in+1) % sai_qbuf_len;
      }
    }
  }

  /* Mute on */
  IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);

  return true;
}


IOIF_SAISoundState_t IOIF_SAI_PlayBeep(uint32_t freq_hz, uint16_t volume, uint32_t time_ms)
{
	bool ret = false;

	uint32_t pre_time;
	int32_t sample_rate = sai_sample_rate;
	int32_t num_samples = IOIF_SAI_BUF_MS * sai_sample_rate / 1000;
	float sample_point;
	int16_t sample_index = 0;
	int16_t div_freq;
	int32_t volume_out;
	uint32_t buf_len;

	// volume control within 0 - 100
	volume = (volume < 0) ? 0 : ((volume > 100) ? 100 : volume);
	volume_out = (INT16_MAX / 40) * volume / 100;

	sai_qbuf_in = 0;
	sai_qbuf_out = 0;

	/* SAI Transmit DMA Start */
	if(BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA) == BSP_OK)
		sai_stop = false;

	div_freq = (float)sample_rate / (float)freq_hz; // 주파수로 나누지 않고 주파수를 바로 사용

	//pre_time = BSP_GetTick();
	pre_time = HAL_GetTick();

	/* Mute off */
	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_SET);

	//while (BSP_GetTick() - pre_time <= time_ms) {
	while (HAL_GetTick() - pre_time <= time_ms) {
		buf_len = ((sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len);
		buf_len = (sai_qbuf_len - buf_len) - 1;

		if (buf_len > 0) {
			for (int i = 0; i < num_samples; i++) {
				sample_point = MakeSineWave(2 * M_PI * (float)(sample_index) / ((float)div_freq));

#ifdef _USE_SOUND_MONO
				sai_q_buf[sai_qbuf_in * sai_q_buf_len + i] = (int16_t)(sample_point * volume_out);
#else
				sai_q_buf[sai_qbuf_in*sai_q_buf_len + i].left  = (int16_t)(sample_point * volume_out);
				sai_q_buf[sai_qbuf_in*sai_q_buf_len + i].right = (int16_t)(sample_point * volume_out);
#endif
				sample_index = (sample_index + 1) % (int)div_freq;
			}

			if (((sai_qbuf_in + 1) % sai_qbuf_len) != sai_qbuf_out) {
				sai_qbuf_in = (sai_qbuf_in + 1) % sai_qbuf_len;
			}
		}
	}
	/* Mute on */
	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);

	ret = true;

	return ret;
}


IOIF_SAISoundState_t IOIF_SAI_StartBeep(uint32_t volume)
{

	uint16_t beepsoundfreq = 1000;					//frequency
	int note_durations[] = {10, 16, 16};			//durations

	for(uint8_t i=0; i<2; i++)
	{
		int note_duration = 1000 / note_durations[i];
		IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
		//osDelay(note_duration * 0.3);
		HAL_Delay(note_duration * 0.3);
	}

	return true;
}


IOIF_SAISoundState_t IOIF_SAI_ErrorBeep(uint32_t volume)
{
	uint16_t beepsoundfreq = 1500;
	int note_durations = 10;

	int note_duration = 1000 / note_durations;
	IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
	//osDelay(note_duration * 0.3);
	HAL_Delay(note_duration * 0.3);

	return true;
}


IOIF_SAISoundState_t IOIF_SAI_OffBeep(uint32_t volume)
{

	uint16_t beepsoundfreq = 1050;					//frequency
	int note_durations[] = {10, 10, 16, 16};			//durations

	for(uint8_t i=0; i<3; i++)
	{
		int note_duration = 1000 / note_durations[i];
		IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
		//osDelay(note_duration * 0.3);
		HAL_Delay(note_duration * 0.3);
	}

	return true;
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static void SaiAudioDMATxCB(void* param)
{
	uint32_t len;

	if (sai_stop == true)																						// if sai transmit is completed
		return;

	len = (sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len;											// sai queue buffer length check

	if (len > 0)																								// if sound data exists in sai queue buffer,
	{
		//HAL_SAI_Transmit_DMA(hsai, (uint8_t*)&sai_q_buf[sai_qbuf_out*sai_q_buf_len], sai_q_buf_len * 2);		// sai DMA trasmit start from next frame
		BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)&sai_q_buf[sai_qbuf_out*sai_q_buf_len], &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA);
		if (sai_qbuf_out != sai_qbuf_in)
		   sai_qbuf_out = (sai_qbuf_out + 1) % sai_qbuf_len;													// queue buffer index increase
	}
	else
		//HAL_SAI_Transmit_DMA(hsai, (uint8_t*)sai_q_buf_zero, sai_q_buf_len * 2);								// first transmit
		BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA);
}


static float SaiGetNoteHz(int8_t octave, int8_t note)
{
  float hz;
  float f_note;

  if (octave < 1) octave = 1;
  if (octave > 8) octave = 8;

  if (note <  1) note = 1;
  if (note > 12) note = 12;

  f_note = (float)(note-10)/12.0f;

  hz = pow(2, (octave-1)) * 55 * pow(2, f_note);

  return hz;
}


static float MakeSineWave(float x)
{
  const float B = 4 / M_PI;
  const float C = -4 / (M_PI * M_PI);

  return -(B * x + C * x * ((x < 0) ? -x : x));
}



#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */
void CLI_RunSAI(cli_args_t *args)
{
	bool ret = false;

	uint32_t cliVolume = 0;
	int8_t cliOctave = 0; int8_t cliNote = 0; uint16_t cliNoteVol = 0; uint32_t cliNoteTime_ms = 0;

	/* Function : Play Beep [start/error/off] */
	if(args->argc == 3 && args->cmpStr(0, "playbeep") == true)
	{
		if(args->cmpStr(1, "start") == true)
		{
			cliVolume = args -> getData(2);
			IOIF_SAI_StartBeep(cliVolume);
			CLI_Printf(" * Start Beep Done. *  \r\n");

			ret = true;
		}
		else if(args->cmpStr(1, "error") == true)
		{
			cliVolume = args -> getData(2);
			IOIF_SAI_ErrorBeep(cliVolume);

			CLI_Printf(" * Error Beep Done. *  \r\n");
			ret = true;
		}
		else if(args->cmpStr(1, "off") == true)
		{
			cliVolume = args -> getData(2);
			IOIF_SAI_OffBeep(cliVolume);

			CLI_Printf(" * Off Beep Done. *  \r\n");
			ret = true;
		}
		else
		{
			ret = false;
		}
	}

	/* Function : Play Note Functions */
	if(args->argc == 5 && args->cmpStr(0, "playnote") == true)
	{
		cliOctave = args -> getData(1);
		cliOctave = (cliOctave < 1) ? 1 : ((cliOctave > 8) ? 8 : cliOctave);							// 1 to 8 limitation
		cliNote = args -> getData(2);
		cliNote = (cliNote < 1) ? 1 : ((cliNote > 12) ? 12 : cliNote);									// 1 to 12 limitation
		cliNoteVol = args -> getData(3);
		cliNoteVol = (cliNoteVol < 0) ? 0 : ((cliNoteVol > 100) ? 100 : cliNoteVol);					// 1 to 100 limitation
		cliNoteTime_ms = args -> getData(4);

		IOIF_SAI_PlayNote(cliOctave, cliNote, cliNoteVol, cliNoteTime_ms);
		CLI_Printf(" * Playing Note Done. \r\n");
		CLI_Printf(" * Octave : %d, Note : %s \r\n", cliOctave, NoteStr[cliNote - 1]);
		CLI_Printf(" * Volume : %d, Duration : %d \r\n", cliNoteVol, cliNoteTime_ms);

		ret = true;
	}


	if(ret == false)		//help
	{
		CLI_Printf(" * Audio Play * \r\n");
		CLI_Printf("   Playing beep or note through SAI. \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   sai_basic playbeep [start/off/error] [volume(1~100)] \r\n");
		CLI_Printf("   sai_basic playnote [octave(1~8)] [note(1~12)] [volume(0~100)] [time(ms)] \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   sai_basic playbeep start 50 \r\n");
	}
}
#endif /* _USE_DEBUG_CLI */

#endif /*IOIF_SAI_COMMON_ENABLED*/
#endif /* BSP_SAI_MODULE_ENABLED */
