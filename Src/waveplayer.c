/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/waveplayer.c 
  * @author  MCD Application Team
  * @brief   I2S Audio player program. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define AUDIO_BUFFER_SIZE             4096

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* LED State (Toggle or OFF)*/
__IO uint32_t LEDsState;

extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


extern char songs[255][100];
extern char* name_song;
extern uint8_t next;
extern uint8_t index_song;
extern unsigned int count;

/* Audio Play Start variable. 
   Defined as external in main.c*/
__IO uint32_t AudioPlayStart = 0;

/* Audio wave data length to be played */
static uint32_t WaveDataLength = 0;

/* Audio wave remaining data length to be played */
static __IO uint32_t AudioRemSize = 0;

/* Ping-Pong buffer used for audio play */
uint8_t Audio_Buffer[AUDIO_BUFFER_SIZE];

/* Position in the audio play buffer */
__IO BUFFER_StateTypeDef buffer_offset = BUFFER_OFFSET_NONE;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
static uint8_t Volume = 70;

/* Variable used to indicate audio mode (play, record or stop). */
/* Defined in main.c */
extern __IO uint32_t CmdIndex;

//Variable used by FFT
uint8_t ifftFlag = 0;
uint8_t doBitReverse = 1;
uint16_t SAMPLES =  2048;
uint16_t FFT_SIZE = 1024;
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 s_CFFT;
float32_t output[1024];
float32_t maxValue;
uint32_t testIndex;
float32_t input[2048];
float32_t dutys[8];
		int temp;		

/* Variable used by FatFs*/
FIL FileRead;
DIR Directory;

/* Variable used to switch play from audio sample available on USB to recorded file. */
/* Defined in waverecorder.c */
extern uint32_t WaveRecStatus;

/* Variable to indicate USB state (start/idle) */
/* Defined in main.c */
extern MSC_ApplicationTypeDef AppliState;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void FFT(void){	
	arm_status status;

	for (int i = 0; i < SAMPLES; i++){
		input[i] = (float)Audio_Buffer[i];
	}
	status = arm_cfft_radix4_init_f32(&s_CFFT,FFT_SIZE,ifftFlag,doBitReverse);
	if (status == ARM_MATH_SUCCESS){
		arm_cfft_radix4_f32(&s_CFFT,input);
		arm_cmplx_mag_f32(input,output,FFT_SIZE);
		
		for (int i = 0; i< 8;i++){
			dutys[i] = output[(i+1)*10];
		}
		
		arm_max_f32(dutys,8,&maxValue,&testIndex);
		
		//PB4
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,PWM_duty(dutys[0],maxValue));
			
		//PB5
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PWM_duty(dutys[1],maxValue));
			
		//PB0
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,PWM_duty(dutys[2],maxValue));
			
		//PB1
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,PWM_duty(dutys[3],maxValue));
		
		//PA15
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_duty(dutys[4],maxValue));
		
		//PA1
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,PWM_duty(dutys[5],maxValue));

		//PA2
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,PWM_duty(dutys[6],maxValue));

		//PA3
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,PWM_duty(dutys[7],maxValue));
				
	}else {
		Error_Handler();
	}
}


/**
  * @brief  Plays Wave from a mass storage.
  * @param  AudioFreq: Audio Sampling Frequency
  * @retval None
*/
void WavePlayBack(uint32_t AudioFreq)
{ 
  UINT bytesread = 0;
  
  /* Start playing */
  AudioPlayStart = 1;
  RepeatState = REPEAT_ON;
  
  /* Initialize Wave player (Codec, DMA, I2C) */
  if(WavePlayerInit(AudioFreq) != 0)
  {
    Error_Handler();
  }

  /* Get Data from USB Flash Disk */
  f_lseek(&FileRead, 0);
  f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
  AudioRemSize = WaveDataLength - bytesread;
  
  /* Start playing Wave */
  BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
  LEDsState = LED6_TOGGLE;
  PauseResumeStatus = RESUME_STATUS;
  PressCount = 0;
 
  /* Check if the device is connected.*/
  while((AudioRemSize != 0) && (AppliState != APPLICATION_IDLE))
  { 
		if (next){
			next = 0;			
			WavePlayer_CallBack();
			for (int i = 0; i<10000000; i++){};
			WavePlayerStart();			
			return;
		}
    /* Test on the command: Playing */		
    if(CmdIndex == CMD_PLAY)
    { 
      if(PauseResumeStatus == PAUSE_STATUS)
      {
        /* Stop Toggling LED2 to signal Pause */
        LEDsState = STOP_TOGGLE;
        /* Pause playing Wave */
        WavePlayerPauseResume(PauseResumeStatus);
        PauseResumeStatus = IDLE_STATUS;
      }
      else if(PauseResumeStatus == RESUME_STATUS)
      {
        /* Toggling LED6 to signal Play */
        LEDsState = LED6_TOGGLE;
        /* Resume playing Wave */
        WavePlayerPauseResume(PauseResumeStatus);
        PauseResumeStatus = IDLE_STATUS;
      }  

      bytesread = 0;
      
      if(buffer_offset == BUFFER_OFFSET_HALF)
      {
        
        f_read(&FileRead, 
               &Audio_Buffer[0], 
               AUDIO_BUFFER_SIZE/2, 
               (void *)&bytesread);           
          buffer_offset = BUFFER_OFFSET_NONE;
      }
      
      if(buffer_offset == BUFFER_OFFSET_FULL)
      {
        f_read(&FileRead, 
               &Audio_Buffer[AUDIO_BUFFER_SIZE/2], 
               AUDIO_BUFFER_SIZE/2, 
               (void *)&bytesread); 
          FFT();
          buffer_offset = BUFFER_OFFSET_NONE;
      } 
      if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2))
      {
        AudioRemSize -= bytesread;
      }
      else
      {
				if (index_song<count-1){
				++index_song;
				}else {
					index_song = 0;
				}
				name_song = songs[index_song];
				next = 1;				
				
        AudioRemSize = 0;
      }
    }
    else 
    {
      /* Stop playing Wave */
      WavePlayerStop();
      /* Close file */
      f_close(&FileRead);
      AudioRemSize = 0;
      RepeatState = REPEAT_ON;
      break;
    }
  }
#ifdef PLAY_REPEAT_DISABLED 
  RepeatState = REPEAT_OFF;
  /* Stop playing Wave */
  WavePlayerStop();
  /* Close file */
  f_close(&FileRead);
  /* Test on the command: Playing */
  if(CmdIndex == CMD_PLAY)
  {
    LEDsState = LED4_TOGGLE;
  }
#else 
  LEDsState = LEDS_OFF;
  RepeatState = REPEAT_ON;
  AudioPlayStart = 0;
  /* Stop playing Wave */
  WavePlayerStop();
  /* Close file */
  f_close(&FileRead);
#endif /* PLAY_REPEAT_DISABLED */
}

/**
  * @brief  Pauses or Resumes a played Wave.
  * @param  state: Player state: Pause, Resume or Idle
  * @retval None
  */
void WavePlayerPauseResume(uint32_t wState)
{
  if(wState == PAUSE_STATUS)
  {
    BSP_AUDIO_OUT_Pause();   
  }
  else
  {
    BSP_AUDIO_OUT_Resume();   
  }
}

/**
  * @brief  Stops playing Wave.
  * @param  None
  * @retval None
  */
void WavePlayerStop(void)
{ 
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}
 
/**
* @brief  Initializes the Wave player.
* @param  AudioFreq: Audio sampling frequency
* @retval None
*/
int WavePlayerInit(uint32_t AudioFreq)
{ 
  /* MEMS Accelerometer configure to manage PAUSE, RESUME operations */
  BSP_ACCELERO_Click_ITConfig();

  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  return(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AudioFreq));  
}

/*--------------------------------
Callbacks implementation:
The callbacks prototypes are defined in the stm32f4_discovery_audio_codec.h file
and their implementation should be done in the user code if they are needed.
Below some examples of callback implementations.
--------------------------------------------------------*/

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
  buffer_offset = BUFFER_OFFSET_HALF;
}

/**
* @brief  Calculates the remaining file size and new position of the pointer.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  buffer_offset = BUFFER_OFFSET_FULL;
  BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE / 2);
}

/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
void BSP_AUDIO_OUT_Error_CallBack(void)
{
  /* Stop the program with an infinite loop */
  while (1)
  {}
  
  /* Could also generate a system reset to recover from the error */
  /* .... */
}

/**
  * @brief  Starts Wave player.
  * @param  None
  * @retval None
  */
void WavePlayerStart(void)
{
	//WavePlayer_CallBack();
  UINT bytesread = 0;
  char path[] = "0:/";
  char* wavefilename = NULL;
  WAVE_FormatTypeDef waveformat;	
  
  /* Get the read out protection status */
  if(f_opendir(&Directory, (const TCHAR*)path) == FR_OK)
  {
    wavefilename = name_song; 
    /* Open the Wave file to be played */
    if(f_open(&FileRead, (const TCHAR*) wavefilename , FA_READ) != FR_OK)
    {
      //BSP_LED_On(LED5);      
    }
    else
    {  
			Name name;
			strcpy(name.name,wavefilename);
			char result[300];
			NameParseJson(name,result);

			Message message;
			strcpy(message.type,"Name");
			strcpy(message.message,result);
			MessageParseJson(message,result);

			HAL_UART_Transmit(&huart3,(uint8_t*)result,strlen(result),10);
			for (int i = 0; i<10000000; i++){};
      /* Read sizeof(WaveFormat) from the selected file */
      f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
      
      /* Set WaveDataLenght to the Speech Wave length */
      WaveDataLength = waveformat.FileSize;

      /* Play the Wave */
      WavePlayBack(waveformat.SampleRate);
			
			free(result);
    }    
  }
}

/**
  * @brief Wave player.
  * @param  None
  * @retval None
  */
void WavePlayer_CallBack(void)
{
  if(AppliState != APPLICATION_IDLE)
  {
    /* Reset the Wave player variables */
    RepeatState = REPEAT_ON;
    AudioPlayStart = 0;
    LEDsState = LEDS_OFF;
    PauseResumeStatus = RESUME_STATUS;
    WaveDataLength =0;
    PressCount = 0;
    
    /* Stop the Codec */
    if(BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW) != AUDIO_OK)
    {
      Error_Handler();
    }
    
    /* Turn OFF LED3, LED4 and LED6 */
    BSP_LED_Off(LED3);
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED6);
  }
} 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
