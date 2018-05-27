
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "dfsdm.h"
#include "lcd.h"
#include "quadspi.h"
#include "sai.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_qspi.h"
#include "stm32l476g_discovery_audio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//QSPI --------------------------------------------------------------------------------------------------------------------
/* Private variables ---------------------------------------------------------*/
#define BUFFER_SIZE         ((uint32_t)0x0200)
#define WRITE_READ_ADDR     ((uint32_t)0x0050)
#define QSPI_BASE_ADDR      ((uint32_t)0x90000000)
uint8_t qspi_aTxBuffer[BUFFER_SIZE];
uint8_t qspi_aRxBuffer[BUFFER_SIZE];
static void     Fill_Buffer (uint8_t *pBuffer, uint32_t uwBufferLength, uint32_t uwOffset);
static uint8_t  Buffercmp   (uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength);
static void qspi_test();
//AUDIO --------------------------------------------------------------------------------------------------------------------
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
 } Record_buffer_offset_Typedef;

#define RECORD_BUFFER_SIZE (uint32_t)2048
 /* Buffer containing the PCM samples coming from the microphone */
 static uint16_t RecordBuffer[RECORD_BUFFER_SIZE];

 /* Buffer used to stream the recorded PCM samples towards the audio codec. */
 static uint16_t PlaybackBuffer[RECORD_BUFFER_SIZE*2];
 static Record_buffer_offset_Typedef  Record_buffer_offset = BUFFER_OFFSET_NONE;
 /* Audio recorder callback functions */
 static void audio_in_transfer_complete_callback(void);
 static void audio_in_half_transfer_callback(void);
 static void audio_in_error_callback(void);
 static void audio_test(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Information indicating which part of the recorded buffer is ready for audio loopback  */


/* Private function prototypes -----------------------------------------------*/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DFSDM1_Init();
  MX_SAI1_Init();
 // MX_LCD_Init(); //musi byc zakomentowane
 // MX_QUADSPI_Init(); //czy musi??
  /* USER CODE BEGIN 2 */
  BSP_LCD_GLASS_Init();
  BSP_QSPI_Init();
  //test QSPI
  qspi_test();
  HAL_Delay(3000);
  //test Audio
  audio_test();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}
static void Fill_Buffer(uint8_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
  uint32_t tmpIndex = 0;
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
}

static uint8_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
      return 1;
    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}

void qspi_test(){
	__IO uint8_t *data_ptr;
	uint32_t index;
	BSP_QSPI_Erase_Block(WRITE_READ_ADDR);			//wyczysc blok pamieci
	Fill_Buffer(qspi_aTxBuffer, BUFFER_SIZE, 0xD20F);//wypelnij losowa wartoscia
	BSP_QSPI_Write(qspi_aTxBuffer, WRITE_READ_ADDR, BUFFER_SIZE); //wpisz bufor pod wskazany adres
	BSP_QSPI_Read(qspi_aRxBuffer, WRITE_READ_ADDR, BUFFER_SIZE);
	if (Buffercmp(qspi_aRxBuffer, qspi_aTxBuffer, BUFFER_SIZE) > 0)
		BSP_LCD_GLASS_DisplayString("err");
	else
		BSP_LCD_GLASS_DisplayString("OK1");
	BSP_QSPI_EnableMemoryMappedMode();
	for (index = 0, data_ptr = (__IO uint8_t *) (QSPI_BASE_ADDR + WRITE_READ_ADDR); index < BUFFER_SIZE; index++, data_ptr++) {
		if (*data_ptr != qspi_aTxBuffer[index]) {
			BSP_LCD_GLASS_DisplayString("err2");
			break;
		}
	}
	if (index == BUFFER_SIZE)
		BSP_LCD_GLASS_DisplayString("OK2");
}

void audio_test(void)
{
  uint32_t audio_out_start = RESET ;
  uint32_t i;
  Record_buffer_offset = BUFFER_OFFSET_NONE; //ustaw offset

  if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE,  80,  BSP_AUDIO_FREQUENCY_8K) != AUDIO_OK) //inicjuj audio output
	  BSP_LCD_GLASS_DisplayString("Aerr"); //jezeli blad

  BSP_AUDIO_OUT_ChangeAudioConfig(BSP_AUDIO_OUT_CIRCULARMODE|BSP_AUDIO_OUT_STEREOMODE); //ustaw audio circular i tryb stereo

  if (BSP_AUDIO_IN_Init(BSP_AUDIO_FREQUENCY_8K, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK) //inicjuj audio in
	  BSP_LCD_GLASS_DisplayString("Aerr2");	//jezeli blad

  BSP_AUDIO_IN_RegisterCallbacks(audio_in_error_callback, audio_in_half_transfer_callback, audio_in_transfer_complete_callback); //ustaw callbacki

  if (BSP_AUDIO_IN_Record(RecordBuffer, RECORD_BUFFER_SIZE) != AUDIO_OK) //inicjuj audio input
	  BSP_LCD_GLASS_DisplayString("Aerr3"); //jezeli blad

  while (1) //odtwarzaj w petli
  {
    if(Record_buffer_offset != BUFFER_OFFSET_NONE) //kiedy 1sza lub 2ga polowa jest gotowa do skopiowania (callback od record in)
    {
      if(Record_buffer_offset == BUFFER_OFFSET_HALF) //kiedy 1sza polowa jest gotowa do skopiowania
      {
        for(i = 0; i < (RECORD_BUFFER_SIZE/2); i++) //kopiuj jako stereo
        {
          PlaybackBuffer[2*i]     = RecordBuffer[i];
          PlaybackBuffer[(2*i)+1] = PlaybackBuffer[2*i];
        }
        if (audio_out_start == RESET) //1szy raz tutaj - wlacz auido
        {
          if (BSP_AUDIO_OUT_Play(PlaybackBuffer, RECORD_BUFFER_SIZE*2) != AUDIO_OK)
        	  BSP_LCD_GLASS_DisplayString("Aerr4"); //jezeli blad

          audio_out_start = SET;
        }
      }
      else { //jezeli druga polowa bufora jest gotowa
        for(i = (RECORD_BUFFER_SIZE/2); i < RECORD_BUFFER_SIZE; i++) //kopiuj jako stereo
        {
          PlaybackBuffer[2*i]     = RecordBuffer[i];
          PlaybackBuffer[(2*i)+1] = PlaybackBuffer[2*i];
        }
      }
      Record_buffer_offset = BUFFER_OFFSET_NONE; //po przekopiowaniu ustaw znowu flage
    }
  }
}
/**
  * @brief Callback function invoked when half of the PCM samples have been
  *        DM Atransfered from the DFSDM channel.
  * @param  None
  * @retval None
  */
void audio_in_transfer_complete_callback(void)
{

  Record_buffer_offset = BUFFER_OFFSET_FULL;
}

/**
  * @brief Callback function invoked when all the PCM samples have been
  *        DMA transfered from the DFSDM channel.
  * @param  None
  * @retval None
  */
void audio_in_half_transfer_callback(void)
{
  Record_buffer_offset = BUFFER_OFFSET_HALF;
}

/**
  * @brief Callback function invoked when an error occured durint he DMA
  *        transfer of the PCM samples from the DFSDM channel.
  * @param  None
  * @retval None
  */
void audio_in_error_callback(void)
{
  /* Stop the program with an infinite loop */
  Error_Handler();
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 3;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
