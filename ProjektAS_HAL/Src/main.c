
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
#include "dma.h"
#include "sai.h"
#include "gpio.h"
#include "lcd.h"
#include "quadspi.h"
/* USER CODE BEGIN Includes */
#include "cs43l22.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_qspi.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*////////////////////////////////////////////// UWAGA ////////////////////////////////////////////////////////////////////////
 * 						PRZY PRZEBUDOWYWANIU PROJEKTU W CUBE
 * 						USTAW W SYSTEMCLOCK_CONFIG
 * 						RCC_OscInitStruct.PLL.PLLM = 1;
 *						ZAKOMENTUJ //MX_LCD_INIT();
 *						KOLEJNOSC INICJALIZACJI PERYFERIOW MOZE MIEC ZNACZENIE?
 *
 *
 */
/* Private variables ---------------------------------------------------------*/
//AUDIO-------------------------------------------------------
#define AUDIO_FILE_ADDRESS   0x08080000
#define AUDIO_FILE_SIZE      (180*1024)
#define PLAY_HEADER          0x2C
#define PLAY_BUFF_SIZE       4096
AUDIO_DrvTypeDef* audio_drv;

uint16_t                     PlayBuff[PLAY_BUFF_SIZE];
__IO int16_t                 UpdatePointer = -1;
//QSPI--------------------------------------------------------
#define BUFFER_SIZE         ((uint32_t)0x0200)
#define WRITE_READ_ADDR     ((uint32_t)0x0050)
#define QSPI_BASE_ADDR      ((uint32_t)0x90000000)
uint8_t qspi_aTxBuffer[BUFFER_SIZE];
uint8_t qspi_aRxBuffer[BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void codec_init();
static void codec_start();
static void qspi_test();
static void     Fill_Buffer (uint8_t *pBuffer, uint32_t uwBufferLength, uint32_t uwOffset);
static uint8_t  Buffercmp   (uint8_t* pBuffer1, uint8_t* pBuffer2, uint32_t BufferLength);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t PlaybackPosition   = PLAY_BUFF_SIZE + PLAY_HEADER;
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
  MX_DMA_Init();
  MX_SAI1_Init();
  //MX_QUADSPI_Init();
  BSP_LCD_GLASS_Init();
  BSP_LCD_GLASS_DisplayString("elo");
  /* USER CODE BEGIN 2 */

  qspi_test(); //ta funkcja rowniez inicjalizuje bsp_qspi!
  codec_init();

  if(*((uint64_t *)AUDIO_FILE_ADDRESS) != 0x017EFE2446464952 ) Error_Handler();
  for(int i=0; i < PLAY_BUFF_SIZE; i+=2)
   {
     PlayBuff[i]=*((__IO uint16_t *)(AUDIO_FILE_ADDRESS + PLAY_HEADER + i));
   }
  codec_start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while(1)
	    {
	  //    /* Wait a callback event */
	      while(UpdatePointer==-1);
	  //
	      int position = UpdatePointer;
	      UpdatePointer = -1;
	  //
	      /* Upate the first or the second part of the buffer */
	      for(int i = 0; i < PLAY_BUFF_SIZE/2; i++)
	      {
	        PlayBuff[i+position] = *(uint16_t *)(AUDIO_FILE_ADDRESS + PlaybackPosition);
	        PlaybackPosition+=2;
	      }

	  //    /* check the end of the file */
	      if((PlaybackPosition+PLAY_BUFF_SIZE/2) > AUDIO_FILE_SIZE)
	      {
	        PlaybackPosition = PLAY_HEADER;
	      }

	      if(UpdatePointer != -1)
	      {
	        /* Buffer update time is too long compare to the data transfer time */
	        Error_Handler();
	      }
	    }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
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
	BSP_QSPI_Init();
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


void codec_init(){
		if (CS43L22_ID != cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS)) {
			Error_Handler();
		}

		audio_drv = &cs43l22_drv;
		audio_drv->Reset(AUDIO_I2C_ADDRESS);
		if (audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 80, AUDIO_FREQUENCY_22K) != 0) {
			Error_Handler();
		}

}
void codec_start(){
	if(0 != audio_drv->Play(AUDIO_I2C_ADDRESS, NULL, 0))
	  {
	    Error_Handler();
	  }
	  if(HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)PlayBuff, PLAY_BUFF_SIZE))
	  {
	    Error_Handler();
	  }
}
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{

  UpdatePointer = PLAY_BUFF_SIZE/2;
}


void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{

  UpdatePointer = 0;
}
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
