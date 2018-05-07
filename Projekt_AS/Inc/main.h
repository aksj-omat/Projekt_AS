/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define joyCenter_Pin GPIO_PIN_0
#define joyCenter_GPIO_Port GPIOA
#define joyLeft_Pin GPIO_PIN_1
#define joyLeft_GPIO_Port GPIOA
#define joyRight_Pin GPIO_PIN_2
#define joyRight_GPIO_Port GPIOA
#define joyUp_Pin GPIO_PIN_3
#define joyUp_GPIO_Port GPIOA
#define joyDown_Pin GPIO_PIN_5
#define joyDown_GPIO_Port GPIOA
#define LCD_SEG23_USER_Pin GPIO_PIN_6
#define LCD_SEG23_USER_GPIO_Port GPIOA
#define LCD_SEG0_USER_Pin GPIO_PIN_7
#define LCD_SEG0_USER_GPIO_Port GPIOA
#define LCD_SEG22_USER_Pin GPIO_PIN_4
#define LCD_SEG22_USER_GPIO_Port GPIOC
#define LCD_SEG1_USER_Pin GPIO_PIN_5
#define LCD_SEG1_USER_GPIO_Port GPIOC
#define LCD_SEG21_USER_Pin GPIO_PIN_0
#define LCD_SEG21_USER_GPIO_Port GPIOB
#define LCD_SEG2_USER_Pin GPIO_PIN_1
#define LCD_SEG2_USER_GPIO_Port GPIOB
#define ledRed_Pin GPIO_PIN_2
#define ledRed_GPIO_Port GPIOB
#define ledGreen_Pin GPIO_PIN_8
#define ledGreen_GPIO_Port GPIOE
#define LCD_SEG20_USER_Pin GPIO_PIN_12
#define LCD_SEG20_USER_GPIO_Port GPIOB
#define LCD_SEG3_USER_Pin GPIO_PIN_13
#define LCD_SEG3_USER_GPIO_Port GPIOB
#define LCD_SEG19_USER_Pin GPIO_PIN_14
#define LCD_SEG19_USER_GPIO_Port GPIOB
#define LCD_SEG4_USER_Pin GPIO_PIN_15
#define LCD_SEG4_USER_GPIO_Port GPIOB
#define LCD_SEG18_USER_Pin GPIO_PIN_8
#define LCD_SEG18_USER_GPIO_Port GPIOD
#define LCD_SEG5_USER_Pin GPIO_PIN_9
#define LCD_SEG5_USER_GPIO_Port GPIOD
#define LCD_SEG17_USER_Pin GPIO_PIN_10
#define LCD_SEG17_USER_GPIO_Port GPIOD
#define LCD_SEG6_USER_Pin GPIO_PIN_11
#define LCD_SEG6_USER_GPIO_Port GPIOD
#define LCD_SEG16_USER_Pin GPIO_PIN_12
#define LCD_SEG16_USER_GPIO_Port GPIOD
#define LCD_SEG7_USER_Pin GPIO_PIN_13
#define LCD_SEG7_USER_GPIO_Port GPIOD
#define LCD_SEG15_USER_Pin GPIO_PIN_14
#define LCD_SEG15_USER_GPIO_Port GPIOD
#define LCD_SEG8_USER_Pin GPIO_PIN_15
#define LCD_SEG8_USER_GPIO_Port GPIOD
#define LCD_SEG14_USER_Pin GPIO_PIN_6
#define LCD_SEG14_USER_GPIO_Port GPIOC
#define LCD_SEG9_USER_Pin GPIO_PIN_7
#define LCD_SEG9_USER_GPIO_Port GPIOC
#define LCD_SEG13_USER_Pin GPIO_PIN_8
#define LCD_SEG13_USER_GPIO_Port GPIOC
#define LCD_SEG10_USER_Pin GPIO_PIN_15
#define LCD_SEG10_USER_GPIO_Port GPIOA
#define LCD_SEG11_USER_Pin GPIO_PIN_4
#define LCD_SEG11_USER_GPIO_Port GPIOB
#define LCD_SEG12_USER_Pin GPIO_PIN_5
#define LCD_SEG12_USER_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
