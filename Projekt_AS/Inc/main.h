
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
#define ledRed_Pin GPIO_PIN_2
#define ledRed_GPIO_Port GPIOB
#define ledGreen_Pin GPIO_PIN_8
#define ledGreen_GPIO_Port GPIOE

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
