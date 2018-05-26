
#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#include "gpio.h"
#include "lcd.h"

typedef enum {
	mainMenu, msgSelection
} menu;


int isJoyClicked(uint16_t joyX_Pin, GPIO_TypeDef * joyX_GPIO_Port) {
	return HAL_GPIO_ReadPin(joyX_GPIO_Port,joyX_Pin) == ENABLE;
}

void doJoyAction(uint16_t joyX_Pin, menu dest) {
	if (dest == mainMenu) {
		switch (joyX_Pin) {
		case joyCenter_Pin:
			BSP_LCD_GLASS_Clear();
			 BSP_LCD_GLASS_DisplayString("srodek");
			break;
		case joyDown_Pin:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString("dol");
			break;
		case joyUp_Pin:
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString("gora");
			break;
		default:
			break;
		}
	}
}

#endif /* JOYSTICK_H_ */
