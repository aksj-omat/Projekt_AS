
#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#include "gpio.h"

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
			break;
			//case inne opcje
		default:
			break;
		}
	}
}

#endif /* JOYSTICK_H_ */
