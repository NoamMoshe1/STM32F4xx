/*
 * 004gpio_freq.c
 *
 *  Created on: Feb 27, 2024
 *      Author: non55
 */
#include <stdint.h>

#include "../Driver/Inc/stm32f407xx.h"
#include "../Driver/Inc/stm32f407xx_gpio_driver.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_Handle_t GpioLed;

	//this is led gpio configuration

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);


	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
	}
	return 0;
}