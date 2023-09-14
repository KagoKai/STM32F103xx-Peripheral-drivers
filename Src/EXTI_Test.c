#include <string.h>
#include "stm32f1xx_gpio.h"

/*
 * Turn on the LED connected to PA11 and PA12 whenever the buttons connected
 * to PA1 and PA2 are pressed correspondingly.
 * The interrupts on PA1 and PA2 are falling edge triggered.
 */

/* GPIOA pins
 * PA1  - Input pull-up (Alternate Input Falling-triggered)
 * PA2  - Input pull-up (Alternate Input Falling-triggered)
 * PA11 - Output push-pull
 * PA12 - Output push-pull
 */

int main(void)
{
	GPIOx_ClkControl(GPIOA, ENABLE);

	GPIOx_Handle_t pinHandler;

	pinHandler.pGPIOx = GPIOA;
	pinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_11;
	pinHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_PP;
	pinHandler.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIOx_Init(&pinHandler);

	pinHandler.pGPIOx = GPIOA;
	pinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_1;
	pinHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	pinHandler.GPIO_pinConfig.GPIO_PinPull = GPIO_PULLUP;
	GPIOx_Init(&pinHandler);
	GPIOx_IRQControl(IRQ_NO_EXTI1, ENABLE);

	while(1);
}

void EXTI1_IRQHandler(void)
{
	GPIOx_IRQHandler(PIN_1);
	GPIOx_PinToggle(GPIOA, PIN_11);
}

void EXTI2_IRQHandler(void)
{
	GPIOx_IRQHandler(PIN_2);
	GPIOx_PinToggle(GPIOA, PIN_12);
}
