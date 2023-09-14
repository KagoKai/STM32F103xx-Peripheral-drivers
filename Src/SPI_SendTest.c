#include <string.h>
#include "stm32f1xx_gpio.h"
#include "stm32f1xx_spi.h"

/* SPI test requirement:
 * Send the "Hello World" message using SPI2 in master mode
 * everytime a button connected to PB1 is pressed.
 * SCLK should be as fast as possible.
 * DFF in 8-bit and 16-bit mode.
 */

/* SPI2 pins
 * MOSI: PB15 - Alternate Output push-pull
 * MISO: PB14 - Not required.
 * SCLK: PB13 - Alternate Output push-pull
 * NSS:  PB12 - Not required
 *
 * Button pin
 * Button: PB1 - External interrupt pull-up.
 */

uint8_t isSending = 0;

int main(void)
{
	// Enable the peripheral clock for GPIO port B.
	AFIO_CLK_EN();
	GPIOx_ClkControl(GPIOB, ENABLE);

	/*** Set up the button pin ***/
	GPIOx_Handle_t btnHandler;
	memset(&btnHandler, 0, sizeof(btnHandler));
	btnHandler.pGPIOx = GPIOB;
	btnHandler.GPIO_pinConfig.GPIO_PinNum = PIN_1;
	btnHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	btnHandler.GPIO_pinConfig.GPIO_PinPull = GPIO_PULLUP;
	GPIOx_Init(&btnHandler);
	GPIOx_IRQControl(IRQ_NO_EXTI1, ENABLE);

	/*** Set up the SPI pins ***/
	GPIOx_Handle_t periPinHandler;
	memset(&periPinHandler, 0, sizeof(periPinHandler));
	// Initialize the MOSI pin.
	periPinHandler.pGPIOx = GPIOB;
	periPinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_15;
	periPinHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_AF_OP_PP;
	periPinHandler.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOx_Init(&periPinHandler);
	// Initialize the SCLK pin.
	periPinHandler.pGPIOx = GPIOB;
	periPinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_13;
	periPinHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_AF_OP_PP;
	periPinHandler.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOx_Init(&periPinHandler);

	/*** Set up the SPI peripheral ***/
	SPIx_ClkControl(SPI2, ENABLE);
	SPIx_Handle_t hspi2;
	memset(&hspi2, 0, sizeof(hspi2));
	hspi2.pSPIx = SPI2;
	hspi2.SPI_config.SPI_Mode = SPI_MODE_MASTER;
	hspi2.SPI_config.SPI_BusMode = SPI_BUSMODE_HALFDUPLEX;
	hspi2.SPI_config.SPI_Speed = SPI_SPEED_DIV2;
	hspi2.SPI_config.SPI_CLKMode = SPI_CLK_MODE_0;
	hspi2.SPI_config.SPI_DFF = SPI_DFF_8BITS;
	hspi2.SPI_config.SPI_SSM = SPI_SSM_SOFTWARE;
	SPIx_Init(&hspi2);

	uint8_t data[] = {0xF1, 0xF2, 0xF3};
	uint32_t dataLen = sizeof(data) / sizeof(data[0]);

	while(1)
	{
		if (isSending)
		{
			// Start the SPI2 peripheral
			SPIx_Start(hspi2.pSPIx);
			SPIx_Send(hspi2.pSPIx, data, dataLen);
			SPIx_Stop(hspi2.pSPIx);
			isSending = 0;
		}
	}
}

void EXTI1_IRQHandler(void)
{
	GPIOx_IRQHandler(PIN_1);
	isSending = 1;
}
