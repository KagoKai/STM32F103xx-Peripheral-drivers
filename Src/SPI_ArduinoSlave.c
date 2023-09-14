#include <string.h>
#include "stm32f1xx_gpio.h"
#include "stm32f1xx_spi.h"

/* Exercise requirement:
 * Send string data to the Arduino slave when a
 * button (PB1) is pressed. Using hardware slave
 * management. The length of the frame is sent
 * before the data.
 *
 * Data Frame: 8-bit
 * SCLK: 2 MHz
 * f_clk: 16 MHz
 */

/* SPI2 pins
 * MOSI: PB15 - Alternate Output push-pull
 * MISO: PB14 - Input pull-up
 * SCLK: PB13 - Alternate Output push-pull
 * NSS:  PB12 - Alternate Output push-pull
 *
 * Button pin
 * Button: PB1 - External interrupt falling-triggered.
 */

SPIx_Handle_t hspi2;
uint8_t isSending;

void GPIO_Setup(void);
void SPI_Setup(SPIx_Handle_t *hspi);

int main(void)
{
	GPIO_Setup();
	SPI_Setup(&hspi2);

	uint8_t data[] = {0xF1, 0xF2, 0xF3};
	uint8_t dataLen = sizeof(data) / sizeof(data[0]);
	while (1)
	{
		if (isSending)
		{
			SPIx_Start(hspi2.pSPIx);
			SPIx_Send(hspi2.pSPIx, &dataLen, 1);
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

void GPIO_Setup(void)
{
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

	/*** Set up the SPI peripheral pins ***/
	GPIOx_Handle_t spiPinHandler;
	memset(&spiPinHandler, 0, sizeof(spiPinHandler));
	// MOSI
	spiPinHandler.pGPIOx = GPIOB;
	spiPinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_15;
	spiPinHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_AF_OP_PP;
	spiPinHandler.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;
	GPIOx_Init(&spiPinHandler);
	// SCLK
	spiPinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_13;
	GPIOx_Init(&spiPinHandler);
	// NSS
	spiPinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_12;
	GPIOx_Init(&spiPinHandler);
	// MISO
	spiPinHandler.GPIO_pinConfig.GPIO_PinNum = PIN_14;
	spiPinHandler.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	spiPinHandler.GPIO_pinConfig.GPIO_PinPull = GPIO_PULLUP;
	GPIOx_Init(&spiPinHandler);
}

void SPI_Setup(SPIx_Handle_t *hspi)
{
	hspi->pSPIx = SPI2;
	SPIx_ClkControl(hspi->pSPIx, ENABLE);
	hspi->SPI_config.SPI_Mode = SPI_MODE_MASTER;
	hspi->SPI_config.SPI_BusMode = SPI_BUSMODE_FULLDUPLEX;
	hspi->SPI_config.SPI_Speed = SPI_SPEED_DIV8;
	hspi->SPI_config.SPI_DFF = SPI_DFF_8BITS;
	hspi->SPI_config.SPI_CLKMode = SPI_CLK_MODE_0;
	hspi->SPI_config.SPI_MultiMaster = SPI_MM_DISABLE;
	hspi->SPI_config.SPI_SSM = SPI_SSM_HARDWARE;
	SPIx_Init(hspi);
}
