/* Exercise requirement:
 * Send command data (string) to the Arduino slave
 * when a button (PB1) is pressed.
 *
 * SSM: Hardware
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

#include <string.h>
#include "stm32f1xx_gpio.h"
#include "stm32f1xx_spi.h"

// Command macros
#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ 	0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_ON				1U
#define LED_OFF				0U

#define NACK 				0xA5
#define ACK					0xF5

// Arduino slave analog pins
#define ANALOG_PIN0   		0
#define ANALOG_PIN1  		1
#define ANALOG_PIN2   		2
#define ANALOG_PIN3   		3
#define ANALOG_PIN4   		4

SPIx_Handle_t hspi2;
uint8_t buttonPressed;
uint8_t cmdArr = {CMD_LED_CTRL, CMD_SENSOR_READ, CMD_LED_READ};

void GPIO_Setup(void);
void SPI_Setup(SPIx_Handle_t *hspi);

int main(void)
{
	GPIO_Setup();
	SPI_Setup(&hspi2);

	uint8_t *command = cmdArr;
	uint8_t sensorVal = 0;
	uint8_t ledVal = 0;
	uint8_t ackBit = 0;

	while (1)
	{
		if (buttonPressed)
		{
			SPIx_Start(hspi2.pSPIx);
			SPIx_Send(hspi2.pSPIx, command, 1);
			SPIx_Receive(hspi2.pSPIx, &ackBit, 1);
			SPIx_Stop(hspi2.pSPIx);

			if (ackBit == ACK)
			{
				if (*command == CMD_LED_CTRL)
				{
					//TODO: Implementation
				}
				else if (*command == CMD_SENSOR_READ)
				{
					//TODO: Implementation
				}
				else if (*command == CMD_LED_READ)
				{
					//TODO: Implementation
				}
				else if (*command == CMD_PRINT)
				{
					//TODO: Implementation
				}
				else if (*command == CMD_ID_READ)
				{
					//TODO: Implementation
				}
			}
			//isSending = 0;
		}
	}

}

void EXTI1_IRQHandler(void)
{
	GPIOx_IRQHandler(PIN_1);
	buttonPressed = 1;
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
