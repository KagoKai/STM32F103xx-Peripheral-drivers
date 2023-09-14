/*
 * stm32f1xx_gpio.h
 *
 *  Created on: Jun 19, 2023
 *      Author: ADMIN
 */

#ifndef INC_STM32F1XX_GPIO_H_
#define INC_STM32F1XX_GPIO_H_

#include "stm32f103xx.h"

/******************** GPIO specific definitions & macros ********************/

/* GPIO control register mode setting macros */
#define  GPIO_CR_MODE_INPUT         0x0u 		/*!< 00: Input mode (reset state)  */
#define  GPIO_CR_CNF_ANALOG         0x0u 		/*!< 00: Analog mode  */
#define  GPIO_CR_CNF_INPUT_FLOATING 0x1u 		/*!< 01: Floating input (reset state)  */
#define  GPIO_CR_CNF_INPUT_PU_PD    0x2u 		/*!< 10: Input with pull-up / pull-down  */
#define  GPIO_CR_CNF_GP_OUTPUT_PP   0x0u 		/*!< 00: General purpose output push-pull  */
#define  GPIO_CR_CNF_GP_OUTPUT_OD   0x1u 		/*!< 01: General purpose output Open-drain  */
#define  GPIO_CR_CNF_AF_OUTPUT_PP   0x2u 		/*!< 10: Alternate function output Push-pull  */
#define  GPIO_CR_CNF_AF_OUTPUT_OD   0x3u 		/*!< 11: Alternate function output Open-drain  */

/* GPIO Pin number macro */
#define PIN_0		0u
#define PIN_1		1u
#define PIN_2		2u
#define PIN_3		3u
#define PIN_4		4u
#define PIN_5		5u
#define PIN_6		6u
#define PIN_7		7u
#define PIN_8		8u
#define PIN_9		9u
#define PIN_10		10u
#define PIN_11		11u
#define PIN_12		12u
#define PIN_13		13u
#define PIN_14		14u
#define PIN_15		15u

/* GPIO mode macros */
#define GPIO_MODE_INPUT			0u
#define GPIO_MODE_OUTPUT_PP 	1u
#define GPIO_MODE_OUTPUT_OD		2u
#define GPIO_MODE_ANALOG		3u
#define GPIO_MODE_AF_OP_PP		4u
#define GPIO_MODE_AF_OP_OD 		5u

/* GPIO External Interrupt mode macros */
#define GPIO_MODE_IT_RT			6u // 110
#define GPIO_MODE_IT_FT			7u // 111
#define GPIO_MODE_IT_RFT		8u // 1000

/* GPIO Output speed macro */
#define GPIO_SPEED_LOW		0x2u	/* MODEy = GPIO_MODE_OUTPUT + GPIO_SPEED_z */
#define GPIO_SPEED_MEDIUM	0x1u
#define GPIO_SPEED_HIGH		0x3u

/* GPIO pull state */
#define GPIO_NOPULL			0u
#define GPIO_PULLUP			1u
#define GPIO_PULLDOWN		2u

typedef struct
{
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPull;
}GPIOx_PinConfig_t;

typedef struct
{
	GPIOx_Reg_t *pGPIOx;
	GPIOx_PinConfig_t GPIO_pinConfig;
}GPIOx_Handle_t;

/****************************************************************************************
 *					APIs supported by this driver                                       *
 *					Read the function definition for more information                   *
 ****************************************************************************************/

/********** Port clock control **********/

/** @brief Control the bus clock for a GPIO port
 *
 *  @param *pGPIOx The pointer to the base address of GPIO port.
 *  @param enableState Can be ENABLE or DISABLE macro, controls the clock accordingly.
 *  @return Void.
 */
void GPIOx_ClkControl(GPIOx_Reg_t *pGPIOx, uint8_t enableState);


/********** Port init and deinit **********/

/** @brief Initializes a GPIO pin with the specified
 * 		   configuration.
 *
 * 	@param *pGPIOx_Handle The handle pointer with setup configuration.
 * 	@return Void.
 */
void GPIOx_Init(GPIOx_Handle_t *pGPIOx_Handle);

/** @brief Deinitializes a GPIO port and resets it back to the
 * 		   default state.
 *
 *  @param *pGPIOx The pointer to the base address of the GPIO port.
 *  @return Void.
 */
void GPIOx_DeInit(GPIOx_Reg_t *pGPIOx);


/********** Read/write operations **********/

/** @brief Reads the value of a pin and returns its value
 *
 *  @param *pGPIOx The pointer to the base address of the GPIO port.
 *  @param pinNumber The read pin number (0->15).
 *  @return The pin current value.
 */
uint8_t GPIOx_PinRead(GPIOx_Reg_t *pGPIOx, uint8_t pinNumber);

/** @brief Reads the value of a GPIO port and returns its value
 *
 *  @param *pGPIOx The pointer to the base address of the GPIO port.
 *  @return The current port value.
 */
uint16_t GPIOx_PortRead(GPIOx_Reg_t *pGPIOx);

/** @brief Writes to a GPIO pin
 *
 *  @param *pGPIOx The pointer to the base address of the GPIO port.
 *  @param pinNumber The written pin number (0->15).
 *  @param pinValue The written value, can be SET or RESET.
 *  @return Void.
 */
void GPIOx_PinWrite(GPIOx_Reg_t *pGPIOx, uint8_t pinNumber, uint8_t pinValue);

/** @brief Writes to a GPIO port
 *
 *  @param *pGPIOx The pointer to the base address of the GPIO port.
 *  @param portValue The written value, must be a 16-bit value.
 *  @return Void.
 */
void GPIOx_PortWrite(GPIOx_Reg_t *pGPIOx, uint16_t portValue);

/** @brief Toggles to a GPIO pin, inverts its current value
 *
 *  @param *pGPIOx The pointer to the base address of the GPIO port.
 *  @param pinNumber The toggled pin number (0->15).
 *  @return Void.
 */
void GPIOx_PinToggle(GPIOx_Reg_t *pGPIOx, uint8_t pinNumber);


/********** Interrupts control and handler **********/

/** @brief Checks if the GPIO mode is an External Interrupt mode.
 *
 *  @param Mode - Current GPIO mode.
 *  @return 1 if True
 *  		0 if False
 */
uint8_t GPIOx_IsEXTI(uint8_t Mode);

/** @brief Enables or disables an I/O interrupt (on the NVIC side).
 *
 *  @param IRQ_Number The interrupt to be controlled.
 *  @param enableState Can be ENABLE or DISABLE, modifies the interrupt state accordingly.
 *  @return Void.
 */
void GPIOx_IRQControl(uint8_t IRQ_Number, uint8_t enableState);

/** @brief Changes an interrupt's priority.
 *
 *  @param IRQ_Number The interrupt to be controlled.
 *  @param IRQ_Priority The final priority level.
 *  @return Void.
 *
 * 	@Note Only the 4 MSB of each section are used to control the
 * 		  priority level in STM32F1xx.
 */
void GPIOx_IRQSetPriority(uint8_t IRQ_Number, uint32_t IRQ_Priority);

/** @brief Additional handling for an IRQ when its ISR fires.
 *
 * 	@param pinNumber The pin on which interrupt signal occurs.
 * 	@return Void.
 */
void GPIOx_IRQHandler(uint8_t pinNumber);

#endif /* INC_STM32F1XX_GPIO_H_ */
