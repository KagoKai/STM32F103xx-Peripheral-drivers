/*
 * stm32f1xx_gpio.c
 *
 *  Created on: Jun 19, 2023
 *      Author: ADMIN
 */

#include "stm32f1xx_gpio.h"

/********** Port clock control **********/

void GPIOx_ClkControl(GPIOx_Reg_t *pGPIOx, uint8_t enableState)
{
	if (enableState == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
	}
	else if (enableState == DISABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}
	}
}


/********** Port init and deinit **********/

void GPIOx_Init(GPIOx_Handle_t *pGPIOx_Handle)
{
	uint8_t MODE = 0;
	uint8_t CNF = 0;

	uint8_t position = pGPIOx_Handle->GPIO_pinConfig.GPIO_PinNum;
	uint8_t configOffset = (position < PIN_8) ? (position * 4) : ((position - 8) * 4);

	switch (pGPIOx_Handle->GPIO_pinConfig.GPIO_PinMode)
	{
		case GPIO_MODE_OUTPUT_PP:
			MODE = pGPIOx_Handle->GPIO_pinConfig.GPIO_PinSpeed;
			CNF = GPIO_CR_CNF_GP_OUTPUT_PP;
			break;
		case GPIO_MODE_OUTPUT_OD:
			MODE = pGPIOx_Handle->GPIO_pinConfig.GPIO_PinSpeed;
			CNF = GPIO_CR_CNF_GP_OUTPUT_OD;
			break;
		case GPIO_MODE_AF_OP_PP:
			MODE = pGPIOx_Handle->GPIO_pinConfig.GPIO_PinSpeed;
			CNF = GPIO_CR_CNF_AF_OUTPUT_PP;
			break;
		case GPIO_MODE_AF_OP_OD:
			MODE = pGPIOx_Handle->GPIO_pinConfig.GPIO_PinSpeed;
			CNF = GPIO_CR_CNF_AF_OUTPUT_OD;
			break;
		case GPIO_MODE_IT_RT:
		case GPIO_MODE_IT_FT:
		case GPIO_MODE_IT_RFT:
		case GPIO_MODE_INPUT:
			MODE = GPIO_CR_MODE_INPUT;
			switch (pGPIOx_Handle->GPIO_pinConfig.GPIO_PinPull)
			{
				case GPIO_NOPULL:
					CNF = GPIO_CR_CNF_INPUT_FLOATING;
					break;
				case GPIO_PULLUP:
					CNF = GPIO_CR_CNF_INPUT_PU_PD;
					pGPIOx_Handle->pGPIOx->ODR |= (1 << position);
					break;
				case GPIO_PULLDOWN:
					CNF = GPIO_CR_CNF_INPUT_PU_PD;
					pGPIOx_Handle->pGPIOx->ODR &= ~(1 << position);
					break;
			}
			break;
		case GPIO_MODE_ANALOG:
			MODE = GPIO_CR_MODE_INPUT;
			CNF = GPIO_CR_CNF_ANALOG;
			break;
	}

	// Extra configuration for Interrupt modes (on the peripheral side).
	if (GPIOx_IsEXTI(pGPIOx_Handle->GPIO_pinConfig.GPIO_PinMode))
	{
		// Step 1: Select trigger edge.
		if (pGPIOx_Handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			SET_BIT(EXTI->RTSR, position);
			CLEAR_BIT(EXTI->FTSR, position);
		}
		else if (pGPIOx_Handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			CLEAR_BIT(EXTI->RTSR, position);
			SET_BIT(EXTI->FTSR, position);
		}
		else if (pGPIOx_Handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			SET_BIT(EXTI->RTSR, position);
			SET_BIT(EXTI->FTSR, position);
		}

		// Step 2: Control the port mapped to the EXTI interrupts.
		uint8_t portSelectVal = EXTI_PORT_SELECT_VAL(pGPIOx_Handle->pGPIOx); // Generate the code to select which port is connected to the EXTI line
		uint8_t EXTI_configOffset = (position % 4) * 4;
		__vo uint32_t *EXTI_configReg = NULL;
		switch (position / 4)
		{
			case 0:
				EXTI_configReg = &AFIO->EXTICR1;
				break;
			case 1:
				EXTI_configReg = &AFIO->EXTICR2;
				break;
			case 2:
				EXTI_configReg = &AFIO->EXTICR3;
				break;
			case 3:
				EXTI_configReg = &AFIO->EXTICR4;
				break;
			default:
				break;
		}
		AFIO_CLK_EN();
		uint32_t clearMask = CLEAR_MASK_GEN(4) << EXTI_configOffset;
		MODIFY_REG(*EXTI_configReg, clearMask, portSelectVal << EXTI_configOffset);

		// Step 3: Enable the EXTI interrupts.
		SET_BIT(EXTI->IMR, position);
	}

	__vo uint32_t *configReg = (position < PIN_8) ? &pGPIOx_Handle->pGPIOx->CRL : &pGPIOx_Handle->pGPIOx->CRH;
	uint32_t config = (CNF << 2) | MODE;

	// Clear both CNFy and MODEy
	uint32_t clearMask = CLEAR_MASK_GEN(4) << configOffset;
	// Set CNFy and MODEy according to configuration
	MODIFY_REG(*configReg, clearMask, config << configOffset);
}

void GPIOx_DeInit(GPIOx_Reg_t *pGPIOx)
{
	uint8_t rstBit = 0;

	if (pGPIOx == GPIOA)
	{
		rstBit = 2;
	}
	else if (pGPIOx == GPIOB)
	{
		rstBit = 3;
	}
	else if (pGPIOx == GPIOC)
	{
		rstBit = 4;
	}
	else if (pGPIOx == GPIOD)
	{
		rstBit = 5;
	}
	else if (pGPIOx == GPIOE)
	{
		rstBit = 6;
	}

	RCC->APB2RSTR |= (1 << rstBit);
	RCC->APB2RSTR &= ~(1 << rstBit);
}


/********** Read/write operations **********/

uint8_t GPIOx_PinRead(GPIOx_Reg_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t pinVal = 0;
	pinVal = (uint8_t)(pGPIOx->IDR & (0x00000001 << pinNumber));
	return pinVal;
}

uint16_t GPIOx_PortRead(GPIOx_Reg_t *pGPIOx)
{
	uint16_t portVal = 0;
	portVal = (uint16_t)(pGPIOx->IDR);
	return portVal;
}

void GPIOx_PinWrite(GPIOx_Reg_t *pGPIOx, uint8_t pinNumber, uint8_t pinValue)
{
	if (pinValue == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (pinValue << pinNumber);
	}
	else if (pinValue == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(pinValue << pinNumber);
	}
}

void GPIOx_PortWrite(GPIOx_Reg_t *pGPIOx, uint16_t portValue)
{
	pGPIOx->ODR = portValue;
}

void GPIOx_PinToggle(GPIOx_Reg_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}


/********** Interrupts control and handler **********/

uint8_t GPIOx_IsEXTI(uint8_t mode)
{
	return ((mode == GPIO_MODE_IT_RT) || (mode == GPIO_MODE_IT_FT) || (mode == GPIO_MODE_IT_RFT)) ? 1 : 0;
}

void GPIOx_IRQControl(uint8_t IRQ_Number, uint8_t enableState)
{
	uint8_t NVIC_regNum = IRQ_Number / 32;
	uint8_t NVIC_configOffset = IRQ_Number % 32;

	__vo uint32_t *configReg = NULL;

	if (enableState == ENABLE)
	{
		configReg = (NVIC_regNum == 0) ? NVIC_ISER0 :
					(NVIC_regNum == 1) ? NVIC_ISER1 : NULL;
	}
	else if (enableState == DISABLE)
	{
		configReg = (NVIC_regNum == 0) ? NVIC_ICER0 :
					(NVIC_regNum == 1) ? NVIC_ICER1 : NULL;
	}

	/* Mask or un-mask an interrupt */
	SET_BIT(*configReg, NVIC_configOffset);
}

void GPIOx_IRQSetPriority(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
	uint8_t Priority_regNum = IRQ_Number / 4;
	uint8_t Priority_configOffset = (IRQ_Number % 4) * 8 + (8 - IMPLEMENTED_PR_BITS);

	uint8_t clearMask = CLEAR_MASK_GEN(4) << Priority_configOffset;

	MODIFY_REG(*(NVIC_IPRx_BASEADDR + Priority_regNum), clearMask, IRQ_Priority << Priority_configOffset);
}

void GPIOx_IRQHandler(uint8_t pinNumber)
{
	// Clear the pending bit in the EXTI peripheral.
	if ( ((EXTI->PR >> pinNumber) & 1) == 1 )
	{
		SET_BIT(EXTI->PR, pinNumber);
	}
}
