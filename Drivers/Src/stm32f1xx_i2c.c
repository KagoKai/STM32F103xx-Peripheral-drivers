/*
 * stm32f1xx_i2c.c
 *
 *  Created on: Sep 9, 2023
 *      Author: ADMIN
 */

#include "stm32f1xx_i2c.h"
#include <math.h>

/********** I2Cx clock control **********/

void I2Cx_ClkControl(I2Cx_Reg_t *pI2Cx, uint8_t enableState)
{
	if (enableState == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_CLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_EN();
		}
	}
	else if (enableState == DISABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_CLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_CLK_DI();
		}
	}
}

#define HSI_CLK_FREQ		8000000u

#define RCC_SYSTEM_CLK_HSI		0u
#define RCC_SYSTEM_CLK_HSE		1u
#define RCC_SYSTEM_CLK_PLL		2u

#define RCC_APB1_CLK_DIV2		4u
#define RCC_APB1_CLK_DIV4		5u
#define RCC_APB1_CLK_DIV8		6u
#define RCC_APB1_CLK_DIV16		7u

#define RCC_AHB_CLK_DIV2		8u
#define RCC_AHB_CLK_DIV4		9u
#define RCC_AHB_CLK_DIV8		10u
#define RCC_AHB_CLK_DIV16		11u
#define RCC_AHB_CLK_DIV64		12u
#define RCC_AHB_CLK_DIV128		13u
#define RCC_AHB_CLK_DIV256		14u
#define RCC_AHB_CLK_DIV512		15u

uint32_t RCC_GetPCLK1(void)
{
	uint32_t clkVal = 0;

	uint8_t sysclkSrc = RCC->CFGR & (0x3 << 2);
	uint8_t hclkDiv	= RCC->CFGR & (0xF << 4);
	uint8_t pclk1Div = RCC->CFGR & (0x7 << 8);

	uint8_t divNum[2];
	// Calculate the AHB bus division
	if (hclkDiv < RCC_AHB_CLK_DIV2)
	{
		divNum[0] = 1;
	}
	else if (hclkDiv <= RCC_AHB_CLK_DIV16)
	{
		divNum[0] = pow(2, hclkDiv - 7);
	}
	else
	{
		divNum[0] = pow(2, hclkDiv - 6);
	}
	// Calculate the APB1 bus division
	if (pclk1Div < RCC_APB1_CLK_DIV2)
	{
		divNum[1] = 1;
	}
	else
	{
		divNum[1] = pow(2, pclk1Div - 3);
	}

	if (sysclkSrc == RCC_SYSTEM_CLK_HSI)
	{
		clkVal = HSI_CLK_FREQ;
		for (int i=0; i < 2; i++)
		{
			clkVal /= divNum[i];
		}
	}

	return clkVal;
}
/********** I2Cx init & denit **********/

void I2Cx_Init(I2Cx_Handle_t *pI2Cx_Handle)
{
	I2Cx_Reg_t *pI2Cx = pI2Cx_Handle->pI2Cx;
	I2Cx_Config_t config = pI2Cx_Handle->I2C_config;

	uint32_t tmpVal = 0;

	/* 1. Control the clock */
	// Set the six FREQ bits
	tmpVal = RCC_GetPCLK1();
	MODIFY_REG(pI2Cx->CR1, 0x3F, tmpVal & 0x3F);

	// Set the SCL mode
	if (config.I2C_ClockSpeed <= I2C_SCL_STANDARDMODE_MAX)
	{
		// Standard mode
		CLEAR_BIT(pI2Cx->CCR, 15);

		// Calculate the CCR value
		tmpVal = RCC_GetPCLK1() / (2 * config.I2C_ClockSpeed);
	}
	else if (config.I2C_ClockSpeed <= I2C_SCL_FASTMODE_MAX)
	{
		// Fast mode
		SET_BIT(pI2Cx->CCR, 15);

		// Set the duty
		uint8_t fastModeDuty = config.I2C_FastModeDutyCycle;
		if (fastModeDuty == I2C_FM_DUTY_2)
		{
			CLEAR_BIT(pI2Cx->CCR, 14);
			tmpVal = RCC_GetPCLK1() / (3 * config.I2C_ClockSpeed);
		}
		else if (fastModeDuty == I2C_FM_DUTY_16_9)
		{
			SET_BIT(pI2Cx->CCR, 14);
			tmpVal = RCC_GetPCLK1() / (25 * config.I2C_ClockSpeed);
		}
	}
	tmpVal &= 0xFFF; 	// Only consider the first 12 bits
	MODIFY_REG(pI2Cx->CCR, CLEAR_MASK_GEN(12), tmpVal);

	/* 2. Configure the operation mode (ACK bit, IRQ, etc ...) */
	if (config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		SET_BIT(pI2Cx->CR1, 10);
	}
	else if (config.I2C_ACKControl == I2C_ACK_DISABLE)
	{
		CLEAR_BIT(pI2Cx->CR1, 10);
	}

	// 3. Set the device's address
	tmpVal = config.I2C_OwnAddress;
	MODIFY_REG(pI2Cx->OAR1, CLEAR_MASK_GEN(7) << 1, (tmpVal & 0x7F) << 1);
}

void I2Cx_DeInit(I2Cx_Reg_t *pI2Cx)
{

}
