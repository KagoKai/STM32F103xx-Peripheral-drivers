/*
 * stm32f1xx_spi.c
 *
 *  Created on: Jul 3, 2023
 *      Author: ADMIN
 */

#include "stm32f1xx_spi.h"

/********** SPIx clock control **********/

void SPIx_ClkControl(SPIx_Reg_t *pSPIx, uint8_t enableState)
{
	if (enableState == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
	}
	else if (enableState == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_CLK_DI();
		}
	}
}


/********** SPIx init & denit **********/

void SPIx_Init(SPIx_Handle_t *pSPIx_Handle)
{
	SPIx_Reg_t *pSPIx = pSPIx_Handle->pSPIx;
	SPIx_Config_t config = pSPIx_Handle->SPI_config;

	// 1. Control the peripheral mode
	WRITE_BIT(pSPIx->CR1, config.SPI_Mode, 2);

	// 2. Set the bus mode
	if (config.SPI_BusMode == SPI_BUSMODE_FULLDUPLEX)
	{
		// TODO: Implement with BIDIMODE = 0 and D
		CLEAR_BIT(pSPIx->CR1, 15);
	}
	else if (config.SPI_BusMode == SPI_BUSMODE_HALFDUPLEX)
	{
		// TODO: Implement with BIDIMODE = 1 and BIDIOE = 1
		SET_BIT(pSPIx->CR1, 15);
		SET_BIT(pSPIx->CR1, 14);
	}
	else if (config.SPI_BusMode == SPI_BUSMODE_SIMPLEX_RX)
	{
		// TODO: Implement with BIDIMODE = 0 and RXONLY = 1
		CLEAR_BIT(pSPIx->CR1, 15);
		SET_BIT(pSPIx->CR1, 10);
	}
	else if (config.SPI_BusMode == SPI_BUSMODE_SIMPLEX_TX)
	{
		// TODO: Implement with BIDIMODE = 0 and RXONLY = 0
		CLEAR_BIT(pSPIx->CR1, 15);
		CLEAR_BIT(pSPIx->CR1, 10);
	}

	// 3. Set the SPI speed division coefficient of SCLK
	if (config.SPI_Mode == SPI_MODE_MASTER)
	{
		uint32_t clearMask = CLEAR_MASK_GEN(3) << 3;
		MODIFY_REG(pSPIx->CR1, clearMask, config.SPI_Speed << 3);
	}

	// 4. Set the mode using CPOL and CPHA
	uint8_t CPHA = (config.SPI_CLKMode >> 0) & 1;
	uint8_t CPOL = (config.SPI_CLKMode >> 1) & 1;
	WRITE_BIT(pSPIx->CR1, CPHA, 0);
	WRITE_BIT(pSPIx->CR1, CPOL, 1);

	// 5. Set the data frame format
	WRITE_BIT(pSPIx->CR1, config.SPI_DFF, 11);

	// 6. Set the bit direction
	WRITE_BIT(pSPIx->CR1, config.SPI_DataDirection, 7);

	//7. Control the NSS pin
	if (config.SPI_SSM == SPI_SSM_HARDWARE)
	{
		CLEAR_BIT(pSPIx->CR1, 9);
		WRITE_BIT(pSPIx->CR2, config.SPI_MultiMaster, 2);
	}
	else if (config.SPI_SSM == SPI_SSM_SOFTWARE)
	{
		SET_BIT(pSPIx->CR1, 9);
		WRITE_BIT(pSPIx->CR1, ~config.SPI_Mode, 8);
	}
}

void SPIx_DeInit(SPIx_Reg_t *pSPIx)
{
	uint8_t rstBit = 0;

	if (pSPIx == SPI1)
	{
		rstBit = 12;
		SET_BIT(RCC->APB2RSTR, rstBit);
		CLEAR_BIT(RCC->APB2RSTR, rstBit);
		return;
	}
	else if (pSPIx == SPI2)
	{
		rstBit = 14;
	}
	else if (pSPIx == SPI3)
	{
		rstBit = 15;
	}
	SET_BIT(RCC->APB1RSTR, rstBit);
	CLEAR_BIT(RCC->APB1RSTR, rstBit);
}

uint8_t SPIx_GetFlag(SPIx_Reg_t *pSPIx, uint8_t flag)
{
	if (((pSPIx->SR >> flag) & 1) == FLAG_SET)	return FLAG_SET;
	else return FLAG_CLEAR;
}


/********** Data transfer & receive **********/

void SPIx_Start(SPIx_Reg_t *pSPIx)
{
	SET_BIT(pSPIx->CR1, 6);
}

void SPIx_Stop(SPIx_Reg_t *pSPIx)
{
	CLEAR_BIT(pSPIx->CR1, 6);
}

void SPIx_Send(SPIx_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen)
{
	/* Everytime the Data Register is written, the MCU sends its value to the TxBuffer.
	 * Data transmission starts once Tx is written, after this the Tx_Empty flag is set.
	 * The SPI_SR_BSY flag is set when the peripheral is busy, or the Tx is not empty.
	 */
	const uint8_t DFF = READ_BIT(pSPIx->CR1, 11); // 1 for 16-bit, 0 for 8-bit

	while (dataLen > 0)
	{
		// Wait until the Tx buffer is empty (TXE is set).
		while (SPIx_GetFlag(pSPIx, SPI_FLAG_TXE) != FLAG_SET);

		if (DFF == 1)
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
		}
		else
		{
			pSPIx->DR = *pTxBuffer;
		}

		dataLen -= (1+DFF);
		pTxBuffer += (1+DFF);
	}

	// Wait until the end of transfer (TXE is set and BSY is cleared)
	while ((SPIx_GetFlag(pSPIx, SPI_FLAG_TXE) != FLAG_SET) && (SPIx_GetFlag(pSPIx, SPI_FLAG_BSY) != FLAG_CLEAR));
}

void SPIx_Receive(SPIx_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen)
{
	uint8_t DFF = READ_BIT(pSPIx->CR1, 11);

	while (dataLen > 0)
	{
		while ( (SPIx_GetFlag(pSPIx, SPI_FLAG_BSY) == FLAG_SET) & (SPIx_GetFlag(pSPIx, SPI_FLAG_RXNE) != FLAG_SET) );

		if (DFF == 1)
		{
			*pRxBuffer = pSPIx->DR;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
		}

		dataLen -= (1+DFF);
		pRxBuffer += (1+DFF);
	}
}

void SPIx_SendIT(SPIx_Handle_t *hspix, uint8_t *pTxBuffer, uint32_t dataLen)
{
	// TODO: Enable the TXE interrupt flag on the peripheral side
	SET_BIT(hspix->pSPIx->CR2, 7);

}


/********** Interrupts control and handler **********/

void SPIx_IRQControl(uint8_t IRQ_Number, uint8_t enableState)
{
	uint8_t NVIC_regNum = IRQ_Number / 32;
	uint8_t NVIC_configOffset = IRQ_Number % 32;

	// Mask or un-mask an interrupt
	switch (NVIC_regNum) {
		case 0:
			if (enableState == ENABLE)
			{
				SET_BIT(*NVIC_ISER0, NVIC_configOffset);
			}
			else if (enableState == DISABLE)
			{
				SET_BIT(*NVIC_ICER0, NVIC_configOffset);
			}
			break;
		case 1:
			if (enableState == ENABLE)
			{
				SET_BIT(*NVIC_ISER1, NVIC_configOffset);
			}
			else if (enableState == DISABLE)
			{
				SET_BIT(*NVIC_ICER1, NVIC_configOffset);
			}
			break;
		default:
			break;
	}
}

void SPIx_IRQSetPriority(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
	uint8_t Priority_regNum = IRQ_Number / 4;
	uint8_t Priority_configOffset = (IRQ_Number % 4) * 8 + (8 - IMPLEMENTED_PR_BITS);

	uint8_t clearMask = CLEAR_MASK_GEN(4) << Priority_configOffset;

	MODIFY_REG(*(NVIC_IPRx_BASEADDR + Priority_regNum), clearMask, IRQ_Priority << Priority_configOffset);
}
