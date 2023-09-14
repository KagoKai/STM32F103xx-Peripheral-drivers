/*
 * stm32f1xx_spi.h
 *
 *  Created on: Jun 30, 2023
 *      Author: ADMIN
 */

#ifndef INC_STM32F1XX_SPI_H_
#define INC_STM32F1XX_SPI_H_

#include "stm32f103xx.h"

/******************** SPI specific definitions & macros ********************/

/* SPI mode macros */
#define SPI_MODE_SLAVE			0u
#define SPI_MODE_MASTER			1u

/* SPI bus mode macros */
#define SPI_BUSMODE_FULLDUPLEX			0u
#define SPI_BUSMODE_HALFDUPLEX			1u
#define SPI_BUSMODE_SIMPLEX_TX			2u	// Two simplex modes refer to the single-line configuration
#define SPI_BUSMODE_SIMPLEX_RX			3u

/* SPI baud rate macros (SCLK frequency) */
#define SPI_SPEED_DIV2			0u
#define SPI_SPEED_DIV4			1u
#define SPI_SPEED_DIV8			2u
#define SPI_SPEED_DIV16			3u
#define SPI_SPEED_DIV32			4u
#define SPI_SPEED_DIV64			5u
#define SPI_SPEED_DIV128		6u
#define SPI_SPEED_DIV256		7u

/* Data Frame Format macros */
#define SPI_DFF_8BITS			0u
#define SPI_DFF_16BITS			1u

/* Frame format macros */
#define SPI_DIRECTION_MSBFIRST			0u
#define SPI_DIRECTION_LSBFIRST			1u

/* Clock mode macros */
#define SPI_CLK_MODE_0			0u	// 00
#define SPI_CLK_MODE_1			1u	// 01
#define SPI_CLK_MODE_2			2u	// 10
#define SPI_CLK_MODE_3			3u	// 11

/* Slave management macros */
#define SPI_SSM_HARDWARE		0u
#define SPI_SSM_SOFTWARE		1u

/* Multi-Master mode */
#define SPI_MM_ENABLE			0u
#define SPI_MM_DISABLE			1u

/* Status flag macros */
#define SPI_FLAG_RXNE				0u
#define SPI_FLAG_TXE				1u
#define SPI_FLAG_BSY				7u

typedef struct
{
	uint8_t SPI_Mode;			// Determines Master or Slave mode
	uint8_t SPI_BusMode;		// Determines the duplex mode
	uint8_t SPI_Speed;			// Determines the SCLK speed (division of f_clk)
	uint8_t SPI_DFF;			// Data Frame Format
	uint8_t SPI_DataDirection;	// LSB first or MSB first
	uint8_t SPI_CLKMode;		// Control using CPOL and CPHA
	uint8_t SPI_MultiMaster;    // Control the Output Enable mode of NSS pin
	uint8_t SPI_SSM;			// Control the slave select pin
}SPIx_Config_t;

typedef struct
{
	SPIx_Reg_t *pSPIx;
	SPIx_Config_t SPI_config;
}SPIx_Handle_t;


/****************************************************************************************
 *					APIs supported by this driver                                       *
 *					Read the function definition for more information                   *
 ****************************************************************************************/

/********** SPIx clock control **********/

/** @brief Control the bus clock for a SPI peripheral
 *
 *  @param *pSPIx The pointer to the base address of a SPI.
 *  @enableState Can be ENABLE or DISABLE, controls the clock accordingly.
 *  @return Void.
 */
void SPIx_ClkControl(SPIx_Reg_t *pSPIx, uint8_t enableState);


/********** SPIx init & denit **********/

/** @brief Initializes a SPI with the specified
 * 		   configuration.
 *
 * 	@param *pSPIx_Handle The handle pointer with setup configuration.
 * 	@return Void.
 */
void SPIx_Init(SPIx_Handle_t *pSPIx_Handle);

/** @brief Deinitializes a SPI and resets it back to the
 * 		   default state.
 *
 *  @param *pSPIx The pointer to the base address of a SPI.
 *  @return Void.
 */
void SPIx_DeInit(SPIx_Reg_t *pSPIx);

/** @brief Return the corresponding flag value in the Status
 * 		   register.
 *
 *  @param *pSPIx The pointer to the base address of a SPI.
 *  @param flagName The macro of the flag to read.
 *  @return True or False.
 */
uint8_t SPIx_GetFlag(SPIx_Reg_t *pSPIx, uint8_t flag);


/********** Data transfer & receive **********/

/** @brief Starts the SPI peripheral.
 *
 *  @param *pSPIx The pointer to the base address of a SPI.
 *  @return void
 */
void SPIx_Start(SPIx_Reg_t *pSPIx);

/** @brief Stops the SPI peripheral.
 *
 *  @param *pSPIx The pointer to the base address of a SPI.
 *  @return void
 */
void SPIx_Stop(SPIx_Reg_t *pSPIx);

/** @brief Sends the data from the TxBuffer to the MOSI/MISO
 * 		   line (blocking).
 *
 *  @param pSPIx The pointer to the SPI peripheral.
 *  @param pTxBuffer The pointer to the TxBuffer of the SPI.
 *  @param dataLen The length of the data frame in byte.
 *  @return Void.
 */
void SPIx_Send(SPIx_Reg_t *pSPIx, uint8_t *pTxBuffer, uint32_t dataLen);

/** @brief Receives the data on the MOSI/MISO line and
 * 		   writes it to the RxBuffer (blocking).
 *
 *  @param pSPIx The pointer to the SPI peripheral.
 *  @param pRxBuffer The pointer to the RxBuffer of the SPI.
 *  @param dataLen The length of the data frame in byte.
 *  @return Void.
 */
void SPIx_Receive(SPIx_Reg_t *pSPIx, uint8_t *pRxBuffer, uint32_t dataLen);

/** @brief Sets up the peripheral for data transmission
 * 		   using interrupt services.
 *
 *  @param
 *  @return
 */
void SPIx_SendIT(SPIx_Handle_t *hspix, uint8_t *pTxBuffer, uint32_t dataLen);

/** @brief Sets up the peripheral for data reception
 * 		   using interrupt services.
 *
 *  @param
 *  @return
 */
void SPIx_ReceiveIT(void);


/********** Interrupts control and handler **********/

/** @brief Enables or disables an I/O interrupt (on the NVIC side).
 *
 *  @param IRQ_Number The interrupt to be controlled.
 *  @param enableState Can be ENABLE or DISABLE, modifies the interrupt state accordingly.
 *  @return Void.
 */
void SPIx_IRQControl(uint8_t IRQ_Number, uint8_t enableState);

/** @brief Changes an interrupt's priority.
 *
 *  @param IRQ_Number The interrupt to be controlled.
 *  @param IRQ_Priority The final priority level.
 *  @return Void.
 *
 * 	@Note Only the 4 MSB of each section are used to control the
 * 		  priority level in STM32F1xx.
 */
void SPIx_IRQSetPriority(uint8_t IRQ_Number, uint32_t IRQ_Priority);

/** @brief Handles interrupts (TXE, RXNE, etc.) for a SPI peripheral
 *
 *  @param
 *  @return Void.
 */
void SPIx_IRQHandler(void);

#endif /* INC_STM32F1XX_SPI_H_ */
