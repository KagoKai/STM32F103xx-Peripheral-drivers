/*
 * stm32f1xx_i2c.h
 *
 *  Created on: Sep 9, 2023
 *      Author: ADMIN
 */

#ifndef INC_STM32F1XX_I2C_H_
#define INC_STM32F1XX_I2C_H_

#include "stm32f103xx.h"

/******************** I2C specific definitions & macros ********************/

/* I2C clock speed macros */
#define I2C_SCL_STANDARDMODE_MAX			100000u
#define I2C_SCL_FASTMODE_MAX				400000u

/* I2C clock stretching macros */
#define I2C_CLK_STRETCH_ENABLE		0u
#define I2C_CLK_STRETCH_DISABLE 	1u

/* I2C ACK control macros */
#define I2C_ACK_DISABLE				0u
#define I2C_ACK_ENABLE				1u

/* I2C FM duty cycle macros */
#define I2C_FM_DUTY_2				0u
#define I2C_FM_DUTY_16_9			1u

typedef struct
{
	uint32_t I2C_ClockSpeed;			// In Hz
	uint8_t I2C_ClockStretching;
	uint8_t I2C_OwnAddress; 			// Using only 7-bit address for now
	uint8_t I2C_ACKControl;
	uint16_t I2C_FastModeDutyCycle;
}I2Cx_Config_t;

typedef struct
{
	I2Cx_Reg_t *pI2Cx;
	I2Cx_Config_t I2C_config;
}I2Cx_Handle_t;

/****************************************************************************************
 *					APIs supported by this driver                                       *
 *					Read the function definition for more information                   *
 ****************************************************************************************/

/********** I2Cx clock control **********/

/** @brief Control the bus clock for a I2C peripheral
 *
 *  @param *pI2Cx The pointer to the base address of a SPI.
 *  @enableState Can be ENABLE or DISABLE, controls the clock accordingly.
 *  @return Void.
 */
void I2Cx_ClkControl(I2Cx_Reg_t *pI2Cx, uint8_t enableState);


/********** I2Cx init & denit **********/

/** @brief Initializes a I2C with the specified
 * 		   configuration.
 *
 * 	@param *pI2Cx_Handle The handle pointer with setup configuration.
 * 	@return Void.
 */
void I2Cx_Init(I2Cx_Handle_t *pI2Cx_Handle);

/** @brief Deinitializes a I2C and resets it back to the
 * 		   default state.
 *
 *  @param *pI2Cx The base address of the I2C peripheral.
 *  @return Void.
 */
void I2Cx_DeInit(I2Cx_Reg_t *pI2Cx);


/********** Data transfer & receive **********/

/** @brief  Generates the START condition (Master).
 *
 *  @param *pI2Cx The base address of the I2C peripheral.
 *  @return void
 */
void I2Cx_Start(I2Cx_Reg_t *pI2Cx);

/** @brief Generates the STOP condition (Master).
 *
 *  @param *pI2Cx The base address of the I2C peripheral.
 *  @return void
 */
void I2Cx_Stop(I2Cx_Reg_t *pI2Cx);

#endif /* INC_STM32F1XX_I2C_H_ */
