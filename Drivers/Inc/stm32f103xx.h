 /*
 * stm32f103xx.h
 *
 *  Created on: Jun 15, 2023
 *      Author: ADMIN
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define __vo volatile

/******************** Cortex processors specific macros ********************/
/*
 * NVIC IRQs enable and disable registers.
 */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)

#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)

#define NVIC_IPRx_BASEADDR 		((__vo uint32_t*)0xE000E400)

/******************** Function macros ********************/
#define SET_BIT(REG, POS)     ((REG) |= (1 << POS))
#define CLEAR_BIT(REG, POS)   ((REG) &= ~(1 << POS))
#define READ_BIT(REG, POS)    ((REG) & (1 << POS))
#define WRITE_BIT(REG, VAL, POS)	((REG) = ((CLEAR_BIT(REG, POS)) | (VAL << POS)))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define CLEAR_MASK_GEN(BITNUM)	(((int)pow(2, (BITNUM)) - 1))

#define EXTI_PORT_SELECT_VAL(PORT)	  (	((PORT) == GPIOA) ? 0 : \
										((PORT) == GPIOB) ? 1 : \
										((PORT) == GPIOC) ? 2 : \
										((PORT) == GPIOD) ? 3 : \
										((PORT) == GPIOE) ? 4 : \
										((PORT) == GPIOF) ? 5 : \
										((PORT) == GPIOG) ? 6 : 0 )


/******************** Macros for memory domains ********************/
/*
 * C macros for MCU memories: Flash, ROM, SRAM
 */
#define FLASH_BASEADDR					0x08000000u
#define SRAM_BASEADDR					0x20000000u
#define ROM_BASEADDR					0x1FFFF000u
#define SRAM 							SRAM_BASEADDR

/*
 * C macros for peripherals bus domains: AHB, APB1, APB2
 */
#define PERIPHERALS_BASEADDR			0x40000000u
#define APB1_BASEADDR					PERIPHERALS_BASEADDR
#define APB2_BASEADDR 					0x40010000u
#define AHB_BASEADDR					0x40018000u

/*
 * APB1 peripherals memory locations
 */
#define TIM2_BASEADDR 					(APB1_BASEADDR + 0x0000)
#define TIM3_BASEADDR					(APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR 					(APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR					(APB1_BASEADDR + 0x0C00)
#define TIM6_BASEADDR 					(APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR					(APB1_BASEADDR + 0x1400)
#define TIM12_BASEADDR					(APB1_BASEADDR + 0x1800)
#define TIM13_BASEADDR					(APB1_BASEADDR + 0x1C00)
#define TIM14_BASEADDR					(APB1_BASEADDR + 0x2000)
#define RTC_BASEADDR					(APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR					(APB1_BASEADDR + 0x2C00) // Window watchdog
#define IWDG_BASEADDR					(APB1_BASEADDR + 0x3000) // Independent watchdog
#define SPI2_BASEADDR					(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1_BASEADDR + 0x3C00)
#define USART2_BASEADDR					(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR					(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1_BASEADDR + 0x5000)
#define I2C1_BASEADDR					(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1_BASEADDR + 0x5800)
#define USB_BASEADDR					(APB1_BASEADDR + 0x5C00)
#define USB_CAN_SRAM_BASEADDR			(APB1_BASEADDR + 0x6000) // Shared USB/CAN SRAM
#define CAN1_BASEADDR					(APB1_BASEADDR + 0x6400)
#define CAN2_BASEADDR					(APB1_BASEADDR + 0x6800)
#define BKP_BASEADDR					(APB1_BASEADDR + 0x6C00) // Backup registers
#define PWR_BASEADDR					(APB1_BASEADDR + 0x7000) // Power control
#define DAC_BASEADDR					(APB1_BASEADDR + 0x7400)

/*
 * APB2 peripherals memory locations
 */
#define AFIO_BASEADDR					(APB2_BASEADDR + 0x0000)
#define EXTI_BASEADDR					(APB2_BASEADDR + 0x0400)
#define GPIOA_BASEADDR					(APB2_BASEADDR + 0x0800)
#define GPIOB_BASEADDR					(APB2_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR					(APB2_BASEADDR + 0x1000)
#define GPIOD_BASEADDR					(APB2_BASEADDR + 0x1400)
#define GPIOE_BASEADDR					(APB2_BASEADDR + 0x1800)
#define GPIOF_BASEADDR					(APB2_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR					(APB2_BASEADDR + 0x2000)
#define ADC1_BASEADDR 					(APB2_BASEADDR + 0x2400)
#define ADC2_BASEADDR					(APB2_BASEADDR + 0x2800)
#define ADC3_BASEADDR					(APB2_BASEADDR + 0x3C00)
#define TIM1_BASEADDR 					(APB2_BASEADDR + 0x2C00)
#define TIM8_BASEADDR					(APB2_BASEADDR + 0x3400)
#define TIM9_BASEADDR					(APB2_BASEADDR + 0x4C00)
#define TIM10_BASEADDR					(APB2_BASEADDR + 0x5000)
#define TIM11_BASEADDR					(APB2_BASEADDR + 0x5400)
#define SPI1_BASEADDR					(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR					(APB2_BASEADDR + 0x3800)

/*
 * AHB peripherals memory locations
 */
#define DMA1_BASEADDR 					(AHB_BASEADDR + 0x8000)
#define DMA2_BASEADDR 					(AHB_BASEADDR + 0x8400)
#define RCC_BASEADDR 					(AHB_BASEADDR + 0x9000)


/******************** Peripherals registers definition ********************/
/*
 * RCC peripheral registers structure
 */
typedef struct
{
	__vo uint32_t CR;			// Clock control register. 						Offset: 0x00
	__vo uint32_t CFGR;			// Clock configuration register. 				Offset: 0x04
	__vo uint32_t CIR;			// Clock interrupt register. 					Offset: 0x08
	__vo uint32_t APB2RSTR;		// APB2 peripheral reset register. 				Offset: 0x0C
	__vo uint32_t APB1RSTR;		// APB1 peripheral reset register. 				Offset: 0x10
	__vo uint32_t AHBENR;		// AHB Peripheral Clock enable register. 		Offset: 0x14
	__vo uint32_t APB2ENR;		// APB2 Peripheral Clock enable register. 		Offset: 0x18
	__vo uint32_t APB1ENR;		// APB1 Peripheral Clock enable register. 		Offset: 0x1C
	__vo uint32_t BDCR;			// Backup domain control register.				Offset: 0x20
	__vo uint32_t CSR;			// Control/status register. 					Offset: 0x24
	__vo uint32_t AHBSTR;		// AHB peripheral clock reset register. 		Offset: 0x28
	__vo uint32_t CFGR2;		// Clock configuration register2. 				Offset: 0x2C
}RCC_Reg_t;

/*
 * EXTI peripheral registers structure
 */
typedef struct
{
	__vo uint32_t IMR;			// Interrupt mask register.						Offset: 0x00
	__vo uint32_t EMR;			// Event mask register.							Offset: 0x04
	__vo uint32_t RTSR;			// Rising trigger selection register.			Offset: 0x08
	__vo uint32_t FTSR;			// Falling trigger selection register.			Offset: 0x0C
	__vo uint32_t SWIER;		// Software interrupt event register.			Offset: 0x10
	__vo uint32_t PR;			// Pending register.							Offset: 0x14
}EXTI_Reg_t;

/*
 * GPIOx peripheral registers structure
 */
typedef struct
{
	__vo uint32_t CRL;			// Port configuration register low. 		Offset: 0x00
	__vo uint32_t CRH;			// Port configuration register high. 		Offset: 0x04
	__vo uint32_t IDR;			// Port input data register. 				Offset: 0x08
	__vo uint32_t ODR;			// Port output data register. 				Offset: 0x0C
	__vo uint32_t BRSS;			// Port bit set/reset register. 			Offset: 0x10
	__vo uint32_t BRR;			// Port bit reset register. 				Offset: 0x14
	__vo uint32_t LCKR;			// Port configuration lock register. 		Offset: 0x18
}GPIOx_Reg_t;

/*
 * AFIO registers structure
 */
typedef struct
{
	__vo uint32_t EVCR;			// Event control register.								Offset: 0x00
	__vo uint32_t MAPR;			// Alternate function remap & debug register.			Offset: 0x04
	__vo uint32_t EXTICR1;		// External interrupt configuration register 1. 		Offset: 0x08
	__vo uint32_t EXTICR2;		// External interrupt configuration register 2.			Offset: 0x0C
	__vo uint32_t EXTICR3;		// External interrupt configuration register 3.			Offset: 0x10
	__vo uint32_t EXTICR4;		// External interrupt configuration register 4.			Offset: 0x14
	__vo uint32_t MAPR2;		// Alternate function remap & debug register 2.			Offset: 0x18
}AFIO_Reg_t;

/*
 * SPI peripheral registers structure
 */
typedef struct
{
	__vo uint32_t CR1;			// SPI configuration register 1.			Offset: 0x00
	__vo uint32_t CR2;			// SPI configuration register 2.			Offset: 0x04
	__vo uint32_t SR;			// SPI status register.						Offset: 0x08
	__vo uint32_t DR;			// SPI data register.						Offset: 0x0C
	__vo uint32_t CRCPR;		// SPI CRC polynomial register.				Offset: 0x10
	__vo uint32_t RXCRCR;		// SPI RX CRC register.						Offset: 0x14
	__vo uint32_t TXCRCR;		// SPI TX CRC register.						Offset: 0x18
	__vo uint32_t I2SCFGR;		// I2S configuration register.				Offset: 0x1C
	__vo uint32_t I2SPR;		// I2S prescaler register.					Offset: 0x20
}SPIx_Reg_t;

/*
 * I2C peripheral registers structure
 */
typedef struct
{
	__vo uint32_t CR1;			// I2C configuration register 1.			Offset: 0x00
	__vo uint32_t CR2;			// I2C configuration register 2.			Offset: 0x04
	__vo uint32_t OAR1;			// I2C own address register 1.				Offset: 0x08
	__vo uint32_t OAR2;			// I2C own address register 2.				Offset: 0x0C
	__vo uint32_t DR;			// I2C data register.						Offset: 0x10
	__vo uint32_t SR1;			// I2C status register 1.					Offset: 0x14
	__vo uint32_t SR2;			// I2C status register 2.					Offset: 0x18
	__vo uint32_t CCR;			// I2C clock control register.				Offset: 0x1C
	__vo uint32_t TRSISE;		// I2C TRISE register.						Offset: 0x20
}I2Cx_Reg_t;

/*
 * Peripheral definition macros
 */
#define RCC								((RCC_Reg_t*)RCC_BASEADDR)
#define EXTI 							((EXTI_Reg_t*)EXTI_BASEADDR)
#define AFIO 							((AFIO_Reg_t*)AFIO_BASEADDR)

#define GPIOA							((GPIOx_Reg_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIOx_Reg_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIOx_Reg_t*)GPIOC_BASEADDR)
#define GPIOD							((GPIOx_Reg_t*)GPIOD_BASEADDR)
#define GPIOE							((GPIOx_Reg_t*)GPIOE_BASEADDR)
#define GPIOF							((GPIOx_Reg_t*)GPIOF_BASEADDR)
#define GPIOG							((GPIOx_Reg_t*)GPIOG_BASEADDR)

#define SPI1							((SPIx_Reg_t*)SPI1_BASEADDR)
#define SPI2							((SPIx_Reg_t*)SPI2_BASEADDR)
#define SPI3							((SPIx_Reg_t*)SPI3_BASEADDR)

#define I2C1							((I2Cx_Reg_t*)I2C1_BASEADDR)
#define I2C2							((I2Cx_Reg_t*)I2C2_BASEADDR)

/*
 * GPIOx Clock Enable macros
 */
#define AFIO_CLK_EN()  ( RCC->APB2ENR |= (1 << 0) )
#define GPIOA_CLK_EN() ( RCC->APB2ENR |= (1 << 2) )
#define GPIOB_CLK_EN() ( RCC->APB2ENR |= (1 << 3) )
#define GPIOC_CLK_EN() ( RCC->APB2ENR |= (1 << 4) )
#define GPIOD_CLK_EN() ( RCC->APB2ENR |= (1 << 5) )
#define GPIOE_CLK_EN() ( RCC->APB2ENR |= (1 << 6) )

/*
 * I2Cx Clock Enable macros
 */
#define I2C1_CLK_EN() ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_CLK_EN() ( RCC->APB1ENR |= (1 << 22) )

/*
 * SPIx Clock Enable macros
 */
#define SPI1_CLK_EN() ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_CLK_EN() ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_CLK_EN() ( RCC->APB1ENR |= (1 << 15) )

/*
 * UARTx/USARTx Clock Enable macros
 */
#define USART1_CLK_EN() ( RCC->APB2ENR |= (1 << 14) )
#define USART2_CLK_EN() ( RCC->APB1ENR |= (1 << 17) )
#define USART3_CLK_EN() ( RCC->APB1ENR |= (1 << 18) )
#define UART4_CLK_EN()  ( RCC->APB1ENR |= (1 << 19) )
#define UART5_CLK_EN()  ( RCC->APB1ENR |= (1 << 20) )

/*
 * GPIOx Clock Disable macros
 */
#define AFIO_CLK_DI()  ( RCC->APB2ENR &= ~(1 << 0) )
#define GPIOA_CLK_DI() ( RCC->APB2ENR &= ~(1 << 2) )
#define GPIOB_CLK_DI() ( RCC->APB2ENR &= ~(1 << 3) )
#define GPIOC_CLK_DI() ( RCC->APB2ENR &= ~(1 << 4) )
#define GPIOD_CLK_DI() ( RCC->APB2ENR &= ~(1 << 5) )
#define GPIOE_CLK_DI() ( RCC->APB2ENR &= ~(1 << 6) )

/*
 * I2Cx Clock Disable macros
 */
#define I2C1_CLK_DI() ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_CLK_DI() ( RCC->APB1ENR &= ~(1 << 22) )

/*
 * SPIx Clock Disable macros
 */
#define SPI1_CLK_DI() ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_CLK_DI() ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_CLK_DI() ( RCC->APB1ENR &= ~(1 << 15) )


/******************** Generic macros ********************/
/*
 * C macros for IRQ number
 */
#define IRQ_NO_EXTI0				6u
#define IRQ_NO_EXTI1				7u
#define IRQ_NO_EXTI2				8u
#define IRQ_NO_EXTI3				9u
#define IRQ_NO_EXTI4				10u
#define IRQ_NO_EXTI9_5				23u
#define IRQ_NO_EXTI15_10			40u
#define IRQ_NO_SPI1 				35u
#define IRQ_NO_SPI2					36u
#define IRQ_NO_SPI3					51u

/* The number of bits used control priority in the device */
#define IMPLEMENTED_PR_BITS			4u

/*
 * Generic values
 */
#define ENABLE 			1u
#define DISABLE 		0u
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET
#define FLAG_SET		SET
#define FLAG_CLEAR		RESET

#endif /* INC_STM32F103XX_H_ */
