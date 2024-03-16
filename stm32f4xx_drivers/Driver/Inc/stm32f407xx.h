/*
 * stm32f407xx.h
 *
 *  Created on: Feb 2, 2024
 *      Author: non55
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0 			( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2  		( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3			( (__vo uint32_t*)0XE000E18C )


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define NVIC_PR_BASE_ADDR 	( (__vo uint32_t*)0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */

#define NO_PR_BITS_IMPLEMENTED  4

/*
 * Base addresses of Flash and SRAM memories
 * FROM 2.Memory and bus architecture
 */

#define FLASHE_BASEADDR 						0x08000000U		//Main memory
#define SRAM1_BASEADDR							0x20000000U		//112KB
#define SRAM2_BASESDDR							0x2001C000U		//16KB
#define ROM_BASEADDR							0x1FFF0000U		//System memory
#define SRAM									SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define BUS_PERIPH_BASEADDR						0x40000000U
#define APB1_BUS_BASEADDR						BUS_PERIPH_BASEADDR
#define APB2_BUS_BASEADDR						0x40010000U
#define AHB1_BUS_BASEADDR						0x40020000U
#define AHB2_BUS_BASEADDR						0x50000000U


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define TIM2_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x0000)
#define TIM3_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x0400)
#define TIM4_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x0800)
#define TIM5_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x0C00)
#define TIM6_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x1000)
#define TIM7_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x1400)
#define TIM12_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x1800)
#define TIM13_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x1C00)
#define TIM14_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x4000)

#define RTC_N_BKP_REG_PERIPH_BASEADDR			(APB1_BUS_BASEADDR + 0x2800)
#define WWDG_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x2C00)
#define IWDG_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x3000)
#define I2S2EXT_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x3400)
#define I2S3EXT_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x4000)

#define SPI2_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x3800)
#define SPI3_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x3C00)

#define USART2_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x4400)
#define USART3_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x4800)

#define UART4_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x4C00)
/*#define UART5_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x5000)
#define UART7_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x7800)
#define UART8_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x7C00)
*/
#define I2C1_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x5400)
#define I2C2_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x5800)
#define I2C3_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x5C00)

#define CAN1_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x6400)
#define CAN2_PERIPH_BASEADDR					(APB1_BUS_BASEADDR + 0x6800)
#define PWR_PERIPH_BASEADDR						(APB1_BUS_BASEADDR + 0x7000)
#define DAC_PERIPH_BASEADDR						(APB1_BUS_BASEADDR + 0x7400)

/*
* Base address of peripherals which are hanging on APB2 bus
*/

#define TIM1_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x0000)
#define TIM8_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x0400)
#define TIM9_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x4000)
#define TIM10_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x4400)
#define TIM11_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x4800)

#define USART1_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x1000)
//#define USART6_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x1400)

#define ADC1_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x2000)
#define ADC2_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x2100)
#define ADC3_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x2200)

#define SDIO_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x2C00)

#define SPI1_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x3000)
/*#define SPI4_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x3400)
#define SPI5_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x5000)
#define SPI6_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x5400)
*/
#define SYSCFG_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x3800)
#define EXTI_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x3C00)

#define SAI1_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x5800)
#define LCD_TFT_PERIPH_BASEADDR					(APB2_BUS_BASEADDR + 0x6800)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define	GPIOA_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x0000)
#define	GPIOB_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x0400)
#define GPIOC_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x0800)
#define GPIOD_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x0C00)
#define GPIOE_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x1000)
#define GPIOF_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x1400)
#define GPIOG_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x1800)
#define GPIOH_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x1C00)
#define GPIOI_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x2000)
#define GPIOK_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x2400)
#define GPIOJ_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x2800)

#define CRC_PERIPH_BASEADDR						(AHB1_BUS_BASEADDR + 0x3000)
#define RCC_PERIPH_BASEADDR						(AHB1_BUS_BASEADDR + 0x3800)
#define FIR_PERIPH_BASEADDR						(AHB1_BUS_BASEADDR + 0x3C00)
#define BKPSRAM_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x4000)
#define DMA1_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x6000)
#define DMA2_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0x6400)
#define ETHERNET_MAC_PERIPH_BASEADDR			(AHB1_BUS_BASEADDR + 0x8000)
#define DMA2D_PERIPH_BASEADDR					(AHB1_BUS_BASEADDR + 0xB000)
#define USB_OTG_HS_PERIPH_BASEADDR				(AHB1_BUS_BASEADDR + 0x00020000)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define USB_OTG_FS_PERIPH_BASEADDR				(AHB2_BUS_BASEADDR + 0x0000)
#define DCMI_PERIPH_BASEADDR					(AHB2_BUS_BASEADDR + 0x00050000)
#define CRYP_PERIPH_BASEADDR					(AHB2_BUS_BASEADDR + 0x00060000)
#define HASH_PERIPH_BASEADDR					(AHB2_BUS_BASEADDR + 0x00060400)
#define RNG_PERIPH_BASEADDR						(AHB2_BUS_BASEADDR + 0x00060800)

/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;			/*!< GPIO port mode register, Address offset: 0x00
	 	 	 	 	 	 	 	 	 	 Bits 2y:2y+1 MODERy[1:0]: Port x configuration bits (y = 0..15)
	 	 	 	 	 	 	 	 	 	 These bits are written by software to configure the I/O direction mode.
										 00: Input (reset state)
										 01: General purpose output mode
										 10: Alternate function mode
										 11: Analog mode			*/

	__vo uint32_t OTYPER;			/*!< Address offset: 0x04
	 	 	 	 	 	 	 	 	 	 Bits 31:16 Reserved, must be kept at reset value.
										 Bits 15:0 OTy: Port x configuration bits (y = 0..15)
										 These bits are written by software to configure the output type of the I/O port.
										 0: Output push-pull (reset state)
										 1: Output open-drain			*/

	__vo uint32_t OSPEEDR;			/*|< Address offset: 0x08
	 	 	 	 	 	 	 	 	 	 Bits 2y:2y+1 OSPEEDRy[1:0]: Port x configuration bits (y = 0..15)
										 These bits are written by software to configure the I/O output speed.
										 00: Low speed
							 	 	  	 01: Medium speed
										 10: High speed
									     11: Very high speed
										 Note: Refer to the product datasheets for the values of OSPEEDRy bits versus VDD
									 	 range and external load.			*/

	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;


/*
 * peripheral register definition structure for RCC
 */

typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */

typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */

typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */

typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA  				( (GPIO_RegDef_t*) GPIOA_PERIPH_BASEADDR )
#define GPIOB  				( (GPIO_RegDef_t*) GPIOB_PERIPH_BASEADDR )
#define GPIOC  				( (GPIO_RegDef_t*) GPIOC_PERIPH_BASEADDR )
#define GPIOD  				( (GPIO_RegDef_t*) GPIOD_PERIPH_BASEADDR )
#define GPIOE  				( (GPIO_RegDef_t*) GPIOE_PERIPH_BASEADDR )
#define GPIOF  				( (GPIO_RegDef_t*) GPIOF_PERIPH_BASEADDR )
#define GPIOG  				( (GPIO_RegDef_t*) GPIOG_PERIPH_BASEADDR )
#define GPIOH  				( (GPIO_RegDef_t*) GPIOH_PERIPH_BASEADDR )
#define GPIOI  				( (GPIO_RegDef_t*) GPIOI_PERIPH_BASEADDR )

#define RCC 				( (RCC_RegDef_t*) RCC_PERIPH_BASEADDR )
#define EXTI				( (EXTI_RegDef_t*) EXTI_PERIPH_BASEADDR )
#define SYSCFG				( (SYSCFG_RegDef_t*) SYSCFG_PERIPH_BASEADDR )


#define SPI1  				((SPI_RegDef_t*)SPI1_PERIPH_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_PERIPH_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_PERIPH_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_PERIPH_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_PERIPH_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_PERIPH_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_PERIPH_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_PERIPH_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_PERIPH_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_PERIPH_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_PERIPH_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_PERIPH_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() (RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC -> APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripheralsbu
 */

#define SPI1_PCLK_EN() (RCC -> APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC -> APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCCK_EN() (RCC -> APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC -> APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC -> APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC -> APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC -> APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC -> APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() (RCC -> APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()

////////////////////////////////////////////////////////////////////

/*
 *  Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()	do{ (RCC -> AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
								        (x == GPIOE) ? 4 : \
								        (x == GPIOF) ? 5 : \
								        (x == GPIOG) ? 6 : \
								        (x == GPIOH) ? 7 : \
								        (x == GPIOI) ? 8 : 0)

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to MCU
 * TODO: Complete this list for other peripherals
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRI0      0
#define NVIC_IRQ_PRI1      1
#define NVIC_IRQ_PRI2      2
#define NVIC_IRQ_PRI3      3
#define NVIC_IRQ_PRI4      4
#define NVIC_IRQ_PRI5      5
#define NVIC_IRQ_PRI6      6
#define NVIC_IRQ_PRI7      7
#define NVIC_IRQ_PRI8      8
#define NVIC_IRQ_PRI9      9
#define NVIC_IRQ_PRI10     10
#define NVIC_IRQ_PRI11     11
#define NVIC_IRQ_PRI12     12
#define NVIC_IRQ_PRI13     13
#define NVIC_IRQ_PRI14     14
#define NVIC_IRQ_PRI15     15

//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */

#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */

#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */

#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/*
 *  Macros to reset GPIOx peripherals
 */

#define SPI1_REG_RESET()	do{ (RCC -> APB2RSTR |= (1 << 12)); (RCC->AHB1RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC -> APB1RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC -> APB1RSTR |= (1 << 15)); (RCC->AHB1RSTR &= ~(1 << 15)); }while(0)

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

#endif /* INC_STM32F407XX_H_ */
