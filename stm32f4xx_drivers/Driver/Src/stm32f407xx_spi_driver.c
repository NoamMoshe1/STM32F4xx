/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Mar 2, 2024
 *      Author: non55
 */

#include "stm32f407xx_spi_driver.h"

/*
 * Peripheral Clock setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
		else
		{
			//TODO
		}
}

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable

	SPI_PeriClockControl(pSPIHandle -> pSPIx, ENABLE);

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle -> SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2. Configure the bus config
	if(pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}
	else if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle -> SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle -> SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle -> SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle -> SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle -> SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle -> SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle -> pSPIx -> CR1 = tempreg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();		//TODO WRITE #DEFINE
		}
	else if (pSPIx == SPI2)			//TODO WRITE #DEFINE
		{
			SPI2_REG_RESET();
		}
	else if (pSPIx == SPI3)			//TODO WRITE #DEFINE
		{
			SPI3_REG_RESET();
		}
}
////////////////////////////////////////////
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx -> SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/////////////////////////////////////////////

/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until TXE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

			//2. check the DFF bit in CR1
		 	if( (pSPIx -> CR1 & ( 1 << SPI_CR1_DFF ) ) )
			{
				//16 bit DFF
				//1. load the data in to the DR
				pSPIx -> DR = *( (uint16_t*)pTxBuffer );
				Len--;
				Len--;
				(uint16_t*)pTxBuffer++;
			}
		 	else
			{
				//8 bit DFF
				pSPIx -> DR = *pTxBuffer;
				Len--;
				pTxBuffer++;
			}
		}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
		 	if( (pSPIx -> CR1 & ( 1 << SPI_CR1_DFF ) ) )
			{
				//16 bit DFF
		 		//1. load the data from DR to Rxbuffer address
		 		 *((uint16_t*)pRxBuffer) = pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}
		 	else
			{
				//8 bit DFF
		 		*pRxBuffer = pSPIx -> DR;
				Len--;
				pRxBuffer++;
			}
		}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		//Set - enable
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		//Clear - disable
		pSPIx -> CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			//Set - enable
			pSPIx -> CR1 |=  (1 << SPI_CR1_SSI);
		}
	else
		{
			//Clear - disable
			pSPIx -> CR1 &=  ~(1 << SPI_CR1_SSI);
		}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			//Set - enable
			pSPIx -> CR2 |=  (1 << SPI_CR2_SSOE);
		}
	else
		{
			//Clear - disable
			pSPIx -> CR2 &=  ~(1 << SPI_CR2_SSOE);
		}
}
