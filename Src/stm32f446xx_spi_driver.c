/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 5 Nov 2021
 *      Author: eegam
 */


#include "stm32f446xx_spi_driver.h"

#include <stdint.h>

// Peripheral Clock setup


void SPI_PeriClockControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	  if(EnorDi == ENABLE)
	  {
		  if(pSPIx == SPI1)
		  {
			  SPI1_PCLK_EN();
		  }else if(pSPIx == SPI2)
		  {
			  SPI2_PCLK_EN();
		  }else if(pSPIx == SPI3)
		  {
			  SPI3_PCLK_EN();
		  }else if(pSPIx == SPI4)
		  {
			  SPI4_PCLK_EN();
		  }
	  }else
	  {
		  if(pSPIx == SPI1)
		  {
			  SPI1_PCLK_DI();
		  }else if(pSPIx == SPI2)
		  {
			  SPI2_PCLK_DI();
		  }else if(pSPIx == SPI3)
		  {
			  SPI3_PCLK_DI();
		  }else if(pSPIx == SPI4)
		  {
			  SPI4_PCLK_DI();
		  }
	  }

}

//This function initialise all the required configuration for  the given SPI port

void SPI_Init(SPI_Handle_t  *pSPIHandle)
{
    // Initialising the clock for the peripheral
	  SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;
// 1. Configure the device mode
	 tempreg |= pSPIHandle->SPIConfig.SPI_Devicemode <<  SPI_CR1_MSTR;


// 2.  Configure the BusConfig
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDIMODE must be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDIOE must be SET
		tempreg |=  (1 << SPI_CR1_BIDIMODE);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
	{
		// BIDIMODE must be cleared and RX only must be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}
//3. Configure the SclkSpeed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

// 4. Configure the DFF
	tempreg |=  pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

//5. Configure the CPHA
	tempreg |=  pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA ;

//5. Configure the CPOL
	tempreg |=  pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

//5. Configure the SSM
	tempreg |=  pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Writing back all the configured bits in CRC1 register
	pSPIHandle->pSPIx->CR1 = tempreg;



}

//This function enables or disables peripheral clock for the given SPI
void SPI_DeInit(SPI_Regdef_t  *pSPIx)
{

	  if(pSPIx == SPI1)
	  {
		  SPI1_REG_RESET();
	  }else if(pSPIx == SPI2)
	  {
		  SPI2_REG_RESET();
	  }else if(pSPIx == SPI3)
	  {
		  SPI3_REG_RESET();
	  }else if(pSPIx == SPI4)
	  {
		  SPI4_REG_RESET();
	  }


}

uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx, uint32_t FlagName)
{
	if( pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

// Data Send and Receive

void SPI_SendData(SPI_Regdef_t  *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{

   while(Len>0)
   {
	    	   //1. wait until TxBuffer is empty (TXE is set)
	     while(!(pSPIx->SR & (1<<1)));
	   //2. Check the DFF bit in CR1
            if((pSPIx ->CR1	&	(1<< SPI_CR1_DFF)))
            {
            	// 16 bit data loaded in to the DR register
            	pSPIx->DR = *(uint16_t*)pTxBuffer;
            	Len --;
            	Len --;
            	(uint16_t*)pTxBuffer++;

            }else
            {
            	// load the 8 bit data in to the DR register
            	*((volatile uint32_t *)(0x4000380C)) = 0x45;
            	pSPIx->DR = 0x4545;
            	Len --;
            	pTxBuffer++;

            }
            uint8_t dummy = pSPIx->DR;
            //while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)	==	FLAG_RESET);

   }
}

//This function enables the clock for the spi peripheral
void SPI_PeripheralControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}



void SPI_SSIConfigure(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}


