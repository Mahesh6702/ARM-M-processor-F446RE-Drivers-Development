/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: 5 Nov 2021
 *      Author: eegam
 */


#include "stm32f446xx_spi_driver.h"

#include <stdint.h>


/* ================================Creation of API's prototype =======================================
 *                                 @SPIO_PeriClockControl
 *                                 @SPI_Init
 *                                 @SPI_DeInit
 *                                 @SPI_SendData
 *                                 @SPI_ReceiveData
 *                                 @SPI_IRQInterruptConfig
 *                                 @SPI_IRQPriorityConfig
 *                                 @SPI_IRQHandling
 *
 * */


// Peripheral Clock setup
/**********************************************************************
 * @fn               - SPIO_PeriClockControl
 *
 * @brief            - This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]        - Base address of the given SPI
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */

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


/*****************************     Init and De-init  *****************************************
 *
 * @fn               - SPI_Init
 *
 * @brief            - This function initialise all the required configuration for  the given SPI port
 *
 * @param[in]        - Base address of the given SPI port
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */

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



/**********************************************************************
 * @fn               - SPI_DeInit
 *
 * @brief            - This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]        - Base address of the given SPI
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 * @Note             * None

 */
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


/**********************************************************************
 * @fn               - SPI_SendDatal
 *
 * @brief            - This function sends the data from MCU(Master) to any sepicified slave using spi protocol
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Transmitter buffer register
 * @param[in]        - Length of the buffer register
 *
 * @return           - None
 *
 * @Note             * Blocking calls (while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);) CAN BE BLOCKED PERMANNETLY

 */

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



/**********************************************************************
 * @fn               - SPI_ReceiveData
 *
 * @brief            - This function receives the data from slave to MCU using spi protocol
 *
 * @param[in]        - Receiver buffer
 * @param[in]        - Length of the receiver buffer
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */
void SPI_ReceiveData(SPI_Regdef_t  *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{


}

// IRQ Configuration and ISR handling

/**********************************************************************
 * @fn               - SPI_IRQInterruptConfig
 *
 * @brief            - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{


}

// Peripheral Clock setup
/**********************************************************************
 * @fn               - SPI_IRQPriorityConfig
 *
 * @brief            - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{


}


// Peripheral Clock setup
/**********************************************************************
 * @fn               - SPI_IRQHandling
 *
 * @brief            - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{


}


/**********************************************************************
 * @fn               - SPI_PeripheralControl
 *
 * @brief            - This function enables the clock for the spi peripheral
 *
 * @param[in]        - Base address of the given SPIx port
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */


void SPI_PeripheralControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/**********************************************************************
 * @fn               - SPI_SSIConfigure
 *
 * @brief            - This function sets SSI bit
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Enable or Disable
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * Clears the SSI, in order to overcome the MODEF error(By NSS bit is internally connected +vcc)

 */


void SPI_SSIConfigure(SPI_Regdef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}

}


