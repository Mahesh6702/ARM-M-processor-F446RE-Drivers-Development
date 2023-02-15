/*
 * stm32f446xx_i2c_driver.c
 *
 *  Created on: Nov 14, 2021
 *      Author: eegam
 */


#include "stm32f446xx_i2c_driver.h"
#include <stdint.h>


uint16_t ahb_prescalar[8]= {2,4,8,16,64,128,256,512};
uint8_t aPb_prescalar[4] = {2,4,8,16};

static void I2C_GenerateStartCondition(I2C_Regdef_t  *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_Regdef_t *pI2C, uint8_t SlaveAddress);
static void  I2C_ClearADDRFlag(I2C_Regdef_t *PI2Cx);
static void I2C_GenerateStopCondition(I2C_Regdef_t  *pI2Cx);

//This function generates the start condition for  I2C
static void I2C_GenerateStartCondition(I2C_Regdef_t  *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START); // Start bit is enabled in the CR1 register

}

/**********************************************************************
 * @fn               - I2C_ExecuteAddressPhase
 *
 * @brief            - This function excutes the slave address connected to   I2C
 *
 * @param[in]        - Base address of the given I2C
 * @param[in]        - Slave address
 * @param[in]        -
 *
 * @return           - None
 *
 * @Note             * None

 */
static void I2C_ExecuteAddressPhase(I2C_Regdef_t *pI2Cx, uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress  << 1;
	SlaveAddress &= ~(1); // Slave addres is slave address + r/w operation
	pI2Cx->DR = SlaveAddress;

}

//This function generates the start condition for  I2C
void static I2C_ClearADDRFlag(I2C_Regdef_t *pI2Cx)
{

	uint32_t dummyRead = pI2Cx->SR1;
	//uint32_t dummyRead = pI2Cx->SR2;
	(void)dummyRead;

}

//This function generates the stops condition for  I2C

static void I2C_GenerateStopCondition(I2C_Regdef_t  *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP); // Stop bit is enabled in the CR1 register

}

//This function enables or disables peripheral for the given I2C
void I2C_PeripheralControl(I2C_Regdef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE );
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE );
	}
}



// Peripheral Clock setup
void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx, uint8_t EnorDi)
{
	  if(EnorDi == ENABLE)
	  {
		  if(pI2Cx == I2C1)
		  {
			  I2C1_PCLK_EN();
		  }else if(pI2Cx == I2C2)
		  {
			  I2C2_PCLK_EN();
		  }else if(pI2Cx == I2C3)
		  {
			  I2C3_PCLK_EN();
		  }else if(pI2Cx == I2C4)
		  {
			  I2C4_PCLK_EN();
		  }
	  }else
	  {
		  if(pI2Cx == I2C1)
		  {
			  I2C1_PCLK_DI();
		  }else if(pI2Cx == I2C1)
		  {
			  I2C2_PCLK_DI();
		  }else if(pI2Cx == I2C1)
		  {
			  I2C3_PCLK_DI();
		  }else if(pI2Cx == I2C1)
		  {
			  I2C4_PCLK_DI();
		  }
	  }

}

// Init and De-init


/*void RCC_GetPLLOutputClock()
{

} */


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp_i2c1, temp_i2c2, ahb_p ,apb_p;

	clksrc = ((RCC->CFGR >>2)  & 0x3); // 2nd and 3rd bit positon of CFGR are brought at lsb and store in clksrc variable, to get system clock

	if(clksrc == 0)
	{
		SystemClk = 160000000; // HS1 clock selection
	}else if(clksrc == 1)
	{
		SystemClk = 8000000; // HSE clock selection
	} /*else if (clksrc ==2)
	{
		SystemClk = RCC_GetPLLOutputClock(); // PLL clock selection
	}*/

	// ahb clock configuration
	temp_i2c1 = ((RCC->CFGR >> 4) & 0xF);  // lift shifted by four bit positions and masked with 1, to get HPRE bits in LSB

	if(temp_i2c1 < 8)
	{
		ahb_p = 1;
	}
	else
	{
		ahb_p = ahb_prescalar[temp_i2c1 - 8];
	}


	// apb clock configuration
	temp_i2c2 = ((RCC->CFGR >> 10) & 0xF);  // lift shifted by 10 bit positions and masked with 1, to get PPRE1 bits in LSB

	if(temp_i2c2 <4)
	{
		apb_p = 1;
	}
	else
	{
		apb_p = aPb_prescalar[temp_i2c2 - 4];
	}
	pclk1 = ((SystemClk / ahb_p) / apb_p);

	return pclk1;
}

void I2C_Init(I2C_Handle_t  *pI2CHandle)
{
	uint32_t temp_i2c = 0;

	// initialising the ACK
	temp_i2c |= pI2CHandle-> I2CConfig.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR2 = temp_i2c;

	// configure the FREQ field of cr2(clock configuration)
	temp_i2c = 0;
	temp_i2c |= RCC_GetPCLK1Value() / 1000000U ;
	pI2CHandle->pI2Cx->CR2 = (temp_i2c & 0x3F) ; // 6 LSB Bits are used for FREQ in CR2 register

	//program the device own address
	temp_i2c |= pI2CHandle->I2CConfig.I2C_DeviceAddress << 1; // enabling the device address bit in OAR1 register
	temp_i2c |=(1 << 14);
	pI2CHandle->pI2Cx->OAR1 = temp_i2c;


    //CCR calculations
	uint16_t ccr_value = 0;
	temp_i2c =0;
	if (pI2CHandle->I2CConfig.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard mode

		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2CConfig.I2C_SCLSpeed)); // calculation of internal frequency
		temp_i2c |= (ccr_value & 0xFFF);  // only 12 bits are used for ccr bit
	}
	else
	{
		// mode is fast
		temp_i2c |= (1 << 15); // setting 15 bit position in CCR register for fast mode
		temp_i2c |= (pI2CHandle->I2CConfig.I2C_FMDutyCycle <<  14); // FMdutycycle is user selection, two macros are defined for this

		// calculation of ccr value
		if(pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2CConfig.I2C_SCLSpeed)); //I2C_FM_DUTY_2 mode
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2CConfig.I2C_SCLSpeed)); //I2C_FM_DUTY_16_9
		}
		temp_i2c |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = temp_i2c ; // copy back to ccr register

}


void I2C_DeInit(I2C_Regdef_t  *pI2Cx)
{
	  if(pI2Cx == I2C1)
	  {
		  I2C1_REG_RESET();
	  }else if(pI2Cx == I2C2)
	  {
		  I2C2_REG_RESET();
	  }else if(pI2Cx == I2C3)
	  {
		  I2C3_REG_RESET();
	  }else if(pI2Cx == I2C4)
	  {
		  I2C4_REG_RESET() ;
	  }

}

// Flag status of I2C
uint8_t I2C_GetFlagStatus(I2C_Regdef_t *pI2Cx, uint32_t FlagName)
 {
   if(pI2Cx->SR1 & FlagName)
   {
	   return FLAG_SET ;
   }

	return FLAG_RESET;
 }

// This function sends the data, as master mode

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint32_t SlaveAddress )
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle ->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note : Until SB is cleared SCL will be stretched (pulled to LOW)
	while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) ); // wait SB flag is set

	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	 I2C_ExecuteAddressPhase(pI2CHandle ->pI2Cx, SlaveAddress);

	 // 4. Confirm that address phase is completed by checking the ADDR flag is
	 while( ! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_ADDR) ); // wait ADDR flag is set

	 //5. Clear the ADDR flag according to its software sequence
	 // Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	 I2C_ClearADDRFlag(pI2CHandle ->pI2Cx);

	 //6. Send the data until Len becomes 0
	 while(Len > 0)
	 {
		 while( ! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_TxE) ); // wait TxE flag is set
		 pI2CHandle->pI2Cx->DR = *pTxbuffer;
		 *pTxbuffer;
		 Len--;
	 }
	 //7. when Len becomes 0 wait for TxE =1 and BTF =1 befor generating the STOP condition
	 //   Note: TxE =1, BTF =1, means that both Sr Dr are empty and nextr treansmission should begin
	 //   when BTf = 1 SCl will be stretched (pulled to Low)
	 while( ! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_TxE) ); // wait TxE flag is set


	 while( ! I2C_GetFlagStatus(pI2CHandle-> pI2Cx, I2C_FLAG_BTF) ); // wait BTF flag is set


	 ///8. Generate STOp condition and master need not to wait for the completion of stop condition.
	 // Note: generating StOP, automatically clears the BTF
	   I2C_GenerateStopCondition(pI2CHandle-> pI2Cx);
}



