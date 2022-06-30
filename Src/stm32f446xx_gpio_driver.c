/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 1 Nov 2021
 *      Author: eegam
 */


#include "stm32f446xx_gpio_driver.h"

#include <stdint.h>

/* ================================Creation of API's defintions =======================================
 *                                 @GPIO_PeriClockControl
 *                                 @GPIO_Init
 *                                 @GPIo_DeInit
 *                                 @GPIO_ReadFromInputPi
 *                                 @GPIO_ReadFromInputPort
 *                                 @GPIO_WriteToOutputPin
 *                                 @GPIO_WriteToOutputPort
 *                                 @GPIO_ToggleOutputPi
 *                                 @GPIO_IRQInterruptConfig
 *                                 @GPIO_IRQPriorityConfig
 *                                 @GPIO_IRQHandling
 *
 * */




// Peripheral Clock setup
/**********************************************************************
 * @fn               - GPIO_PeriClockControl
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

void GPIO_PeriClockControl(GPIOx_Regdef_t *pGPIOx, uint8_t EnorDi)
{
  if(EnorDi == ENABLE)
  {
	  if(pGPIOx == GPIOA)
	  {
		  GPIOA_PCLK_EN();
	  }else if(pGPIOx == GPIOB)
	  {
	    GPIOB_PCLK_EN();
	  }else if(pGPIOx == GPIOC)
	  {
	    GPIOC_PCLK_EN();
	  }else if(pGPIOx == GPIOD)
	  {
	    GPIOD_PCLK_EN();
	  }else if(pGPIOx == GPIOE)
	  {
	    GPIOE_PCLK_EN();
	  }else if(pGPIOx == GPIOF)
	  {
	    GPIOF_PCLK_EN();
	  }else if(pGPIOx == GPIOG)
	  {
	    GPIOG_PCLK_EN();
	  }else if(pGPIOx == GPIOH)
	  {
	    GPIOH_PCLK_EN();
	  }
  }else
  {
	  if(pGPIOx == GPIOA)
	  {
	    GPIOA_PCLK_DI();
	  }else if(pGPIOx == GPIOB)
	  {
	    GPIOB_PCLK_DI();
	  }else if(pGPIOx == GPIOC)
	  {
	    GPIOC_PCLK_DI();
	  }else if(pGPIOx == GPIOD)
	  {
	    GPIOD_PCLK_DI();
	  }else if(pGPIOx == GPIOE)
	  {
	    GPIOE_PCLK_DI();
	  }else if(pGPIOx == GPIOF)
	  {
	    GPIOF_PCLK_DI();
	  }else if(pGPIOx == GPIOG)
	  {
	    GPIOG_PCLK_DI();
	  }else if(pGPIOx == GPIOH)
	  {
	    GPIOH_PCLK_DI();
	  }
  }
}

// Init and De-init

/**********************************************************************
 * @fn               - GPIO_Init
 *
 * @brief            - initialise the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        -
 * @param[in]        -
 *
 * @return           - none
 *
 * @Note             - none
 *

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	  // Initialising the peripheral clock
	    GPIO_PeriClockControl(pGPIOHandle ->pGPIOx,ENABLE);

	  uint32_t temp;
   //1. configure the mode of the gpio pin
	   if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)                                     /*  This information is at -->  */
	   {
		   // then it is non interrupt mode

		   temp =(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber)); // setting
		   pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);                 // clearing
	       pGPIOHandle->pGPIOx->MODER |= temp;
	   }
	   else
	   {
            // interrupt mode
		   if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		   {
			   //1. configure the FTSR
			   EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			   EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber); // Clearing the corresponding RTSR bit
		   }
		   else if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RT )
		   {
			   //1. configure the RTSR
			   EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			   EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber); // Clearing the corresponding FTSR bit
		   }
		   if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		   {
			   //1. configure the FTSR and RTSR
			   EXTI->RTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
			   EXTI->FTSR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);
		   }
		   // 2. configure the GPIO port selection in SYSCFG_EXTICR
               uint8_t temp1 = (pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber / 4);
               uint8_t temp2 = (pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber % 4);
               uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
               SYSCFG_PCLK_EN();
               SYSCFG->EXTICR[temp1] = portcode  << (temp2 *4);
		   // 3. Enable the exti interrupt delivery using IMR
		   EXTI->IMR |= (1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);

	   }
	   temp = 0;
  //2. configure the speed
	   /*  This information is at --> @GPIO_PinSpeed  */
	   temp =(pGPIOHandle->GPIO_pinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber)); // setting
	   pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);               // clearing
       pGPIOHandle->pGPIOx->OSPEEDER |= temp;

       temp = 0;
  //3. configure the pupd settings
       /*  This information is at --> @GPIO_PinPuPdControl  */
	   temp =(pGPIOHandle->GPIO_pinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber)); // setting
	   pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);                         // clearing
       pGPIOHandle->pGPIOx->PUPDR |= temp;

       temp =0;
  //4. configure the optype
       /*  This information is at --> @GPIO_PinOPtype  */
	   temp =(pGPIOHandle->GPIO_pinConfig.GPIO_PinOPtype <<( pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber));         // setting
	   pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber);                        // clearing
       pGPIOHandle->pGPIOx->OTYPER |= temp;

   //5. configure the alt functionality
       if(pGPIOHandle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
       {
    	   uint8_t temp1, temp2 ;

		   temp1 = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber / 8;
    	   temp2 = pGPIOHandle->GPIO_pinConfig.GPIO_PinNumber % 8;
    	   pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xFF << (4 * temp2));                                           // clearing
    	   pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_pinConfig.GPIO_PinAltFunMode << (4 * temp2));  // setting
       }
}


/**********************************************************************
 * @fn               - GPIO_DeInit
 *
 * @brief            - de-initialise the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        -
 * @param[in]        -
 *
 * @return           - none
 *
 * @Note             - none
 *

 */
void GPIo_DeInit(GPIOx_Regdef_t *pGPIOx)
{

	  if(pGPIOx == GPIOA)
	  {
	    GPIOA_REG_RESET();
	  }else if(pGPIOx == GPIOB)
	  {
	    GPIOB_REG_RESET();
	  }else if(pGPIOx == GPIOC)
	  {
	    GPIOC_REG_RESET();
	  }else if(pGPIOx == GPIOD)
	  {
	    GPIOD_REG_RESET();
	  }else if(pGPIOx == GPIOE)
	  {
	    GPIOE_REG_RESET();
	  }else if(pGPIOx == GPIOF)
	  {
	    GPIOF_REG_RESET();
	  }else if(pGPIOx == GPIOG)
	  {
	    GPIOG_REG_RESET();
	  }else if(pGPIOx == GPIOH)
	  {
	    GPIOH_REG_RESET();
	  }


}


// Data read and write
/**********************************************************************
 * @fn               - GPIO_ReadFromInputPin
 *
 * @brief            - This function reads the data from the given GPIO port pin
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - PinNumber of the the given GPIO port
 * @param[in]        -
 *
 * @return           - return binary data
 *
 * @Note             - none

 */

uint16_t GPIO_ReadFromInputPin(GPIOx_Regdef_t *pGPIOx, uint16_t PinNumber)
{
	uint16_t value;
	value = (uint16_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**********************************************************************
 * @fn               - GPIO_ReadFromInputPort
 *
 * @brief            - Reads the data for the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        -
 * @param[in]        -
 *
 * @return           - return hexadecimal value
 *
 * @Note             - none

 */

uint16_t GPIO_ReadFromInputPort(GPIOx_Regdef_t *pGPIOx )
{
	uint16_t value1;
	value1 = (uint16_t)(pGPIOx->IDR );
	return value1;

}


/**********************************************************************
 * @fn               - GPIO_WriteToOutputPin
 *
 * @brief            - This function helps to write the data for the given GPIO port pin
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Given GPIO port pin Number
 * @param[in]        - binary value
 *
 * @return           - none
 *
 * @Note             - none

 */
void GPIO_WriteToOutputPin(GPIOx_Regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	if(Value == GPIO_PIN_SET )
	{
		pGPIOx->ODR |= (1 << PinNumber); // write 1 to the output data register at the corresponding bit to the pin number
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber); // write 0 to the output data register at the corresponding bit to the pin number
	}

}

/**********************************************************************
 * @fn               - GPIO_WriteToOutputPort
 *
 * @brief            - This function helps to write the data into the given GPIO port
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - Hexadecimal value into the given GPIO port
 * @param[in]        -
 *
 * @return           - none
 *
 * @Note             - none

 */
void GPIO_WriteToOutputPort(GPIOx_Regdef_t *pGPIOx,  uint16_t Value)
{

	 pGPIOx->ODR = Value; // writing  16bit value to  the GPIO port register
}


/**********************************************************************
 * @fn               - GPIO_ToggleOutputPin
 *
 * @brief            - This function helps to toggle the given GPIO port pin
 *
 * @param[in]        - Base address of the given GPIO port
 * @param[in]        - binary value into the given GPIO port pin
 * @param[in]        -
 *
 * @return           - none
 *
 * @Note             - none

 */
void GPIO_ToggleOutputPin(GPIOx_Regdef_t *pGPIOx, uint8_t PinNumber)
{
	 pGPIOx->ODR ^= (1 << PinNumber); // toggling the corresponding pin number of the GPIO port
}



// IRQ Configuration and ISR handling
/**********************************************************************
 * @fn               - GPIO_IRQConfig
 *
 * @brief            - This function configures the Interrupt
 *
 * @param[in]        -
 * @param[in]        - priority of the interrupt
 * @param[in]        - enable or disable
 *
 * @return           - none
 *
 * @Note             - none

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
  // processor specific interrupt

	if(EnorDi == ENABLE )
	{
		if(IRQNumber<= 31)
		{
			// program ISER0 register
			*NVIC_ISER0 |=(1 << IRQNumber);

		}else if(IRQNumber>= 32 && IRQNumber <=64)
		{
			// program ISER0 register
			*NVIC_ISER1 |=(1 << (IRQNumber % 32));

		}else if(IRQNumber>= 65 && IRQNumber <=96)
		{
			// program ISER0 register
			*NVIC_ISER3 |=(1 << (IRQNumber % 64));

		}
	}

	else
	{
		if(IRQNumber<= 31)
		{
			// program ISER0 register
			*NVIC_ICER0 |=(1 << IRQNumber);

		}else if(IRQNumber>= 32 && IRQNumber <=64)
		{
			// program ISER0 register
			*NVIC_ICER1 |=(1 << (IRQNumber % 32));

		}else if(IRQNumber>= 65 && IRQNumber <=96)
		{
			// program ISER0 register
			*NVIC_ICER3 |=(1 << (IRQNumber % 64));

		}

	}
}

/**********************************************************************
 * @fn               - GPIO_IRQHandling
 *
 * @brief            - This function handles the priority of the interrupt from the pin
 *
 * @param[in]        - pin number
 * @param[in]        -
 * @param[in]        -
 *
 * @return           - none
 *
 * @Note             - none

 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
     //1. First lets find out the ipr register

	   uint8_t iprx  = IRQNumber / 4 ; // finds the which suitable register
	   uint8_t iprx_section = IRQNumber % 4; // finds the which suitable section should be implemented
	   uint8_t shift_amount  = (8 * iprx_section) + (8 - NO_OR_BITS_IMPLEMENTED); // amount value should be shifted to set the priority

	   *(NVIC_PR_BASEADDR +(iprx * 4)) |= (IRQPriority  << shift_amount);

}




/**********************************************************************
 * @fn               - GPIO_IRQHandling
 *
 * @brief            - This function handles the interrupt from the pin
 *
 * @param[in]        - pin number
 * @param[in]        -
 * @param[in]        -
 *
 * @return           - none
 *
 * @Note             - none

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

	// clear the exti pr register corresponding to the pin number
	if(EXTI ->PR &(1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}
