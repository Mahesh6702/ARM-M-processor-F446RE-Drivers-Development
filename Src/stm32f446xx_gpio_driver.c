/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 1 Nov 2021
 *      Author: eegam
 */


#include "stm32f446xx_gpio_driver.h"

#include <stdint.h>

//This function enables or disables peripheral clock for the given GPIO port

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

//initialise the given GPIO port
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

//de-initialise the given GPIO port
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

//This function reads the data from the given GPIO port pin

uint16_t GPIO_ReadFromInputPin(GPIOx_Regdef_t *pGPIOx, uint16_t PinNumber)
{
	uint16_t value;
	value = (uint16_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

//Reads the data for the given GPIO port
uint16_t GPIO_ReadFromInputPort(GPIOx_Regdef_t *pGPIOx )
{
	uint16_t value1;
	value1 = (uint16_t)(pGPIOx->IDR );
	return value1;

}

// This function helps to write the data for the given GPIO port pin
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

//function helps to write the data into the given GPIO port
void GPIO_WriteToOutputPort(GPIOx_Regdef_t *pGPIOx,  uint16_t Value)
{

	 pGPIOx->ODR = Value; // writing  16bit value to  the GPIO port register
}

//toggle the given GPIO port pin
void GPIO_ToggleOutputPin(GPIOx_Regdef_t *pGPIOx, uint8_t PinNumber)
{
	 pGPIOx->ODR ^= (1 << PinNumber); // toggling the corresponding pin number of the GPIO port
}

}
