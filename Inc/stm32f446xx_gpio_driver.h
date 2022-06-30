/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 1 Nov 2021
 *      Author: eegam
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"



typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinOPtype;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct
{
	GPIOx_Regdef_t *pGPIOx;            // This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_pinConfig;  // This holds GPIO pin Configuration Settings
}GPIO_Handle_t;



// Setting the GPIOx pin number
// ******** @GPIO_PinNumber
#define GPIO_PinNumber_P0    0
#define GPIO_PinNumber_P1    1
#define GPIO_PinNumber_P2    2
#define GPIO_PinNumber_P3    3
#define GPIO_PinNumber_P4    4
#define GPIO_PinNumber_P5    5
#define GPIO_PinNumber_P6    6
#define GPIO_PinNumber_P7    7
#define GPIO_PinNumber_P8    8
#define GPIO_PinNumber_P9    9
#define GPIO_PinNumber_P10   10
#define GPIO_PinNumber_P11   11
#define GPIO_PinNumber_P12   12
#define GPIO_PinNumber_P13   13
#define GPIO_PinNumber_P14   14
#define GPIO_PinNumber_P15   15

// Setting the GPIOx mode
// ******** @GPIO_MODE
#define GPIO_MODE_IN         0
#define GPIO_MODE_OUT        1
#define GPIO_MODE_ALTFN      2
#define GPIO_MODE_ANALOG     3
#define GPIO_MODE_IT_FT      4    // Mode as falling edge
#define GPIO_MODE_IT_RT      5    // mode as rising edge
#define GPIO_MODE_IT_RFT     6    // Mode as rising and falling trigger


// Setting the GPIOx pin output type
// ******** @GPIO_PinOPtype
#define GPIO_OTYPE_PPL      0   // Output type is push pull
#define GPIO_OTYPE_OPD      1   // Output type is open drain

// Setting the GPIOx pin speed
// ******** @GPIO_PinSpeed
#define GPIO_OUT_LS        0
#define GPIO_OUT_MS        1
#define GPIO_OUT_HS        2
#define GPIO_OUT_FS        3

// Setting the GPIOx PULL UP OR PULL DOWN
// ******** @GPIO_PinPuPdControl
#define GPIO_PINPUPD_NO     0
#define GPIO_PINPU          1
#define GPIO_PINPD          2



/* ================================Creation of API's prototype =======================================
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
void GPIO_PeriClockControl(GPIOx_Regdef_t *pGPIOx, uint8_t EnorDi);

// Init and De-init

void GPIO_Init(GPIO_Handle_t *GPIOHandle);
void GPIo_DeInit(GPIOx_Regdef_t *pGPIOx);


// Data read and write
uint16_t GPIO_ReadFromInputPin(GPIOx_Regdef_t *pGPIOx, uint16_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIOx_Regdef_t *pGPIOx );
void GPIO_WriteToOutputPin(GPIOx_Regdef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIOx_Regdef_t *pGPIOx,  uint16_t Value);
void GPIO_ToggleOutputPin(GPIOx_Regdef_t *pGPIOx, uint8_t PinNumber);



// IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
