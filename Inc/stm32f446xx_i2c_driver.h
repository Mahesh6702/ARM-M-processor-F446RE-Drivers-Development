/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: Nov 14, 2021
 *      Author: eegam
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_
#include "stm32f446xx.h"
#include<stdint.h>


// Configuration structure of SPIx peripheral
typedef struct
{
	uint8_t I2C_SCLSpeed;       // I2C speed, normal mode or fast mode(1MHz / 4 MHz)
	uint8_t I2C_DeviceAddress;  // User selection address
	uint8_t I2C_ACKControl;     // acknowledge control
	uint8_t I2C_FMDutyCycle;    // Clock  control settings
}I2C_Config_t;


// Handle structure for I2C
typedef struct
{
	I2C_Regdef_t   *pI2Cx;            // This holds the base address of the I2Cx to which the pin belongs
	I2C_Config_t    I2CConfig;        // This holds I2C pin Configuration Settings
}I2C_Handle_t;

// setting I2C speed mode
#define I2C_SCL_SPEED_SM      100000
#define I2C_SCL_SPEED_FM      400000

// Setting I2C ACKcontrol
#define I2C_ACK_ENABLE         1
#define I2C_ACK_DISABLE        0

//  Setting FMDuty cycle
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1

// I2C Flag macros

#define I2C_FLAG_SB             0
#define I2C_FLAG_ADDR           1
#define I2C_FLAG_BTF            2
#define I2C_FLAG_STOPF          4
#define I2C_FLAG_RxNE           6
#define I2C_FLAG_TxE            7
#define I2C_FLAG_BERR           8
#define I2C_FLAG_ARLO           9
#define I2C_FLAG_AF             10
#define I2C_FLAG_OVR            11
#define I2C_FLAG_PECERR         12
#define I2C_FLAG_TIMEOUT        14
#define I2C_FLAG_SMBALERT       15

// API's prototypes
// Peripheral Enable or Disable
void I2C_PeripheralControl(I2C_Regdef_t *pI2Cx, uint8_t EnorDi);

// Peripheral Clock setup
void I2C_PeriClockControl(I2C_Regdef_t *pI2Cx, uint8_t EnorDi);

// Init and De-init

void I2C_Init(I2C_Handle_t  *pI2CHandle);
void I2C_DeInit(I2C_Regdef_t  *pI2Cx);

// Flag status of I2C
 uint8_t I2C_GetFlagStatus(I2C_Regdef_t *pI2Cx, uint32_t FlagName);

 // Master send data
 void I2C_MasterSendData(I2C_Handle_t *pI2cHandle, uint8_t *pTxbuffer, uint32_t Len, uint32_t SlaveAddress );


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
