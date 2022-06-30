/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: 5 Nov 2021
 *      Author: eegam
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

 #include "stm32f446xx.h"

// Configuration structure of SPIx peripheral
typedef struct
{
	uint8_t SPI_Devicemode;  // SPI in device in Master mode or Slave mode
	uint8_t SPI_BusConfig;  // SPI in Full duplex, Half duplex or Simple duplex
	uint8_t SPI_SclkSpeed;  // selecting user speed
	uint8_t SPI_DFF;        // SPI's shift register in 8/ 16 bit register mode
	uint8_t SPI_CPHA;       // User application these bits are sets
	uint8_t SPI_CPOL;       // User application these bits are sets
	uint8_t SPI_SSM;        // Slave select management

}SPI_Config_t;


// Handle structure for SPI
typedef struct
{
	SPI_Regdef_t    *pSPIx;            // This holds the base address of the SPIx to which the pin belongs
	SPI_Config_t    SPIConfig;        // This holds GPIO pin Configuration Settings
}SPI_Handle_t;


/*
 *            @SPI_BusConfig Macros
 */

#define   SPI_DEVICE_MODE_MASTER     1
#define   SPI_DEVICE_MODE_SLAVE      0


/*
 *            @SPI_BusConfig Macros
 */
#define   SPI_BUS_CONFIG_FD          1
#define   SPI_BUS_CONFIG_HD          2
#define   SPI_BUS_SIMPLEX_RXONLY     3

/*
 *            @SPI_DFF Macros
 */
#define SPI_DFF_8BITS                0
#define SPI_DFF_16BITS               1

/*
 *            @SPI_CPHA Macros
 */
#define SPI_CPHA_HIGH                1
#define SPI_CPHA_LOW                 0

/*
 *            @SPI_CPOL Macros
 */
#define SPI_CPOL_HIGH                1
#define SPI_CPOL_LOW                 0

/*
 *            @SPI_SSM Macros
 */
#define SPI_SSM_EN                   1
#define SPI_SSM_DI                   0

/*
 *            @ SPI_Speed Macros
 */

#define  SPI_SPEED_DIV2               0
#define  SPI_SPEED_DIV4               1
#define  SPI_SPEED_DIV8               2
#define  SPI_SPEED_DIV16              3
#define  SPI_SPEED_DIV32              4
#define  SPI_SPEED_DIV64              5
#define  SPI_SPEED_DIV128             6
#define  SPI_SPEED_DIV256             7

/*
 *         SPI related status flags definitions
 */

#define SPI_TXE_FLAG                  (1 << SPI_SR_TXE)
#define SPI_BUSY_FLAG                 (1 << SPI_SR_BSY)







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
void SPI_PeriClockControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi);

// Init and De-init

void SPI_Init(SPI_Handle_t  *pSPIHandle);
void SPI_DeInit(SPI_Regdef_t  *pSPIx);

// Flag status of SPI
uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx, uint32_t FlagName);


// Data Send and Receive

void SPI_SendData(SPI_Regdef_t  *pSPIx,uint8_t *pTxBuffer, uint32_t Len);

void SPI_ReceiveData(SPI_Regdef_t  *pSPIx,uint8_t *pRxBuffer, uint32_t Len);

// IRQ Configuration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// other peripheral control APIs

void SPI_PeripheralControl(SPI_Regdef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfigure(SPI_Regdef_t *pSPIx, uint8_t EnorDi);


#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
