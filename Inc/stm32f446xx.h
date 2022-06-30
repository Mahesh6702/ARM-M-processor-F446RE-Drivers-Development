/*
 * stm32f446xx.h
 *
 *  Created on: Oct 30, 2021
 *      Author: eegam
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>

#define __vo volatile  // __vo is volatile keyword used for memory location which can changed be any time in the program


/******************************************START:Processor Specific Details *****************************************************************
 *
 * ARM Cortex M4 Processor NVIC ISERx register Address
 */

// Macros for interrupt Set-enable Registers

#define NVIC_ISER0                      ((__vo uint32_t*)0xE000E100) // Base address of NVIC_ISER0
#define NVIC_ISER1                      ((__vo uint32_t*)0xE000E104) // Base address of NVIC_ISER1
#define NVIC_ISER2                      ((__vo uint32_t*)0xE000E108) // Base address of NVIC_ISER2
#define NVIC_ISER3                      ((__vo uint32_t*)0xE000E10C) // Base address of NVIC_ISER3

//Macros for Interrupt Clear-enable Registers

#define NVIC_ICER0                       ((__vo uint32_t*)0XE000E180) // Base address of NVIC_ICER0
#define NVIC_ICER1                       ((__vo uint32_t*)0XE000E184) // Base address of NVIC_ICER1
#define NVIC_ICER2                       ((__vo uint32_t*)0XE000E188) // Base address of NVIC_ICER2
#define NVIC_ICER3                       ((__vo uint32_t*)0XE000E18C) // Base address of NVIC_ICER3

// ARm Cortex M4 processor priority Register Address calculation

#define NVIC_PR_BASEADDR                 ((__vo uint32_t*)0xE000E400) // Base address of NVIC_ICER0




// ARm Cortex M4 processor number of priority bits implemented in Priority Register

#define NO_OR_BITS_IMPLEMENTED                4


#define FLASH_BASEADDR                   0x08000000U  // Base address of Flash memory
#define SRAM_BASEADDR                    0x20000000U  // Base address of SRAM
#define SRMA1_BASEADDR                   0x20000000U  // Base address of SRAM1
#define SRAM2_BASEADDR                   0x2001C000U  // Base address of SRAM2
#define ROM__BASEADDR                    0x1FFF0000U  // Base address of ROM memory


// defining base address for Busses

#define AHB1_BASEADDR                    0x40020000U  // Base address of AHB1 bus
#define AHB2_BASEADDR                    0x50000000U  // Base address of AHB2 bus
#define AHB3_BASEADDR                    0x60000000U  // Base address of AHB3 bus
#define APB1_BASEADDR                    0x40010000U  // Base address of APB1 bus
#define APB2_BASEADDR                    0x40000000U  // Base address of APB2 bus


// defining base address for Peripherals hanging on AHB1 Bus

#define GPIOA_BASEADDR                   0x40020000U   // Base address of GPIOA
#define GPIOB_BASEADDR                   0x40020400U   // Base address of GPIOB
#define GPIOC_BASEADDR                   0x40020800U   // Base address of GPIOC
#define GPIOD_BASEADDR                   0X40020C00U   // Base address of GPIOD
#define GPIOE_BASEADDR                   0x40021000U   // Base address of GPIOE
#define GPIOF_BASEADDR                   0x40021400U   // Base address of GPIOF
#define GPIOG_BASEADDR                   0x40021800U   // Base address of GPIOG
#define GPIOH_BASEADDR                   0x40021C00U   // Base address of GPIOH
#define CRC_BASEADDR                     0x40023000U   // Base address of CRC
#define RCC_BASEADDR                     0x40023800U   // Base address of RCC
#define Flash_interface_BASEADDR         0x40023C00U   // Base address of Flash interface
#define BKPSRAM_BASEADDR                 0x40024000U   // Base address of BKPSRAM
#define DMA1_BASEADDR                    0x40026000U   // Base address of DMA1
#define DMA2_BASEADDR                    0x40026400U   // Base address of DMA2
#define USB_OTG_HS_BASEADDR              0x40040000U   // Base address of USB_OTG_HS

// defining base address for Peripherals hanging on AHB2 Bus
#define USB_OTG_FS_BASEADDR              0x50000000U   // Base address of USB_OTG_FS
#define DCMI_BASEADDR                    0x50050000U   // Base address of DCMI


// defining base address for Peripherals hanging on AHB3 Bus

#define FMC_bank_1_BASEADDR               0x60000000U  // Base address of FMC bank 1
#define FMC_bank_3_BASEADDR               0x80000000U  // Base address of FMC bank 3
#define QuadSPI_BASEADDR                  0x90000000U  // Base address of QuadSPI
#define FMC_control_register_BASEADDR     0xA0000000U  // Base address of FMC control register
#define QuadSPI_control_register_BASEADDR 0xA0001000U  // Base address of QuadSPI control register
#define FMC_bank_5_BASEADDR               0xC0000000U  // Base address of FMC bank 5
#define FMC_bank_6_BASEADDR               0xD0000000U  // Base address of FMC bank 6

// defining base address for Peripherals hanging on APB1 Bus
#define TIM2_BASEADDR                    0x40000000U   // Base address of TIM2
#define TIM3_BASEADDR                    0x40000400U   // Base address of TIM3
#define TIM4_BASEADDR                    0x40000800U   // Base address of TIM4
#define TIM5_BASEADDR                    0x40000C00U   // Base address of TIM5
#define TIM6_BASEADDR                    0x40001000U   // Base address of TIM6
#define TIM7_BASEADDR                    0x40001400U   // Base address of TIM7
#define TIM12_BASEADDR                   0x40001800U   // Base address of TIM12
#define TIM13_BASEADDR                   0x40001C00U   // Base address of TIM13
#define TIM14_BASEADDR                   0x40002000U   // Base address of TIM14
#define RTC_BKP_Registers                0x40002800U   // Base address of RTC_&_BKP Registers
#define WWDG                             0x40002C00U   // Base address of WWDG
#define IWDG                             0x40003000U   // Base address of IWDG
#define SPI2_I2S2_BASEADDR               0x40003800U   // Base address of SPI2 / I2S2
#define SPI3_I2S3_BASEADDR               0x40003C00U   // Base address of SPI3 / I2S3
#define SPDIFRX                          0x40004000U   // Base address of SPDIFRX
#define USART2_BASEADDR                  0x40004400U   // Base address of USART2
#define USART3_BASEADDR                  0x40004800U   // Base address of USART3
#define UART4_BASEADDR                   0x40004C00U   // Base address of UART4
#define UART5_BASEADDR                   0x40005000U   // Base address of UART5
#define I2C1_BASEADDR                    0x40005400U   // Base address of I2C1
#define I2C2_BASEADDR                    0x40005800U   // Base address of I2C2
#define I2C3_BASEADDR                    0x40005C00U   // Base address of I2C3
#define FMPI2C1                          0x40006000U   // Base address of FMPI2C1
#define CAN1                             0x40006400U   // Base address of CAN1
#define CAN2                             0x40006800U   // Base address of CAN2
#define HDMI_CEC                         0x40006C00U   // Base address of HDMI-CEC
#define PWR                              0x40007000U   // Base address of PWR
#define DAC                              0x40007400U   // Base address of DAC



// defining base address for Peripherals hanging on APB2 Bus

#define TIM1_BASEADDR                     0x40010000U  // Base address of TIM1
#define TIM8_BASEADDR                     0x40010400U  // Base address of TIM8
#define USART1_BASEADDR                   0x40011000U  // Base address of USART1
#define USART6_BASEADDR                   0x40011400U  // Base address of USART6
#define ADC1_BASEADDR                     0x40012000   // Base address of ADC1
#define ADC2_BASEADDR                     (ADC1_BASEADDR + 0x100) // Base address of ADC2
#define ADC3_BASEADDR                     (ADC1_BASEADDR + 0x200) // Base address of ADC3
#define SDIO_BASEADDR                     0x40012C00U  // Base address of SDIO
#define SPI1_BASEADDR                     0x40013000U  // Base address of SPI1
#define SPI4_BASEADDR                     0x40013400U  // Base address of SPI4
#define SYSCFG_BASEADDR                   0x40013800U  // Base address of SYSCFG
#define EXTI_BASEADDR                     0x40013C00U  // Base address of EXTI
#define TIM9_BASEADDR                     0x40014000U  // Base address of TIM9
#define TIM10_BASEADDR                    0x40014400U  // Base address of TIM10
#define TIM11_BASEADDR                    0x40014800U  // Base address of TIM11
#define SAI1_BASEADDR                     0x40015800U  // Base address of SAI1
#define SAI2_BASEADDR                     0x40015C00U  // Base address of SAI2



/* =====================********************** Peripheral Structure creation****************************************************
                                                @GPIO(General purpose input and output)
                                                @RCC(Reset clock and control)
                                                @EXTI(Interrupts and events)
                                                @SYSCFG(System configuration controller)
                                                @SPI(Serial peripheral interface)


*/
//Peripheral structure creation for GPIOx ports

typedef struct
{
	__vo uint32_t MODER;     // Base address of GPIOx mode
	__vo uint32_t OTYPER;    // Base address of GPIOx OTYPER
	__vo uint32_t OSPEEDER;  // Base address of GPIOx OSPEEDE
	__vo uint32_t PUPDR;     // Base address of GPIOx PUPDR
	__vo uint32_t IDR;       // Base address of GPIOx IDR
	__vo uint32_t ODR;       // Base address of GPIOx ODR
	__vo uint32_t BSRR;      // Base address of GPIOx BSRR
	__vo uint32_t LCKR;      // Base address of GPIOx LCKR
	__vo uint32_t AFR[2];    // Base address of GPIOx AFRL is AFRL[0] and AFRH is AFRH[1]


}GPIOx_Regdef_t;


//Peripheral structure creation for RCC peripherals
typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1_RSTR;
	__vo uint32_t AHB2_RSTR;
	__vo uint32_t AHB3_RSTR;
	  uint32_t    reserved_1;
	__vo uint32_t APB1_RSTR;
	__vo uint32_t APB2_RSTR;
	  uint32_t    reserved_2[2];
	__vo uint32_t AHB1_ENR;
	__vo uint32_t AHB2_ENR;
	__vo uint32_t AHB3_ENR;
	  uint32_t    reserved_3;
	__vo uint32_t APB1_ENR;
	__vo uint32_t APB2_ENR;
	  uint32_t    reserved_4[2];
	__vo uint32_t AHB1_LPENR;
	__vo uint32_t AHB2_LPENR;
	__vo uint32_t AHB3_LPENR;
	  uint32_t    reserved_5;
	__vo uint32_t APB1_LPENR;
	__vo uint32_t APB2_LPENR;
	  uint32_t    reserved_6[2];
    __vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	  uint32_t    reserved_7[2];
    __vo uint32_t RCC_SS_CGR;
    __vo uint32_t RCC_PLLI2_SCRGR;
    __vo uint32_t RCC_PLL_SAI_CFGR;
    __vo uint32_t RCC_DCK_CFGR;
    __vo uint32_t RCC_CK_GATENR;
    __vo uint32_t RCC_DCK_CFGR2;

}RCC_Regdef_t;

 // GPIO_Regdef_t *GPIOA =(GPIO_Regdef_t*) MODER; // Can also be written as macro below

//Peripheral structure creation for EXTI peripheral

typedef struct
{
	__vo uint32_t IMR;            // Base address of EXTI_IMR
	__vo uint32_t EMR;            // Base address of EXTI_EMR
	__vo uint32_t RTSR;          // Base address of EXTI_RTSR
	__vo uint32_t FTSR;         // Base address of  EXTI_FTSR
	__vo uint32_t SWIER;       // Base address of EXTI_SWIER
	__vo uint32_t PR;         // Base address of EXTI_PR



}EXTI_Regdef_t;

//Peripheral structure creation for SYSCFGperipheral
typedef struct
{
	__vo uint32_t MEMRMP;          // Base address of SYSCFG_MEMRMP
	__vo uint32_t PMC;            // Base address of SYSCFG_PMC
	__vo uint32_t EXTICR[4];     // Base address of SYSCFG_EXTICR1, SYSCFG_EXTICR2, SYSCFG_EXTICR3, SYSCFG_EXTICR4
	__vo uint32_t CMPCR;        // Base address of SYSCFG_CMPCR
	__vo uint32_t CFGR;        // Base address of SYSCFG_CFGR

}SYSCFG_Regdef_t;

//Peripheral structure creation for SPIx ports

typedef struct
{
	__vo uint32_t CR1;     // Base address of SPIx control register 1
	__vo uint32_t CR2;     // Base address of SPIx control register 2
	__vo uint32_t SR;      // Base address of SPIx status register
	__vo uint32_t DR;      // Base address of SPIx Data register
	__vo uint32_t CRCPR;   // Base address of SPIx CRC polynomial register
	__vo uint32_t RXCRCR;   // Base address of SPIx RX CRC register
	__vo uint32_t TXCRCR;   // Base address of SPIx TX CRC register
	__vo uint32_t I2SCFGR;  // Base address of SPI_I2S Configuration register
	__vo uint32_t I2SPR;    // pre scalar register


}SPI_Regdef_t;


//Peripheral structure creation for I2Cx ports

typedef struct
{
	__vo uint32_t CR1;       // Base address of I2Cx control register 1
	__vo uint32_t CR2;       // Base address of I2Cx control register 2
	__vo uint32_t OAR1;      // Base address of I2Cx address register 1
	__vo uint32_t OAR2;      // Base address of I2Cx address register 2
	__vo uint32_t DR;        // Base address of I2Cx data register
	__vo uint32_t SR1;       // Base address of I2Cx status register 1
	__vo uint32_t SR2;       // Base address of I2Cx status register 2
	__vo uint32_t CCR;       // Base address of I2Cx clock control register 1
	__vo uint32_t TRISE;     // Base address of I2Cx clock TRISE
	__vo uint32_t FLTR;      // Base address of I2Cx clock  FLTR


}I2C_Regdef_t;

/* =====================********************** Peripheral Structure definition creation****************************************************
                                                @GPIO(General purpose input and output)
                                                @RCC(Reset clock and control)
                                                @EXTI(Interrupts and events)
                                                @SYSCFG(System configuration controller)
                                                @SPI(Serial peripheral interface)


*/

// Peripheral definitions of GPIO

#define GPIOA  ((GPIOx_Regdef_t*)GPIOA_BASEADDR)  // GPIOx_Regdef_t structure as GPIOA Peripheral
#define GPIOB  ((GPIOx_Regdef_t*)GPIOB_BASEADDR)  // GPIOx_Regdef_t structure as GPIOB Peripheral
#define GPIOC  ((GPIOx_Regdef_t*)GPIOC_BASEADDR)  // GPIOx_Regdef_t structure as GPIOC Peripheral
#define GPIOD  ((GPIOx_Regdef_t*)GPIOD_BASEADDR)  // GPIOx_Regdef_t structure as GPIOD Peripheral
#define GPIOE  ((GPIOx_Regdef_t*)GPIOE_BASEADDR)  // GPIOx_Regdef_t structure as GPIOE Peripheral
#define GPIOF  ((GPIOx_Regdef_t*)GPIOF_BASEADDR)  // GPIOx_Regdef_t structure as GPIOF Peripheral
#define GPIOG  ((GPIOx_Regdef_t*)GPIOG_BASEADDR)  // GPIOx_Regdef_t structure as GPIOG Peripheral
#define GPIOH  ((GPIOx_Regdef_t*)GPIOH_BASEADDR)  // GPIOx_Regdef_t structure as GPIOH Peripheral

// Peripheral definitions for Reset and clock control(RCC)
#define RCC       ((RCC_Regdef_t*)RCC_BASEADDR)         // Taking the base address using pointer variable RCC

// Peripheral definitions for interrupt(EXTI)
#define EXTI      ((EXTI_Regdef_t*)EXTI_BASEADDR)      // Taking the base address using pointer variable EXTI

// Peripheral definitions for System configuration controller(SYSCFG)
#define SYSCFG      ((SYSCFG_Regdef_t*)SYSCFG_BASEADDR) // Taking the base address using pointer variable SYSCFG

// Peripheral definitions for SPI(Serial peripheral interface)
#define SPI1         ((SPI_Regdef_t*)SPI1_BASEADDR)          // SPI_Regdef_t structure as SPI peripheral
#define SPI2         ((SPI_Regdef_t*)SPI2_I2S2_BASEADDR)     // SPI_Regdef_t structure as SPI peripheral
#define SPI3         ((SPI_Regdef_t*)SPI3_I2S3_BASEADDR)     // SPI_Regdef_t structure as SPI peripheral
#define SPI4         ((SPI_Regdef_t*)SPI4_BASEADDR)          // SPI_Regdef_t structure as SPI peripheral


// Peripheral definitions for I2C(Serial peripheral interface)
#define I2C1         ((I2C_Regdef_t*)I2C1_BASEADDR)          // I2C_Regdef_t structure as I2C1 peripheral
#define I2C2         ((I2C_Regdef_t*)I2C2_BASEADDR)          // I2C_Regdef_t structure as I2C2 peripheral
#define I2C3         ((I2C_Regdef_t*)I2C3_BASEADDR)          // I2C_Regdef_t structure as I2C3 peripheral
#define I2C4         ((I2C_Regdef_t*)FMPI2C1)                // I2C_Regdef_t structure as I2C4 peripheral




/* =====================**********************  Enable the  clock for  peripherals****************************************************
                                                @GPIO(General purpose input and output)
                                                @RCC(Reset clock and control)
                                                @EXTI(Interrupts and events)
                                                @SYSCFG(System configuration controller)
                                                @SPI(Serial peripheral interface)


*/
// Enable the GPIO clock peripherals

#define GPIOA_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 0) )  // Enabling the clock for GPIOA port
#define GPIOB_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 1) )  // Enabling the clock for GPIOB port
#define GPIOC_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 2) )  // Enabling the clock for GPIOC port
#define GPIOD_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 3) )  // Enabling the clock for GPIOD port
#define GPIOE_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 4) )  // Enabling the clock for GPIOE port
#define GPIOF_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 5) )  // Enabling the clock for GPIOF port
#define GPIOG_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 6) )  // Enabling the clock for GPIOG port
#define GPIOH_PCLK_EN()        ( (RCC->AHB1_ENR)|= (1 << 7) )  // Enabling the clock for GPIOH port

// Enable the SPIx clock peripherals

#define SPI1_PCLK_EN()        ( (RCC->APB2_ENR)|= (1 << 12) )  // Enabling the clock for SPI1 PORT
#define SPI2_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 14) )  // Enabling the clock for SPI2 port
#define SPI3_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 15) )  // Enabling the clock for SPI3 port
#define SPI4_PCLK_EN()        ( (RCC->APB2_ENR)|= (1 << 13) )  // Enabling the clock for SPI4 port

// Enable the I2Cx clock peripherals
#define I2C1_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 21) )  // Enabling the clock for I2C1 port
#define I2C2_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 22) )  // Enabling the clock for I2C2 port
#define I2C3_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 23) )  // Enabling the clock for I2C3 port
#define I2C4_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 24) )  // Enabling the clock for I2C4 port


// Enable the USARTx and UARTx clock peripherals

#define USART1_PCLK_EN()        ( (RCC->APB2_ENR)|= (1 << 4) )   // Enabling the clock for USART1 PORT
#define USART2_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 17) )  // Enabling the clock for USART2 port
#define USART3_PCLK_EN()        ( (RCC->APB1_ENR)|= (1 << 18) )  // Enabling the clock for USART3 port
#define UART4_PCLK_EN()         ( (RCC->APB1_ENR)|= (1 << 19) )  // Enabling the clock for UART4 port
#define UART5_PCLK_EN()         ( (RCC->APB1_ENR)|= (1 << 20) )  // Enabling the clock for UART5 port
#define USART6_PCLK_EN()        ( (RCC->APB2_ENR)|= (1 << 5) )   // Enabling the clock for USART6 PORT

// Enable the SYSCFG clock peripherals

#define SYSCFG_PCLK_EN()        ( (RCC->APB2_ENR)|= (1 << 14) )  // Enabling the clock for SYSCFG PORT



/* =====================**********************  Disable the  clock for  peripherals****************************************************
                                                @GPIO(General purpose input and output)
                                                @RCC(Reset clock and control)
                                                @EXTI(Interrupts and events)
                                                @SYSCFG(System configuration controller)
                                                @SPI(Serial peripheral interface)


*/
// Disable the GPIO clock peripherals

#define GPIOA_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 0) )  // Disabling the clock for GPIOA port
#define GPIOB_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 1) )  // Disabling the clock for GPIOB port
#define GPIOC_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 2) )  // Disabling the clock for GPIOC port
#define GPIOD_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 3) )  // Disabling the clock for GPIOD port
#define GPIOE_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 4) )  // Disabling the clock for GPIOE port
#define GPIOF_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 5) )  // Disabling the clock for GPIOF port
#define GPIOG_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 6) )  // Disabling the clock for GPIOG port
#define GPIOH_PCLK_DI()        ( (RCC->AHB1_ENR)&= ~(1 << 7) )  // Disabling the clock for GPIOH port

// Disable the SPIx clock peripherals

#define SPI1_PCLK_DI()        ( (RCC->APB2_ENR)&= ~(1 << 12) )  // Disabling the clock for SPI1 PORT
#define SPI2_PCLK_DI()        ( (RCC->APB1_ENR)&= ~(1 << 14) )  // Disabling the clock for SPI2 port
#define SPI3_PCLK_DI()        ( (RCC->APB1_ENR)&= ~(1 << 15) )  // Disabling the clock for SPI3 port
#define SPI4_PCLK_DI()        ( (RCC->APB2_ENR)&= ~(1 << 13) )  // Disabling the clock for SPI4 port

// Disable the I2Cx clock peripherals
#define I2C1_PCLK_DI()        ( (RCC->APB1_ENR)&= ~(1 << 21) )  // Disabling the clock for I2C1 port
#define I2C2_PCLK_DI()        ( (RCC->APB1_ENR)&= ~(1 << 22) )  // Disabling the clock for I2C2 port
#define I2C3_PCLK_DI()        ( (RCC->APB1_ENR)&= ~(1 << 23) )  // Disabling the clock for I2C3 port
#define I2C4_PCLK_DI()        ( (RCC->APB1_ENR)&= ~(1 << 24) )  // Disabling the clock for I2C3 port


// Disable the USARTx and UARTx clock peripherals

#define USART1_PCLK_DI()        ( (RCC->APB2_ENR)& = ~(1 << 4) )   // Disabling the clock for USART1 PORT
#define USART2_PCLK_DI()        ( (RCC->APB1_ENR)& = ~(1 << 17) )  // Disabling the clock for USART2 port
#define USART3_PCLK_DI()        ( (RCC->APB1_ENR)& = ~(1 << 18) )  // Disabling the clock for USART3 port
#define UART4_PCLK_DI()         ( (RCC->APB1_ENR)& = ~(1 << 19) )  // Disabling the clock for UART4 port
#define UART5_PCLK_DI()         ( (RCC->APB1_ENR)& = ~(1 << 20) )  // Disabling the clock for UART5 port
#define USART6_PCLK_DI()        ( (RCC->APB2_ENR)& = ~(1 << 5) )   // Disabling the clock for USART6 PORT

// Disable the SYSCFG clock peripherals

#define SYSCFG_PCLK_DI()        ( (RCC->APB2_ENR)& = ~(1 << 14) )  // Disabling the clock for SYSCFG PORT


// Macro's for GPIO peripheral reset register
#define GPIOA_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 0) );  ( (RCC->AHB1_RSTR)&= ~(1 << 0) ); } while(0)
#define GPIOB_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 1) );  ( (RCC->AHB1_RSTR)&= ~(1 << 1) ); } while(0)
#define GPIOC_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 2) );  ( (RCC->AHB1_RSTR)&= ~(1 << 2) ); } while(0)
#define GPIOD_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 3) );  ( (RCC->AHB1_RSTR)&= ~(1 << 3) ); } while(0)
#define GPIOE_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 4) );  ( (RCC->AHB1_RSTR)&= ~(1 << 4) ); } while(0)
#define GPIOF_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 5) );  ( (RCC->AHB1_RSTR)&= ~(1 << 5) ); } while(0)
#define GPIOG_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 6) );  ( (RCC->AHB1_RSTR)&= ~(1 << 6) ); } while(0)
#define GPIOH_REG_RESET()       do{ ( (RCC->AHB1_RSTR)|= (1 << 7) );  ( (RCC->AHB1_RSTR)&= ~(1 << 7) ); } while(0)



// Macro's for SPI peripheral reset register

#define SPI1_REG_RESET()        do{ ( (RCC->APB2_RSTR)|= (1 << 12) );  ( ( (RCC->APB2_RSTR)&= ~(1 << 12) ) ); } while(0)     // Reseting the SPI1
#define SPI2_REG_RESET()        do{ ( (RCC->APB1_RSTR)|= (1 << 14) );  ( ( (RCC->APB2_RSTR)&= ~(1 << 14) ) ); } while(0)     // Reseting the SPI2
#define SPI3_REG_RESET()        do{ ( (RCC->APB1_RSTR)|= (1 << 15) );  ( ( (RCC->APB2_RSTR)&= ~(1 << 15) ) ); } while(0)     // Reseting the SPI3
#define SPI4_REG_RESET()        do{ ( (RCC->APB2_RSTR)|= (1 << 13) );  ( ( (RCC->APB2_RSTR)&= ~(1 << 13) ) ); } while(0)     // Reseting the SPI4


// Macro's for I2C peripheral reset register
#define I2C1_REG_RESET()        do{ ( (RCC->APB1_RSTR)|= (1 << 21) );  ( ( (RCC->APB1_RSTR)&= ~(1 << 21) ) ); } while(0)     // Reseting the I2C1
#define I2C2_REG_RESET()        do{ ( (RCC->APB1_RSTR)|= (1 << 22) );  ( ( (RCC->APB1_RSTR)&= ~(1 << 22) ) ); } while(0)     // Reseting the I2C2
#define I2C3_REG_RESET()        do{ ( (RCC->APB1_RSTR)|= (1 << 23) );  ( ( (RCC->APB1_RSTR)&= ~(1 << 23) ) ); } while(0)     // Reseting the I2C3
#define I2C4_REG_RESET()        do{ ( (RCC->APB1_RSTR)|= (1 << 24) );  ( ( (RCC->APB1_RSTR)&= ~(1 << 24) ) ); } while(0)     // Reseting the I2C4

// some generic macros

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET
#define FLAG_SET                SET
#define FLAG_RESET              RESET



// Macros for PORTcode of GPIOx ports            // conditional operators
#define GPIO_BASEADDR_TO_CODE(x)               ((x == GPIOA)?0:\
                                                (x == GPIOB)?1:\
                                                (x == GPIOC)?2:\
                                                (x == GPIOD)?3:\
                                                (x == GPIOE)?4:\
                                   				(x == GPIOF)?5:\
                          						(x == GPIOG)?6:\
                  								(x == GPIOH)?7:0)


// Macros for IRQ number(Interrupt Request) of STM32f446Rx MCU

#define IRQ_NO_EXTI0              6
#define IRQ_NO_EXTI1              7
#define IRQ_NO_EXTI3              9
#define IRQ_NO_EXTI4              10
#define IRQ_NO_EXTI9_5            23
#define IRQ_NO_EXTI15_10          40


// MACROS for all possible priority levels

#define NVIC_IRQ_PRI0             0
#define NVIC_IRQ_PRI1             1
#define NVIC_IRQ_PRI2             2
#define NVIC_IRQ_PRI3             3
#define NVIC_IRQ_PRI4             4
#define NVIC_IRQ_PRI5             5
#define NVIC_IRQ_PRI6             6
#define NVIC_IRQ_PRI7             7
#define NVIC_IRQ_PRI8             8
#define NVIC_IRQ_PRI9             9
#define NVIC_IRQ_PRI10            10
#define NVIC_IRQ_PRI11            11
#define NVIC_IRQ_PRI12            12
#define NVIC_IRQ_PRI13            13
#define NVIC_IRQ_PRI14            14
#define NVIC_IRQ_PRI15            15


/***************************************************************************************************************************************
 * Bit position definitions of SPI peripheral
 ***************************************************************************************************************************************/
      // Bit position macros for CR1 register
#define  SPI_CR1_CPHA            0
#define  SPI_CR1_CPOL            1
#define  SPI_CR1_MSTR            2
#define  SPI_CR1_BR              3
#define  SPI_CR1_SPE             6
#define  SPI_CR1_LSB_FIRST       7
#define  SPI_CR1_SSI             8
#define  SPI_CR1_SSM             9
#define  SPI_CR1_RXONLY          10
#define  SPI_CR1_DFF             11
#define  SPI_CR1_CRC_NEXT        12
#define  SPI_CR1_CRC_EN          13
#define  SPI_CR1_BIDIOE          14
#define  SPI_CR1_BIDIMODE        15

// Bit position macros for CR2 register

#define  SPI_CR2_RXDMAEN         0
#define  SPI_CR2_TXDMAEN         1
#define  SPI_CR2_SSOE            2
#define  SPI_CR2_FRF             4
#define  SPI_CR2_ERRIE           5
#define  SPI_CR2_RXNEIE          6
#define  SPI_CR2_TXEIE           7


// Bit position macros for SR register
#define  SPI_SR_RXNE             0
#define  SPI_SR_TXE              1
#define  SPI_SR_CHSIDE           2
#define  SPI_SR_UDR              3
#define  SPI_SR_CRCERR           4
#define  SPI_SR_MODF             5
#define  SPI_SR_OVR              6
#define  SPI_SR_BSY              7
#define  SPI_SR_FRE              8


/***************************************************************************************************************************************
 * Bit position definitions of I2C peripheral
 ***************************************************************************************************************************************/
      // Bit position macros for CR1 register

#define  I2C_CR1_PE              0
#define  I2C_CR1_SMBUS           1
#define  I2C_CR1_SMB_TYPE        3
#define  I2C_CR1_ENARP           4
#define  I2C_CR1_ENPEC           5
#define  I2C_CR1_ENGC            6
#define  I2C_CR1_NOSTRETCH       7
#define  I2C_CR1_START           8
#define  I2C_CR1_STOP            9
#define  I2C_CR1_ACK             10
#define  I2C_CR1_POS             11
#define  I2C_CR1_PEC             12
#define  I2C_CR1_ALERT           13
#define  I2C_CR1_SWRST           15


// Bit position macros for CR2 register

#define  I2C_CR2_FREQ             0
#define  I2C_CR2_ITERREN          8
#define  I2C_CR2_ITEVTEN          9
#define  I2C_CR2_ITBUF            10
#define  I2C_CR2_DMAEN            11
#define  I2C_CR2_LAST             12

// Bit position macros for SR1 register

#define I2C_SR1_SB                 0
#define I2C_SR1_ADDR               1
#define I2C_SR1_BTF                2
#define I2C_SR1_ADD10              3
#define I2C_SR1_STOPF              4
#define I2C_SR1_RxNE               6
#define I2C_SR1_TxE                7
#define I2C_SR1_BERR               8
#define I2C_SR1_ARLO               9
#define I2C_SR1_AF                 10
#define I2C_SR1_OVR                11
#define I2C_SR1_PECERR             12
#define I2C_SR1_TIMEOUT            14
#define I2C_SR1_SMBALERT           15

// Bit position macros for SR2 register

#define I2C_SR2_MSL                0
#define I2C_SR2_BUSY               1
#define I2C_SR2_TRA                2
#define I2C_SR2_GENCALL            4
#define I2C_SR2_SMBDEFAULT         5
#define I2C_SR2_SMBHOST            6
#define I2C_SR2_DUALF              7
#define I2C_SR2_PEC                8


#include "stm32f446xx_gpio_driver.h"

#include "stm32f446xx_spi_driver.h"

#include "stm32f446xx_i2c_driver.h"


#endif /* INC_STM32F446XX_H_ */
