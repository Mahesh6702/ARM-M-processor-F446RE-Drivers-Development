// Communication between St microcontroller and Arduino uno using SPI protocol

#include<string.h>
#include "stm32f446xx.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB10 -> SPI2_SCLK
 * PB9 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_pinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_pinConfig.GPIO_PinOPtype = GPIO_OTYPE_PPL;
	SPIPins.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_PINPUPD_NO;
	SPIPins.GPIO_pinConfig.GPIO_PinSpeed = GPIO_OUT_FS;

	//SCLK
	SPIPins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PinNumber_P10;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PinNumber_P15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);


	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	//GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{

	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_Devicemode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SPEED_DIV2;//generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //software slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "Hello world";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfigure(SPI2,ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	//to send data

	SPI2->CR1 |= (1 << SPI_CR1_SPE);
	SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));
		//for(unsigned long int i=0;i<50000000;i++);

	//lets confirm SPI is not busy
	while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1)

	return 0;

}
