# Driver development for (GPIO, I2C and SPI) STM32f446RE
## Development of communication protocols libraries by understanding user manual and datasheet microcontroller.
Testing developed libraries using basic projects.
##### Table of Contents  
[1.Mcu_strcuture_headerfile](#headers)
[](#emphasis)  
[2.GPIO_headerfile](#headers) 
[](#emphasis)   
[3.SPI_headerfile](#headers)
[](#emphasis)   
[4.I2C_headerfile](#headers)
[](#emphasis)   
[5.GPIO_library](#headers)
[](#emphasis)   
[6.SPI_library](#headers)
[](#emphasis)   
[7.I2C_library](#headers)
[](#emphasis)   

 <a name="headers"/>
 
## 1.Mcu_strcuture_headerfile
This header file contains the base address peripherals like (GPIOX, SPIX,I2CX,USARTX/UARTX,TIMERSX..etc) lieing on buses(APHB1, APHB2, APHB3, APB1, APB2). And some of the peripherals structure of GPIOX, SPIX,I2CX. Even with the required macros of communicaiton protocols.
## 2.GPIO_headerfile
Contains GPIO handle structure, which can handle GPIO port (A...H) and configurations like pin(number, mode, type, speed ..etc,) and  with required API  prototype declaration like Initialise, De-initiaslise,read, write, and toggle..etc,.Even with the required macros of GPIOX.
## 3.SPI_headerfile
Contains SPI handle structure, which can handle SPI1, SPI2, SPI3, and SPI4 and configurations like device mode (Master or Slave mode), busconfiguration (Full duplex, Half duplex or Simple duplex), speed selection, data register (8bit or 16bit register), slave select management, CPHA, CPOL along with required API  prototype declaration like Initialise, De-initiaslise,send,receive data..etc,,Even with the required macros of SPIX.
## 4.I2C_headerfile
Contains I2C handle structure, which can handle I2C1, I2C2,I2C3, and I2C4 and configurations like device mode (Master or Slave mode), busconfiguration (Full duplex, Half duplex or Simple duplex), speed selection, data register (8bit or 16bit register), slave select management, CPHA, CPOL along with required API  prototype declaration like Initialise, De-initiaslise,send,receive data..etc,.Even with the required macros of I2CX.
## 5.GPIO_library
## 6.SPI_library
## 7.I2C_library
