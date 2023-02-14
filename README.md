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
 
## Mcu_strcuture_headerfile
This header file contains the base address peripherals like (GPIOX, SPIX,I2CX,USARTX/UARTX,TIMERSX..etc) lieing on buses(APHB1, APHB2, APHB3, APB1, APB2). And some of the peripherals structure of GPIOX, SPIX,I2CX. Even with the required macros of communicaiton protocols.
