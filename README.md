## Driver development for (GPIO, I2C and SPI) STM32f446RE
## Development of communication protocols libraries by understanding user manual and datasheet microcontroller.
### Table of Contents  
# Table of Contents
1. [Mcu_strcuture_headerfile](#example)
2. [GPIO_headerfile](#example2)
3. [SPI_headerfile](#third-example)
4. [I2c_headerfile](#fourth-example)
5. [GPIO_library](#fifth-example)
6. [SPI_library](#sixth-example)
7. [I2C_library](#seventh-example)
8. [Projects](#eighth-example)
9. [Documents](#nineth-example)

 <a name="headers"/>
 
### Mcu_strcuture_headerfile
This header file contains the base address peripherals like (GPIOX, SPIX,I2CX,USARTX/UARTX,TIMERSX..etc) lieing on buses(APHB1, APHB2, APHB3, APB1, APB2). And some of the peripherals structure of GPIOX, SPIX,I2CX. Even with the required macros of communicaiton protocols.
### GPIO_headerfile
Contains GPIO handle structure, which can handle GPIO port (A...H) and configurations like pin(number, mode, type, speed ..etc,) and  with required API  prototype declaration like Initialise, De-initiaslise,read, write, and toggle..etc,.Even with the required macros of GPIOX.
### SPI_headerfile
In this header file it contains SPI handle structure, which can handle SPI1, SPI2, SPI3, and SPI4 and configurations like device mode (Master or Slave mode), busconfiguration (Full duplex, Half duplex or Simple duplex), speed selection, data register (8bit or 16bit register), slave select management, CPHA, CPOL along with required API  prototype declaration like Initialise, De-initiaslise,send,receive data..etc,,Even with the required macros of SPIX.
### I2C_headerfile
This header file has I2C handle structure, which can handle I2C1, I2C2,I2C3, and I2C4 and configurations like device mode (Master or Slave mode), busconfiguration (Full duplex, Half duplex or Simple duplex), speed selection, data register (8bit or 16bit register), slave select management, CPHA, CPOL along with required API  prototype declaration like Initialise, De-initiaslise,send,receive data..etc,.Even with the required macros of I2CX.
### GPIO_library
This library contains the defintion for Intialise, Deintialise, Toggle, read and write API as menitoned in respective headerfile.
### SPI_library
Spi library contians funciton definitions for the funciton declare in the header file
### I2C_library
It contians defintions for the PeripheralControl, Init, DeInit, MasterSendData API's mentioned in the I2C headerfile.
### Projects
The created libraries are tested with a simple project.
Using SPI protocol, a communication is established between ST microcontroller and Arduino. ST controller has master, which been sending char array data "Hello World" was received at the slave arduino sucessfully.
### Documents
Schematic, Reference manual and datasheet is avilable in the following below links
Reference manual(https://[pages.github.com](https://www.st.com/resource/en/reference_manual/dm00135183-stm32f446xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)/).
Datasheet(https://[pages.github.com](https://www.st.com/resource/en/datasheet/stm32f446re.pdf))/).
Schematic (https://[pages.github.com](https://www.st.com/en/evaluation-tools/nucleo-f446re.html#cad-resources)/).

