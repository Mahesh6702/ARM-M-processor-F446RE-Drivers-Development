

#include<stdint.h>


int main(void)
{

	uint32_t *pClkCtrlReg =   (uint32_t*)0x40023830;// memory location rcc for Gpio port A
	uint32_t *pPortAModeReg = (uint32_t*)0x40020000 ; // memory location GpioA peripherial register with enabling mode 
	uint32_t *pPortAOutReg =  (uint32_t*)0x40020014;  // memory location of GpioA output registers


	// 1. enable the clock for GPIOA peripheral in the AHB1ENR
	/*
	 *
	 * unint32_t temp = *pClkCtrlReg; // reading operation
	   temp = temp | 0x01; // modify for enabling the GPIOA clock
	  *pClkCtrlReg  = temp; // write back into original variable

	*/
	 // in short we can write the above code as
	//*pClkCtrlReg |= 0x01;
	 *pClkCtrlReg |= 0x01;
			 ;

	// 2. Configure the clock for GPIOA peripheral in the AHB!ENR
	//a. clear the 10th and 11th pins
	//*pPortAModeReg &= 0xFFFFF3FF;
	 *pPortAModeReg &= ~(3 << 10);


	//b. setting the above 10th pin
	//*pPortAModeReg |=  0x00000400;
	 *pPortAModeReg |= (1 << 10);

	// 3. Set the 5th bit of the output data register to make I/O pin-12 HIGH
	  //*pPortAOutReg |= 0x0020;
	  *pPortAOutReg |= (1 << 5);

	while(1);
}

