
#include <stdint.h>



int main(void)
{


	uint32_t *pClkCtrlReg =   (uint32_t*)0x40023830;// memory location rcc for Gpio port A
	uint32_t *pPortAModeReg = (uint32_t*)0x40020000 ; // memory location GpioA peripherial register with enabling mode
	uint32_t *pPortAOutReg =  (uint32_t*)0x40020014;  // memory location of GpioA output registers

	uint32_t *pPortCModeReg = (uint32_t*)0x40020800 ; // memory location GpioC peripherial register with enabling mode
	uint32_t *pPortCInReg =  (uint32_t*)0x40020810;  // memory location of GpioC Input registers


//Enabling the LDR2 on board which is connected to PA5 pin

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
            // 1. enable the clock for GPIOC peripheral in the AHB1ENR
            *pClkCtrlReg |= (1 << 2); // setting the GPIOC bit which is the 2nd bit position in RCC


    // 2. Enable the PA5 pin mode as output mode

	  //a. clear the 10th and 11th pins
	      //*pPortAModeReg &= 0xFFFFF3FF;
	      *pPortAModeReg &= ~(3 << 10); // PA5 pin bit position are 10,11 are clearing in output register


	  //b. setting the above 10th pin

	      //*pPortAModeReg |=  0x00000400;
	       *pPortAModeReg |= (1 << 10); // setting PA5 pin bit position are 10 in output register

	       // 2.Enable the PC13 pin as input mode
	        *pPortCModeReg &= ~(3 << 26); // clearing PC13 pin bit position are 26,27 in mode register



    // 3. Enable the PA5 pin as output write
	        // *pPortAOutReg |= 0x0020;
	         //  *pPortAOutReg |= (1 << 5);
	        *pPortCInReg &= 0x00000000;

//Enabling the push button which is connected to pin PC13

	        while(1)
	        {
	        	// 3. Enable the PC13 pin as input read
	        	//*pPortCInReg |=0x00002000;
	        	   uint32_t button_status = (uint32_t)(*pPortCInReg & (1 << 13));

		        if(button_status )
		             {

	                // 3. Enable the PA5 pin as output write(LD2)
	      	        //*pPortAOutReg |= 0x0020;
	      	            *pPortAOutReg |= (1 << 5); // On  the LED2

		             }

		           else
		              {
		        	     *pPortAOutReg &= ~(1 << 5); // OFF the  LED2

		               }

	            }

}
