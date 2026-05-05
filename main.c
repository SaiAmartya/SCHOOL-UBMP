/*==============================================================================
 Project: Lab 1
 
This lab allows us to explore the functionality of the UBMP specifically
looking at using the buttons as inputs and using the lights and buzzer as
outputs
==============================================================================*/

#include    "xc.h"              // Microchip XC8 compiler include file
#include    "stdint.h"          // Include integer definitions
#include    "stdbool.h"         // Include Boolean (true/false) definitions

#include    "UBMP420.h"         // Include UBMP4.2 constants and functions

//Initialize variables here!!!
int loop = 0;

// TODO Set linker ROM ranges to 'default,-0-7FF' under "Memory model" pull-down.
// TODO Set linker code offset to '800' under "Additional options" pull-down.

// The main function is required, and the program begins executing from here.

int main(void)
{

    // Configure oscillator and I/O ports. These functions run once at start-up.
    OSC_config();               // Configure internal oscillator for 48 MHz
    UBMP4_config();             // Configure on-board UBMP4 I/O devices
    
    // Code in this while loop runs repeatedly.
    while(1)
	{
            
        if(SW2 == 0)
        {
            loop = loop + 1;
            if(loop > 4)
            {
                loop = 1;
            }
        }
        else
        {
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
            LED5 = 0;
            
            if(loop == 1) LED2 = 1;
            if(loop == 2) LED3 = 1;
            if(loop == 3) LED4 = 1;
            if(loop == 4) LED5 = 1;
        }

        // Activate bootloader if SW1 is pressed.
        if(SW1 == 0)
        {
            RESET();
        }
    }
}
