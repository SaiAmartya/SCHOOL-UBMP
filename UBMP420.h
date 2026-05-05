/*==============================================================================
 File: UBMP420.h
 Date: April 2, 2022
 
 UBMP4.2 (PIC16F1459) symbolic constant and function definitions
 
 I/O pin definitions section:
 Inputs read port registers (e.g. PORTC) and outputs write to port latches
 (e.g. LATC). Use this file to create unique symbolic names for each input
 and output device connected to the I/O pins of the PIC16F1459 of UBMP4.2.
 
 ADC input channel definitions section:
 Definitions representing ADCON0 channel select (CHS) bit for the ADC channels
 available on the UMBP4.2.
 
 Function prototypes section:
 Functions in the UBMP420.c file must be defined before use in the main user
 program. This header file defines function prototypes for functions located
 in the UBMP420.c file.
==============================================================================*/

// TODO - Add/modify user constant definitions for UBMP hardware here.

// PORTA I/O pin definitions
#define PROG        PORTAbits.RA3   // SW1/PROG/Reset (MCLR) pushbutton input
#define MCLR        PORTAbits.RA3   // SW1/PROG/Reset (MCLR) pushbutton input
#define S1          PORTAbits.RA3   // SW1/PROG/Reset (MCLR) pushbutton input
#define SW1         PORTAbits.RA3   // SW1/PROG/Reset (MCLR) pushbutton input
#define BEEPER      LATAbits.LATA4  // Piezo beeper (LS1) output
#define	LS1         LATAbits.LATA4  // Piezo beeper (LS1) output
#define D1          LATAbits.LATA5  // LED D1/Run LED output (active-low)
#define LED1        LATAbits.LATA5  // LED D1/Run LED output (active-low)
#define RUNLED      LATAbits.LATA5  // LED D1/Run LED output (active-low)

// PORTB I/O pin definitions
#define S2          PORTBbits.RB4   // Pushbutton SW2 input
#define SW2         PORTBbits.RB4   // Pushbutton SW2 input
#define S3          PORTBbits.RB5   // Pushbutton SW3 input
#define SW3         PORTBbits.RB5   // Pushbutton SW3 input
#define S4          PORTBbits.RB6   // Pushbutton SW4 input
#define SW4         PORTBbits.RB6   // Pushbutton SW4 input
#define S5          PORTBbits.RB7   // Pushbutton SW5 input
#define SW5         PORTBbits.RB7   // Pushbutton SW5 input

// PORTC I/O pin definitions
#define H1IN        PORTCbits.RC0   // External I/O header H1 input
#define H1OUT       LATCbits.LATC0  // External I/O header H1 output

#define H2IN        PORTCbits.RC1   // External I/O header H2 input
#define H2OUT       LATCbits.LATC1  // External I/O header H2 output

#define H3IN        PORTCbits.RC2   // External I/O header H3 input
#define H3OUT       LATCbits.LATC2  // External I/O header H3 output 
#define IR          PORTCbits.RC2   // IR demodulator (U2) input
#define U2          PORTCbits.RC2   // IR demodulator (U2) input

#define H4IN        PORTCbits.RC3   // External I/O header H4 input
#define H4OUT       LATCbits.LATC3  // External I/O header H4 output
#define Q1          PORTCbits.RC3   // Phototransistor/ambient light sensor (Q1) input

#define H5IN        PORTCbits.RC4   // External I/O header H5 input
#define H5OUT       LATCbits.LATC4  // External I/O header H5 output
#define D2          LATCbits.LATC4  // LED D2 output
#define LED2        LATCbits.LATC4  // LED D2 output

#define H6IN        PORTCbits.RC5   // External I/O header H6 input
#define H6OUT       LATCbits.LATC5  // External I/O header H6 output
#define D3          LATCbits.LATC5  // LED D3 output
#define LED3        LATCbits.LATC5  // LED D3 output
#define D6          LATCbits.LATC5  // LED D6/IR LED output
#define LED6        LATCbits.LATC5  // LED D6/IR LED output
#define IRLED       LATCbits.LATC5  // LED6/IR LED output

#define H7IN        PORTCbits.RC6   // External I/O header H7 input
#define H7OUT       LATCbits.LATC6  // External I/O header H7 output
#define D4          LATCbits.LATC6  // LED D4 output
#define LED4        LATCbits.LATC6  // LED D4 output

#define H8IN        PORTCbits.RC7   // External I/O header H8 input
#define H8OUT       LATCbits.LATC7  // External I/O header H8 output
#define D5          LATCbits.LATC7  // LED D5 output
#define LED5        LATCbits.LATC7  // LED D5 output

// ADC (A-D converter) input channel definitions for read_ADC() function
#define AN4         0b00010000      // A-D converter channel 4 input
#define ANH1        0b00010000      // External H1 header analogue input (Ch4))
#define AN5         0b00010100      // A-D converter channel 5 input
#define ANH2        0b00010100      // External H2 header analogue input (Ch5)
#define AN6         0b00011000      // A-D converter channel 6 input
#define ANH3        0b00011000      // External H3 header analogue input (Ch6)
#define AN7         0b00011100      // A-D converter channel 7 input
#define ANH4        0b00011100      // External H4 header analogue input (Ch7)
#define ANQ1        0b00011100      // Q1 phototransistor A-D input (Ch7)
#define AN8         0b00100000      // A-D converter channel 8 input
#define ANH7        0b00100000      // External H7 header analogue input (Ch8)
#define AN9         0b00100100      // A-D converter channel 9 input
#define ANH8        0b00100100      // External H8 header analogue input (Ch9)
#define AN10        0b00101000      // A-D converter channel 10 input (SW2)
#define AN11        0b00101100      // A-D converter channel 11 input (SW3)
#define ANTIM       0b01110100      // On-die temperature indicator module input

// Clock frequency definition for delay macros and simulation
#define _XTAL_FREQ  48000000        // Set clock frequency for time delays

// Prototypes for UBMP420.c functions:

/**
 * Function: void OSC_config(void)
 * 
 * Configure oscillator for 48 MHz operation (required for USB bootloader).
 */
void OSC_config(void);

/**
 * Function: void UBMP4_config(void)
 * 
 * Configure hardware ports and peripherals for on-board UBMP4 I/O devices.
 */
void UBMP4_config(void);

/**
 * Function: void ADC_config(void)
 * 
 * Configure ADC for 8-bit conversion from on-board phototransistor Q1 (AN7).
 */
void ADC_config(void);

/**
 * Function: void ADC_select_channel(unsigned char channel)
 * 
 * Enable ADC and switch ADC input mux to the channel specified by one of the
 * channel constants defined above.
 * 
 * Example usage: ADC_select_channel(ANTIM);
 */
void ADC_select_channel(unsigned char);

/**
 * Function: unsigned char ADC_read(void)
 * 
 * Convert currently selected channel and return an 8-bit conversion result.
 * 
 * Example usage: light_level = ADC_read();
 */
unsigned char ADC_read(void);

/**
 * Function: unsigned char ADC_read_channel(unsigned char channel)
 * 
 * Enable ADC, switch to the channel specified by one of the channel constants
 * defined above, and return an 8-bit conversion result.
 * 
 * Example usage: light_level = ADC_read_channel(ANQ1);
 */
unsigned char ADC_read_channel(unsigned char);

// TODO - Add additional function prototypes for new functions in UBMP420.c here.
