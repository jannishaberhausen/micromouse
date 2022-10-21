/// Configuration Bits---------------------------

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Start with Internal RC Oscillator
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS1            // Watchdog Timer Postscaler (1:1)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = ON              // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


/// Include headers-------------------------------
#include "xc.h"
#include "IOconfig.h"
#include "button.h"
#include "timer.h"
#include "pwm.h"
#include "adc.h"
#include "dma.h"
#include "pathfinder.h"
#include "pathfinder_tests.h"
#include "mouse_motion.h"


int main() 
{
    
     /*
      * oscillator setup --------------------------------------------------
      * The external oscillator runs at 20MHz
      * PLL is used to generate 80MHz clock (FOSC)
      * The relationship between oscillator and cycle frequency: FCY = FOSC/2
      * Have a look at "PLL Configuration" paragraph in the mcu manual
      * 
      * N1 = 3  to reach VCO input between 0.8 .. 8MHz          (20MHz / 3 = 6.667MHz)
      * M  = 24 to reach VCO output between 100 .. 200MHz       (6.667MHz * 24 = 160MHz)
      * N2 = 2  to reach output frequency between 12.5 .. 80MHz (160MHz / 2 = 80MHz)
      * 
      * Result: FCY = 0.5 * (20MHz*24/(3*2)) = 40 MIPS, Tcycle=25nsec
      * ------------------------------------------------------------------*/
    PLLFBDbits.PLLDIV = 22;           //set PPL to M=24 (22+2)
    CLKDIVbits.PLLPRE = 1;            //N1 = input/3
    CLKDIVbits.PLLPOST = 0;           //N2 = output/2    
    
        /* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH( 0x03 );            // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    
    while( OSCCONbits.COSC != 0b011 );
    
    // In reality, give some time to the PLL to lock
    while (OSCCONbits.LOCK != 1); //Wait for PLL to lock
 
    setupIO(); //configures inputs and outputs
    
    //set up the 10ms timer used as base for all tests.
    /*
     * Test cases are implemented in the timer ISR. More details in timer.c
     */
    initTimer(10000);
    
    setupLED24();
    
    setupMotors();
    setupSensors();
    setupEncoders();
    resetController();
    
    // start the timer.
    /*
     * Uncomment the test case you want to run in the timer ISR.
     */
    startTimer();
    
    //testMotionSequence();
    //testMouseRaceSequence();
    
    // PLANNER STATE MACHINE
    plannerFSM();
    while(1)
    {
        
       
    }
    
    return 0;
}

