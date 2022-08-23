#include "xc.h"
#include "IOconfig.h"

#include "pwm.h"
#include "button.h"
#include "motorEncoders.h"
#include "dma.h"
#include "adc.h"
#include "sharp.h"



void setupIO()
{

    int i;
    
    
    //all pins are now digital, by default they are analogue
    AD1PCFGL=0xFFFF;
    
    //sharp sensors at AN2, AN3, and AN4 are analogue
    AD1PCFGLbits.PCFG2 = 0;
    AD1PCFGLbits.PCFG3 = 0;
    AD1PCFGLbits.PCFG4 = 0;
    
    // set LEDs as output
    //TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    //TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB15 = 0;
    
    // UART1 TX as output
    TRISCbits.TRISC1 = 0;
    

    ///////////////////////// PIN MAPPING ///////////////////////////
    
    //before we map, we need to unlock
    // clear bit 6 (unlock, they are usually write protected)
    __builtin_write_OSCCONL(OSCCON & 0xbf);
    
    
    // PERIPHERAL receives data from which INPUT PIN
    
    // UART1 RX on RP16
    RPINR18bits.U1RXR = 16;

    // QE1 on RP20 and RP21
    RPINR14bits.QEA1R = 20;
    RPINR14bits.QEB1R = 21;
    
    // QE2 on RP24 and RP25
    RPINR16bits.QEA2R = 24;
    RPINR16bits.QEB2R = 25;
    
    
    
    
    //OUTPUT PIN receives data from which PERIPHERAL, 
    //see table 11-2 in datasheet to check peripheral codes 
    
    // UART1 TX goes to RP17
    RPOR8bits.RP17R = 0b00011; //output bin RP2 gets data from peripheral U1 TX 

    
    
    
   
    //after mapping we lock again
    // Lock PPS registers (lock again!)
     __builtin_write_OSCCONL(OSCCON | 0x40);
     
    // short dirty delay for changes to take effect
    for (i = 0; i < 30000; i++);

    
}

/////////////////////////////////////////////////////////////////////////////
//    Wrapper functions initializing the necessary hardware modules for    //
//                         high-level functions                            //
/////////////////////////////////////////////////////////////////////////////

/*
 * These functions call the low-level initialization functions whenever the
 * corresponding module is required for the higher-level function.
 * Therefore, some setup functions will be called more often than necessary.
 * This is prefered over calling them manually due to simplicity, and ensures
 * that the modules are always in a known sate.
 * Other setup functions may modify that state (e.g. for PWM that's done all 
 * the time), but not completely reinitialize them if they are not directly
 * needed.
 */



/**
 * Uses LED2 and LED4 as output.
 * 
 * For binary output without PWM, makes it possible to use these LEDs 
 * at the same time as the motors or the switch.
 * 
 * Changes only the LED2/4 pins, 
 * motors, encoders, switch, and sensors are unaffected.
 * 
 * Calls no other setup functions.
 */
void setupLED24() {
    // disable PWM if necessary
    if(PWM1CON1bits.PEN1L == 1) {
        // LED2 is set to PWM. Disable it
        PWM1CON1bits.PEN1L = 0;
    }
    if(PWM1CON1bits.PEN2L == 1) {
        // LED4 is set to PWM. Disable it
        PWM1CON1bits.PEN2L = 0;
    }
}


/**
 * Uses LED2 and LED4 as PWM output.
 * 
 * Can not me used while the motors are active, but can be used 
 * together with the switch.
 * 
 * Changes the LED2/4 output pins and disables. 
 * Encoders, switch and sensors are unaffected.
 * 
 * Calls setupPWM().
 */
void setupLED24_PWM() {
    // reset PWM module
    initPWM();
    
    // disable motors
    MOTENL = 0;
    MOTENR = 0;
    
    // enable PWM on LEDs
    PWM1CON1bits.PEN1L = 1;
    PWM1CON1bits.PEN2L = 1;
}


/**
 * Enables the motors.
 * 
 * Cannot be used together with LED2/4 in PWM mode or together with the switch.
 * 
 * Activates the motors, disables PWM on the LEDs and disables the switch.
 * Encoders and sensors are unaffected.
 * 
 * Calls setupPWM().
 */
void setupMotors() {
    // disable interrupts on the switch
    CNEN2bits.CN27IE = 0;
    // TODO disable internal pullup
    // CNPU2bits.CN27PUE = 0;
    
    // Do we have to change TRISBbits.TRISB5 to output ?
    
    // reset PWM module
    initPWM();
    
    // set motor directions to save, known state
    MOTINL1 = 0;
    MOTINL2 = 0;
    MOTINR1 = 0;
    MOTINR2 = 0;
    
    // enable PWM on the motors
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2H = 1;
}


/**
 * Enables the switch.
 * 
 * Can not be used while the motors are active.
 * Can be used together with the LEDs in any mode.
 * 
 * Enables the switch, disables the motors
 * 
 * Calls initSwitch() and startSwitch()
 */
void setupSwitch() {
    // disable PWM on the motors
    PWM1CON1bits.PEN1H = 0;
    PWM1CON1bits.PEN2H = 0;
    
    // disable motors
    MOTENL = 0;
    MOTENR = 0;
    
    // Activate the switch
    // Do we have to set TRISBbits.TRISB5 to input?
    initSwitch();
    startSwitch();
}


/**
 * Enables the encoders.
 * 
 * Can be used in any configuration.
 * 
 * Only activates the encoders, and probably has to be called only once.
 * 
 * Calls initQEI().
 */
void setupEncoders() {
    initQEI(0, 0);
}


/**
 * Enables the sharp sensors.
 * 
 * Can be used in any configuration.
 * 
 * Only activates the sharp distance sensors, and probably has to be called 
 * only once.
 * 
 * Calls initDMA(), initADC(), and initSharp().
 */
void setupSensors() {
    // start DMA for reading the converted values
    initDMA();
    // start ADC for converting readings
    initADC();
    // start sharp sensors
    initSharp();
}