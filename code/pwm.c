
#include <xc.h>

#include "pwm.h"
#include "IOconfig.h"


/**
 * set up PWM1
 * 
 * Uses a frequency of 250Hz.
 */
void initPWM()
{
    /* 
     * PWM1:
     * Using FCY = 40MHz, FPWM = 250Hz, the base time is
     * P1TPER = FCY / (FPWM * TCKPS) -1 = 9999
     * where the prescale TCKPS=16 is chosen to fit P1TPER into 15bit.
     */
    P1TCONbits.PTEN = 0; // Switch off PWM generator
    P1TCONbits.PTCKPS = 0b10; // Sets prescaler, available are 1(00),4(01),16(10) or 64(11)
    P1TPER = 9999; //15 bit register (Time Base Period )
    PWM1CON1bits.PMOD1 = 1; // set PWM unit 1 to independent mode
    PWM1CON1bits.PMOD2 = 1;
    PWM1CON1bits.PMOD3 = 1;
    // diable all PWM pins
    PWM1CON1bits.PEN1H = 0; // disable PWM driver PWM1H1 (MOTORL, LED1)
    PWM1CON1bits.PEN2H = 0; // disable PWM driver PWM1H2 (MOTORR, LED3)
    PWM1CON1bits.PEN3H = 0; // disable PWM driver
    PWM1CON1bits.PEN1L = 0; // disable PWM driver PWM1L1 (LED2)
    PWM1CON1bits.PEN2L = 0; // disable PWM driver PWM1L2 (LED4)
    PWM1CON1bits.PEN3L = 0; // disable PWM driver

    P1TCONbits.PTEN = 1; // Switch on PWM generator
    // zero duty cycle initially
    P1DC1 = 0;
    P1DC2 = 0;
    P1DC3 = 0;
}

