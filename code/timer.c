
#include <xc.h>

#include "timer.h"
#include "IOconfig.h"
//#include "hw_tests.h"
//#include "mouse_tests.h"
#include "mouse_motion.h"

/**
 * set up TIMER1
 * 
 * Initialize TIMER1 with a specified time, choosing the minimum prescale.
 * Uses a clock frequency of 40MHz.
 * 
 * @param t time in µs
 */
void initTimer(unsigned long int t) {
    
    
    unsigned long ticks = t * 40;           // clock frequency in MHz
    T1CON = 0;                              // ensure Timer 1 is in reset state
    
    // max. clock counter value: 2^16-1
    
    if (ticks < 65535) {
        // prescale 1: ticks < max ctr
        T1CONbits.TCKPS = 0b00;
        PR1 = (int) ticks;
    } else if (ticks < 524280) {
        // prescale 8: ticks < 8*max ctr
        T1CONbits.TCKPS = 0b01;
        PR1 = (int) (ticks >> 3);
    } else if (ticks < 4194240) {
        // prescale 64. ticks < 64*max ctr
        T1CONbits.TCKPS = 0b10;
        PR1 = (int) (ticks >> 6);
    } else {
        // prescale 256
        T1CONbits.TCKPS = 0b11;
        PR1 = (int) (ticks >> 8);
    }
    
    
    T1CONbits.TCS = 0;      // select internal FCY clock source
    T1CONbits.TGATE = 0;    // gated time accumulation disabled
    T1CONbits.TON = 0;      // leave timer disabled initially
    
    IFS0bits.T1IF = 0;      // reset Timer 1 interrupt flag
    IPC0bits.T1IP = 4;      // set Timer1 interrupt priority level to 4
    IEC0bits.T1IE = 1;      // enable Timer 1 interrupt
    
    TMR1 = 0;               // reset Timer 1 value
}


/**
 * activate TIMER1
 * 
 * To be used after configuring it via setupTimer().
 */
void startTimer() {
    T1CONbits.TON = 1;      // start Timer 1
}


// ISR for Timer 1
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;           // reset Timer 1 interrupt flag
    
    // Uncomment the test case you want to run. Only run one at a time,
    // some of them contain conflicting hardware configurations.
    // Later test cases rely on functions tested in the previous ones. 
    // If something doesn't work, everything after that probably won't work either.
    
    // Please double-check whether the motor pins are set correctly while the motor
    // isn't used. I did that, but the motors are too expensive to make a typo.
    // See IOConfig.c
    
    //testTimer();
    //testSwitch();
    //LED2 = SW1;
    //testLedPWM();
    //testMotorPWM();
    //testEncoders();
    //testSensorsLR();
    //testSensorsF();
    //testMotion_P();
    //testMotion_PID(10, 10);
    //testMotion_nested();
    //testMotion_rotate();
    //testMotion_turn();
    //testMouseSlowMotionForward();
    //testMouseSlowMotionForwardEncoderControl();
    //testMotorBreak();
    //testRotation();
    //testMouse180DegreesRotation();
    //testMouseMotionAlongCorridor();
    //testMouseMotionBackAndForthInCorridor();
    //testMouseStopBeforeWall();
    //testMouseOnlyRelyOnOneSensor();
    //testMouseAlwaysFollowRightWall();
    //testRightTurn(360);
    //testStraightCorridor();
    //testRotation();
    motionFSM();
}