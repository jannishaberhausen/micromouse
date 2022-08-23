
#include <xc.h>

#include "button.h"
#include "IOconfig.h"
#include "timer.h"

/**
 * set up SW1
 */
void initSwitch() {
    //select CN27 for interrupts
    CNEN2bits.CN27IE = 1;
    
    //set priority, reset flag
    _CNIP = 5;
    _CNIF = 0;
}


/**
 * activate CN interrupts
 * 
 * To be used after configuring it via initSwitch().
 */
void startSwitch() {
    _CNIE = 1;
}


//ISR for change notification interrupt
void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
     _CNIF = 0; 

     // Toggle LED2 and LED4
     LED2 = !LED2;
     LED4 = !LED4;
}  
