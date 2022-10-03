
#include <xc.h>

#include "button.h"
#include "IOconfig.h"
#include "timer.h"

#include "pathfinder.h"

/**
 * set up SW1
 */
void initSwitch() {
    //select CN27 for interrupts
    CNEN2bits.CN27IE = 1;
    // TODO enable internal pullup
    //CNPU2bits.CN27PUE = 1;
    
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

     // switch to the next planner state
     if(SW1) {
         if (current_state_planner == WAIT_EXPLORE)
             current_state_planner = EXPLORE;
         else if (current_state_planner == WAIT_EXPLOIT)
             current_state_planner = EXPLOIT;
     }
}  
