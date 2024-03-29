#ifndef HWTEST_H
#define	HWTEST_H

#include "pathfinder.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
    /* flag to be set when running the test function for the first time, so we 
     * can call the corresponding setup functions only when they are needed 
     * (eliminates possibility of destroying stuff we don't even want to test 
     * yet)
     */
    extern int setup;
    
    // for communication with pathfinder
    extern direction mouseState;
    
    void testSwitch();
    void testTimer();
    void testLedPWM();
    void testMotorPWM();
    void testEncoders();
    void testSensorsLR();
    void testSensorsF();
    void testSensorsLRWalls();

#ifdef	__cplusplus
}
#endif

#endif	/* HWTEST_H */

