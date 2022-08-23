#ifndef TIMER_H
#define	TIMER_H

#ifdef	__cplusplus
extern "C" {
#endif

    void initTimer(unsigned long int t);
    void startTimer();
    
    // test case to execute next
    extern int testNr;
    // flag to be set when changing the test case, so we can run the
    // corresponding setup functions ony when they are needed
    // (eliminates possibility of destroying stuff we don't even want to test yet)
    extern int setup;


#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_H */

