#ifndef HWTEST_H
#define	HWTEST_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    // flag to be set when running the test function for the first time, 
    // so we can call the corresponding setup functions only when they are needed
    // (eliminates possibility of destroying stuff we don't even want to test yet)
    extern int setup;
    
    enum state {
        FORWARD=0,
        BREAK=1,
        ROTATE=2
    };
    typedef enum state state;
    
    void testSwitch();
    void testTimer();
    void testLedPWM();
    void testMotorPWM();
    void testEncoders();
    void testSensorsLR();
    void testSensorsF();
<<<<<<< HEAD
    void testMouseSlowMotionForward();
    void testMouseSlowMotionForwardEncoderControl();
    void testMotorBreak();
    void testMouse180DegreesRotation();
    void testMouseMotionAlongCorridor();
    void testMouseMotionBackAndForthInCorridor();
=======
    void testMotion_P();
    void testMotion_sharp();
    void testMotion_rotate();
    void testMotion_turn();

>>>>>>> 295c5fbbc24fdaceba247710da0528fdeb39717c

#ifdef	__cplusplus
}
#endif

#endif	/* HWTEST_H */

