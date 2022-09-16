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
        ROTATE=2,
        RIGHT_TURN=3,
        LEFT_TURN=4
    };
    typedef enum state state;
    
    void testSwitch();
    void testTimer();
    void testLedPWM();
    void testMotorPWM();
    void testEncoders();
    void testSensorsLR();
    void testSensorsF();
    void testMotion_P();
    void testMotion_sharp();
    void testMotion_rotate();
    void testMotion_turn();
    void testMouseSlowMotionForward();
    void testMouseSlowMotionForwardEncoderControl();
    void testMotorBreak();
    void testMouse180DegreesRotation();
    void testMouseMotionAlongCorridor();
    void testMouseMotionBackAndForthInCorridor();
    void testMouseStopBeforeWall();
    void testMouseOnlyRelyOnOneSensor();
    void testMouseAlwaysFollowRightWall();
    
    
    void testRightTurn(int degrees);
    void motorBreak();

#ifdef	__cplusplus
}
#endif

#endif	/* HWTEST_H */

