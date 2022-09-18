#ifndef MOUSETEST_H
#define	MOUSETEST_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    /* define states of finite state machine (FSM) that implements the mouse's
     * behavior.
     */
    enum state {
        FORWARD=0,
        BRAKE=1,
        RIGHT_TURN=2,
        LEFT_TURN=3
    };
    typedef enum state state;
   
    void testSlowMotionForwardNoControl();
    void testSlowMotionForwardEncoderControl();
    
    
    void testMotorBreak();
    void testMouse180DegreesRotation();
    void testStraightCorridor();
    void testMouseMotionBackAndForthInCorridor();
    void testMouseStopBeforeWall();
    void testMouseOnlyRelyOnOneSensor();
    void testMouseAlwaysFollowRightWall();
    
    
    void testRightTurn(int degrees);
    void motorBreak();

#ifdef	__cplusplus
}
#endif

#endif	/* MOUSETEST_H */

