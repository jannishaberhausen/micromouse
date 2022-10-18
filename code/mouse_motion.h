#ifndef MOUSEMOTION_H
#define	MOUSEMOTION_H

#include "pathfinder.h"

// desired driving speed of the micromouse
#define BASE_SPEED 15
#define MIN_SPEED 0.1

#ifdef	__cplusplus
extern "C" {
#endif
    
    extern int delay;
    
    void setMotionState(direction newState);
    int getMotionCompleted();
    int getRotationCompleted();
    void motionFSM();
    
    void controlFixedSpeed(float left_speed, float right_speed);
    
    // Motion functions
    void resetController();
    void driveForward();
    void driveRightTurn(int degrees);
    void driveLeftTurn(int degrees);
    void driveControlledRightTurn(int degrees);
    void driveSmoothRightTurn(int degrees);
    void driveSmoothLeftTurn(int degrees);
    void brake();

    // State Transition Checks 
    int checkForWallAhead(int front);
    int checkForRightCorner();
    int checkForLeftCorner();
    int checkForGoal();

    // Hardware Macros
    void setMotorDirections_Disable();
    void setMotorDirections_Forward();
    void setMotorDirections_RightTurn();
    void setMotorDirections_LeftTurn();
    void setMotorDirections_Backward();
    void setMotorDirectionLeft_Forward();
    void setMotorDirectionLeft_Backward();
    void setMotorDirectionRight_Forward();
    void setMotorDirectionRight_Backward();
    int distanceFromEncoderReadings();

#ifdef	__cplusplus
}
#endif

#endif	/* MOUSEMOTION_H */

