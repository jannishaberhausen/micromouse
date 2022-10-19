#ifndef MOUSEMOTION_H
#define	MOUSEMOTION_H

#include "pathfinder.h"

// desired driving speed of the micromouse
// #define BASE_SPEED 15
#define MIN_SPEED 0.1

#ifdef	__cplusplus
extern "C" {
#endif
    
    extern int delay;
    extern int BASE_SPEED;
    
    void setMotionState(direction newState);
    void setRaceMotionState(direction newState);
    int getMotionCompleted();
    int getRaceMotionCompleted();
    void setRaceMotionCompleted();
    int getRotationCompleted();
    int getRaceRotationCompleted();
    void motionFSM();
    void raceMotionFSM();
    
    void controlFixedSpeed(float left_speed, float right_speed);
    
    // Motion functions
    void resetController();
    void driveForward();
    void raceForward();
    void driveRightTurn(int degrees);
    void driveLeftTurn(int degrees);
    void driveControlledRightTurn();
    void raceControlledRightTurn();
    void driveControlledLeftTurn();
    void raceControlledLeftTurn();
    void driveSmoothRightTurn();
    void driveSmoothLeftTurn();
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

