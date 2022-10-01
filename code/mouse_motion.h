#ifndef MOUSEMOTION_H
#define	MOUSEMOTION_H

// desired driving speed of the micromouse
#define BASE_SPEED 10

#ifdef	__cplusplus
extern "C" {
#endif
    
    // Motion functions
    void driveForward();
    void driveRightTurn(int degrees);
    void driveLeftTurn(int degrees);
    void brake();

    // State Transition Checks 
    int checkForWallAhead(int front);
    int checkForRightCorner(int right);
    int checkForLeftCorner(int left);
    int checkForGoal();

    // Hardware Macros
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

