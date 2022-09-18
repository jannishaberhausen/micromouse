#ifndef MOUSEMOTION_H
#define	MOUSEMOTION_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    // fraction of base speed of left and right motors
    float vbl = 0.04;
    float vbr = 0.04;
    
    // Motion functions
    void driveForward(int distance_in_cm, short number_of_cells);
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
    int distanceFromEncoderReadings();

#ifdef	__cplusplus
}
#endif

#endif	/* MOUSEMOTION_H */

