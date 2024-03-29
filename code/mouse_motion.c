/*
 * File:   mouse_motion.c
 * Author: janni
 *
 * Created on 18. September 2022, 18:57
 */


#include "xc.h"

#include "IOconfig.h"

#include "pwm.h"
#include "motorEncoders.h"
#include "dma.h"
#include "adc.h"
#include "sharp.h"
#include "mouse_motion.h"

#include "pathfinder.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// length of one cell: 18 cm / (6 pi cm / 16*33*4 ticks) = 2016,8 ticks/cell
int length_of_cell = 2017;
// length of one cell: (18cm / 6 pi cm) * 2 pi = 6 rad
//float length_of_cell = 6;
// Wheel rotation for a 90� robot rotation
//float length_of_curve = 2.408553844;
int length_of_curve = 809;

int race_length_of_curve = 1400;

// control parameters
float k_p = 400;
float k_i = 0.5;
float k_d = 0;

float sensor_k_p = 0.003;

// error terms used by the controller. Have to be defined globally
float correction_left = 0;
float correction_right = 0;
float last_error_left = 0;
float last_error_right = 0;

float acc_error_left = 0;
float acc_error_right = 0;

int BASE_SPEED = 15;

// desired velocity in ticks per sample,
// 2112 ticks/rotation at 0.5 rotations/second
// and 100 samples/second => 10.56 ticks per sample
float v_dest_left;
float v_dest_right;

// fraction of base speed of left and right motors
float vbl = 0.04;
float vbr = 0.04;

// counter for dirty delay to avoid infinite spinning in curves.
int delay = 0;

// stores the starting position so we know how far to drive
float start_position = 0;

// used for recalibrating the encoders:
// Wall configuratin at the last measurement
int last_wall_l, last_wall_f, last_wall_r;
// true if more than half of the distance has been driven.
// Then, the sensors look into the new cell
int passed_archway;

// state variable for the motor control FSM
direction motionState = STOP;
direction raceMotionState = STOP;


/*#############################################################################
 ############################## HELPER FUNCTIONS ##############################
 ############################################################################*/

/**
 * Reset the encoders and all error terms.
 * 
 * Has to be called after every state change of the motor controller,
 * otherwise the integral part of the controller can cause problems.
 */
void resetController() {
    
        // unnecessary, maybe remove. But does not hurt to do this.
        setupMotors();
        setupSensors();
        
        // reset encoders
        setupEncoders();
        
        // set motor directions: left fwd, right fwd
        setMotorDirections_Forward();
        
        // reset error terms
        correction_left = 0;
        correction_right = 0;
        last_error_left = 0;
        last_error_right = 0;
        
        acc_error_left = 0;
        acc_error_right = 0;
        
        v_dest_left = BASE_SPEED;
        v_dest_right = BASE_SPEED;
        
        get_walls(&last_wall_l, &last_wall_f, &last_wall_r);
}


/**
 * Distance driven since the last encoder reset.
 * 
 * Returns the distance driven in the current motion, in encoder ticks.
 * Used to check for completion of a motion.
 * 
 * @return the distance driven since the last encoder reset.
 */
int distanceFromEncoderReadings() {
    return ((getPositionInCounts_1() + getPositionInCounts_2()) / 2);
}

/*#############################################################################
 ################# FUNCTIONS DEFINING THE MOTION OF THE MOUSE #################
 ############################################################################*/

/**
 * Controls both motors individually to a desired speed.
 * 
 * Implements a closed-loop PID controller.
 * Helper function for all driveForward functions.
 * 
 * @param v_l Desired speed of the left motor
 * @param v_r Desired speed of the right motor
 */
void controlFixedSpeed(float v_l, float v_r) {
    
    float left = fabs(getVelocityInCountsPerSample_1());
    float right = fabs(getVelocityInCountsPerSample_2());
    
    // P-part
    float error_l = v_l-left;
    float error_r = v_r-right;
    
    // I-part
    acc_error_left += error_l;
    acc_error_right += error_r;
    
    //D-part
    // float d_error_l = error_l - last_error_left;
    // float d_error_r = error_r - last_error_right;
    
    last_error_left = error_l;
    last_error_right = error_r;
    
    // uses the parameters from the top of this file
    correction_left = 800 + error_l * k_p + acc_error_left * k_i; // + d_error_l * k_d;
    correction_right = 800 + error_r * k_p + acc_error_right * k_i; // + d_error_r * k_d;
    
    
    if(correction_left < MOTOR_MAX) {
        if (correction_left < 0) {
            MOTORL = 0;
        } else {
            MOTORL = correction_left;
        }
    } else {
        MOTORL = MOTOR_MAX;
    }
    
    if(correction_right < MOTOR_MAX) {
        if (correction_right < 0) {
            MOTORR = 0;
        } else {
            MOTORR = correction_right;
        }
    } else {
        MOTORR = MOTOR_MAX;
    }
    
}


/**
 * Controls the mouse along a straight line.
 * 
 * Uses no sensor input, only relies on the encoders.
 */
void driveForwardBlind() {
    controlFixedSpeed(BASE_SPEED, BASE_SPEED);
}



/**
 * Controls the mouse's motion along a straight corridor. 
 * 
 * Uses the right and left sensor to stabilize the mouse in the middle of the 
 * corridor. Implements a closed-loop nested controller to do so. 
 * Uses controlFixedSpeed as the inner loop.
 */
void driveForwardAlongCorridor() {
    
    setMotorDirections_Forward();
    
    ////////////////////////////////////////////////////////////////////////
    //            outer control loop: sideways using sensors              //
    ////////////////////////////////////////////////////////////////////////
    
    int sensor_left, unused, sensor_right;
    sharpRaw(&sensor_left, &unused, &sensor_right);
    
    // v_dest ~ 10, sensor diff for 1cm ~ 800,
    // so we have to use *very* small weights
    
    v_dest_left = BASE_SPEED + (sensor_left - sensor_right) * sensor_k_p;
    v_dest_right = BASE_SPEED + (sensor_right - sensor_left) * sensor_k_p;
    
    
    ///////////////////////////////////////////////////////////////////////
    //         inner control loop, bring v_dest to the motors            //
    ///////////////////////////////////////////////////////////////////////
    
    controlFixedSpeed(v_dest_left, v_dest_right);
}


/**
 * Controls the mouse's motion along a straight wall. 
 * 
 * Uses the only the left sensor to stabilize the mouse in the middle of the 
 * corridor. Implements a closed-loop nested controller to do so. 
 * Uses controlFixedSpeed as the inner loop.
 */
void driveForwardAlongLeftWall() {
    
    setMotorDirections_Forward();
    
    ////////////////////////////////////////////////////////////////////////
    //            outer control loop: sideways using sensors              //
    ////////////////////////////////////////////////////////////////////////
    
    int sensor_left, unused, sensor_right;
    sharpRaw(&sensor_left, &unused, &sensor_right);
    
    // v_dest ~ 10, sensor diff for 1cm ~ 800,
    // so we have to use *very* small weights
    
    // (senosr_left - sensor_right) = (sensor_left - (1000-sensor_left))
    // = (2*sensor_left - 1000)
    v_dest_left = BASE_SPEED + (sensor_left - 1000) * sensor_k_p;
    v_dest_right = BASE_SPEED + (1000 - sensor_left) * sensor_k_p;
    
    
    ///////////////////////////////////////////////////////////////////////
    //         inner control loop, bring v_dest to the motors            //
    ///////////////////////////////////////////////////////////////////////
    
    controlFixedSpeed(v_dest_left, v_dest_right);
}


/**
 * Controls the mouse's motion along a straight wall. 
 * 
 * Uses only the right sensor to stabilize the mouse in the middle of the 
 * corridor. Implements a closed-loop nested controller to do so. 
 * Uses controlFixedSpeed as the inner loop.
 */
void driveForwardAlongRightWall() {
    
    setMotorDirections_Forward();
    
    ////////////////////////////////////////////////////////////////////////
    //            outer control loop: sideways using sensors              //
    ////////////////////////////////////////////////////////////////////////
    
    int sensor_left, unused, sensor_right;
    sharpRaw(&sensor_left, &unused, &sensor_right);
    
    // v_dest ~ 10, sensor diff for 1cm ~ 800,
    // so we have to use *very* small weights
    
    // (senosr_left - sensor_right) = (sensor_left - (1000-sensor_left))
    // = (2*sensor_left - 1000)
    v_dest_left = BASE_SPEED + (1000 - sensor_right) * sensor_k_p;
    v_dest_right = BASE_SPEED + (sensor_right - 1000) * sensor_k_p;
    
    
    ///////////////////////////////////////////////////////////////////////
    //         inner control loop, bring v_dest to the motors            //
    ///////////////////////////////////////////////////////////////////////
    
    controlFixedSpeed(v_dest_left, v_dest_right);
}


/**
 * Controls the mouse's motion along a wall on the left side. 
 * 
 * Uses the left and right sensors to stabilize the mouse in the middle of the 
 * corridor. Implements a closed-loop nested controller to do so. 
 * Decides what walls are available at the current position and calls the
 * appropriate driveForward* function.
 */
void driveForward() {
    setMotorDirections_Forward();
    
    // decide which controller to use
    int wall_left, wall_front, wall_right;
    get_walls(&wall_left, &wall_front, &wall_right);
    
    if (wall_left) {
        if (wall_right) {
            // both walls present
            driveForwardAlongCorridor();
        } else {
            // only left wall present
            driveForwardAlongLeftWall();
        }
    } else {
        if(wall_right) {
            // only right wall present
            driveForwardAlongRightWall();
        } else {
            // no walls present
            driveForwardBlind();
        }
    }
}


/**
 * Controls the mouse's motion along a wall on the left side. 
 * 
 * Uses the left and right sensors to stabilize the mouse in the middle of the 
 * corridor. Implements a closed-loop nested controller to do so. 
 * Decides what walls are available at the current position and calls the
 * appropriate driveForward* function.
 */
void raceForward() {
    setMotorDirections_Forward();
    
    // decide which controller to use
    int wall_left, wall_front, wall_right;
    get_walls(&wall_left, &wall_front, &wall_right);
    
    if (wall_left) {
        if (wall_right) {
            // both walls present
            driveForwardAlongCorridor();
        } else {
            // only left wall present
            driveForwardAlongLeftWall();
        }
    } else {
        if(wall_right) {
            // only right wall present
            driveForwardAlongRightWall();
        } else {
            // no walls present
            driveForwardBlind();
        }
    }
}


/**
 * Lets the mouse turn clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveRightTurn(int degrees) {
    delay = 0;
    resetController();
    
    float encoder_start = getAvgPositionInRad();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *0.24
    float rotation_in_rad = (float) degrees * 0.01745 * 1.533333;

    setMotorDirections_RightTurn();

    MOTORL = MIN_SPEED * MOTOR_MAX;
    MOTORR = MIN_SPEED * MOTOR_MAX;

    while (fabs(getAvgPositionInRad() - encoder_start) < rotation_in_rad);
}


/**
 * Lets the mouse turn counter-clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveLeftTurn(int degrees) {
    delay = 0;
    resetController();
    
    float encoder_start = getAvgPositionInRad();
    
    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *1.533
    float rotation_in_rad = (float) degrees * 0.01745 * 1.533333;

    setMotorDirections_LeftTurn();

    MOTORL = MIN_SPEED * MOTOR_MAX;
    MOTORR = MIN_SPEED * MOTOR_MAX;

    while (fabs(getAvgPositionInRad() - encoder_start) < rotation_in_rad);
}


/**
 * Lets the mouse turn clockwise by 90 degrees.
 */
void driveControlledRightTurn() {
    delay = 0;
    
    setMotorDirections_RightTurn();

    controlFixedSpeed(15, 15);
}


/**
 * Lets the mouse turn clockwise by 90 degrees.
 */
void raceControlledRightTurn() {
    /* wait until right turn becomes available 
     * (if encoder ticks send the mouse too short)
     */
    int wall_left, wall_front, wall_right;
    get_walls(&wall_left, &wall_front, &wall_right);
    //while (wall_right);
    
    controlFixedSpeed(BASE_SPEED * 0.5 * 4.5, BASE_SPEED * 0.5);
}


/**
 * Lets the mouse perform a smooth right turn. 
 */
void driveControlledLeftTurn() {
    delay = 0;
    
    setMotorDirections_LeftTurn();

    controlFixedSpeed(15, 15);
}


/**
 * Lets the mouse perform a smooth left turn.
 */
void raceControlledLeftTurn() {
    /* wait until right turn becomes available 
     * (if encoder ticks send the mouse too short)
     */
    int wall_left, wall_front, wall_right;
    get_walls(&wall_left, &wall_front, &wall_right);
    //while (wall_left);
    
    controlFixedSpeed(BASE_SPEED * 0.5, BASE_SPEED * 0.5 * 4.5);
}


/**
 * Slows the mouse down quickly by turning the motors backward. This results in 
 * a faster stop as compared to letting the wheels spin freely. Exit the 
 * function when the remaining wheel speed is below a threshold
 */
void brake() {
    setMotorDirections_Disable();
    MOTORL = 0;
    MOTORR = 0;
}

/*#############################################################################
 ###################### STATE TRANSITION CHECKS FOR FSM #######################
 ############################################################################*/

/**
 * Checks whether there is a wall of the mouse. This can be a dead end or a 
 * right and/or left corner.
 * 
 * @param
 *      front (int): distance measurement from front sensor
 * @return
 *      1 = there is a wall ahead, 0 = there is no wall ahead
 */
int checkForWallAhead(int front) {
    if(front > 1000){
        return 1;
    } else {
        return 0;
    }
}


/**
 * Checks whether there is a corner to the right. Does not exclude the 
 * possibility of a corner to the left or other type of junctions.
 * 
 * @return
 *      1 = there is a corner to the right, 0 = there is no corner to the right
 */
int checkForRightCorner() {
    int left, right, front;
    get_walls(&left, &front, &right);
    return right;
}


/**
 * Checks whether there is a corner to the left. Does not exclude the 
 * possibility of a corner to the right or other type of junctions.
 * 
 * @return
 *      1 = there is a corner to the left, 0 = there is no corner to the left
 */
int checkForLeftCorner() {
    int left, right, front;
    get_walls(&left, &front, &right);
    return left;
}

/*#############################################################################
 ############################## HARDWARE MACROS ###############################
 ############################################################################*/

/**
 * Sets the direction of the motors to disable.
 */
void setMotorDirections_Disable() {
    MOTINR1 = 1;
    MOTINR2 = 1;
    MOTINL1 = 1;
    MOTINL2 = 1;
}

/**
 * Sets the direction of the motors to both go forward.
 */
void setMotorDirections_Forward() {
    setMotorDirectionLeft_Forward();
    setMotorDirectionRight_Forward();
}

/**
 * Sets the direction of the motors such that the mouse turns right.
 * Left wheel goes forward. Right wheel goes backward.
 */
void setMotorDirections_RightTurn() {
    setMotorDirectionLeft_Forward();
    setMotorDirectionRight_Backward();
}

/**
 * Sets the direction of the motors such that the mouse turns left.
 * Left wheel goes backward. Right wheel goes forward.
 */
void setMotorDirections_LeftTurn() {
    setMotorDirectionLeft_Backward();
    setMotorDirectionRight_Forward();
}

/**
 * Sets the direction of the motors to both go backward.
 */
void setMotorDirections_Backward() {
    setMotorDirectionLeft_Backward();
    setMotorDirectionRight_Backward();
}

/**
 * Sets the direction of the left motor to go forward.
 */
void setMotorDirectionLeft_Forward() {
    MOTINL1 = 1;
    MOTINL2 = 0;
}

/**
 * Sets the direction of the left motor to go backward.
 */
void setMotorDirectionLeft_Backward() {
    MOTINL1 = 0;
    MOTINL2 = 1;
}

/**
 * Sets the direction of the right motor to go forward.
 */
void setMotorDirectionRight_Forward() {
    MOTINR1 = 1;
    MOTINR2 = 0;
}

/**
 * Sets the direction of the right motor to go backward.
 */
void setMotorDirectionRight_Backward() {
    MOTINR1 = 0;
    MOTINR2 = 1;
}

/*#############################################################################
 ################## HELPER FUNCTIONS OF FINITE STATE MACHINE ##################
 ############################################################################*/

/**
 * Setter for the state of the motor control FSM during explore phase.
 * 
 * Keeps track of the previously executed motions, and resets the controller
 * and the driven distance where necessary. Therefore, don't set the state
 * manually!
 */
void setMotionState(direction newState) {
    if (newState != motionState)
        resetController();
    // always remember the original position to know when to stop
    start_position = getAvgPositionInCounts();
    passed_archway = 0;
    // reset encoders
    //setupEncoders();
    motionState = newState;
}

/**
 * Setter for the state of the motor control FSM during explore phase.
 * 
 * Keeps track of the previously executed motions, and resets the controller
 * and the driven distance where necessary. Therefore, don't set the state
 * manually!
 */
void setSaveRaceState(direction newState, direction nextState) {
    if (newState != motionState)
        resetController();
    // always remember the original position to know when to stop
    start_position = getAvgPositionInCounts();
    passed_archway = 0;
    // reset encoders
    //setupEncoders();
    motionState = newState;
}

/**
 * Setter for the state of the motor control FSM during exploit phase.
 * 
 * Keeps track of the previously executed motions, and resets the controller
 * and the driven distance where necessary. Therefore, don't set the state
 * manually!
 */
void setRaceMotionState(direction newState) {
    //if (newState != raceMotionState) {
        resetController();
    //}
    // always remember the original position to know when to stop
    start_position = getAvgPositionInCounts();
    raceMotionState = newState;
}

/**
 * Function used by the motion planner to check completion of a motion during 
 * explore phase.
 * 
 * @return 1 if completed, 0 otherwise.
 */
int getRotationCompleted() {
    switch(motionState) {
        case LEFT:
            return (getAvgPositionInCounts() - start_position) > length_of_curve;
        case RIGHT:
            return (getAvgPositionInCounts() - start_position) > length_of_curve;
        case BACK:
            return (getAvgPositionInCounts() - start_position) > 2*length_of_curve;
        default:
            return 0;
    }
}


/**
 * Function used by the motion planner to check completion of a motion during 
 * explore phase.
 * 
 * @return 1 if completed, 0 otherwise.
 */
int getMotionCompleted() {

    int unused, distance_front;
    sharpRaw(&unused, &distance_front, &unused);

    switch(motionState) {
        case FRONT:
            // Recalibrate on front walls ony if there is a wall now
            if (distance_front > 650) {
                // We are driving too far, brake!
                if (distance_front > 1200) {
                    // no more forward motion now !!! 
                    return 1;
                } else {
                    // Drive until we go to the first case
                    // disable motionComplete-mechanism
                    return 0;
                }
            }

            return (distanceFromEncoderReadings() - start_position) > length_of_cell;
        case STOP:
            return 1;
        default:
            return 0;
    }
}


/**
 * Function used by the motion planner to check completion of a motion during 
 * exploit phase.
 * 
 * @return 1 if completed, 0 otherwise.
 */
int getRaceRotationCompleted() {
    switch(raceMotionState) {
        case LEFT:
            return (getAvgPositionInCounts() - start_position) > length_of_curve;
        case RIGHT:
            return (getAvgPositionInCounts() - start_position) > length_of_curve;
        case BACK:
            return (getAvgPositionInCounts() - start_position) > 2*length_of_curve;
        default:
            return 0;
    }
}


/**
 * Function used by the motion planner to check completion of a motion during 
 * exploit phase.
 * 
 * @return 1 if completed, 0 otherwise.
 */
int getRaceMotionCompleted() {

    // Recalibrate using the front wall
    int distance_left, distance_front, distance_right;
    sharpRaw(&distance_left, &distance_front, &distance_right);
    int wall_left = distance_left > 650;
    int wall_right = distance_right > 650;

    switch(raceMotionState) {
        case FRONT:
            // Recalibrate on front walls ony if there is a wall now
            if (distance_front > 400) {
                // We are driving too far, brake!
                if (distance_front > 500) {
                    // no more forward motion now !!! 
                    return 1;
                } else {
                    // Drive until we go to the first case
                    // disable motionComplete-mechanism
                    return 0;
                }
            }
            if(last_wall_l != wall_left || last_wall_r != wall_right) {
                start_position = getAvgPositionInCounts() - length_of_cell + 504;
            }
            last_wall_l = wall_left;
            last_wall_r = wall_right;
            return (getAvgPositionInCounts() - start_position) > length_of_cell;
        case LEFT:
            return (getAvgPositionInCounts() - start_position) > race_length_of_curve;
        case RIGHT:
            return (getAvgPositionInCounts() - start_position) > race_length_of_curve;
        case STOP:
            return 1;
        case ROTATE_RIGHT:
            return (getAvgPositionInCounts() - start_position) > length_of_curve;
            break;
        case ROTATE_LEFT:
            return (getAvgPositionInCounts() - start_position) > length_of_curve;
            break;
        case HALF_FRONT:
            return (distanceFromEncoderReadings() - start_position) > length_of_cell/2;
            break;
        default:
            return 0;
    }
}

/*#############################################################################
 ##################### MOTION STATE MACHINES OF THE MOUSE #####################
 ############################################################################*/

/**
 * Main function of the motor control.
 * 
 * Implements the finite state machine, and should be called by the timer ISR.
 * Instructions should be sent to the motor control unit by changing the state 
 * variable using setMotionState.
 */
void motionFSM() {
    delay ++;
    if(delay < 50 && (motionState == LEFT || motionState == RIGHT)) {
        //setMotionState(FRONT);
    }
    
    
    // Rotation functions use busy waiting - completion check for the
    // motion planner and multiple calls during one motion are only relevant
    // for forward control!
    
    switch (motionState) {
        case FRONT:
            // drive forward, be done.
            driveForward();
            break;
        case RIGHT:
            // first rotate, move as next step
            driveControlledRightTurn();
            if(getRotationCompleted()) {
                resetController();
                setMotionState(FRONT);
            }
            break;
        case LEFT:
            // first rotate, move as next step
            driveControlledLeftTurn();
            if(getRotationCompleted()) {
                resetController();
                setMotionState(FRONT);
            }
            break;
        case BACK:
            // first rotate, move as next step
            driveControlledRightTurn();
            if(getRotationCompleted()) {
                resetController();
                setMotionState(FRONT);
            }
            break;
        case STOP:
            // completed.
            brake();
            resetController();
            break;
        case EMPTY:
            break;
        default:
            break;
    }
}


/**
 * Main function of the motor control during exploit phase.
 * 
 * Implements the finite state machine, and should be called by the timer ISR.
 * Instructions should be sent to the motor control unit by changing the state 
 * variable using setRaceMotionState.
 */
void raceMotionFSM() {
    switch (raceMotionState) {
        case FRONT:
            // race forward, be done.
            raceForward();
            break;
        case RIGHT:
            // smooth 90 degrees right turn, driving from entry to exit of cell
            raceControlledRightTurn();
            break;
        case LEFT:
            // smooth 90 degrees left turn, driving from entry to exit of cell
            raceControlledLeftTurn();
            break;
        case STOP:
            // completed.
            brake();
            resetController();
            break;
        case EMPTY:
            break;
        case ROTATE_RIGHT:
            // rotate 90 degrees to the right (only needed in first state of exploit phase)
            driveControlledRightTurn();
            break;
        case ROTATE_LEFT:
            // rotate 90 degrees to the left (only needed in first state of exploit phase)
            driveControlledLeftTurn();
            break;
        case HALF_FRONT:
            // drive forward by half a cell
            raceForward();
            break;
        default:
            break;
    }
}