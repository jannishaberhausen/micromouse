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


// control parameters
float k_p = 150;
float k_i = 2;
float k_d = 0;


// error terms used by the controller. Have to be defined globally
float correction_left = 0;
float correction_right = 0;
float last_error_left = 0;
float last_error_right = 0;

float acc_error_left = 0;
float acc_error_right = 0;

// desired velocity in ticks per sample,
// 2112 ticks/rotation at 0.5 rotations/second
// and 100 samples/second => 10.56 ticks per sample
float v_dest_left = BASE_SPEED;
float v_dest_right = BASE_SPEED;


// fraction of base speed of left and right motors
float vbl = 0.04;
float vbr = 0.04;

// counter for dirty delay to avoid infinite spinning in curves.
int delay = 0;

// stores the starting position so we know how far to drive
int start_position = 0;

// state variable for the motor control FSM
direction motionState;


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
    
    float left = getVelocityInCountsPerSample_1();
    float right = getVelocityInCountsPerSample_2();
    
    // P-part
    float error_l = v_l-left;
    float error_r = v_r-right;
    
    // I-part
    acc_error_left += error_l;
    acc_error_right += error_r;
    
    //D-part
    float d_error_l = error_l - last_error_left;
    float d_error_r = error_r - last_error_right;
    
    last_error_left = error_l;
    last_error_right = error_r;
    
    // uses the parameters from the top of this file
    correction_left = error_l * k_p + acc_error_left * k_i + d_error_l * k_d;
    correction_right = error_r * k_p + acc_error_right * k_i + d_error_r * k_d;
    
    
    if(BASE_SPEED + correction_left < MOTOR_MAX)
        MOTORL = BASE_SPEED + correction_left;
    
    if(BASE_SPEED + correction_right < MOTOR_MAX)
        MOTORR = BASE_SPEED + correction_right;
    
}



/**
 * Drives the wheels to a fixed position.
 */
void controlFixedPosition(float x_l, float x_r) {
    
    
    float left = getPositionInRad_1();
    float right = getPositionInRad_2();
    
    // P-part
    float error_l = x_l-left;
    float error_r = x_r-right;
    
    // I-part
    acc_error_left += error_l;
    acc_error_right += error_r;
    
    //D-part
    float d_error_l = error_l - last_error_left;
    float d_error_r = error_r - last_error_right;
    
    last_error_left = error_l;
    last_error_right = error_r;
    
    // uses the parameters from the top of this file
    correction_left = error_l * k_p + acc_error_left * k_i + d_error_l * k_d;
    correction_right = error_r * k_p + acc_error_right * k_i + d_error_r * k_d;
    
    
    if(BASE_SPEED + correction_left < MOTOR_MAX)
        MOTORL = BASE_SPEED + correction_left;
    
    if(BASE_SPEED + correction_right < MOTOR_MAX)
        MOTORR = BASE_SPEED + correction_right;
    
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
    
    v_dest_left = 10 + (sensor_left - sensor_right) * 0.003;
    v_dest_right = 10 + (sensor_right - sensor_left) * 0.003;
    
    
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
    v_dest_left = 10 + (sensor_left - 1000) * 0.003;
    v_dest_right = 10 + (1000 - sensor_left) * 0.003;
    
    
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
    v_dest_left = 10 + (1000 - sensor_right) * 0.003;
    v_dest_right = 10 + (sensor_right - 1000) * 0.003;
    
    
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
 * Lets the mouse turn clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveRightTurn(int degrees) {
    delay = 0;
    
    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *0.24
    float rotation_in_rad = (float) degrees * 0.01745 * 0.98685488;

    setMotorDirections_RightTurn();

    MOTORL = 0.15 * MOTOR_MAX;
    MOTORR = 0.15 * MOTOR_MAX;

    while (abs(getPositionInRad_2() - encoder_start) < rotation_in_rad);
}


/**
 * Lets the mouse turn counter-clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveLeftTurn(int degrees) {
    delay = 0;
    
    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *1.55
    float rotation_in_rad = (float) degrees * 0.01745 * 0.98685488;

    setMotorDirections_LeftTurn();

    MOTORL = 0.15 * MOTOR_MAX;
    MOTORR = 0.15 * MOTOR_MAX;

    while (abs(getPositionInRad_2() - encoder_start) < rotation_in_rad);
}


void driveSmoothRightTurn(int degrees) {
    delay = 0;
    
    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *1.55
    float rotation_in_rad = (float) degrees * 0.01745 * 0.98685488;

    setMotorDirections_Forward();

    // ratio v_left_wheel to v_right_wheel: 3.52 to 1
    MOTORL = 0.1 * 3.52 * MOTOR_MAX;
    MOTORR = 0.1 * MOTOR_MAX;

    while (abs(getPositionInRad_2() - encoder_start) < rotation_in_rad);
    
    MOTORL = 0;
    MOTORR = 0;
}


void driveSmoothLeftTurn(int degrees) {
    delay = 0;
    
    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *1.55
    float rotation_in_rad = (float) degrees * 0.01745 * 0.98685488;

    setMotorDirections_Forward();

    // ratio v_left_wheel to v_right_wheel: 1 to 3.52
    MOTORL = 0.1 * MOTOR_MAX;
    MOTORR = 0.1 * 3.52 * MOTOR_MAX;

    while (abs(getPositionInRad_2() - encoder_start) < rotation_in_rad);
    
    MOTORL = 0;
    MOTORR = 0;
}


/**
 * Slows the mouse down quickly by turning the motors backward. This results in 
 * a faster stop as compared to letting the wheels spin freely. Exit the 
 * function when the remaining wheel speed is below a threshold
 */
void brake() {
    // define errors as remaining velocity in the wheels
    int error_left = getVelocityInCountsPerSample_1();
    int error_right = getVelocityInCountsPerSample_2();
    
    // TODO: have not verified thresholds!!!
    if(error_left < 100 && error_right < 100) {
        return;
    }

    // set motor directions according to the errors
    if (error_left > 0) {
        setMotorDirectionLeft_Backward();
    } else {
        setMotorDirectionLeft_Forward();
    }

    if (error_right > 0) {
        setMotorDirectionRight_Backward();
    } else {
        setMotorDirectionRight_Forward();
    }

    // factor for P control
    float kp = 1;

    // update motor speeds based on error readings
    MOTORL += kp * error_left;
    MOTORR -= kp * error_right;
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


/**
 * Checks whether the mouse has reached the goal.
 * 
 * @return
 *      1 = goal reached, 0 = goal not reached
 */
int checkForGoal() {
    
}


/*#############################################################################
 ############################## HARDWARE MACROS ###############################
 ############################################################################*/

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


/**
 * Distance driven since the last encoder reset.
 * 
 * Returns the distance driven in the current motion, in encoder ticks.
 * Used to check for completion of a motion.
 * 
 * @return the distance driven since the last encoder reset.
 */
int distanceFromEncoderReadings() {
    return ((getPositionInCounts_1() + getPositionInCounts_2()) / 2)
                - start_position;
}


/**
 * Setter for the state of the motor control FSM.
 * 
 * Keeps track of the previously executed motions, and resets the controller
 * and the driven distance where necessary. Therefore, don't set the state
 * manually!
 */
void setMotionState(direction newState) {
    
    // Actions only necessary when changing to forward control.
    if(newState == FRONT) {
        if (motionState != FRONT) {
            // If we were rotating before, reset the controller to avoid 
            // messing up the integral part
            resetController();
        }
        // always remember the original position to know when to stop
        start_position = (getPositionInCounts_1()+getPositionInCounts_2()) / 2;
    }
    motionState = newState;
}


/**
 * Function used by the motion planner to check completion of a motion.
 * 
 * @return 1 if completed, 0 otherwise.
 */
int getMotionCompleted() {
    if (motionState == FRONT)
        // length of one cell: 16.5 cm / (6 pi cm / 16*33*4 ticks) = 1849 ticks/cell
        return distanceFromEncoderReadings() > 1849;
    
    return 0;
}


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
            LED2 = LEDON;
            LED4 = LEDON;
            driveForward();
            break;
        case RIGHT:
            // first rotate, move as next step
            LED2 = LEDON;
            LED4 = LEDOFF;
            driveRightTurn(90);
            setMotionState(FRONT);
            break;
        case LEFT:
            // first rotate, move as next step
            LED2 = LEDOFF;
            LED4 = LEDON;
            driveLeftTurn(90);
            setMotionState(FRONT);
            break;
        case BACK:
            // first rotate, move as next step
            LED2 = LEDOFF;
            LED4 = LEDOFF;
            driveRightTurn(180);
            setMotionState(FRONT);
            break;
        case STOP:
            // completed.
            LED2 = LEDOFF;
            LED4 = LEDOFF;
            brake();
            break;
        default:
            LED2 = LEDOFF;
            LED4 = LEDOFF;
    }
}