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
// #include "hw_tests.h"
#include "mouse_tests.h"
#include "mouse_motion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// control parameters
float k_p = 2;
float k_i = 0.001;
float k_d = 80;


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
 * Controls the mouse's motion along a straight corridor. Uses the right and 
 * left sensor to stabilize the mouse in the middle of the corridor. Implements
 * a closed-loop control to do so. Either accepts as input the desired distance 
 * to drive in cm or the number of cells. If both inputs are null then the mouse
 * drives forward until it reaches a junction or a dead end and changes state.
 * 
 * @params 
 *      distance_in_cm (int):   optional, distance to drive forward in cm
 *      number_of_cells (int):  optional, number of cells to drive forward
 */
void driveForward() {
    
    ////////////////////////////////////////////////////////////////////////
    //            outer control loop: sideways using sensors              //
    ////////////////////////////////////////////////////////////////////////
    
    int sensor_left, unused, sensor_right;
    sharpRaw(&sensor_left, &unused, &sensor_right);
    
    // v_dest ~ 10, sensor diff for 1cm ~ 800,
    // so we have to use *very* small weights
    
    v_dest_left = 10 - (sensor_left - sensor_right) * 0.003;
    v_dest_right = 10 - (sensor_right - sensor_left) * 0.003;
    
    
    ///////////////////////////////////////////////////////////////////////
    //         inner control loop, bring v_dest to the motors            //
    ///////////////////////////////////////////////////////////////////////
    
    float left = getVelocityInCountsPerSample_1();
    float right = getVelocityInCountsPerSample_2();
    
    // P-part
    float error_l = v_dest_left-left;
    float error_r = v_dest_right-right;
    
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
    
    
    if(MOTORL + correction_left < MOTOR_MAX)
        MOTORL += correction_left;
    
    if(MOTORR + correction_right < MOTOR_MAX)
        MOTORR += correction_right;
}


/**
 * Lets the mouse turn clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveRightTurn(int degrees) {
    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *1.55
    float rotation_in_rad = (float) degrees * 0.02755 * 1.55;

    setMotorDirections_RightTurn();

    MOTORL = vbl * MOTOR_MAX;
    MOTORR = vbr * MOTOR_MAX;

    while (abs(getPositionInRad_2() - encoder_start) < rotation_in_rad);

    brake();
}


/**
 * Lets the mouse turn counter-clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveLeftTurn(int degrees) {
    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    // mouse rotation to wheel rotation: *1.55
    float rotation_in_rad = (float) degrees * 0.02755 * 1.55;

    setMotorDirections_LeftTurn();

    MOTORL = vbl * MOTOR_MAX;
    MOTORR = vbr * MOTOR_MAX;

    while (abs(getPositionInRad_2() - encoder_start) < rotation_in_rad);

    brake();
    
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
 * @param
 *      right (int): distance measurement from right sensor
 * @return
 *      1 = there is a corner to the right, 0 = there is no corner to the right
 */
int checkForRightCorner(int right) {
    
}


/**
 * Checks whether there is a corner to the left. Does not exclude the 
 * possibility of a corner to the right or other type of junctions.
 * 
 * @param
 *      left (int): distance measurement from left sensor
 * @return
 *      1 = there is a corner to the left, 0 = there is no corner to the left
 */
int checkForLeftCorner(int left) {
    
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


int distanceFromEncoderReadings() {
    // not implemented
}