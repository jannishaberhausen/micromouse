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
 * @param 
 *      distance_in_cm (int): optional, distance to drive forward in cm
 *      number_of_cells (int): optional, number of cells to drive forward
 */
void driveForward(int distance_in_cm, short number_of_cells) {
    // set motor directions. Both forward.
    setMotorDirections_Forward();
    
    // int distance_travelled = 0;
    // while(distance_travelled < distance_in_cm) {
    
    // (re)set motor speeds
    MOTORL = vbl * MOTOR_MAX;
    MOTORR = vbr * MOTOR_MAX;

    int left, right, front;
    sharpRaw(&left, &front, &right);

    // define error as signed distance from middle of straight corridor
    int error = right - left;

    // factor for P control
    float kp = 0.1;

    // update motor speeds based on error readings
    MOTORL += kp * error;
    MOTORR -= kp * error;

    // distance_travelled += distanceFromEncoderReadings(); 
    //}
}


/**
 * Lets the mouse turn clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveRightTurn(int degrees) {
    
}


/**
 * Lets the mouse turn counter-clockwise by the specified angle.
 * 
 * @param
 *      degrees (int): size of turning angle 
 */
void driveLeftTurn(int degrees) {
    
}


/**
 * Slows the mouse down quickly by turning the motors backward. This results in 
 * a faster stop as compared to letting the wheels spin freely.
 */
void brake() {
    
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
    MOTINL1 = 1;
    MOTINL2 = 0;
    MOTINR1 = 1;
    MOTINR2 = 0;
}


/**
 * Sets the direction of the motors such that the mouse turns right.
 * Left wheel goes forward. Right wheel goes backward.
 */
void setMotorDirections_RightTurn() {
    MOTINL1 = 1;
    MOTINL2 = 0;
    MOTINR1 = 0;
    MOTINR2 = 1;
}

/**
 * Sets the direction of the motors such that the mouse turns left.
 * Left wheel goes backward. Right wheel goes forward.
 */
void setMotorDirections_LeftTurn() {
    MOTINL1 = 0;
    MOTINL2 = 1;
    MOTINR1 = 1;
    MOTINR2 = 0;
}

/**
 * Sets the direction of the motors to both go backward.
 */
void setMotorDirections_Backward() {
    MOTINL1 = 0;
    MOTINL2 = 1;
    MOTINR1 = 0;
    MOTINR2 = 1;
}


int distanceFromEncoderReadings() {
    // not implemented
}