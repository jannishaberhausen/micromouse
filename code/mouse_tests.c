/*
 * File:   mouse_tests.c
 * Author: janni
 *
 * Created on 18. September 2022, 18:18
 */


#include "xc.h"

#include "IOconfig.h"

#include "pwm.h"
#include "motorEncoders.h"
#include "dma.h"
#include "adc.h"
#include "sharp.h"
#include "hw_tests.h"
#include "mouse_tests.h"
#include "mouse_motion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// TODO: Question: is setup set to 1? Can NOT set it here because it was 
// declared in hw_tests.h and defined in hw_tests.c
// Is the alternative really to define two setup variables in hw_tests.h AND
// in mouse_tests.h??? 
// setup = 1;

int perform_only_once = 1;

direction mouseState;

int error_previous = 0;

int desired_distance_to_front_wall = 1000;
int distance_to_front_wall_threshold = 100;


/**
 * Turns both motors forward. No control.
 * 
 * No special test setup.
 * 
 * Tests the motors.
 */
void testSlowMotionForwardNoControl() {
    if (setup == 1) {
        setupMotors();

        // set motor directions. Both forward.
        setMotorDirections_Forward();

        // set motor speeds
        MOTORL = 0.04 * MOTOR_MAX;
        MOTORR = 0.04 * MOTOR_MAX;

        setup = 0;
    }
}


/**
 * Turns both motors forward. Simple encoder velocity control.
 * 
 * No special test setup.
 * 
 * Tests motors and encoders. 
 */
void testSlowMotionForwardEncoderControl() {
    if (setup == 1) {
        setupMotors();
        setupEncoders();

        // set motor directions. Both forward.
        setMotorDirections_Forward();

        setup = 0;
    }

    // (re)set motor speeds
    MOTORL = 0.04 * MOTOR_MAX;
    MOTORR = 0.04 * MOTOR_MAX;

    // define error as difference in encoder readings
    int error = getVelocityInCountsPerSample_1() - getVelocityInCountsPerSample_2();

    // factor for P control
    float kp = 1;

    // update motor speeds based on error readings
    MOTORL += kp * error;
    MOTORR -= kp * error;
}


/**
 * Controls the mouse's motion in a straight corridor. Implements a simple 
 * controller using the right and left sharp sensor.
 * 
 * Test setup: setup a straight corridor with no corners.
 * 
 * Tests motors and sensors.
 */
void testStraightCorridor() {
    if (setup == 1) {
        setupMotors();
        setupSensors();
        setupEncoders();
        resetController();

        setup = 0;
    }

    driveForward();
}


/**
 * Tests the brake controller. The brake controller is a simple encoder control
 * that accelerates the motors such that the wheels turn as little as possible
 * even under external forces.
 * 
 * No special test setup.
 * 
 * Tests motors and encoders.
 */
void testBrake() {
    if (setup == 1) {
        setupMotors();
        setupEncoders();

        setup = 0;
    }
    
    brake();
}


/**
 * Tests the brake controller. The brake controller is a simple encoder control
 * that accelerates the motors such that the wheels turn as little as possible
 * even under external forces. In this test the mouse follows a straight 
 * corridor and brakes when it reaches the end of it.
 * 
 * Test setup: setup a straight corridor with no corners.
 * 
 * Tests motors, sensors, and encoders.
 */
void testBrakeInStraightCorridor() {
    if (setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();

        // set motor directions. both forward
        setMotorDirections_Forward();

        setup = 0;
    }

    driveForward();
    
    int left, right, front;
    sharpRaw(&left, &front, &right);
    if(checkForWallAhead(front)) {
        brake();
    }
}


/**
 * In this test the mouse performs four consecutive 90 degrees right turns,
 * followed by four consecutive left turns. Afterward, the mouse performs two
 * more 180 degrees right turns, and two 180 degrees left turns. 
 * 
 * No special test setup.
 * 
 * Tests motors and encoders.
 */
void testRotation() {
    if (!perform_only_once) {
        return;
    }
    perform_only_once = 0;

    if (setup == 1) {
        setupMotors();
        setupEncoders();

        setup = 0;
    }
    
    for(int i=0; i<4; i++) {
        driveRightTurn(90);
    }
    for(int i=0; i<4; i++) {
        driveLeftTurn(90);
    }
    for(int i=0; i<2; i++) {
        driveRightTurn(180);
    }
    for(int i=0; i<2; i++) {
        driveLeftTurn(180);
    }
}


/**
 * This test uses the simple controller from testMouseMotionAlongCorridor(). 
 * Additionally, the mouse registers the end of the corridor and performs a 180 
 * degrees turn before going back the corridor. The switching between states is 
 * implemented by a simple FSM. 
 * 
 * Test setup: setup a straight corridor with no corners.
 * 
 * Tests motors, encoders, and sensors.
 */
void testMouseMotionBackAndForthInCorridor() {
    if (setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();

        mouseState = FRONT;

        setup = 0;
    }
    
    delay ++;
    
    if(mouseState == RIGHT && delay < 50)
        mouseState = FRONT;

    switch (mouseState) {
        case FRONT:
            driveForward();

            int left, right, front;
            sharpRaw(&left, &front, &right);
            if(checkForWallAhead(front)){
                mouseState = STOP;
            }
            break;
            
        case STOP:
            brake();
            mouseState = RIGHT;
            break;
            
        case RIGHT:
            driveRightTurn(180);
            mouseState = FRONT;
            //resetController();
            break;
        default:
            break;
    }
}


void testMouseSmoothRightTurn(){
    if (setup == 1) {
        setupMotors();
        setupEncoders();

        // controlFixedSpeed(sth, sth);
        setup = 0;
    }   
}


void testMouseSmoothLeftTurn(){
    if (setup == 1) {
        setupMotors();
        setupEncoders();

        // controlFixedSpeed(sth, sth);
        setup = 0;
    }   
}


void testMouseRaceSequence() {
    if (setup == 1) {
        setupMotors();
        setupEncoders();
        
        setup = 0;
    }
    BASE_SPEED = 30;
    direction path[9];
    path[0] = FRONT;
    path[1] = LEFT;
    path[2] = FRONT;
    path[3] = FRONT;
    path[4] = RIGHT;
    path[5] = FRONT;
    path[6] = RIGHT;
    path[7] = FRONT;
    path[8] = STOP;
    for(int i = 0; path[i] != STOP; i++) {
        setRaceMotionState(path[i]);
        // wait for completion
        while (!getRaceMotionCompleted());
    }
    setRaceMotionState(STOP);
}

/* #############################################################################
 * ######## HAVE NOT CHANGED FUNCTIONs BELOW TO MATCH THE NEW FORMAT ###########
 * ###########################################################################*/

/**
 * In this test the mouse should move along a straight corridor only relying on 
 * one sensor. This behavior is necessary at corners. Here it can be tested in a 
 * longer corridor.
 * 
 * Test setup: setup a straight wall with no corners and no walls on the 
 * opposite side.
 * 
 * Tests the motors and sensors.
 */
void testMouseOnlyRelyOnOneSensor() {
    if (setup == 1) {
        setupMotors();
        setupSensors();

        // set motor directions. both forward
        setMotorDirections_Forward();

        setup = 0;
    }
    // int only_right = 1;
    int error;

    // variables for controller
    float kp_sensor = 0.3;
    // float ki_sensor = 0.01;

    int left, right, front;
    sharpRaw(&left, &front, &right);

    /*
    if(only_right) { // true: only rely on right sensor. false: only on left
        error = right - 1000;
    } else {
        error = left - 1000;
    }
     */

    error = right - 1000;

    float update = kp_sensor * error; //+ ki_sensor * (error+error_previous);

    MOTORL = 0.04 * MOTOR_MAX + update;
    MOTORR = 0.04 * MOTOR_MAX - update;

    // just for debugging
    if (front > 2000) {
        MOTORL = 0;
        MOTORR = 0;
    }
}

void testMouseAlwaysFollowRightWall() {
    if (setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();

        mouseState = FRONT;

        setup = 0;
    }

    int left, right, front;
    sharpRaw(&left, &front, &right);

    float encoder_start;
    float rotation;

    switch (mouseState) {
        case FRONT:
            // set motor directions. both forward
            MOTINL1 = 1;
            MOTINL2 = 0;
            MOTINR1 = 1;
            MOTINR2 = 0;

            // variables for controller
            float kp_sensor = 0.3;

            int error = right - 1000;

            float update = kp_sensor * error;

            MOTORL = 0.04 * MOTOR_MAX + update;
            MOTORR = 0.04 * MOTOR_MAX - update;

            if (front > desired_distance_to_front_wall) {
                mouseState = STOP;
            }

            break;

        case STOP:
            /*
            if(abs(front - desired_distance_to_front_wall) <= distance_to_front_wall_threshold) {
                // stop motors
                MOTORL = 0;
                MOTORR = 0;
                
                mouseState = RIGHT;
            }
            
            // define error as distance from desired wall distance
            error = front - desired_distance_to_front_wall;

            if(error > 0) {
                MOTINL1 = 0;
                MOTINL2 = 1;
                MOTINR1 = 0;
                MOTINR2 = 1;
            } else {
                MOTINL1 = 1;
                MOTINL2 = 0;
                MOTINR1 = 1;
                MOTINR2 = 0;
            }
            
            // turn both motors with a slow speed
            MOTORL = base_motor_speed_left * MOTOR_MAX;
            MOTORR = base_motor_speed_right * MOTOR_MAX;
             */

            MOTORL = 0;
            MOTORR = 0;

            mouseState = RIGHT;

            break;

        case RIGHT:
            encoder_start = getPositionInRad_2();

            // calculate rad from degrees: degrees * 0.02755 = rad
            rotation = 90.0 * 0.02755;

            // set motor directions. turn left motor forward, turn right motor backward
            MOTINL1 = 1;
            MOTINL2 = 0;
            MOTINR1 = 0;
            MOTINR2 = 1;

            MOTORL = 0.04 * MOTOR_MAX;
            MOTORR = 0.04 * MOTOR_MAX;

            while (getPositionInRad_2() + encoder_start > rotation);

            MOTORL = 0;
            MOTORR = 0;

            mouseState = FRONT;

            break;
        default:
            break;
    }
}


// actually a left turn at the moment

void testRightTurn(int degrees) {
    if (!perform_only_once) {
        return;
    }
    perform_only_once = 0;

    if (setup == 1) {
        setupMotors();
        setupEncoders();

        setup = 0;
    }

    float encoder_start = getPositionInRad_2();

    // calculate rad from degrees: degrees * 0.02755 = rad
    float rotation = (float) degrees * 0.02755;

    // set motor directions. turn left motor forward, turn right motor backward
    MOTINL1 = 0;
    MOTINL2 = 1;
    MOTINR1 = 1;
    MOTINR2 = 0;

    MOTORL = 0.04 * MOTOR_MAX;
    MOTORR = 0.04 * MOTOR_MAX;

    while (getPositionInRad_2() - encoder_start < rotation);

    MOTORL = 0;
    MOTORR = 0;

    //motorBreak();
}

/* ############################################################################
 * ################## MOVE TO UTILS FILE ######################################
 * ############################################################################
 */

void motorBreak() {
    while (getVelocityInCountsPerSample_1() > 1000 || getVelocityInCountsPerSample_2() > 1000) {
        // define error as remaining velocity in the motors
        int error_left_breaking;
        int error_right_breaking;

        error_left_breaking = getVelocityInCountsPerSample_1();
        error_right_breaking = getVelocityInCountsPerSample_2();

        // set motor directions for breaking
        if (error_left_breaking > 0) {
            MOTINL1 = 0;
            MOTINL2 = 1;
        } else {
            MOTINL1 = 1;
            MOTINL2 = 0;
        }
        if (error_right_breaking > 0) {
            MOTINR1 = 0;
            MOTINR2 = 1;
        } else {
            MOTINR1 = 1;
            MOTINR2 = 0;
        }

        float kp_encoder = 1;

        MOTORL = 0.04 * MOTOR_MAX - kp_encoder * error_left_breaking;
        MOTORR = 0.04 * MOTOR_MAX - kp_encoder * error_right_breaking;
    }
}
