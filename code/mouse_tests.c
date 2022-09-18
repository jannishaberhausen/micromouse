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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


// TODO: Question: is setup set to 1? Can NOT set it here because it was 
// declared in hw_tests.h and defined in hw_tests.c
// Is the alternative really to define two setup variables in hw_tests.h AND
// in mouse_tests.h??? 
// setup = 1;

int perform_only_once = 1;

state mouseState;

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
    if(setup == 1) {
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
    if(setup == 1) {
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
    if(setup == 1) {
        setupMotors();
        setupSensors();
        
        setup = 0;
    }
    
    driveForward();
}

/* #############################################################################
 * ######## HAVE NOT CHANGED FUNCTIONs BELOW TO MATCH THE NEW FORMAT ###########
 * ###########################################################################*/

void testMotorBreak() {
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();
        setupLED24();
        
        // set motor directions. both forward
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;
        
        setup = 0;
    }
    
    // turn both motors with a slow speed
    MOTORL = 0.04 * MOTOR_MAX;
    MOTORR = 0.03 * MOTOR_MAX;
    
    int left, right, front;
    sharpRaw(&left, &front, &right);
    
    if(front > 1000) {
        // verify sensor read
        LED4 = 1;
        
        // define error as difference in encoder readings
        int error_left = getVelocityInCountsPerSample_1();
        int error_right = getVelocityInCountsPerSample_2();
        
        // break motors fast
        if(error_left > 0) {
            MOTINL1 = 0;
            MOTINL2 = 1;
        } else {
            MOTINL1 = 1;
            MOTINL2 = 0;
        }
        
        if(error_right > 0) {
            MOTINR1 = 0;
            MOTINR2 = 1;
        } else {
            MOTINR1 = 1;
            MOTINR2 = 0;
        }
        
        float kp = 1;
        
        MOTORL += kp * error_left;
        MOTORR -= kp * error_right;
    }
}

/**
 * Tests the motors and encoders.
 * 
 * The mouse should perform a 180 degrees rotation. This maneuver is only 
 * performed once
 */
void testMouse180DegreesRotation() {
    if(!perform_only_once) {
        return;
    }
    perform_only_once = 0;
    
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupLED24();
        
        setup = 0;
    }
    
    float encoder_start = getPositionInRad_2();
    
    /**
     * distance between wheels: 9.5 cm
     * diameter of wheels: 6 cm
     * cirfumference of wheels: pi * 6 cm = 18.85 cm
     * circumference of mouse rotating on the spot: pi * 9.5 cm = 29.85 cm
     * distance for a 180 degrees rotation: 29.85/2 = 14.92 cm
     * fraction of one wheel rotation for 180 degrees turn: 14.92/18.85 = 0.79
     * wheel rotation in rad for a 180 degree turn: 2*pi*0.79 = 4.96
     */
    float half_rotation = 4.96;
    
    // set motor directions. turn left motor backward, turn right motor forward
    MOTINL1 = 0;
    MOTINL2 = 1;
    MOTINR1 = 1;
    MOTINR2 = 0;
    
    MOTORL = 0.05 * MOTOR_MAX;
    MOTORR = 0.05 * MOTOR_MAX;
    
    while(getPositionInRad_2() - encoder_start < half_rotation){
        LED4 = 1;
    }
    
    // stop motors
    MOTINL1 = 0;
    MOTINL2 = 0;
    MOTINR1 = 0;
    MOTINR2 = 0;
}


/**
 * Test motors, encoders, and sensors. 
 * 
 * Uses the simple controller from testMouseMotionAlongCorridor(). Additionally, 
 * the mouse registers the end of the corridor and performs a 180 degrees turn 
 * before going back the corridor. The switching between states is implemented
 * by a simple FSM.
 */
void testMouseMotionBackAndForthInCorridor() {
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();
        
        mouseState = FORWARD;
        
        setup = 0;
    }
    
    int error_left_breaking;
    int error_right_breaking;
    
    switch(mouseState) {
        case FORWARD:
            // set motor directions. both forward
            MOTINL1 = 1;
            MOTINL2 = 0;
            MOTINR1 = 1;
            MOTINR2 = 0;
            
            // factors for controller
            float kp_sensor = 0.1;
            float ki_sensor = 0.01;
            
            // turn both motors with a slow speed
            MOTORL = 0.04 * MOTOR_MAX;
            MOTORR = 0.04 * MOTOR_MAX;

            int left, right, front;
            sharpRaw(&left, &front, &right);

            // define error as signed distance from middle of straight corridor
            int error = right - left;
            
            float update = kp_sensor * error + ki_sensor * (error+error_previous);

            MOTORL += update;
            MOTORR -= update;
            
            error_previous = error;
            
            if(front > 2000) {
                mouseState = BRAKE;
            }
            
            break;
        case BRAKE:
            // define error as remaining velocity in the motors
            error_left_breaking = getVelocityInCountsPerSample_1();
            error_right_breaking = getVelocityInCountsPerSample_2();

            // break motors fast
            if(error_left_breaking > 0) {
                MOTINL1 = 0;
                MOTINL2 = 1;
            } else {
                MOTINL1 = 1;
                MOTINL2 = 0;
            }

            if(error_right_breaking > 0) {
                MOTINR1 = 0;
                MOTINR2 = 1;
            } else {
                MOTINR1 = 1;
                MOTINR2 = 0;
            }

            float kp_encoder = 1;

            MOTORL += kp_encoder * error_left_breaking;
            MOTORR -= kp_encoder * error_right_breaking;
            
            mouseState = RIGHT_TURN;
            
            break;
        case RIGHT_TURN:
            testMouse180DegreesRotation();
            mouseState = FORWARD;
            break;
    }
}


/**
 * Tests the motors and the sensors.
 * 
 * The mouse should follow a straight corridor until it reaches the end of the 
 * corridor. The mouse should stop exactly in the middle of the cell.
 */
void testMouseStopBeforeWall() {
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();
        
        mouseState = FORWARD;
        
        setup = 0;
    }
    
    int left, right, front;
    sharpRaw(&left, &front, &right);
    
    switch(mouseState) {
        case FORWARD:
            // set motor directions. both forward
            MOTINL1 = 1;
            MOTINL2 = 0;
            MOTINR1 = 1;
            MOTINR2 = 0;
            
            // factors for controller
            float kp_sensor = 0.1;
            float ki_sensor = 0.01;
            
            // turn both motors with a slow speed
            MOTORL = 0.04 * MOTOR_MAX;
            MOTORR = 0.04 * MOTOR_MAX;

            // define error as signed distance from middle of straight corridor
            int error = right - left;
            
            float update = kp_sensor * error + ki_sensor * (error+error_previous);

            MOTORL += update;
            MOTORR -= update;
            
            error_previous = error;
            
            if(front > desired_distance_to_front_wall) {
                mouseState = BRAKE;
            }
            
            break;
        case BRAKE:
            if(abs(front - desired_distance_to_front_wall) <= distance_to_front_wall_threshold) {
                // stop motors
                MOTORL = 0;
                MOTORR = 0;
            }
            
            // define error as distance from desired wall distance
            error = front - desired_distance_to_front_wall;

            // break motors fast
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
            MOTORL = 0.04 * MOTOR_MAX;
            MOTORR = 0.04 * MOTOR_MAX;

            break;
    }
}


/**
 * Tests the motors and sensors.
 * 
 * The mouse should move along a straight corridor only relying on one sensor. 
 * This behavior is necessary at corners. Here it can be tested in a longer 
 * corridor.
 */
void testMouseOnlyRelyOnOneSensor() {
    if(setup == 1) {
        setupMotors();
        setupSensors();
        
        // set motor directions. both forward
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;
        
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

    float update = kp_sensor * error;  //+ ki_sensor * (error+error_previous);

    MOTORL = 0.04 * MOTOR_MAX + update;
    MOTORR = 0.04 * MOTOR_MAX - update;
    
    // just for debugging
    if(front > 2000) {
        MOTORL = 0;
        MOTORR = 0;
    }
}


void testMouseAlwaysFollowRightWall() {
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();
        
        mouseState = FORWARD;
        
        setup = 0;
    }
    
    int left, right, front;
    sharpRaw(&left, &front, &right);
    
    float encoder_start;
    float rotation;
    
    switch(mouseState) {
        case FORWARD:
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
            
            if(front > desired_distance_to_front_wall) {
                mouseState = BRAKE;
            }
            
            break;
            
        case BRAKE:
            /*
            if(abs(front - desired_distance_to_front_wall) <= distance_to_front_wall_threshold) {
                // stop motors
                MOTORL = 0;
                MOTORR = 0;
                
                mouseState = RIGHT_TURN;
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
            
            mouseState = RIGHT_TURN;

            break;
            
        case RIGHT_TURN:
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

            while(getPositionInRad_2() + encoder_start > rotation);

            MOTORL = 0;
            MOTORR = 0;
            
            mouseState = FORWARD;
            
            break;
    }
}


// actually a left turn at the moment
void testRightTurn(int degrees) {
    if(!perform_only_once) {
        return;
    }
    perform_only_once = 0;
    
    if(setup == 1) {
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
    
    while(getPositionInRad_2() - encoder_start < rotation);
    
    MOTORL = 0;
    MOTORR = 0;
    
    //motorBreak();
}


/* ############################################################################
 * ################## MOVE TO UTILS FILE ######################################
 * ############################################################################
 */

void motorBreak() {
    while(getVelocityInCountsPerSample_1() > 1000 || getVelocityInCountsPerSample_2() > 1000) {
        // define error as remaining velocity in the motors
        int error_left_breaking;
        int error_right_breaking;

        error_left_breaking = getVelocityInCountsPerSample_1();
        error_right_breaking = getVelocityInCountsPerSample_2();

        // set motor directions for breaking
        if(error_left_breaking > 0) {
            MOTINL1 = 0;
            MOTINL2 = 1;
        } else {
            MOTINL1 = 1;
            MOTINL2 = 0;
        }
        if(error_right_breaking > 0) {
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
