
#include <xc.h>

#include "IOconfig.h"

#include "pwm.h"
#include "motorEncoders.h"
#include "dma.h"
#include "adc.h"
#include "sharp.h"
#include "hw_tests.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int setup = 1;

int ctr = 0;

int perform_only_once = 1;

state mouseState;
float base_motor_speed_left = 0.04;
float base_motor_speed_right = 0.04;

int error_previous = 0;

int desired_distance_to_front_wall = 1000;
int distance_to_front_wall_threshold = 100;

// dutycycle to make motors turn slowly
float dc = 0.05;


/**
 * Tests the switch.
 * 
 * Pressing the switch should toggle LED2 and LED4.
 * 
 * Also tests the two LEDs.
 */
void testSwitch() {
    if(setup == 1) {
        setupSwitch();
        setup = 0;
    }
    
    // nothing else to do, action happens in the button.c ISR
}


/**
 * Tests the oscillator setup and the timer
 * 
 * Blinks LED2 at 50Hz. 
 */
void testTimer() {
    if(setup == 1) {
        setupLED24();
        setup = 0;
    }
    LED2 = !LED2;
    LED4 = !LED4;
}


/**
 * Tests the PWM on LED2 and LED4.
 * 
 * Sine LED2/4 with 1Hz. LED2 shifted 90° wrt LED4.
 * Uses up to 100% dutycycle, so test this with motors disconnected first.
 */
void testLedPWM() {
    if(setup == 1) {
        setupLED24_PWM();
        ctr = 0;
        setup = 0;
    }
    
    if(ctr > 50) 
        ctr = 0;

    // (2*pi)/50 ~ 0.125663706
    LED4DC = (1-(sin((float)ctr * 0.125663706)+1)*0.5) * PWM_MAX;
    LED2DC = (1-(sin((float)ctr * 0.125663706)+1)*0.5) * PWM_MAX;
    
    ctr ++;
}


/**
 * Test the PWM for motor control.
 * 
 * Sine the PWM signals for motors with 1Hz.
 * Right motor shifted by 90° wrt the left one.
 * Uses up to 2/3 dutycycle for the motors.
 */
void testMotorPWM() {
    if(setup == 1) {
        setupMotors();
        
        // set motor directions: left fwd, right bwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;
        
        ctr = 0;
        setup = 0;
    }
    
    if(ctr > 50) 
        ctr = 0;

    // (2*pi)/50 ~ 0.125663706
    MOTORL = (1-(sin((float)ctr * 0.125663706)+1)*0.5) * MOTOR_MAX;
    MOTORR = (1-(sin((float)ctr * 0.125663706)+1)*0.5) * MOTOR_MAX;
    
    ctr ++;
}


/**
 * Tests the motor encoders.
 * 
 * Set the brightness / DC of LED2/4 proportional to the
 * position of the right/left motors.
 */
void testEncoders() {
    if(setup == 1) {
        setupLED24_PWM();
        setupEncoders();
        setup = 0;
    }

    float left = getPositionInRad_1();
    float right = getPositionInRad_2();

    // 1/(2*pi) = 0.159155
    LED2DC = (1 - sin(left * 0.159155))  * PWM_MAX;
    LED4DC = (1 - sin(right * 0.159155)) * PWM_MAX;
}

void testMotion_P() {
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;
        setup = 0;
    }
    
    MOTORL = dc*MOTOR_MAX;
    MOTORR = dc*MOTOR_MAX;
    
    // controller
    
    float left = getVelocityInCountsPerSample_1();
    float right = getVelocityInCountsPerSample_2();
    
    
    float correction_l = (right-left) * 0.5;
    float correction_r = (left-right) * 0.5;
     
    if(MOTORL + correction_l < MOTOR_MAX)
        MOTORL += correction_l;
    
    if(MOTORR + correction_r < MOTOR_MAX)
        MOTORR += correction_r;
}

void testMotion_sharp() {
    
    if(setup == 1) {
        setupMotors();
        setupSensors();
        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;
        setup = 0;
    }
    
    
    MOTORL = dc*MOTOR_MAX;
    MOTORR = dc*MOTOR_MAX;
    
    // controller
    
    int left, front, right;
    sharpRaw(&left, &front, &right);
    
    float correction_l, correction_r;
    
    correction_l = (left - right) * 0.1;
    correction_r = (right - left) * 0.1;
     
    if(MOTORL + correction_l < MOTOR_MAX)
        MOTORL += correction_l;
    
    if(MOTORR + correction_r < MOTOR_MAX)
        MOTORR += correction_r;
    
}


void testMotion_rotate() {
    
    int origin;
    
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupLED24();
        // set motor directions: left bwd, right fwd
        MOTINL1 = 0;
        MOTINL2 = 1;
        MOTINR1 = 1;
        MOTINR2 = 0;
        mouseState = ROTATE;
        origin = getPositionInRad_2();
        setup = 0;
    }
    
    
    if(mouseState == ROTATE) {
        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;
        
        while(getPositionInRad_2() <= origin + 5);
        
        mouseState = FORWARD;
    } else {
        MOTINL1 = 1;
        MOTINL2 = 1;
        MOTINR1 = 1;
        MOTINR2 = 1;
    }
    
}

void testMotion_turn() {
    
    if(setup == 1) {
        setupMotors();
        setupSensors();
        setupEncoders();
        setupLED24();
        mouseState = FORWARD;
        setup = 0;
    }
    
    
    float origin;
    
    
    switch(mouseState) {
    case FORWARD:

        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;

        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;

        // controller


        int left, front, right;
        sharpRaw(&left, &front, &right);
        
        float correction_l, correction_r;

        correction_l = (left - right) * 0.1;
        correction_r = (right - left) * 0.1;

        if(MOTORL + correction_l < MOTOR_MAX)
            MOTORL += correction_l;

        if(MOTORR + correction_r < MOTOR_MAX)
            MOTORR += correction_r;

        
        LED2 = front >= 2000;

        if(front >= 2000) {
            mouseState = ROTATE;
        }
        
        break;
    case 1:
        
        origin = getPositionInRad_2();
        
        MOTINL1 = 0;
        MOTINL2 = 1;
        MOTINR1 = 1;
        MOTINR2 = 0;
        
        MOTORL = 0.2*MOTOR_MAX;
        MOTORR = 0.2*MOTOR_MAX;
        
        while(abs(getPositionInRad_2() - origin) <= 5);
        
        mouseState = FORWARD;
        break;
    default:
        break;
    }
    
}


/**
 * Tests the sharp distance sensors.
 * 
 * Sets the brightness / DC of LED2/4 proportional to the 
 * distance read from the right / left sensors.
 * 
 * Tests the sensors (raw output, no distance measures!), as well as 
 * the ADC and DMA units required to read them.
 * The front sensor is read as well, but displayed only in the next 
 * test case, as the other LEDs are connected to the hbridge.
 */
void testSensorsLR() {
    if(setup == 1) {
        setupLED24_PWM();
        setupSensors();
        //initOutput();
        setup = 0;
    
        //int left, right, front;
        //sharpRaw(&left, &front, &right);
        //putBinary(left);
    }
    
    //refreshBinary();
    int left, right, front;
    sharpRaw(&left, &front, &right);

    LED4DC = (1 - ((float) left  / SHARP_MAX)) * PWM_MAX;
    LED2DC = (1 - ((float) right / SHARP_MAX)) * PWM_MAX;
}


/**
 * Tests the sharp distance sensors.
 * 
 * Sets the brightness / DC of LED2 proportional to the 
 * distance read from the front sensor.
 * 
 * Tests the sensor (raw output, no distance measures!), as well as 
 * the ADC and DMA units required to read them.
 * The left / right sensors are read as well, but displayed only in the 
 * previous test case, as the other LEDs are connected to the hbridge.
 */
void testSensorsF() {
    if(setup == 1) {
        setupLED24_PWM();
        setupSensors();
        setup = 0;
    }
    
    int left, right, front;
    sharpRaw(&left, &front, &right);

    LED2DC = (1 - ((float) front / 1023.0)) * PWM_MAX;
}


/**
 * Tests the motors.
 * 
 * Slowly turns both motors forward.
 */
void testMouseSlowMotionForward() {
    if(setup == 1) {
        setupMotors();
        
        setup = 0;
    }
    
    // set motor directions. both forward
    MOTINL1 = 1;
    MOTINL2 = 0;
    MOTINR1 = 1;
    MOTINR2 = 0;
    
    MOTORL = base_motor_speed_left * MOTOR_MAX;
    MOTORR = base_motor_speed_right * MOTOR_MAX;
}


/**
 * Tests motors and encoders.
 * 
 * Mouse should move slowly forward in a corridor. A simple controller ajusts 
 * the speed of the wheels based on the encoder readings.
 */
void testMouseSlowMotionForwardEncoderControl() {
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        
        // set motor directions. both forward
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;
        
        setup = 0;
    }
    
    MOTORL = 0.05 * MOTOR_MAX;
    MOTORR = 0.05 * MOTOR_MAX;
    
    // define error as difference in encoder readings
    int error = getVelocityInCountsPerSample_1() - getVelocityInCountsPerSample_2();
    
    float kp = 1;
    
    MOTORL += kp * error;
    MOTORR -= kp * error;
}


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
 * Tests motors and sensors. Implements a simple controller.
 * 
 * Mouse should go forward and keep in the middle of a straight corridor.
 */
void testMouseMotionAlongCorridor() {
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

    // turn both motors with a slow speed
    MOTORL = 0.04 * MOTOR_MAX;
    MOTORR = 0.03 * MOTOR_MAX;
    
    int left, right, front;
    sharpRaw(&left, &front, &right);
    
    // define error as signed distance from middle of straight corridor
    int error = right - left;
    
    float kp = 0.1;
    
    MOTORL += kp * error;
    MOTORR -= kp * error;
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
            MOTORL = base_motor_speed_left * MOTOR_MAX;
            MOTORR = base_motor_speed_right * MOTOR_MAX;

            int left, right, front;
            sharpRaw(&left, &front, &right);

            // define error as signed distance from middle of straight corridor
            int error = right - left;
            
            float update = kp_sensor * error + ki_sensor * (error+error_previous);

            MOTORL += update;
            MOTORR -= update;
            
            error_previous = error;
            
            if(front > 2000) {
                mouseState = BREAK;
            }
            
            break;
        case BREAK:
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
            
            mouseState = ROTATE;
            
            break;
        case ROTATE:
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
            MOTORL = base_motor_speed_left * MOTOR_MAX;
            MOTORR = base_motor_speed_right * MOTOR_MAX;

            // define error as signed distance from middle of straight corridor
            int error = right - left;
            
            float update = kp_sensor * error + ki_sensor * (error+error_previous);

            MOTORL += update;
            MOTORR -= update;
            
            error_previous = error;
            
            if(front > desired_distance_to_front_wall) {
                mouseState = BREAK;
            }
            
            break;
        case BREAK:
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
            MOTORL = base_motor_speed_left * MOTOR_MAX;
            MOTORR = base_motor_speed_right * MOTOR_MAX;

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

    MOTORL = base_motor_speed_left * MOTOR_MAX + update;
    MOTORR = base_motor_speed_right * MOTOR_MAX - update;
    
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

            MOTORL = base_motor_speed_left * MOTOR_MAX + update;
            MOTORR = base_motor_speed_right * MOTOR_MAX - update;
            
            if(front > desired_distance_to_front_wall) {
                mouseState = BREAK;
            }
            
            break;
            
        case BREAK:
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

            MOTORL = base_motor_speed_left * MOTOR_MAX;
            MOTORR = base_motor_speed_right * MOTOR_MAX;

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
    
    MOTORL = base_motor_speed_left * MOTOR_MAX;
    MOTORR = base_motor_speed_right * MOTOR_MAX;
    
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

        MOTORL = base_motor_speed_left * MOTOR_MAX - kp_encoder * error_left_breaking;
        MOTORR = base_motor_speed_left * MOTOR_MAX - kp_encoder * error_right_breaking;
    }
}