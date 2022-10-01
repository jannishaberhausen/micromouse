
#include <xc.h>

#include "IOconfig.h"

#include "pwm.h"
#include "motorEncoders.h"
#include "dma.h"
#include "adc.h"
#include "sharp.h"
#include "hw_tests.h"
#include "pathfinder.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int setup = 1;

int ctr = 0;

/*
// dutycycle to make motors turn slowly
float dc = 0.05;

// desired velocity
float v_dest_left;
float v_dest_right;

// accumulated error for I part
float acc_error_left;
float acc_error_right;

// correction during control
float correction_left;
float correction_right;

// error in the previous control iteration, for D part
int last_error_left;
int last_error_right;

// control parameters
float k_p = 2;
float k_i = 0.001;
float k_d = 80;

direction mouseState;

int delay;

*/

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
 * Tests slowly driving forward using a P controller.
 * 
 * Controls only the difference between the two motors, 
 * not the actual speed.
 * 
 * Works, but not great, made obsolete by testMotion_PID().
 */
/*
void testMotion_P() {
    
    // desired velocity in ticks per sample,
    // 2112 ticks/rotation at 0.5 rotations/second
    // and 100 samples/second => 10.56 ticks per sample
    int v = 10;
    
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;

        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;
        
        correction_left = 0;
        correction_right = 0;
        
        setup = 0;
    }
    
    // controller
    
    float left = getVelocityInCountsPerSample_1();
    float right = getVelocityInCountsPerSample_2();
    
    
    correction_left = (v-left) * k_p;
    correction_right = (v-right) * k_p;
     
    if(MOTORL + correction_left < MOTOR_MAX)
        MOTORL += correction_left;
    
    if(MOTORR + correction_right < MOTOR_MAX)
        MOTORR += correction_right;
}
*/

/**
 * Tests slowly driving forward using a PID controller.
 * 
 * Keeps the velocity of both motors independently
 * to half a rotation per second.
 */
/*
void testMotion_PID() {
    
    // desired velocity in ticks per sample,
    // 2112 ticks/rotation at 0.5 rotations/second
    // and 100 samples/second => 10.56 ticks per sample
    int v = 10;
    
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;

        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;
        
        correction_left = 0;
        correction_right = 0;
        last_error_left = 0;
        last_error_right = 0;
        
        acc_error_left = 0;
        acc_error_right = 0;
        
        setup = 0;
    }
    
    // controller
    
    float left = getVelocityInCountsPerSample_1();
    float right = getVelocityInCountsPerSample_2();
    
    // P-part
    float error_l = v-left;
    float error_r = v-right;
    
    // I-part
    acc_error_left += error_l;
    acc_error_right += error_r;
    
    // D-part
    float d_error_l = error_l - last_error_left;
    float d_error_r = error_r - last_error_right;
    
    last_error_left = error_l;
    last_error_right = error_r;
    
    // uses the parameters at the top of this file
    correction_left = error_l * k_p + acc_error_left * k_i + d_error_l * k_d;
    correction_right = error_r * k_p + acc_error_right * k_i + d_error_r * k_d;
    
    
     
    if(MOTORL + correction_left < MOTOR_MAX)
        MOTORL += correction_left;
    
    if(MOTORR + correction_right < MOTOR_MAX)
        MOTORR += correction_right;
}
*/

/**
 * Tests the left-right control using the sensors.
 * 
 * Uses a basic P-controller.
 * 
 * Works, but not great. Made obsolete by testMotion_nested().
 */
/*
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
    
    
    correction_left = (left - right) * 0.1;
    correction_right = (right - left) * 0.1;
     
    if(MOTORL + correction_left < MOTOR_MAX)
        MOTORL += correction_left;
    
    if(MOTORR + correction_right < MOTOR_MAX)
        MOTORR += correction_right;
    
}
*/

/**
 * Extracts the same functionality as testMotion_PID() to its own function.
 * 
 * This function can be used from within others, and will be further changed 
 * until we can directly move it over to a new motor.c
 * 
 * @param v_l The desired volocity of the left motor
 * @param v_r The desired velocity of the right motor
 * @param reset If 1, all configuration will be reset, if 0, ignored
 */
/*
void control_PID(int v_l, int v_r, int reset) {
    if(reset == 1) {
        setupMotors();
        setupEncoders();
        // set motor directions: left fwd, right fwd
        if(v_l >= 0) {
            MOTINL1 = 1;
            MOTINL2 = 0;
        } else {
            MOTINL1 = 0;
            MOTINL2 = 1;
        }
        if(v_r >= 0) {
            MOTINR1 = 1;
            MOTINR2 = 0;
        } else {
            MOTINR1 = 0;
            MOTINR2 = 1;
        }

        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;
        
        correction_left = 0;
        correction_right = 0;
        last_error_left = 0;
        last_error_right = 0;
        
        acc_error_left = 0;
        acc_error_right = 0;
        
        reset = 0;
    }
    
    // controller
    
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
    
    
     
    if(MOTORL + correction_left < MOTOR_MAX)
        MOTORL += correction_left;
    
    if(MOTORR + correction_right < MOTOR_MAX)
        MOTORR += correction_right;
}
*/

/**
 * Tests slowly driving forward using a nested controller,
 * unifying testMotion_Sensor and testMotion_PD.
 */
/*
void testMotion_nested() {
    
    if(setup == 1) {
        setupMotors();
        setupEncoders();
        setupSensors();
        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;

        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;
        
        correction_left = 0;
        correction_right = 0;
        last_error_left = 0;
        last_error_right = 0;
        
        acc_error_left = 0;
        acc_error_right = 0;
        
        // desired velocity in ticks per sample,
        // 2112 ticks/rotation at 0.5 rotations/second
        // and 100 samples/second => 10.56 ticks per sample
        v_dest_left = 10;
        v_dest_right = 10;
        
        
        setup = 0;
    }
    
    // controller
    
    // outer control loop: sideways using sensors
    int sensor_left, unused, sensor_right;
    sharpRaw(&sensor_left, &unused, &sensor_right);
    
    // v_dest ~ 10, sensor diff for 1cm ~ 800,
    // so we have to use *very* small weights
    
    v_dest_left = 10 - (sensor_left - sensor_right) * 0.003;
    v_dest_right = 10 - (sensor_right - sensor_left) * 0.003;
    
    
    // inner control loop, bring v_dest to the motors
    control_PID(v_dest_left, v_dest_right, 0);
}
*/

/**
 * Doesnt work yet. Should apply PID control backwards
 */
/*
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
        mouseState = RIGHT;
    }
    
    
    if(mouseState == RIGHT) {
        control_PID(-10, -10, setup);
        if(setup == 1) {
            origin = getPositionInRad_2();
            setup = 0;
        }
        
        if(getPositionInRad_2() >= origin + 5){
            mouseState = FRONT;
            LED2 = LEDOFF;
        }
    } else {
        MOTINL1 = 1;
        MOTINL2 = 1;
        MOTINR1 = 1;
        MOTINR2 = 1;
    }
    
}
*/

/**
 * Test driving up and down a corridor.
 * 
 * Uses nested control for driving forward, and no control for turning.
 * Should be extended to use control for turning as soon as testMotion_rotate
 * works. Uses a short delay to avoid spinning all the time.
 * Also tests a very basic automaton.
 */
/*
void testMotion_turn() {
    
    if(setup == 1) {
        setupMotors();
        setupSensors();
        setupEncoders();
        setupLED24();
        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;

        MOTORL = dc*MOTOR_MAX;
        MOTORR = dc*MOTOR_MAX;
        
        correction_left = 0;
        correction_right = 0;
        last_error_left = 0;
        last_error_right = 0;
        
        acc_error_left = 0;
        acc_error_right = 0;
        
        mouseState = FRONT;
        setup = 0;
    }
    
    
    float origin;
    
    
    switch(mouseState) {
    case FRONT:
        

        // set motor directions: left fwd, right fwd
        MOTINL1 = 1;
        MOTINL2 = 0;
        MOTINR1 = 1;
        MOTINR2 = 0;

        testMotion_nested();

        int unused1, front, unused2;
        sharpRaw(&unused1, &front, &unused2);
        
        LED2 = front >= 1500;

        if(front >= 1500 && delay >= 50) {
            mouseState = RIGHT;
        }
        
        delay ++;
        
        break;
    case RIGHT:
        
        origin = getPositionInRad_2();
        
        MOTINL1 = 0;
        MOTINL2 = 1;
        MOTINR1 = 1;
        MOTINR2 = 0;
        
        MOTORL = 0.07*MOTOR_MAX;
        MOTORR = 0.07*MOTOR_MAX;
        
        while(abs(getPositionInRad_2() - origin) <= 3);
        
        mouseState = FRONT;
        // reset encoders so the backwards turning motor doesnt mess up the
        // integral part
        setupEncoders();
        // set half a second delay before we can start turning again
        delay = 0;
        break;
    default:
        break;
    }   
}
*/