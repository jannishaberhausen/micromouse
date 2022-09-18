
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

/*

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

*/


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

