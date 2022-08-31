
#include <xc.h>

#include "IOconfig.h"

#include "pwm.h"
#include "motorEncoders.h"
#include "dma.h"
#include "adc.h"
#include "sharp.h"

#include <math.h>
#include <stdio.h>

int setup = 1;

int ctr = 0;


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
    LED2DC = (1-(cos((float)ctr * 0.125663706)+1)*0.5) * PWM_MAX;
    
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
        MOTINR1 = 0;
        MOTINR2 = 1;
        
        ctr = 0;
        setup = 0;
    }
    
    if(ctr > 50) 
        ctr = 0;

    // (2*pi)/50 ~ 0.125663706
    MOTORL = ((sin((float)ctr * 0.125663706)+1)*0.5) * MOTOR_MAX;
    MOTORR = ((cos((float)ctr * 0.125663706)+1)*0.5) * MOTOR_MAX;
    
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
    LED4DC = (1 - (left * 0.159155))  * PWM_MAX;
    LED4DC = (1 - (right * 0.159155)) * PWM_MAX;
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
        setup = 0;
    }
    
    int left, right, front;
    sharpRaw(&left, &front, &right);


    LED4DC = (1 - ((float) left  / 1023.0)) * PWM_MAX;
    LED2DC = (1 - ((float) right / 1023.0)) * PWM_MAX;
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