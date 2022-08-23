#ifndef IOCONFIG_H
#define	IOCONFIG_H

// green LEDs
// NEVR SET LED1 AND LED3 MANUALLY! Those are connected to PWM1H1/2.
//#define LED1 LATBbits.LATB12
#define LED2 LATBbits.LATB13
//#define LED3 LATBbits.LATB14
#define LED4 LATBbits.LATB15

// LED PWM dutycycle
#define LED2DC P1DC2
#define LED4DC P1DC1

// motor PWM duty cycle
#define MOTORL P1DC1
#define MOTORR P1DC2

// motor direction
#define MOTINL1 LATBbits.LATB5
#define MOTINL2 LATBbits.LATB6
#define MOTINR1 LATBbits.LATB7
#define MOTINR2 LATBbits.LATB8

// motor enable bits when motrs are inactive
#define MOTENL LATBbits.LATB14
#define MOTENR LATBbits.LATB12

#define LEDON 0
#define LEDOFF 1

void setupIO();

void setupLED24();
void setupLED24_PWM();
void setupMotors();
void setupSwitch();
void setupEncoders();
void setupSensors();

#endif	/* IOCONFIG_H */

