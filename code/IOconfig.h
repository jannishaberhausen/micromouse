#ifndef IOCONFIG_H
#define	IOCONFIG_H

// green LEDs
#define LED1 LATBbits.LATB12
#define LED2 LATBbits.LATB13
#define LED3 LATBbits.LATB14
#define LED4 LATBbits.LATB15


#define LEDON 0
#define LEDOFF 1

void setupIO();

#endif	/* IOCONFIG_H */

