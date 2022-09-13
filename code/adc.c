#include "adc.h"
#include <xc.h>


/**
 * Set up configuration for ADC1.
 * 
 * After this function has been executed, sampling starts immediatelly.
 * Set up DMA beforehead!
 * 
 * Uses SCAN on CH0
 */
void initADC() {
    
    AD1CON1bits.ADON = 0; //unset on-bit
   
    //ADCON1
    AD1CON1bits.ADSIDL=0;   //no sampling in idle mode
    AD1CON1bits.ADDMABM=1;  //DMA channels are written in order of conversion
    AD1CON1bits.AD12B=1;    //12-bit operation (use 10bit for multi-channel)
    AD1CON1bits.SIMSAM=0;   // simultaneous sampling off (1 for multi-channel)
    AD1CON1bits.FORM=0b00;  //format is unsigned integer
    AD1CON1bits.SSRC=0b111; //auto-convert, see old adc.c for options
    AD1CON1bits.ASAM=0;     //auto-sampling off, sample only when SAMP bit is set
    AD1CON1bits.SAMP=0;     //no sampling, will be set by auto-sample

    //ADCON2
    AD1CON2bits.VCFG=0b000; //select Vref+ and Vref- as internal reference voltage
    AD1CON2bits.CSCNA=1;    //enable analog input SCAN on channel 0
    //AD1CON2bits.CHPS=0b11;  //convert channels 0, 1, 2, and 3 when using multi-channel
    AD1CON2bits.SMPI=3;     //interrupt after every sample (0 for multi-channel)
    AD1CON2bits.BUFM=0;     //always fill buffer starting at address 0x00
    AD1CON2bits.ALTS=0;     //always use channel A and do not alternate

    //ADCON3
    AD1CON3bits.ADRC=0;     //use internal clock source
    AD1CON3bits.SAMC=0x05;  //auto sample time bits, number of Tad
    AD1CON3bits.ADCS=0x06;  //8-bits to derive ADC clock, Tad= TCY ? (ADCS<7:0> + 1)

    //ADCON4
    AD1CON4bits.DMABL=0b000;//allocate one word of memory per input

    //AD1CSSL (input scan select register)
    AD1CSSL= 0b0000000000010011;    //select AN0, AN1, and AN4 (only for SCAN)

    AD1CHS0bits.CH0NA = 0b00;       //negative input for S/H 0 is Vref-
    AD1CHS123bits.CH123NA = 0b00;   //negative input for S/H 123 is Vref-
    //AD1CHS0bits.CH0SA = 0b10;       //sample AN2 on channel 0 (only for multi-channel)
    //AD1CHS123bits.CH123SA = 0b1;    //sample AN3, AN4, and AN5 on S/H 1, 2, and 3 (only for multi-channel)
    
    //interrupt configuration
    IFS0bits.AD1IF = 0;     //reset interrupt flag
    IEC0bits.AD1IE = 0;     //enable interrupts
    IPC3bits.AD1IP = 5;     //medium priority

    
    //start sampling
    AD1CON1bits.ADON=1; //set on-bit
    AD1CON1bits.ASAM=1; //auto-sample
}

