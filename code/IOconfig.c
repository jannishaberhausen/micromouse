#include "xc.h"
#include "IOconfig.h"



void setupIO()
{

    int i;
    
    
    //all pins are now digital, by default they are analogue
    AD1PCFGL=0xFFFF;
    
    //sharp sensors at AN2, AN3, and AN4 are analogue
    AD1PCFGLbits.PCFG2 = 0;
    AD1PCFGLbits.PCFG3 = 0;
    AD1PCFGLbits.PCFG4 = 0;
    
    
    // set LEDs as output
    TRISBbits.TRISB12 = 0;
    TRISBbits.TRISB13 = 0;
    TRISBbits.TRISB14 = 0;
    TRISBbits.TRISB15 = 0;
    
    // UART1 TX as output
    TRISCbits.TRISC1 = 0;
    

    ///////////////////////// PIN MAPPING ///////////////////////////
    
    //before we map, we need to unlock
    // clear bit 6 (unlock, they are usually write protected)
    __builtin_write_OSCCONL(OSCCON & 0xbf);
    
    
    // PERIPHERAL receives data from which INPUT PIN
    
    // UART1 RX on RP16
    RPINR18bits.U1RXR = 16;

    // QE1 on RP20 and RP21
    RPINR14bits.QEA1R = 20;
    RPINR14bits.QEB1R = 21;
    
    // QE2 on RP24 and RP25
    RPINR16bits.QEA2R = 24;
    RPINR16bits.QEB2R = 25;
    
    
    
    
    //OUTPUT PIN receives data from which PERIPHERAL, 
    //see table 11-2 in datasheet to check peripheral codes 
    
    // UART1 TX goes to RP17
    RPOR8bits.RP17R = 0b00011; //output bin RP2 gets data from peripheral U1 TX 

    
    
    
   
    //after mapping we lock again
    // Lock PPS registers (lock again!)
     __builtin_write_OSCCONL(OSCCON | 0x40);
     
    // short dirty delay for changes to take effect
    for (i = 0; i < 30000; i++);

    
}
