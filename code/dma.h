#ifndef DMA_H
#define	DMA_H


#include <xc.h>
    
//allocate four words of memory
extern unsigned int adcData[48]__attribute__((space(dma)));

void initDMA(void);

//fast access to sensor readings
#define SHARP_L adcData[0]
#define SHARP_R adcData[1]
#define SHARP_F adcData[2]



#endif	/* DMA_H */

