
#include <xc.h>

#include "IOconfig.h"
#include "dma.h"
#include "sharp.h"

#define BUF_LEN 10

#ifndef SHARP_L
#warning "Must define SHARP_L, SHARP_F, and SHARP_R "
#warning "to point to the DMA addresses of the corresponding sensors!"
#endif

// ring buffer for filtering
int buffer_l[BUF_LEN];
int buffer_r[BUF_LEN];
int buffer_f[BUF_LEN];
int head = 0;

/**
 * Start the sensor.
 * 
 * Initializes the ring buffer.
 */
void initSharp() {
    int empty;
    for(int i = 0; i < BUF_LEN; i++) {
        sharpRaw(&empty, &empty, &empty);
    }
    head = 0;
}


/**
 * Collect a filtered measurement as returned by the ADC.
 * 
 * No postprocessing / scaling done here.
 * Probably useful for calibrating, and sideways position control.
 * 
 * @param [OUT] left will be set to the reading of the left sensor
 * @param [OUT] front will be set to the reading of the front sensor
 * @param [OUT] right will be set to the reading of the right sensor
 * 
 */
void sharpRaw(int *left, int *front, int *right)
{
    
    buffer_l[head] = SHARP_L;
    buffer_r[head] = SHARP_R;
    buffer_f[head] = SHARP_F;
    
    head ++;
    if(head >= BUF_LEN)
        head = 0;

    
    // sliding-average filter
    int l = 0;
    int r = 0;
    int f = 0;
    for(int i = 0; i < BUF_LEN; i++) {
        l += buffer_l[i];
        r += buffer_r[i];
        f += buffer_f[i];
    }
    l /= BUF_LEN;
    r /= BUF_LEN;
    f /= BUF_LEN;
    
    //output raw values 
    *left = l;
    *right = r;
    *front = f;
}

/**
 * Collect a filtered and scaled measurement.
 * 
 * Output in 0.1 mm.
 * Used for motor control, especially for the front sensor.
 * 
 * @param [OUT] left will be set to the reading of the left sensor
 * @param [OUT] front will be set to the reading of the front sensor
 * @param [OUT] right will be set to the reading of the right sensor
 * 
 */
void sharpDistance(int *left, int *front, int *right)
{
    
    buffer_l[head] = SHARP_L;
    buffer_r[head] = SHARP_R;
    buffer_f[head] = SHARP_F;
    
    head ++;
    if(head >= BUF_LEN)
        head = 0;

    
    // sliding-average filter
    int l = 0;
    int r = 0;
    int f = 0;
    for(int i = 0; i < BUF_LEN; i++) {
        l += buffer_l[i];
        r += buffer_r[i];
        f += buffer_f[i];
    }
    l /= BUF_LEN;
    r /= BUF_LEN;
    f /= BUF_LEN;
    
    // Interpolate to distance
    // we need a good result from 2cm - 8cm, interpolate values for 4cm and 6cm
    l = (int) (400 - (l - 3090.0) / 8.81);
    r = (int) (400 - (r - 3090.0) / 8.81);
    f = (int) (400 - (f - 3090.0) / 8.81);
    
    //output values 
    *left = l;
    *right = r;
    *front = f;
}

/**
 * Get the wall positions
 * 
 * Outputs booleans indicating whether there are walls.
 * 
 * @param [OUT] left will be set to the reading of the left sensor
 * @param [OUT] front will be set to the reading of the front sensor
 * @param [OUT] right will be set to the reading of the right sensor
 * 
 */
void get_walls(int *left, int *front, int *right)
{
    
    buffer_l[head] = SHARP_L;
    buffer_r[head] = SHARP_R;
    buffer_f[head] = SHARP_F;
    
    head ++;
    if(head >= BUF_LEN)
        head = 0;

    
    // sliding-average filter
    int l = 0;
    int r = 0;
    int f = 0;
    for(int i = 0; i < BUF_LEN; i++) {
        l += buffer_l[i];
        r += buffer_r[i];
        f += buffer_f[i];
    }
    l /= BUF_LEN;
    r /= BUF_LEN;
    f /= BUF_LEN;
    
    int limit = 650;
    
    //output values 
    *left = SHARP_L < limit ? 0 : 1;
    *right = SHARP_R < limit ? 0 : 1;
    *front = SHARP_F < limit ? 0 : 1;
}