#ifndef _FASTPID_H
#define _FASTPID_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


#define INTEG_MAX    (INT32_MAX)
#define INTEG_MIN    (INT32_MIN)
#define DERIV_MAX    (INT16_MAX)
#define DERIV_MIN    (INT16_MIN)

#define PARAM_SHIFT  8
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT) 
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)) >> (PARAM_BITS - PARAM_SHIFT)) 


struct FastPID
{
    // Configuration
    uint32_t _p, _i, _d;
    int64_t _outmax, _outmin; 
    bool _cfg_err; 
    
    // State
    int16_t _last_out;
    int64_t _sum;
    int32_t _last_err;
}pid;



void setCfgErr();
void clear();
bool setOutputRange(FastPID*, int16_t min , int16_t max);
bool setOutputConfig(FastPID*, int bits, bool sign);
uint32_t floatToParam(FastPID*, float in);
bool setCoefficients(FastPID*,  float kp, float ki, float kd, float hz);
bool configure(FastPID*, float kp, float ki, float kd, float hz, int bits, bool sign);
int16_t step(FastPID*, int16_t sp , int16_t cp);

#endif