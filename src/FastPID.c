#include "FastPID.h"
#include <stdint.h>
#include <stdbool.h>

// refer to https://www.embeddedrelated.com/showarticle/1015.php to understand fixed point arithmetic 


void setCfgErr(FastPID* pid) 
{
  pid->_cfg_err = true;
  pid->_p = pid->_i = pid->_d = 0;
}

void clear(FastPID* pid)
{
    pid->_cfg_err = false;
    pid->_last_sp = 0; 
    pid->_last_out = 0;
    pid->_sum = 0; 
    pid->_last_err = 0;   
}


bool setOutputRange(FastPID* pid, int16_t min , int16_t max)
{
    if(min >= max)
    {
        setCfgErr(pid);
        return !pid->_cfg_err;
    }

    pid->_outmin = (int64_t)min * PARAM_MULT;
    pid->_outmax = (int64_t)max * PARAM_MULT;
    return ! pid->_cfg_err;
}


bool setOutputConfig(FastPID* pid, int bits, bool sign)
{
    if (bits > 16 || bits < 1) 
    {
        setCfgErr(pid);
    }
    else
    {
        if(bits==16)
        {
            pid->_outmax = (0xFFFFULL >> (17-bits)) * PARAM_MULT;
        }
        else
        {
            pid->_outmax = (0xFFFFULL >> (16 - bits)) * PARAM_MULT;
        }
        if (sign) 
        {
            pid->_outmin = -((0xFFFFULL >> (17 - bits)) + 1) * PARAM_MULT;
        }
        else 
        {
            pid->_outmin = 0;
        }
    }
    return ! pid->_cfg_err;
}


uint32_t floatToParam(FastPID* pid, float in)
{
    if( in > PARAM_MAX || in < 0)
    {
        pid->_cfg_err = true;
        return 0;
    }

    uint32_t param = in * PARAM_MULT; //Shifting the int value to upper 24 bits and fractional part to lower 8 bits

    if( in != 0 && param == 0 )
    {
        pid->_cfg_err = true;
        return 0;
    }

    return param;
}

bool setCoefficients(FastPID* pid, float kp, float ki, float kd, float hz)
{
    pid->_p = floatToParam(pid, pid->kp);
    pid->_i = floatToParam(pid, pid->ki / pid->hz);
    pid->_d = floatToParam(pid, pid->kd * pid->hz);

    return !pid->_cfg_err;
}


bool configure(FastPID* pid, float kp, float ki, float kd, float hz, int bits, bool sign)
{
    clear(pid);
    pid->_cfg_err = false;
    setCoefficients(pid, pid->kp, pid->ki, pid->kd, pid->hz);
    setOutputConfig(pid, pid->bits, pid->sign);
    return pid->_cfg_err;
}

int16_t step(FastPID* pid, int16_t sp , int16_t cp) //Setpoint , Currentpoint
{
    // int16 + int16 = int17
    int32_t err = (int32_t) cp - (int32_t) sp;
    int32_t P = 0, I = 0;
    int32_t D = 0;

    if(pid->_p)
    {
        // uint16 * int16 = int32
        P = (int32_t)pid->_p * (int32_t)err;
    }

    if(pid->_i)
    {
        // int17 * int16 = int33
        pid->_sum += (int64_t)err * (int64_t)pid->_i; 

        // Limit sum to 32-bit signed value so that it saturates, never overflows.
        if (pid->_sum > INTEG_MAX)
        {
          pid->_sum = INTEG_MAX;
        }
        else if (pid->_sum < INTEG_MIN)
        {
          pid->_sum = INTEG_MIN;
        }

        // int32
        I = pid->_sum;
    }

    if(pid->_d)
    {
        int32_t deriv = err - pid->_last_err;
        pid->_last_err = err;

        // Limit the derivative to 16-bit signed value.
        if (deriv > DERIV_MAX)
          deriv = DERIV_MAX;
        else if (deriv < DERIV_MIN)
          deriv = DERIV_MIN;

        D = (int32_t)deriv * (int32_t)pid->_d;
    }

    // int32 (P) + int32 (I) + int32 (D) = int34
    int64_t out = (int64_t)P + (int64_t)I + (int64_t)D;

    // Make the output saturate
    if (out > pid->_outmax) 
      out = pid->_outmax;
    else if (out < pid->_outmin) 
      out = pid->_outmin;

    // Remove the integer scaling factor. 
    int16_t rval = out >> PARAM_SHIFT;

    // Fair rounding.
    if (out & (0x1ULL << (PARAM_SHIFT - 1))) 
    {
      rval++;
    }

    return rval;

}