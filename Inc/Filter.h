#ifndef _FILTER_H_
#define _FILTER_H_
#include "main.h"
#include "fm33ld5xx_fl.h"
#include "fm33ld5xx.h"
#include "User_Param.h"

typedef struct
{
    float states0;
    float states1;
    float b0;
    float b1;
    float b2;
    float a0;
    float a1;
    float a2;
    float gain0;
    float gain1;
} IIR_BUTTERWORTH_DEF;

extern IIR_BUTTERWORTH_DEF WE_IIR_LPF_Para;

extern float WE_IIR_LPF_Coeff[8];

extern void IIR_ButterWorth_Coefficient_Init(float temp[8], IIR_BUTTERWORTH_DEF *iir_bw_lpf_temp);
extern void IIR_ButterWorth(float in, float *out, IIR_BUTTERWORTH_DEF *iir_bw_lpf_temp);

#endif    /* _FILTER_H_ */