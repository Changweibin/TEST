#include "Filter.h"

IIR_BUTTERWORTH_DEF WE_IIR_LPF_Para;

//float WE_IIR_LPF_Coeff[8] = {1.0f, 1.0f, 0.0f, 1.0f, -0.969067417193793f, 0.0f, 0.015466291403103f, 1.0f};    /* Ò»½×ButterWorthµÍÍ¨ÂË²¨Æ÷ÏµÊı */
//float WE_IIR_LPF_Coeff[8] = {1.0f, 1.0f, 0.0f, 1.0f, -0.987511929907314f, 0.0f, 0.006244035046342f, 1.0f};     //1½×20hz
float WE_IIR_LPF_Coeff[8] = {1.0f, 2.0f, 1.0f, 1.0f, -1.955578240315035f, 0.956543676511203f, 0.000241359049041f, 1.0f};

void IIR_ButterWorth_Coefficient_Init(float temp[8], IIR_BUTTERWORTH_DEF *iir_bw_lpf_temp)
{
    iir_bw_lpf_temp->b0 = temp[0];
    iir_bw_lpf_temp->b1 = temp[1];
    iir_bw_lpf_temp->b2 = temp[2];
    iir_bw_lpf_temp->a0 = temp[3];
    iir_bw_lpf_temp->a1 = temp[4];
    iir_bw_lpf_temp->a2 = temp[5];
    iir_bw_lpf_temp->gain0 = temp[6];
    iir_bw_lpf_temp->gain1 = temp[7];
    iir_bw_lpf_temp->states0 = 0.0f;
    iir_bw_lpf_temp->states1 = 0.0f;
}

void IIR_ButterWorth(float in, float *out, IIR_BUTTERWORTH_DEF *iir_bw_lpf_temp)
{
    float temp;
    temp = (iir_bw_lpf_temp->gain0 * in \
            - iir_bw_lpf_temp->a1 * iir_bw_lpf_temp->states0) \
            - (iir_bw_lpf_temp->a2 * iir_bw_lpf_temp->states1);
    *out = ((iir_bw_lpf_temp->b0 * temp + iir_bw_lpf_temp->b1 *iir_bw_lpf_temp->states0) + \
             iir_bw_lpf_temp->b2 *iir_bw_lpf_temp->states1) * iir_bw_lpf_temp->gain1;
             
    iir_bw_lpf_temp->states1 = iir_bw_lpf_temp->states0;
    iir_bw_lpf_temp->states0 = temp;
}
