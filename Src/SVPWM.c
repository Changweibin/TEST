#include "SVPWM.h"
#include "fm33ld5xx.h"
#include "User_Param.h"
#include "arm_math.h"

SVPWM   SVPWM_dq            = SVPWM_DEFAULTS;

/*
** SVPWM计算    方法1
** 按照7段式SVPWM进行推导
** Txyz时间最终等效Tabc三相输入占空比
** Udc/sqrt(3) = 1  单位1
** 调制利用率取（1-0.04）=0.96=96%
** 等幅值变换
*/
//void SVPWM_Cale(p_SVPWM pv)
//{
//    /* 判断空间矢量Uout所在扇区 */
//    pv->u1 = pv->Ubeta;
//    pv->u2 = pv->Ubeta * 0.5f + pv->Ualpha * 0.8660254f;
//    pv->u3 = pv->u2 - pv->u1;
//    /* 根据三相电压符号计算矢量扇区 */
//    pv->VecSector = 3;
//    pv->VecSector = (pv->u2 > 0) ? (pv->VecSector-1) : pv->VecSector;
//    pv->VecSector = (pv->u3 > 0) ? (pv->VecSector-1) : pv->VecSector;
//    pv->VecSector = (pv->u1 < 0) ? (7-pv->VecSector) : pv->VecSector;

//    /* 根据矢量扇区计算矢量占空比Tabc */
//    if((pv->VecSector == 1) || (pv->VecSector == 4))
//    {
//        pv->Ta = pv->u2;
//        pv->Tb = pv->u1 - pv->u3;
//        pv->Tc = -pv->u2;
//        }
//    else if((pv->VecSector == 2) || (pv->VecSector ==5))
//    {
//        pv->Ta = pv->u3 + pv->u2;
//        pv->Tb = pv->u1;
//        pv->Tc = -pv->u1;
//    }
//    else if((pv->VecSector ==3) || (pv->VecSector == 6))
//    {
//        pv->Ta = pv->u3;
//        pv->Tb = -pv->u3;
//        pv->Tc = -(pv->u1 + pv->u2);
//    }
//    else                        /* 异常状态下的判断出的扇区 0---7或者其他就执行0电压矢量 */
//    {
//        pv->Ta = 0;
//        pv->Tb = 0;
//        pv->Tc = 0;
//    }

//    /* Tabc是带正负的浮点型数据，电压 pv->Tabc/Svpwm_Km是标幺值计算， 将Tabc计算成-1---1*/
//    /* 将占空比调节成-1---1，再将PWM半周期的占空比值提出 （-1---1）*50%+50% = 0-100% */
//    if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
//    {
//        PWM_TIM->CCR1 = (u16)(PWM_TIM_PULSE - ((u16)(pv->Ta * Svpwm_Km_Backw * PWM_TIM_PULSE_Half) + PWM_TIM_PULSE_Half));
//        PWM_TIM->CCR2 = (u16)(PWM_TIM_PULSE - ((u16)(pv->Tb * Svpwm_Km_Backw * PWM_TIM_PULSE_Half) + PWM_TIM_PULSE_Half));
//        PWM_TIM->CCR3 = (u16)(PWM_TIM_PULSE - ((u16)(pv->Tc * Svpwm_Km_Backw * PWM_TIM_PULSE_Half) + PWM_TIM_PULSE_Half));
//    }
//    else if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
//    {
//        PWM_TIM->CCR1 = (u16)((pv->Ta * Svpwm_Km_Backw * PWM_TIM_PULSE_Half) + PWM_TIM_PULSE_Half);
//        PWM_TIM->CCR2 = (u16)((pv->Tb * Svpwm_Km_Backw * PWM_TIM_PULSE_Half) + PWM_TIM_PULSE_Half);
//        PWM_TIM->CCR3 = (u16)((pv->Tc * Svpwm_Km_Backw * PWM_TIM_PULSE_Half) + PWM_TIM_PULSE_Half);
//    }
//}

void SVPWM_Cale(p_SVPWM pv)    /* 参考袁雷书  但其计算过程可能是以倒三角计数形式得出！ */
{
    pv->u1 =  pv->Ubeta;
    pv->u2 =  0.866025f * pv->Ualpha - 0.5f * pv->Ubeta;
    pv->u3 = -0.866025f * pv->Ualpha - 0.5f * pv->Ubeta;

    pv->VecSector = 4 * ((pv->u3 > 0) ? 1 : 0) + 2 * ((pv->u2 > 0) ? 1 : 0) + ((pv->u1 > 0) ? 1 : 0);

    if(pv->VecSector == 3)         /* sector 1 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * pv->u2;
        pv->Tb = Svpwm_Km_Backw * TS * pv->u1;

        /* 过调制处理 */            /* 按照公式计算Tx+Ty可能会大于Ts，所以要对实际的Tx和Ty进行修正 */
        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);    /* PWM1模式是否还是该限制条件？ */

        /* 零矢量作用时间 */
        pv->T0 = TS - pv->Ta - pv->Tb;

        if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2/TS*PWM_TIM_PULSE));
            PWM_TIM->CCR2 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
        }
        else if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)((pv->T0/2/TS*PWM_TIM_PULSE));
            PWM_TIM->CCR2 = (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
        }
    }
    else if(pv->VecSector == 1)        /* sector 2 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * (-pv->u2);
        pv->Tb = Svpwm_Km_Backw * TS * (-pv->u3);

        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);

        pv->T0 = TS - pv->Ta - pv->Tb;

        if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
        }
        else if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
        }
    }
    else if(pv->VecSector == 5)        /* sector 3 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * pv->u1;
        pv->Tb = Svpwm_Km_Backw * TS * pv->u3;

        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);

        pv->T0 = TS - pv->Ta - pv->Tb;

        if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
        }
        else if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
        }
    }
    else if(pv->VecSector == 4)        /* sector 4 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * (-pv->u1);
        pv->Tb = Svpwm_Km_Backw * TS * (-pv->u2);

        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);

        pv->T0 = TS - pv->Ta - pv->Tb;

        if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
        }
        else if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
        }
    }
    else if(pv->VecSector == 6)        /* sector 5 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * pv->u3;
        pv->Tb = Svpwm_Km_Backw * TS * pv->u2;

        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);

        pv->T0 = TS - pv->Ta - pv->Tb;

        if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
        }
        else if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
        }
    }
    else if(pv->VecSector == 2)        /* sector 6 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * (-pv->u3);
        pv->Tb = Svpwm_Km_Backw * TS * (-pv->u1);

        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);

        pv->T0 = TS - pv->Ta - pv->Tb;

        if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)PWM_TIM_PULSE - (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
        }
        else if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
        {
            PWM_TIM->CCR1 = (u16)((pv->T0/2)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR2 = (u16)((pv->T0/2 + pv->Ta + pv->Tb)/TS*PWM_TIM_PULSE);
            PWM_TIM->CCR3 = (u16)((pv->T0/2 + pv->Ta)/TS*PWM_TIM_PULSE);
        }
    }
    else
    {
        PWM_TIM->CCR1 = 0;
        PWM_TIM->CCR2 = 0;
        PWM_TIM->CCR3 = 0;
    }
}

///* 中点平移法SVPWM */
//void SVPWM_Cale(p_SVPWM pv)
//{
//    float Vmax_pu = 0, Vmin_pu = 0, Vcom_pu;

//    float oneOverDcBus_invV = 1.0f / ADC_Sample_Para.VBUS;

//    float Valpha_pu = pv->Ualpha * oneOverDcBus_invV;
//    float Vbeta_pu  = pv->Ubeta  * oneOverDcBus_invV;

//    float Va_tmp    = (float)(0.5f * Valpha_pu);
//    float Vb_tmp    = (float)(SVGEN_SQRT3_OVER_2 * Vbeta_pu);

//    float Va_pu     = Valpha_pu;
//    float Vb_pu     = -Va_tmp + Vb_tmp;        /* -0.5*Valpha + sqrt(3)/2*Vbeta */
//    float Vc_pu     = -Va_tmp - Vb_tmp;        /* -0.5*Valpha - sqrt(3)/2*Vbeta */

//    /* Find Vmax and Vmin */
//    if(Va_pu > Vb_pu)
//    {
//        Vmax_pu = Va_pu;
//        Vmin_pu = Vb_pu;
//    }
//    else
//    {
//        Vmax_pu = Vb_pu;
//        Vmin_pu = Va_pu;
//    }

//    if(Vc_pu > Vmax_pu)
//        Vmax_pu = Vc_pu;
//    else if(Vc_pu < Vmin_pu)
//        Vmin_pu = Vc_pu;

//    Vcom_pu = (float)(0.5f * (Vmax_pu + Vmin_pu));

//    pv->Ta = Va_pu - Vcom_pu;
//    pv->Tb = Vb_pu - Vcom_pu;
//    pv->Tc = Vc_pu - Vcom_pu;
//    
//    if(FL_ATIM_OC_MODE_PWM1 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
//    {
//        PWM_TIM->CCR1 = PWM_TIM_PULSE - (u16)((Limit_Sat(pv->Ta, 0.5f, -0.5f) + 0.5f) * PWM_TIM_PULSE);
//        PWM_TIM->CCR2 = PWM_TIM_PULSE - (u16)((Limit_Sat(pv->Tb, 0.5f, -0.5f) + 0.5f) * PWM_TIM_PULSE);
//        PWM_TIM->CCR3 = PWM_TIM_PULSE - (u16)((Limit_Sat(pv->Tc, 0.5f, -0.5f) + 0.5f) * PWM_TIM_PULSE);
//    }
//    else if(FL_ATIM_OC_MODE_PWM2 == FL_ATIM_OC_GetMode(PWM_TIM, FL_ATIM_CHANNEL_1))
//    {
//        PWM_TIM->CCR1 = (u16)((Limit_Sat(pv->Ta, 0.5f, -0.5f) + 0.5f) * PWM_TIM_PULSE);
//        PWM_TIM->CCR2 = (u16)((Limit_Sat(pv->Tb, 0.5f, -0.5f) + 0.5f) * PWM_TIM_PULSE);
//        PWM_TIM->CCR3 = (u16)((Limit_Sat(pv->Tc, 0.5f, -0.5f) + 0.5f) * PWM_TIM_PULSE);
//    }
//    
//}

