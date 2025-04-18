#include "SVPWM.h"
#include "fm33ld5xx.h"
#include "User_Param.h"
#include "arm_math.h"

SVPWM   SVPWM_dq            = SVPWM_DEFAULTS;

/*
** SVPWM����    ����1
** ����7��ʽSVPWM�����Ƶ�
** Txyzʱ�����յ�ЧTabc��������ռ�ձ�
** Udc/sqrt(3) = 1  ��λ1
** ����������ȡ��1-0.04��=0.96=96%
** �ȷ�ֵ�任
*/
//void SVPWM_Cale(p_SVPWM pv)
//{
//    /* �жϿռ�ʸ��Uout�������� */
//    pv->u1 = pv->Ubeta;
//    pv->u2 = pv->Ubeta * 0.5f + pv->Ualpha * 0.8660254f;
//    pv->u3 = pv->u2 - pv->u1;
//    /* ���������ѹ���ż���ʸ������ */
//    pv->VecSector = 3;
//    pv->VecSector = (pv->u2 > 0) ? (pv->VecSector-1) : pv->VecSector;
//    pv->VecSector = (pv->u3 > 0) ? (pv->VecSector-1) : pv->VecSector;
//    pv->VecSector = (pv->u1 < 0) ? (7-pv->VecSector) : pv->VecSector;

//    /* ����ʸ����������ʸ��ռ�ձ�Tabc */
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
//    else                        /* �쳣״̬�µ��жϳ������� 0---7����������ִ��0��ѹʸ�� */
//    {
//        pv->Ta = 0;
//        pv->Tb = 0;
//        pv->Tc = 0;
//    }

//    /* Tabc�Ǵ������ĸ��������ݣ���ѹ pv->Tabc/Svpwm_Km�Ǳ���ֵ���㣬 ��Tabc�����-1---1*/
//    /* ��ռ�ձȵ��ڳ�-1---1���ٽ�PWM�����ڵ�ռ�ձ�ֵ��� ��-1---1��*50%+50% = 0-100% */
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

void SVPWM_Cale(p_SVPWM pv)    /* �ο�Ԭ����  ���������̿������Ե����Ǽ�����ʽ�ó��� */
{
    pv->u1 =  pv->Ubeta;
    pv->u2 =  0.866025f * pv->Ualpha - 0.5f * pv->Ubeta;
    pv->u3 = -0.866025f * pv->Ualpha - 0.5f * pv->Ubeta;

    pv->VecSector = 4 * ((pv->u3 > 0) ? 1 : 0) + 2 * ((pv->u2 > 0) ? 1 : 0) + ((pv->u1 > 0) ? 1 : 0);

    if(pv->VecSector == 3)         /* sector 1 */
    {
        pv->Ta = Svpwm_Km_Backw * TS * pv->u2;
        pv->Tb = Svpwm_Km_Backw * TS * pv->u1;

        /* �����ƴ��� */            /* ���չ�ʽ����Tx+Ty���ܻ����Ts������Ҫ��ʵ�ʵ�Tx��Ty�������� */
        MODIFY_TX_TY(pv->Ta, pv->Tb, pv->Tc, TS);    /* PWM1ģʽ�Ƿ��Ǹ����������� */

        /* ��ʸ������ʱ�� */
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

///* �е�ƽ�Ʒ�SVPWM */
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

