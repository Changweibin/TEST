#include "HFI.h"
#include "FOC.h"
#include "Filter.h"
#include "arm_math.h"
#include "PI_Cale.h"
#include "Coordinate_Trans.h"
#include "MotorCtr.h"

GRamp Speed_Ref_GRamp = GRAMP_DEFAULTS;

///* �����������ʼ�� */
HFI_Para    HFI          = HFI_PARA_DEFAULTS;
Angle_PLL   PLL_HFI_Para = {0};

void Grad_XieLv(p_GRamp pv)
{
    if(pv->Freq_Max < (pv->Freq - pv->Freq_delta))
        pv->Freq -= pv->Freq_delta;
    else if(pv->Freq_Max > (pv->Freq + pv->Freq_delta))
        pv->Freq += pv->Freq_delta;
    else
        pv->Freq = pv->Freq_Max;
}

void PLL_Cale_HFI(p_Angle_PLL pv, float alpha, float beta)
{
    /* ͨ��ʸ����˻�ýǶ���� */
    pv->Theta_Err = -(alpha * arm_sin_f32(pv->PLL_Theta)) + beta * arm_cos_f32(pv->PLL_Theta);
    /* ����PI���������ת�٣�ת��Ϊ�����Ƶ���ٶ� */
    pv->Omega += (pv->Kp * (pv->Theta_Err - pv->Theta_Err_Last) + TS * pv->Ki * pv->Theta_Err);    /* ����ʽPI */
    
//    pv->inter += TS * pv->Ki * pv->Theta_Err;
//    pv->Omega  = pv->inter + (pv->Kp * pv->Theta_Err);
    
    
    pv->Omega = Limit_Sat(pv->Omega, 210.0f, -210.0f);    //1000rpm
    
    /* ����ٶ��˲� */
//    IIR_ButterWorth(PLL_HFI_Para.Omega, &PLL_HFI_Para.OmegaF, &WE_IIR_LPF_Para);

    pv->OmegaF = pv->OmegaF * 0.96954 + pv->Omega * 0.03046f;

    
    /* ��¼�Ƕ���� */
    pv->Theta_Err_Last = pv->Theta_Err;
    
    /* ���ֻ�ȡ��Ƕ� */
    pv->PLL_Theta += TS * pv->OmegaF;
    /* �Ե�ǶȽ����޷� */
    if(pv->PLL_Theta > PIX2)
        pv->PLL_Theta -= PIX2;
    else if(pv->PLL_Theta < 0)
        pv->PLL_Theta += PIX2;
}

//HFI��ʼ��
void HFI_Init(void)
{
    HFI.Theta = 0;
    HFI.Ialpha_F = 0;
    HFI.Ialpha_H = 0;
    HFI.Ialpha_H_A = 0;
    HFI.Ialpha_H_Last = 0;
    HFI.Ialpha_Last = 0;
    HFI.Ibeta_F = 0;
    HFI.Ibeta_H = 0;
    HFI.Ibeta_H_A = 0;
    HFI.Ibeta_H_Last = 0;
    HFI.Ibeta_Last = 0;
    HFI.Id_F = 0;
    HFI.Id_H = 0;
    HFI.Id_S = 0;
    HFI.Id_S_Last = 0;
    HFI.Iq_F = 0;
    HFI.Iq_S = 0;
    HFI.Iq_S_Last = 0;
    
    HFI.Uinj = 0;
    
    /* Kp=2�Ʀ�0/k2  Ki=(��0)^2/k2  K2����  ����50Hz ��ȡ1 */
    PLL_HFI_Para.Kp       = 2 * 0.707f * 314;
    PLL_HFI_Para.Ki       = 314 * 314;
    
    PLL_HFI_Para.PLL_Theta = 0;
}

//HFI�Ƕȼ���
void HFI_Angle_Cale(p_HFI pv)
{
    pv->Ialpha_H = (CLARKE_ICurr.Alpha - pv->Ialpha_Last);
    pv->Ibeta_H  = (CLARKE_ICurr.Beta  - pv->Ibeta_Last);
    
    pv->Ialpha_Last = CLARKE_ICurr.Alpha;
    pv->Ibeta_Last  = CLARKE_ICurr.Beta;

    /* ������ */
    pv->SignVal = Sign(pv->Uinj);
    pv->Ialpha_H_A = (pv->Ialpha_H) * pv->SignVal;
    pv->Ibeta_H_A  = (pv->Ibeta_H)  * pv->SignVal;

    
//    pv->Ialpha_H = (CLARKE_ICurr.Alpha - 2 * pv->Ialpha_Last + pv->Ialpha_LLast) / 4.0f;
//    pv->Ibeta_H  = (CLARKE_ICurr.Beta  - 2 * pv->Ibeta_Last  + pv->Ibeta_LLast)  / 4.0f  ;

//    pv->Ialpha_LLast = pv->Ialpha_Last;
//    pv->Ibeta_LLast  = pv->Ibeta_LLast;

//    pv->Ialpha_Last = CLARKE_ICurr.Alpha;
//    pv->Ibeta_Last  = CLARKE_ICurr.Beta;

//    /* ������ */
//    pv->SignVal = Sign(pv->Uinj);
//    pv->Ialpha_H_A = (pv->Ialpha_H - pv->Ialpha_H_Last) * pv->SignVal;
//    pv->Ibeta_H_A  = (pv->Ibeta_H  - pv->Ibeta_H_Last)  * pv->SignVal;
//    
//    /* ��¼��Ƶ�������� */
//    pv->Ialpha_H_Last = pv->Ialpha_H;
//    pv->Ibeta_H_Last  = pv->Ibeta_H;

    PLL_Cale_HFI((p_Angle_PLL)&PLL_HFI_Para, pv->Ialpha_H_A, pv->Ibeta_H_A);    /* ������Ϊ�˼���SMO PLL */
}

//ת�ӳ�ʼ�Ƕ��뼫�Ա�ʶ
bool HFI_Uinj_Flag = 0;

void HFI_Init_Theta(void)
{
    static u16    HFI_Clock = 0;
    static float  SUM_Id1   = 0;
    static float  SUM_Id2   = 0;
    static u8     HFICnt    = 0;
    
//    if(++HFICnt == 2)
    {
        HFICnt = 0;
        HFI_Clock++;
        HFI_Uinj_Flag = !HFI_Uinj_Flag;
    }
    
    HFI.Id_S = PARK_PCurr.Ds;
    HFI.Iq_S = PARK_PCurr.Qs;
    
    /* ��ȡ�������� */
    HFI.Id_F = (HFI.Id_S + HFI.Id_S_Last) * 0.5f;
    HFI.Iq_F = (HFI.Iq_S + HFI.Iq_S_Last) * 0.5f;
    
    /* ��ȡ��Ƶ������Ӧ���� */
    HFI.Id_H = (HFI.Id_S - HFI.Id_S_Last) * 0.5f;
    
//    /* ��ȡ�������� */
//    HFI.Id_F = (HFI.Id_S + 2 * HFI.Id_S_Last + HFI.Id_S_LLast) / 4.0f;
//    HFI.Iq_F = (HFI.Iq_S + 2 * HFI.Iq_S_Last + HFI.Iq_S_LLast) / 4.0f;
//    
//    /* ��ȡ��Ƶ������Ӧ���� */
//    HFI.Id_H = (HFI.Id_S - 2 * HFI.Id_S_Last + HFI.Id_S_LLast) / 4.0f;
    
    /* ��¼����ֵ */
    HFI.Id_S_LLast = HFI.Id_S_Last;
    HFI.Iq_S_LLast = HFI.Iq_S_Last;
    HFI.Id_S_Last = HFI.Id_S;
    HFI.Iq_S_Last = HFI.Iq_S;
    
//    if(HFI_Clock < 200)    /* 20msע��(Tsamp = 100us) ʶ��ĳ�ʼ��ǶȽ������� */
//    {
//        pi_id.Ref = 0;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//    }
//    else if(HFI_Clock == 200)    /* ��ʼ��Ƕȸ�ֵ */
//    {
//        HFI.Theta_Init = PLL_HFI_Para.PLL_Theta;
//        pi_id.Ref = 0;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//    }
//    else if((HFI_Clock > 200) && (HFI_Clock <= 300))    /* 10ms Idfƫ�õ���ע�� */
//    {
//        pi_id.Ref = HFI_Id_Offset;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//    }
//    else if((HFI_Clock > 300) && (HFI_Clock <= 320))    /* �ۼ�10�� ��Ƶ��������*Id_offset */
//    {
//        pi_id.Ref = HFI_Id_Offset;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//        SUM_Id1 += my_abs(HFI.Id_H * 10 *HFI_Id_Offset);
//    }
//    else if((HFI_Clock > 320) && (HFI_Clock <= 420))    /* 10ms Idƫ�õ������� */
//    {
//        pi_id.Ref = 0;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//    }
//    else if((HFI_Clock > 420) && (HFI_Clock <= 520))    /* 10ms Id��ƫ�õ���ע�� */
//    {
//        pi_id.Ref = -HFI_Id_Offset;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//    }
//    else if((HFI_Clock > 520) && (HFI_Clock <= 530))    /* �ۼ�10�� ��Ƶ��������*Id_offset */
//    {
//        pi_id.Ref = -HFI_Id_Offset;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//        SUM_Id2 += my_abs(-HFI.Id_H * 10 *HFI_Id_Offset);
//    }

    if(HFI_Clock < 1000)    /* 200msע��(Tsamp = 100us) ʶ��ĳ�ʼ��ǶȽ������� */
    {
        pi_id.Ref = 0;
        pi_id.Fbk = HFI.Id_F;
        PI_Controller((p_PI_Control)&pi_id);
    }
    else if(HFI_Clock == 1000)    /* ��ʼ��Ƕȸ�ֵ */
    {
        HFI.Theta_Init = PLL_HFI_Para.PLL_Theta;
        pi_id.Ref = 0;
        pi_id.Fbk = HFI.Id_F;
        PI_Controller((p_PI_Control)&pi_id);
    }
    else if((HFI_Clock > 1000) && (HFI_Clock <= 1200))    /* 10ms Idfƫ�õ���ע�� */
    {
        pi_id.Ref = HFI_Id_Offset;
        pi_id.Fbk = HFI.Id_F;
        PI_Controller((p_PI_Control)&pi_id);
        
        SUM_Id1 += my_abs(HFI.Id_H * 10 *HFI_Id_Offset);
    }
//    else if((HFI_Clock > 300) && (HFI_Clock <= 320))    /* �ۼ�10�� ��Ƶ��������*Id_offset */
//    {
//        pi_id.Ref = HFI_Id_Offset;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//        SUM_Id1 += my_abs(HFI.Id_H * 10 *HFI_Id_Offset);
//    }
//    else if((HFI_Clock > 2100) && (HFI_Clock <= 2200))    /* 10ms Idƫ�õ������� */
//    {
//        pi_id.Ref = 0;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//    }
    else if((HFI_Clock > 1200) && (HFI_Clock <= 1400))    /* 10ms Id��ƫ�õ���ע�� */
    {
        pi_id.Ref = -HFI_Id_Offset;
        pi_id.Fbk = HFI.Id_F;
        PI_Controller((p_PI_Control)&pi_id);
        
        SUM_Id2 += my_abs(HFI.Id_H * 10 *HFI_Id_Offset);
    }
//    else if((HFI_Clock > 520) && (HFI_Clock <= 530))    /* �ۼ�10�� ��Ƶ��������*Id_offset */
//    {
//        pi_id.Ref = -HFI_Id_Offset;
//        pi_id.Fbk = HFI.Id_F;
//        PI_Controller((p_PI_Control)&pi_id);
//        SUM_Id2 += my_abs(-HFI.Id_H * 10 *HFI_Id_Offset);
//    }
    else if(HFI_Clock > 1400)    /* ��ʼ�Ƕ�ʶ����� ����SUM_Id1&2�Ĵ�С�ж�������N�� OR S�� */
    {
        if(SUM_Id1 < SUM_Id2)    /* ������S�� */
        {
            HFI.Theta_Init = PLL_HFI_Para.PLL_Theta + PI;
            if(HFI.Theta_Init >PIX2)
                HFI.Theta_Init -= PIX2;

            PLL_HFI_Para.PLL_Theta = HFI.Theta_Init;
        }

        SUM_Id1 = 0;
        SUM_Id2 = 0;

        pi_id.Ref = 0;
        pi_id.Fbk = HFI.Id_F;
        PI_Controller((p_PI_Control)&pi_id);

        HFI_Clock = 0;

        /* �л�����״̬ */
        Motor.M_State = M_STATE_HFI;
        
//        Motor.M_State = M_STATE_IDLE;
//        motor_start_stop = 0;
////        /* ͣ�� */
////        MCtr_STOP();
//        
////        /* ����� */
////        Variables_Clr();
    }

    /* ������Ƶע�� */
    if(HFI_Uinj_Flag)
    {
        HFI.Uinj = HFI_Uinj_Offset;
    }
    else
    {
        HFI.Uinj = -HFI_Uinj_Offset;
    }

    /* IPARK�任 */
    IPARK_PVdq.Theta = PLL_HFI_Para.PLL_Theta;
    IPARK_PVdq.VQs = 0.0f;
    IPARK_PVdq.VDs = pi_id.Out + HFI.Uinj;
    IPARK_Cale((p_IPARK)&IPARK_PVdq);
}



s16 HFI_Speed = 150;    /* HFIĿ��ת�� */
void HFI_Start_Ctr(void)
{
    float Speed_Ref_EX = 0.0f;
    float outMax_V;
    
    Speed_Ref_GRamp.Freq_delta = Speed_Ref_OD;    /* �����ٶȵ��ݶ�ֵ */
    Speed_Ref_GRamp.Ramp_Timer = Speed_Ref_Timer; /* �����ٶȵ��ݶ�ʱ�� */
    Speed_Ref_GRamp.Freq_Max   = HFI_Speed;       /* �����ݶ����ޣ����ٶȻ�Ŀ��ת�� */
    
    Speed_Ref_GRamp.Timer_Count++;
    if(Speed_Ref_GRamp.Timer_Count > Speed_Ref_GRamp.Ramp_Timer)
    {
        Speed_Ref_GRamp.Timer_Count = 0;
        Grad_XieLv((p_GRamp)&Speed_Ref_GRamp);
    }
    Speed_Ref_EX = Limit_Sat(Speed_Ref_GRamp.Freq, HFI_Speed_Ref_Max, -HFI_Speed_Ref_Max);
    
    HFI_Uinj_Flag = !HFI_Uinj_Flag;
    
    HFI.Id_S = PARK_PCurr.Ds;
    HFI.Iq_S = PARK_PCurr.Qs;
    
    /* ��ȡ�������� */
    HFI.Id_F = (HFI.Id_S + HFI.Id_S_Last) * 0.5f;
    HFI.Iq_F = (HFI.Iq_S + HFI.Iq_S_Last) * 0.5f;
    
    
//    /* ��ȡ�������� */
//    HFI.Id_F = (HFI.Id_S + 2 * HFI.Id_S_Last + HFI.Id_S_LLast) / 4.0f;
//    HFI.Iq_F = (HFI.Iq_S + 2 * HFI.Iq_S_Last + HFI.Iq_S_LLast) / 4.0f;
    
    
    /* ��¼����ֵ */
    HFI.Id_S_LLast = HFI.Id_S_Last;
    HFI.Iq_S_LLast = HFI.Iq_S_Last;
    HFI.Id_S_Last = HFI.Id_S;
    HFI.Iq_S_Last = HFI.Iq_S;
    
    if(++Motor.SpdClsCnt == IRP_PERIOD)
    {
        Motor.SpdClsCnt = 0;
        
//        pi_spd.Ref = Speed_Ref_EX * Motor.Pn;
        pi_spd.Ref = (600 * Motor.Pn);
        pi_spd.Fbk = -PLL_HFI_Para.OmegaF / PIX2 * 60;
        PI_Controller((p_PI_Control)&pi_spd);
    }
    
    pi_id.Ref = 0;
    pi_id.Fbk = HFI.Id_F;
    PI_Controller((p_PI_Control)&pi_id);
    
    /* ��ֹ������ */
    float maxVsMag_V = MaxVsMagPu * 24.0f;//ADC_Sample_Para.VBUS;
    outMax_V = (maxVsMag_V * maxVsMag_V) - (pi_id.Out * pi_id.Out);
    arm_sqrt_f32(outMax_V, &outMax_V);

    pi_iq.Umax = outMax_V;
    pi_iq.Umin = -outMax_V;
    pi_iq.Ref  = pi_spd.Out;
    pi_iq.Fbk  = HFI.Iq_F;
    PI_Controller((p_PI_Control)&pi_iq);

    /* d��ע���Ƶ������ѹ */
    if(HFI_Uinj_Flag)
        HFI.Uinj = HFI_Uinj_Offset;
    else
        HFI.Uinj = -HFI_Uinj_Offset;

    /* PARK��任 */
    IPARK_PVdq.Theta = PLL_HFI_Para.PLL_Theta;
    IPARK_PVdq.VQs = pi_iq.Out;
    IPARK_PVdq.VDs = pi_id.Out + HFI.Uinj;
    IPARK_Cale((p_IPARK)&IPARK_PVdq);
}

