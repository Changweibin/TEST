#include "main.h"
#include "pll.h"
#include "MotorCtr.h"
#include "PI_Cale.h"
#include "FOC.h"
#include "User_Param.h"
#include "user_process.h"
//dfsdfdsfsdfsdf
void FOUT_Init(uint32_t select)
{
    FL_GPIO_InitTypeDef         GPIO_InitStruct={0};
    //ATIM IO ����
    GPIO_InitStruct.pin           = FL_GPIO_PIN_15;//ATIM 
    GPIO_InitStruct.mode          = FL_GPIO_MODE_DIGITAL;
    GPIO_InitStruct.outputType    = FL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.pull          = FL_GPIO_BOTH_DISABLE;
    GPIO_InitStruct.remapPin      = FL_GPIO_PINREMAP_FUNCTON0;
    GPIO_InitStruct.driveStrength = FL_GPIO_DRIVESTRENGTH_HIGH;
    (void)FL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    FL_GPIO_SetFOUT1(GPIO,select);
}


int main(void)
{
    /* Initialize FL Driver Library */
    /* SHOULD BE KEPT!!! */
    FL_Init();

    CMU->RCHFTR =0x71;
    (void)SelRCHFToPLL(168, 4);   //�Ƚ�RCHF��Ƶ��4M��������PLL���໷��Ƶ, PLLʱ�������Ƶ4�������Ƶ��Χ1~8��

//    FOUT_Init(FL_GPIO_FOUT1_SELECT_PLL_DIV64);  //�۲�Ƶ�ʣ�ʵ��Ӧ�ÿ��Բ�д
    
    SysTim_Init();
    Motor_GPIO_init();
    ADC_init();
    TIM_init();
    
    /* ���������ʼ�� */
    Motor_Param_Init();
    /* PI������ʼ�� */
    PI_Pare_Init();
    
    if(0 == get_offset_flag)
    {
        get_offset_flag = 1;
        TIM_CtrlPWMOutputs(PWM_TIM, FL_DISABLE);
        TIM_Cmd(PWM_TIM, FL_ENABLE);
    }
    
    #if __FPU_USED == 0
        #error "__FPU_USED == 0"
    #endif
    
    Motor.Init_Over = 1;

    while(1)
    {
        FL_IWDT_ReloadCounter(IWDT);
        
        /* ʱ������״̬�� */
        if(1 == TaskTimePare.IntClock_1ms)    /* 1ms */
        {
            TaskTimePare.IntClock_1ms = 0;
            
            /* ��ȡ��ѹ����ֵ */
            ADC_start_regular_conv();

            if(++TaskTimePare.Tim10ms_count == 10)
            {
                TaskTimePare.Tim10ms_count = 0;
                TaskTimePare.Tim10ms_flag  = 1;
            }
        }
        if(1 == TaskTimePare.Tim10ms_flag)    /* 10ms */
        {
            TaskTimePare.Tim10ms_flag = 0;
            
            motorCtr();

//            if(Motor.M_State == M_STATE_SPDCLS)
//                ACC_SPEED();
            
            if(++TaskTimePare.Tim100ms_count == 10)
            {
                TaskTimePare.Tim100ms_count = 0;
                TaskTimePare.Tim100ms_flag  = 1;
            }
        }
        if(1 == TaskTimePare.Tim100ms_flag)   /* 100ms */
        {
            TaskTimePare.Tim100ms_flag = 0;
            
            /* 100ms Task */
            motorCtrStateFlow();

//            Vbus_Check_Proc();    /* ��ѹ��� */
            
            if(++TaskTimePare.Tim500ms_count == 5)
            {
                TaskTimePare.Tim500ms_count = 0;
                TaskTimePare.Tim500ms_flag  = 1;
            }
        }
        if(1 == TaskTimePare.Tim500ms_flag)   /* 500ms */
        {
            TaskTimePare.Tim500ms_flag = 0;
            
            /* 500ms Task */
//            Fault_Code_Proc();    /* ������ϴ��� */
            
            if(++TaskTimePare.Tim1s_count == 2)
            {
                TaskTimePare.Tim1s_count = 0;
                TaskTimePare.Tim1s_flag  = 1;
            }
        }
        if(1 == TaskTimePare.Tim1s_flag)      /* 1s */
        {
            TaskTimePare.Tim1s_flag = 0;
            
            /* 1s Task */
//            LED1_TOG();
        }
    }
}
