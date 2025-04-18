#include "pll.h"
#define PLL_TIMEOUT      0xFFFFFFFFU

void CMU_IRQHandler(void)
{
    uint32_t SYSCLKSELErrorIT = 0;
    uint32_t SYSCLKSELErrorFlag = 0;

    SYSCLKSELErrorIT = FL_CMU_IsEnabledIT_SYSCKEWrong();
    SYSCLKSELErrorFlag = FL_CMU_IsActiveFlag_SYSCSEWrong();

    //时钟选择中断处理
    if((SYSCLKSELErrorIT == 0x01UL)
            && (SYSCLKSELErrorFlag == 0x01UL))
    {
        FL_CMU_ClearFlag_SYSCSEWrong();                    //清除标志
    }

}

/**
  * @brief
  * @rmtoll   PLLCR    OSEL    LL_RCC_GetPLLDigitalDomainOutput
  * @param    Source This parameter can be one of the following values:
  *           @arg @ref FL_CMU_PLL_CLK_SOURCE_RCHF
  *           @arg @ref FL_CMU_PLL_CLK_SOURCE_XTHF
  * @param    PLL_R 锁相环的参考时钟需要预分频至4M，再进行PLL倍频 This parameter can be one of the following values:
  *           @arg @ref FL_CMU_PLL_IPSC_DIV1
  *           @arg @ref FL_CMU_PLL_IPSC_DIV2
  *           @arg @ref FL_CMU_PLL_IPSC_DIV4
  *           @arg @ref FL_CMU_PLL_IPSC_DIV8
  *           @arg @ref FL_CMU_PLL_IPSC_DIV12
  *           @arg @ref FL_CMU_PLL_IPSC_DIV16
  *           @arg @ref FL_CMU_PLL_IPSC_DIV24
  *           @arg @ref FL_CMU_PLL_IPSC_DIV32

  */
void RCC_PLL_ConfigDomain_SYS(uint32_t Source, uint32_t PLL_R, uint32_t PLL_DB, uint8_t div)
{
    MODIFY_REG(CMU->PLLCR, CMU_PLLCR_OPSC_Msk | CMU_PLLCR_DB_Msk | CMU_PLLCR_IPRSC_Msk | CMU_PLLCR_INSEL_Msk,
               ((PLL_DB - 1) << CMU_PLLCR_DB_Pos) | PLL_R | Source | (div - 1)<< CMU_PLLCR_OPSC_Pos);
}

/**
  * @brief  选择内部RCHF作为锁相环的参考时钟
  *
  * @note   锁相环的参考时钟需要预分频至4M，再进行PLL倍频
  * @param  clock PLL倍频输出频率
  * @note   输出时钟频率为4M*（clock+1），最高输出频率可达168M
  * @retval none
  */
uint32_t SelRCHFToPLL(uint32_t PLLclock, uint8_t div)
{
    FL_NVIC_ConfigTypeDef    InterruptConfigStruct;

    uint32_t counter = 0;
    uint32_t readystatus = 0;
    uint32_t PLLdiv = FL_CMU_PLL_IPSC_DIV4;

    if(PLLclock > 168)
    {
        return 1;    //超出PLL倍频最高范围
    }
    else
    {

        FL_CMU_ClearFlag_SYSCSEWrong();                  //清除时钟选择错误标志
        FL_CMU_EnableIT_SYSCKEWrong();                   //使能时钟选择错误中断

        InterruptConfigStruct.preemptPriority = 0x00;
        FL_NVIC_Init(&InterruptConfigStruct, CMU_IRQn);    //CMU中断

        FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_2CYCLE);

        RCC_PLL_ConfigDomain_SYS(FL_CMU_PLL_CLK_SOURCE_RCHF, PLLdiv, PLLclock/4, div);

        FL_CMU_PLL_Enable();
        do
        {
            readystatus = FL_CMU_IsActiveFlag_PLLReady();
            counter++;

            if((counter % 100) == 0)
            {
                IWDT->SERV = 0x12345A5AUL;
            }

        } while((readystatus != 0x1U) && (counter != PLL_TIMEOUT));

        FL_CMU_SetAHBPrescaler(FL_CMU_AHBCLK_PSC_DIV1);
        FL_CMU_SetAPB1Prescaler(FL_CMU_APB1CLK_PSC_DIV1);

        if(PLLclock <= 24)
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_0CYCLE);    //  Flash读等待周期配置 0 wait cycle
        }
        else  if(PLLclock <= 48)
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_1CYCLE);    //  Flash读等待周期配置 1 wait cycle
        }
        else  if(PLLclock <= 72)
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_2CYCLE);    //  Flash读等待周期配置 2 wait cycle
        }
        else  if(PLLclock <= 96)
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_3CYCLE);    //  Flash读等待周期配置 3 wait cycle
        }
        else  if(PLLclock <= 120)
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_4CYCLE);    //  Flash读等待周期配置 4 wait cycle
        }
        else  if(PLLclock <= 144)
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_5CYCLE);    //  Flash读等待周期配置 5 wait cycle
        }
        else 
        {
            FL_FLASH_SetCodeReadWait(FLASH,FL_FLASH_READ_WAIT_6CYCLE);    //  Flash读等待周期配置 6 wait cycle
        }

        FL_CMU_SetSystemClockSource(FL_CMU_SYSTEM_CLK_SOURCE_PLL);
    }

    SystemCoreClockUpdate();

    return 0;
}
