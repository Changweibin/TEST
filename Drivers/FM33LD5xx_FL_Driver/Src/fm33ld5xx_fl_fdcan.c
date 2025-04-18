 /**    
  *******************************************************************************************************
  * @file    fm33ld5xx_fl_fdcan.c
  * @author  FMSH Application Team
  * @brief   Source file of CANFD FL Module
  *******************************************************************************************************   
  * @attention    
  * Copyright (c) 2023, SHANGHAI FUDAN MICROELECTRONICS GROUP CO., LTD.(FUDAN MICROELECTRONICS./ FUDAN MICRO.)    
  * All rights reserved.    
  *    
  * Processor:                   FM33LD5xx    
  * http:                        http://www.fmdevelopers.com.cn/    
  *    
  * Redistribution and use in source and binary forms, with or without    
  * modification, are permitted provided that the following conditions are met    
  *    
  * 1. Redistributions of source code must retain the above copyright notice,    
  *    this list of conditions and the following disclaimer.    
  *    
  * 2. Redistributions in binary form must reproduce the above copyright notice,    
  *    this list of conditions and the following disclaimer in the documentation    
  *    and/or other materials provided with the distribution.    
  *    
  * 3. Neither the name of the copyright holder nor the names of its contributors    
  *    may be used to endorse or promote products derived from this software    
  *    without specific prior written permission.    
  *    
  * 4. To provide the most up-to-date information, the revision of our documents     
  *    on the World Wide Web will be the most Current. Your printed copy may be      
  *    an earlier revision. To verify you have the latest information avaliable,    
  *    refer to: http://www.fmdevelopers.com.cn/.    
  *    
  * THIS SOFTWARE IS PROVIDED BY FUDAN MICRO "AS IS" AND ANY EXPRESSED:     
  * ORIMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES     
  * OF MERCHANTABILITY NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE    
  * ARE DISCLAIMED.IN NO EVENT SHALL FUDAN MICRO OR ITS CONTRIBUTORS BE LIABLE     
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL     
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS     
  * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER    
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,     
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISINGIN ANY WAY OUT OF THE     
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OFTHE POSSIBILITY OF SUCH DAMAGE.    
  *
  *******************************************************************************************************
  */    

/* Includes ------------------------------------------------------------------*/
#include "fm33ld5xx_fl.h"
/** @addtogroup FM33LD5XX_FL_Driver
  * @{
  */

/** @addtogroup FDCAN
  * @{
  */

#ifdef FL_FDCAN_DRIVER_ENABLED

/* Private macros ------------------------------------------------------------*/
/** @addtogroup CANFD_FL_Private_Macros
  * @{
  */


#define     IS_CANFD_INSTANCE(INSTANCE)                 ((INSTANCE) == CANFD0) //|| ((INSTANCE) == CANFD1))
                                                          
#define     IS_CANFD_GLOBAL_MODE(__VALUE__)             (((__VALUE__) == FL_CANFD_GLOBAL_MODE_OPERATION)    ||\
                                                          ((__VALUE__) == FL_CANFD_GLOBAL_MODE_RESET)       ||\
                                                          ((__VALUE__) == FL_CANFD_GLOBAL_MODE_HALT)        ||\
                                                          ((__VALUE__) == FL_CANFD_GLOBAL_MODE_KEEP_CURRENT))                                                         
                                                          
#define     IS_CANFD_CHANNEL_MODE(__VALUE__)            (((__VALUE__) == FL_CANFD_CHANNEL_MODE_OPERATION)  ||\
                                                          ((__VALUE__) == FL_CANFD_CHANNEL_MODE_RESET)      ||\
                                                          ((__VALUE__) == FL_CANFD_CHANNEL_MODE_HALT)       ||\
                                                          ((__VALUE__) == FL_CANFD_CHANNEL_MODE_KEEP_CURRENT))                                                          
                                                         
#define     IS_CANFD_NB_SJW(__VALUE__)                  ((__VALUE__) <= 0x7FU)
                                                         
#define     IS_CANFD_NB_TS1(__VALUE__)                  (((__VALUE__) > 0U) &&\
                                                          ((__VALUE__) <= 0xFFU))
                                                          
#define     IS_CANFD_NB_TS2(__VALUE__)                  (((__VALUE__) > 0U) &&\
                                                          ((__VALUE__) <= 0x7FU))

#define     IS_CANFD_DB_SJW(__VALUE__)                  ((__VALUE__) <= 0x0FU)
                                                         
#define     IS_CANFD_DB_TS1(__VALUE__)                  (((__VALUE__) > 0U) &&\
                                                          ((__VALUE__) <= 0x1FU))
                                                          
#define     IS_CANFD_DB_TS2(__VALUE__)                  (((__VALUE__) > 0) &&\
                                                          ((__VALUE__) <= 0x0FU))
                                                         
#define     IS_CANFD0_CLOCK_SOURCE(__VALUE__)           ((__VALUE__) == FL_CMU_CANFD0_CLK_SOURCE_APBCLK)                                                                                                                     
                                                          

#define     IS_CANFD0_CLOCK_SOURCE_RAM(__VALUE__)       ((__VALUE__) == FL_CMU_CANFD0_CLK_SOURCE_RAM_AHBCLK)  
                                                          

#define     IS_CANFD_RXMB_PLS(__VALUE__)                (((__VALUE__) == FL_CANFD_RMPLS_SIZE_8)     ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_12)   ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_16)   ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_20)   ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_24)   ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_32)   ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_48)   ||\
                                                          ((__VALUE__) == FL_CANFD_RMPLS_SIZE_64))
                                                          
#define     IS_CANFD_RXMB_NUM(__VALUE__)                (((__VALUE__) <= 32U))                                                          
                                                          
#define     IS_CANFD_RXFIFO_IGCV(__VALUE__)             (((__VALUE__) == FL_CANFD_RFIGCV_1P8_FULL)  ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_1P4_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_3P8_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_1P2_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_5P8_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_3P4_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_7P8_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIGCV_FULL))

#define     IS_CANFD_RXFIFO_IM(__VALUE__)               (((__VALUE__) == FL_CANFD_RFIM_REACH_RFIGCV)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFIM_EVERY_MESSAGE))
                                                          
#define     IS_CANFD_RXFIFO_DC(__VALUE__)               (((__VALUE__) == FL_CANFD_RFDC_DEPTH_0)     ||\
                                                          ((__VALUE__) == FL_CANFD_RFDC_DEPTH_4)    ||\
                                                          ((__VALUE__) == FL_CANFD_RFDC_DEPTH_8)    ||\
                                                          ((__VALUE__) == FL_CANFD_RFDC_DEPTH_16)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFDC_DEPTH_32)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFDC_DEPTH_48))

#define     IS_CANFD_RXFIFO_PLS(__VALUE__)              (((__VALUE__) == FL_CANFD_RFPLS_SIZE_8)     ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_12)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_16)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_20)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_24)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_32)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_48)   ||\
                                                          ((__VALUE__) == FL_CANFD_RFPLS_SIZE_64))
                                                          
#define     IS_CANFD_COMMONFIFO_DC(__VALUE__)           (((__VALUE__) == FL_CANFD_CFDC_DEPTH_0)     ||\
                                                          ((__VALUE__) == FL_CANFD_CFDC_DEPTH_4)    ||\
                                                          ((__VALUE__) == FL_CANFD_CFDC_DEPTH_8)    ||\
                                                          ((__VALUE__) == FL_CANFD_CFDC_DEPTH_16)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFDC_DEPTH_32)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFDC_DEPTH_48))

#define     IS_CANFD_COMMONFIFO_TML(__VALUE__)          (((__VALUE__) == FL_CANFD_CFTML_TXMB0)  ||\
                                                          ((__VALUE__) == FL_CANFD_CFTML_TXMB1) ||\
                                                          ((__VALUE__) == FL_CANFD_CFTML_TXMB2) ||\
                                                          ((__VALUE__) == FL_CANFD_CFTML_TXMB3))
                                                          
#define     IS_CANFD_COMMONFIFO_IGCV(__VALUE__)         (((__VALUE__) == FL_CANFD_CFIGCV_1P8_FULL)  ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_1P4_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_3P8_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_1P2_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_5P8_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_3P4_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_7P8_FULL)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIGCV_FULL))

#define     IS_CANFD_COMMONFIFO_IM(__VALUE__)           (((__VALUE__) == FL_CANFD_CFIM_REACH_RFIGCV)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFIM_EVERY_MESSAGE))
                                                          
#define     IS_CANFD_COMMONFIFO_ITR(__VALUE__)          (((__VALUE__) == FL_CANFD_CFITR_REF_CLOCK_MUL_1)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFITR_REF_CLOCK_MUL_10))

#define     IS_CANFD_COMMONFIFO_ITSS(__VALUE__)         (((__VALUE__) == FL_CANFD_CFITSS_REF_CLOCK)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFITSS_BIT_CLOCK))

#define     IS_CANFD_COMMONFIFO_MODE(__VALUE__)         (((__VALUE__) == FL_CANFD_CFM_RX_FIFO)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFM_TX_FIFO))
                                                          
#define     IS_CANFD_COMMONFIFO_PLS(__VALUE__)          (((__VALUE__) == FL_CANFD_CFPLS_SIZE_8)     ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_12)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_16)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_20)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_24)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_32)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_48)   ||\
                                                          ((__VALUE__) == FL_CANFD_CFPLS_SIZE_64))                                                          
                                                          
#define     IS_CANFD_TXQUEUE_DC(__VALUE__)              (((__VALUE__) == FL_CANFD_TXQDC_DEPTH_0)    ||\
                                                          ((__VALUE__) == FL_CANFD_TXQDC_DEPTH_3)   ||\
                                                          ((__VALUE__) == FL_CANFD_TXQDC_DEPTH_4))                                                          
                                                          
#define     IS_CANFD_TXQUEUE_IM(__VALUE__)              (((__VALUE__) == FL_CANFD_TXQIM_LAST_MESSAGE_TRANSMITTED)   ||\
                                                          ((__VALUE__) == FL_CANFD_TXQIM_EVERY_TRANSMISSION))                                                          
                                                          
#define     IS_CANFD_TXHISTORYLIST_IM(__VALUE__)        (((__VALUE__) == FL_CANFD_THLIM_REACH_3P4_DEPTH)   ||\
                                                          ((__VALUE__) == FL_CANFD_THLIM_EVERY_TRANSMISSION))

#define     IS_CANFD_GAFL_PAGE_NUM(__VALUE__)           (((__VALUE__) == FL_CANFD_GAFL_PAGE_NUMBER_FIRST)    ||\
                                                         ((__VALUE__) == FL_CANFD_GAFL_PAGE_NUMBER_SECOND))

#define     IS_CANFD_FILTER_filterRPNMD(__VALUE__)          (((__VALUE__) == FL_CANFD_RETURN_PNF_MODE_AF)     	 ||\
                                                          ((__VALUE__) == FL_CANFD_RETURN_PNF_MODE_PNFID_AF) ||\
                                                          ((__VALUE__) == FL_CANFD_RETURN_PNF_MODE_PNF_AF)   ||\
                                                          ((__VALUE__) == FL_CANFD_RETURN_PNF_MODE_PNF))  

#define     IS_CANFD_GAFL_RULE_NUM(__VALUE__)           (((__VALUE__) <= 32U))
                                                          
#define     IS_CANFD_GAFL_IDE(__VALUE__)                (((__VALUE__) == FL_CANFD_GAFL_IDE_STANDARD)   ||\
                                                          ((__VALUE__) == FL_CANFD_GAFL_IDE_EXTENDED))

#define     IS_CANFD_GAFL_RTR(__VALUE__)                (((__VALUE__) == FL_CANFD_GAFL_RTR_DATA_FRAME)   ||\
                                                          ((__VALUE__) == FL_CANFD_GAFL_RTR_REMOTE_FRAME))
                                                          
#define     IS_CANFD_GAFL_IDEM(__VALUE__)               (((__VALUE__) == FL_CANFD_GAFL_IDEM_NOT_CONSIDERED)   ||\
                                                          ((__VALUE__) == FL_CANFD_GAFL_IDEM_CONSIDERED))

#define     IS_CANFD_GAFL_RTRM(__VALUE__)               (((__VALUE__) == FL_CANFD_GAFL_RTRM_NOT_CONSIDERED)   ||\
                                                          ((__VALUE__) == FL_CANFD_GAFL_RTRM_CONSIDERED))

#define     IS_CANFD_GAFL_SINGLE_RXMB(__VALUE__)        (((__VALUE__) == FL_CANFD_GAFL_SINGLE_RXMB_INVALID)   ||\
                                                          ((__VALUE__) == FL_CANFD_GAFL_SINGLE_RXMB_VALID))
                                                          
#define     IS_CANFD_GAFL_RXMB_NUM(__VALUE__)           ((__VALUE__) <= FL_CANFD_RXMB31)                                    
            
#define     IS_CANFD_TX_PRIORITY(__VALUE__)             (((__VALUE__) == FL_CANFD_TX_PRIORITY_ID) ||\
                                                          ((__VALUE__) == FL_CANFD_TX_PRIORITY_BUFFER))
                                                          
#define     IS_CANFD_ESI_CFG(__VALUE__)                 (((__VALUE__) == FL_CANFD_ESI_CONFIGURATION_NODE_STATE) ||\
                                                          ((__VALUE__) == FL_CANFD_ESI_CONFIGURATION_SOFTWARE))
                                                          
#define     IS_CANFD_TDCO_CFG(__VALUE__)                (((__VALUE__) == FL_CANFD_TDCO_CONFIGURATION_MEASURED_WITH_OFFSET) ||\
                                                          ((__VALUE__) == FL_CANFD_TDCO_CONFIGURATION_OFFSET))
                                                          
#define     IS_CANFD_FUNSTATE_TYPE(__VALUE__)           (((__VALUE__) == FL_DISABLE) ||\
                                                          ((__VALUE__) == FL_ENABLE))
														  
// PFL 
#define     IS_CANFD_GPFL_RULE_NUM(__VALUE__)           (((__VALUE__) <= 2U))
                                                          
#define     IS_CANFD_GPFL_IDE(__VALUE__)                (((__VALUE__) == FL_CANFD_GPFL_IDE_STANDARD)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFL_IDE_EXTENDED))

#define     IS_CANFD_GPFL_RTR(__VALUE__)                (((__VALUE__) == FL_CANFD_GPFL_RTR_DATA_FRAME)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFL_RTR_REMOTE_FRAME))
                                                          
#define     IS_CANFD_GPFL_IDEM(__VALUE__)               (((__VALUE__) == FL_CANFD_GPFL_IDEM_NOT_CONSIDERED)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFL_IDEM_CONSIDERED))

#define     IS_CANFD_GPFL_RTRM(__VALUE__)               (((__VALUE__) == FL_CANFD_GPFL_RTRM_NOT_CONSIDERED)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFL_RTRM_CONSIDERED))

#define     IS_CANFD_GPFL_SINGLE_RXMB(__VALUE__)        (((__VALUE__) == FL_CANFD_GPFL_SINGLE_RXMB_INVALID)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFL_SINGLE_RXMB_VALID))	
														  
#define     IS_CANFD_GPFL_RXMB_NUM(__VALUE__)           ((__VALUE__) <= FL_CANFD_RXMB31) 


#define     IS_CANFD_GPFL_GPFLANDOR(__VALUE__)        	(((__VALUE__) == FL_CANFD_GPFLANDOR_AND)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFLANDOR_OR))	

#define     IS_CANFD_GPFL_GPFLRANG0(__VALUE__)        	(((__VALUE__) == FL_CANFD_GPFLRANG0_MATCH)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFLRANG0_RANGE))

#define     IS_CANFD_GPFL_GPFLOUT0(__VALUE__)        	(((__VALUE__) == FL_CANFD_GPFLOUT0_WITHIN)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFLOUT0_OUTSIDE))
														  
#define     IS_CANFD_GPFL_OFFSET0(__VALUE__)           	((__VALUE__) <= 16) 

#define     IS_CANFD_GPFL_GPFLRANG1(__VALUE__)        	(((__VALUE__) == FL_CANFD_GPFLRANG1_MATCH)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFLRANG1_RANGE))

#define     IS_CANFD_GPFL_GPFLOUT1(__VALUE__)        	(((__VALUE__) == FL_CANFD_GPFLOUT1_WITHIN)   ||\
                                                          ((__VALUE__) == FL_CANFD_GPFLOUT1_OUTSIDE))
														  
#define     IS_CANFD_GPFL_OFFSET1(__VALUE__)           	((__VALUE__) <= 16) 






/**
  * @}
  */

/* Private consts ------------------------------------------------------------*/
/** @addtogroup CANFD_FL_Private_Consts
  * @{
  */

const uint8_t RXMB_INDEX_START[4] = {0, 8, 16, 24};


/*	
	CANFD_BUFFER[] (结构体数组)
	│
	└── [0] (CANFD0 的配置)
    ├── RX_FIFO          	→ CANFD0_RX_FIFO (CANFD_RX_FIFO_Type)
    │	└── CFDRF[0]     	→ 操作接收 FIFO 的第1个寄存器
	|	└── CFDRF[1]	  	→ 操作接收 FIFO 的第2个寄存器
    ├── COMMON_FIFO      	→ CANFD0_COMMON_FIFO
    ├── TX_MESSAGE       	→ CANFD0_TX_MESSAGE
	|	└── CFDTM[0]		→ 操作TX MESSAGE BUFF 的第1个寄存器
	|	└── CFDTM[1]		→ ......
	|	└── CFDTM[2]		→ ......
	|	└── CFDTM[3]		→ ......
    ├── TX_HISTORY_LIST  	→ CANFD0_TX_HISTORY_LIST
    └── RX_MESSAGE[4]    	→ {CANFD0_RX_MESSAGE0, ..., CANFD0_RX_MESSAGE3}
	|	└── CFDRM[0]		→ 操作RX MESSAGE BUFF 的第1个寄存器
	|	└── CFDRM[1]		→ ......
	|	......
	|	└── CFDRM[7]		→ ......
	
	
	typedef struct
	{
		CANFD_RX_FIFO_Type         * RX_FIFO;
		CANFD_COMMON_FIFO_Type     * COMMON_FIFO;
		CANFD_TX_MESSAGE_Type      * TX_MESSAGE;
		CANFD_TX_HISTORY_LIST_Type * TX_HISTORY_LIST;
		CANFD_RX_MESSAGE_Type      * RX_MESSAGE[4];
	}CANFD_BUFFER_TYPE;
*/
const CANFD_BUFFER_TYPE CANFD_BUFFER[] = 
{
	{	/* CANFD0 */
		CANFD0_RX_FIFO, 			
		CANFD0_COMMON_FIFO, 		
		CANFD0_TX_MESSAGE, 			
		CANFD0_TX_HISTORY_LIST, 	
		CANFD0_RX_MESSAGE0, 		
		CANFD0_RX_MESSAGE1, 		
		CANFD0_RX_MESSAGE2, 		
		CANFD0_RX_MESSAGE3
	}
    
    /* CANFD1 */
//    {CANFD1_RX_FIFO, CANFD1_COMMON_FIFO, CANFD1_TX_MESSAGE, CANFD1_TX_HISTORY_LIST, \
//    {CANFD1_RX_MESSAGE0, CANFD1_RX_MESSAGE1, CANFD1_RX_MESSAGE2, CANFD1_RX_MESSAGE3}}
};

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/



/* Exported functions --------------------------------------------------------*/
/** @addtogroup CANFD_FL_EF_Init
  * @{
  */

/**
  * @brief  CANFD初始化
  * @param  CANFDx外设入口地址
  * @param  CANFD_InitStruct 指向一个@ref FL_CANFD_InitTypeDef  结构体的指针
  * @retval 错误状态可能值
  *         -FL_FAIL 配置过程发生错误
  *         -FL_PASS 配置成功
  */
FL_ErrorStatus FL_CANFD_Init(CANFD_COMMON_Type *CANFDx, FL_CANFD_InitTypeDef *CANFD_InitStruct)
{
    uint32_t i, index;
    FL_ErrorStatus status = FL_FAIL;
    
    if(CANFD_InitStruct != NULL)
    {
        /*参数检查*/
        assert_param(IS_CANFD_INSTANCE(CANFDx));
        
        /*全局模式和通道模式检查*/
        assert_param(IS_CANFD_GLOBAL_MODE(CANFD_InitStruct->globalMode));
        assert_param(IS_CANFD_CHANNEL_MODE(CANFD_InitStruct->channelMode));
        
        if ((CANFD_InitStruct->globalMode == FL_CANFD_GLOBAL_MODE_RESET) &&\
            (CANFD_InitStruct->channelMode != FL_CANFD_CHANNEL_MODE_RESET))
        {
            return FL_FAIL;
        }
        
        /* 位速率参数检查 */
        assert_param(IS_CANFD_NB_SJW(CANFD_InitStruct->NB_SJW));
        assert_param(IS_CANFD_NB_TS1(CANFD_InitStruct->NB_TS1));
        assert_param(IS_CANFD_NB_TS2(CANFD_InitStruct->NB_TS2));
        assert_param(IS_CANFD_DB_SJW(CANFD_InitStruct->DB_SJW));
        assert_param(IS_CANFD_DB_TS1(CANFD_InitStruct->DB_TS1));
        assert_param(IS_CANFD_DB_TS2(CANFD_InitStruct->DB_TS2));
        
        /* 时钟源检查 */

        assert_param(IS_CANFD0_CLOCK_SOURCE(CANFD_InitStruct->clockSource));
        assert_param(IS_CANFD0_CLOCK_SOURCE_RAM(CANFD_InitStruct->clockSourceRAM));


        /* RxMB参数检查 */
        assert_param(IS_CANFD_RXMB_PLS(CANFD_InitStruct->rxmbCfg.payloadDataSize));
        assert_param(IS_CANFD_RXMB_NUM(CANFD_InitStruct->rxmbCfg.rxmbNum));
        
        /* RxFIFO参数检查 */
        for (i = 0; i < 2; i++)
        {
            if (CANFD_InitStruct->rxfifoCfg[i].Enable == FL_ENABLE)
            {
                assert_param(IS_CANFD_RXFIFO_IGCV(CANFD_InitStruct->rxfifoCfg[i].intGenCounterValue));
                assert_param(IS_CANFD_RXFIFO_IM(CANFD_InitStruct->rxfifoCfg[i].intMode));
                assert_param(IS_CANFD_RXFIFO_DC(CANFD_InitStruct->rxfifoCfg[i].fifoDepth));
                assert_param(IS_CANFD_RXFIFO_PLS(CANFD_InitStruct->rxfifoCfg[i].payloadDataSize));
            }
        }
        
        /* CommonFIFO参数检查 */
        if (CANFD_InitStruct->commonfifoCfg.Enable == FL_ENABLE)
        {
            assert_param(IS_CANFD_COMMONFIFO_DC(CANFD_InitStruct->commonfifoCfg.fifoDepth));
            assert_param(IS_CANFD_COMMONFIFO_TML(CANFD_InitStruct->commonfifoCfg.txmbLink));
            assert_param(IS_CANFD_COMMONFIFO_IGCV(CANFD_InitStruct->commonfifoCfg.intGenCounterValue));
            assert_param(IS_CANFD_COMMONFIFO_IM(CANFD_InitStruct->commonfifoCfg.intMode));
            assert_param(IS_CANFD_COMMONFIFO_ITR(CANFD_InitStruct->commonfifoCfg.intervalTimerResolution));
            assert_param(IS_CANFD_COMMONFIFO_ITSS(CANFD_InitStruct->commonfifoCfg.intervalTimerSource));
            assert_param(IS_CANFD_COMMONFIFO_MODE(CANFD_InitStruct->commonfifoCfg.fifoMode));
            assert_param(IS_CANFD_COMMONFIFO_PLS(CANFD_InitStruct->commonfifoCfg.payloadDataSize));
        }
        
        /* TxQueue参数检查 */
        if (CANFD_InitStruct->txqueueCfg.Enable == FL_ENABLE)
        {
            assert_param(IS_CANFD_TXQUEUE_DC(CANFD_InitStruct->txqueueCfg.queueDepth));
            assert_param(IS_CANFD_TXQUEUE_IM(CANFD_InitStruct->txqueueCfg.intMode));
        }
        
        /* CommonFIFO和TxQueue参数冲突检查 */
        if (CANFD_InitStruct->commonfifoCfg.fifoMode == FL_CANFD_CFM_TX_FIFO)
        {
            if ((CANFD_InitStruct->txqueueCfg.queueDepth == FL_CANFD_TXQDC_DEPTH_4) ||\
                ((CANFD_InitStruct->txqueueCfg.queueDepth == FL_CANFD_TXQDC_DEPTH_3) && (CANFD_InitStruct->commonfifoCfg.txmbLink != FL_CANFD_CFTML_TXMB3)))
            {
                return FL_FAIL;
            }
        }

        /* TxHistoryList参数检查 */
        if (CANFD_InitStruct->txHistoryCfg.Enable == FL_ENABLE)
        {
            assert_param(IS_CANFD_TXHISTORYLIST_IM(CANFD_InitStruct->txHistoryCfg.intMode));
        }
        
		
		/* 滤波器模式参数检查 */
		assert_param(IS_CANFD_FILTER_filterRPNMD(CANFD_InitStruct->filterRPNMD));
        /* AcceptanceFilterList参数检查，包含32个AFL */
        assert_param(IS_CANFD_GAFL_RULE_NUM(CANFD_InitStruct->acceptanceFilterRuleNum));
        for (i = 0; i < 32; i++)
        {
            if (CANFD_InitStruct->acceptanceFilterCfg[i].filterEnable == FL_ENABLE)
            {
                assert_param(IS_CANFD_GAFL_IDE(CANFD_InitStruct->acceptanceFilterCfg[i].filterIDE));
                assert_param(IS_CANFD_GAFL_RTR(CANFD_InitStruct->acceptanceFilterCfg[i].filterRTR));
                assert_param(IS_CANFD_GAFL_IDEM(CANFD_InitStruct->acceptanceFilterCfg[i].filterIDEMask));
                assert_param(IS_CANFD_GAFL_RTRM(CANFD_InitStruct->acceptanceFilterCfg[i].filterRTRMask));
                assert_param(IS_CANFD_GAFL_SINGLE_RXMB(CANFD_InitStruct->acceptanceFilterCfg[i].rxmbValid));
                assert_param(IS_CANFD_GAFL_RXMB_NUM(CANFD_InitStruct->acceptanceFilterCfg[i].rxmbNum));
            }
        }
		
		
		/* Pretended Network Filter List 参数检查，包含2个filter */
        assert_param(IS_CANFD_GPFL_RULE_NUM(CANFD_InitStruct->pflFilterRuleNum));
        for (i = 0; i < 2; i++)
        {
            if (CANFD_InitStruct->pflFilterCfg[i].PFL_Enable == FL_ENABLE)
            {
                assert_param(IS_CANFD_GPFL_IDE(CANFD_InitStruct->pflFilterCfg[i].PFL_IDE));
                assert_param(IS_CANFD_GPFL_RTR(CANFD_InitStruct->pflFilterCfg[i].PFL_RTR));
                assert_param(IS_CANFD_GPFL_IDEM(CANFD_InitStruct->pflFilterCfg[i].PFL_IDEMask));
                assert_param(IS_CANFD_GPFL_RTRM(CANFD_InitStruct->pflFilterCfg[i].PFL_RTRMask));
                assert_param(IS_CANFD_GPFL_SINGLE_RXMB(CANFD_InitStruct->pflFilterCfg[i].PFL_rxmbValid));
                assert_param(IS_CANFD_GPFL_RXMB_NUM(CANFD_InitStruct->pflFilterCfg[i].PFL_rxmbNum));
				assert_param(IS_CANFD_GPFL_GPFLANDOR(CANFD_InitStruct->pflFilterCfg[i].PFL_AND_OR));
				assert_param(IS_CANFD_GPFL_GPFLRANG0(CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLRANG0));
				assert_param(IS_CANFD_GPFL_GPFLOUT0(CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLOUT0));
				assert_param(IS_CANFD_GPFL_OFFSET0(CANFD_InitStruct->pflFilterCfg[i].PFL_OFFSET0));
				assert_param(IS_CANFD_GPFL_GPFLRANG1(CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLRANG1));
				assert_param(IS_CANFD_GPFL_GPFLOUT1(CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLOUT1));
				assert_param(IS_CANFD_GPFL_OFFSET1(CANFD_InitStruct->pflFilterCfg[i].PFL_OFFSET1));
            }
        }
        
        /* 特殊参数检查 */
        assert_param(IS_CANFD_TX_PRIORITY(CANFD_InitStruct->txPriority));
        assert_param(IS_CANFD_ESI_CFG(CANFD_InitStruct->esiCfg));
        assert_param(IS_CANFD_TDCO_CFG(CANFD_InitStruct->tdcoCfg));
        assert_param(IS_CANFD_FUNSTATE_TYPE(CANFD_InitStruct->tdcoEnable));
        assert_param(IS_CANFD_FUNSTATE_TYPE(CANFD_InitStruct->respedEnable));
        assert_param(IS_CANFD_FUNSTATE_TYPE(CANFD_InitStruct->dlcCheckEnable));
        assert_param(IS_CANFD_FUNSTATE_TYPE(CANFD_InitStruct->dlcReplacementEnable));
        assert_param(IS_CANFD_FUNSTATE_TYPE(CANFD_InitStruct->messageIsRejected));
        

		
		/* ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 参数开始配置 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ */
        /* 时钟总线配置，CMU中配置 */
		FL_CMU_EnableGroup3BusClock(FL_CMU_GROUP3_BUSCLK_CANFD0);
		FL_CMU_SetCANFD0ClockSource(CANFD_InitStruct->clockSource);
		FL_CMU_SetCANFD0ClockSourceRam(CANFD_InitStruct->clockSourceRAM);
		FL_CMU_EnableCANFDOperationClock(FL_CMU_OPCLK_CANFD_COMM);
		FL_CMU_EnableCANFDOperationClock(FL_CMU_OPCLK_CANFD_RAM);


        
        /* 等待RAM初始化完成 */
		/* 25.16.13 CFDGSTS[3]:FDCAN RAM初始化完成标志，所有操作均需要等待RAM init finish后进行操作 */
        while(FL_CANFD_Global_IsActiveFlag_GlobalRAMInitialisation(CANFDx))
        {
            /* 直至看门狗溢出复位 
			 * 1’b0: RAM initialisation is finished
			 * 1’b1: RAM initialisation ongoing
			 */
			/* 增加超时退出逻辑 */
			// User Code......
        }
        
        /* 主动复位 */
        FL_CANFD_Global_WriteKeyOfReset(CANFDx, 0xC4);						/* 25.16.52 CFDGRSTC[15:8]:软件复位key(0xC4) */
        FL_CANFD_Global_SetSoftwareReset(CANFDx, FL_CANFD_SRST_RESET);		/* 25.16.52 CFDGRSTC[0]:软件置位 SRST 将使 RS-CANFD 模块全局复位，软件清零撤销复位 */
        FL_CANFD_Global_SetSoftwareReset(CANFDx, FL_CANFD_SRST_NO_RESET);	/* 软件复位不会导致RAM重新初始化 */
        
        /* 退出global sleep和channel sleep */
        FL_CANFD_Global_Disable_Sleep(CANFDx);
        FL_CANFD_CH0_Disable_Sleep(CANFDx);
        
        /* 进入global reset和channel reset */
		/* 部分寄存器，用户只能在 CH_RESET 和 CH_HALT 模式下改写，所以必须先进入RESET状态 */
        FL_CANFD_Global_SetGlobalMode(CANFDx, FL_CANFD_GLOBAL_MODE_RESET);
        FL_CANFD_CH0_SetChannelMode(CANFDx, FL_CANFD_CHANNEL_MODE_RESET);
        
        /* 配置位速率 */
        FL_CANFD_CH0_WriteNominalBitrateTimeSegment1Length(CANFDx, CANFD_InitStruct->NB_TS1);
        FL_CANFD_CH0_WriteNominalBitrateTimeSegment2Length(CANFDx, CANFD_InitStruct->NB_TS2);
        FL_CANFD_CH0_WriteNominalBitrateSyncJumpWidth(CANFDx, CANFD_InitStruct->NB_SJW);
        FL_CANFD_CH0_WriteNominalBitratePrescaler(CANFDx, CANFD_InitStruct->NB_BRP);
        FL_CANFD_CH0_WriteDataBitrateTimeSegment1Length(CANFDx, CANFD_InitStruct->DB_TS1);
        FL_CANFD_CH0_WriteDataBitrateTimeSegment2Length(CANFDx, CANFD_InitStruct->DB_TS2);
        FL_CANFD_CH0_WriteDataBitrateSyncJumpWidth(CANFDx, CANFD_InitStruct->DB_SJW);
        FL_CANFD_CH0_WriteDataBitratePrescaler(CANFDx, CANFD_InitStruct->DB_BRP);        
        
        /* 配置RxMB */
        FL_CANFD_Global_SetRxMBPayloadDataSize(CANFDx, CANFD_InitStruct->rxmbCfg.payloadDataSize);
        FL_CANFD_Global_WriteNumberOfRxMB(CANFDx, CANFD_InitStruct->rxmbCfg.rxmbNum);
        
        /* 配置RxFIFO */
        for (i = 0; i < 2; i++)
        {
            if (CANFD_InitStruct->rxfifoCfg[i].Enable == FL_ENABLE)
            {
                FL_CANFD_Global_SetRxFIFOInterruptGenerationCounterValue(CANFDx, i, CANFD_InitStruct->rxfifoCfg[i].intGenCounterValue);
                FL_CANFD_Global_SetRxFIFOInterruptMode(CANFDx, i, CANFD_InitStruct->rxfifoCfg[i].intMode);
                FL_CANFD_Global_SetRxFIFODepth(CANFDx, i, CANFD_InitStruct->rxfifoCfg[i].fifoDepth);
                FL_CANFD_Global_SetRxFIFOPayloadDataSize(CANFDx, i, CANFD_InitStruct->rxfifoCfg[i].payloadDataSize);
                if (CANFD_InitStruct->rxfifoCfg[i].intEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_EnableIT_RxFIFO(CANFDx, i);                    
                }
                else
                {
                    FL_CANFD_Global_DisableIT_RxFIFO(CANFDx, i);
                }
                /* RxFIFO使能必须在OPERATION模式下配置 */
            }
        }
        
        /* 配置CommonFIFO */
        if (CANFD_InitStruct->commonfifoCfg.Enable == FL_ENABLE)
        {
            FL_CANFD_Global_WriteCommonFIFOIntervalTransmissionTime(CANFDx, CANFD_InitStruct->commonfifoCfg.txInterval);
            FL_CANFD_Global_SetCommonFIFODepth(CANFDx, CANFD_InitStruct->commonfifoCfg.fifoDepth);
            FL_CANFD_Global_SetCommonFIFOTxMBLink(CANFDx, CANFD_InitStruct->commonfifoCfg.txmbLink);
            FL_CANFD_Global_SetCommonFIFOInterruptGenerationCounterValue(CANFDx, CANFD_InitStruct->commonfifoCfg.intGenCounterValue);
            FL_CANFD_Global_SetCommonFIFOInterruptMode(CANFDx, CANFD_InitStruct->commonfifoCfg.intMode);
            FL_CANFD_Global_SetCommonFIFOIntervalTimerResolution(CANFDx, CANFD_InitStruct->commonfifoCfg.intervalTimerResolution);
            FL_CANFD_Global_SetCommonFIFOIntervalTimerSource(CANFDx, CANFD_InitStruct->commonfifoCfg.intervalTimerSource);
            FL_CANFD_Global_SetCommonFIFOMode(CANFDx, CANFD_InitStruct->commonfifoCfg.fifoMode);
            FL_CANFD_Global_SetCommonFIFOPayloadDataSize(CANFDx, CANFD_InitStruct->commonfifoCfg.payloadDataSize);
            if (CANFD_InitStruct->commonfifoCfg.intEnable == FL_ENABLE)
            {
                if (CANFD_InitStruct->commonfifoCfg.fifoMode == FL_CANFD_CFM_TX_FIFO)
                {
                    FL_CANFD_Global_DisableIT_CommonFIFORx(CANFDx);
                    FL_CANFD_Global_EnableIT_CommonFIFOTx(CANFDx);
                }
                else
                {
                    FL_CANFD_Global_DisableIT_CommonFIFOTx(CANFDx);
                    FL_CANFD_Global_EnableIT_CommonFIFORx(CANFDx);
                }
            }            
            /* CommonFIFO使能必须在OPERATION模式下配置 */
        }

        /* 配置TxQueue */
        if (CANFD_InitStruct->txqueueCfg.Enable == FL_ENABLE)
        {
            FL_CANFD_Global_SetTxQueueDepth(CANFDx, CANFD_InitStruct->txqueueCfg.queueDepth);
            FL_CANFD_Global_SetTxQueueInterruptMode(CANFDx, CANFD_InitStruct->txqueueCfg.intMode);
            if (CANFD_InitStruct->txqueueCfg.intEnable == FL_ENABLE)
            {
                FL_CANFD_Global_EnableIT_TxQueueTx(CANFDx);
            }
            else
            {
                FL_CANFD_Global_DisableIT_TxQueueTx(CANFDx);
            }
            /* TxQueue使能必须在OPERATION模式下配置 */
        }
        
        /* 配置TxHistoryList */
        if (CANFD_InitStruct->txHistoryCfg.Enable == FL_ENABLE)
        {
            if (CANFD_InitStruct->txHistoryCfg.dedicatedTxEnable == FL_ENABLE)
            {
                FL_CANFD_Global_Enable_TxHistoryListDedicatedTx(CANFDx);
            }
            else
            {
                FL_CANFD_Global_Disable_TxHistoryListDedicatedTx(CANFDx);
            }
            
            FL_CANFD_Global_SetTxHistoryListInterruptMode(CANFDx, CANFD_InitStruct->txHistoryCfg.intMode);
            if (CANFD_InitStruct->txHistoryCfg.intEnable == FL_ENABLE)
            {
                FL_CANFD_Global_EnableIT_TxHistoryList(CANFDx);
            }
            else
            {
                FL_CANFD_Global_DisableIT_TxHistoryList(CANFDx);
            }
            /* TxQueue使能必须在OPERATION模式下配置 */
        }
       
		
        /* 接收滤波器配置,AFL和PFL组合用法 */
			
		/* 25.16.17 CFDGAFLECTR[8]:接收滤波器列表配置进入控制寄存器，使能后才可以配置滤波器参数*/
        FL_CANFD_Global_Enable_GAFLDataAccess(CANFDx);			
		/* 25.16.17 CFDGAFLECTR[0]：滤波器列表页码 0 or 1*/
        FL_CANFD_Global_SetGAFLPageNumber(CANFDx, FL_CANFD_GAFL_PAGE_NUMBER_FIRST);
		/* 25.16.18 CFDGAFLCFG[21:16]:接收规则数目 1~32 */
        FL_CANFD_Global_WriteGAFLRuleNumber(CANFDx, CANFD_InitStruct->acceptanceFilterRuleNum);
		
        for (i = 0; i < 32; i++)	/* 共32个AFL元素 */
        {
            if (i == 16)
            {
				/* 当配置规则数超过半数时，设置页面到 second page */
                FL_CANFD_Global_SetGAFLPageNumber(CANFDx, FL_CANFD_GAFL_PAGE_NUMBER_SECOND);
            }
            
            index = i & 0xF;	/* 只取i的低半字节，或者2个page的配置索引元素号 */
            if (CANFD_InitStruct->acceptanceFilterCfg[i].filterEnable == FL_ENABLE)		
            {
                FL_CANFD_Global_SetGAFLIDE(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].filterIDE);
                FL_CANFD_Global_SetGAFLRTR(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].filterRTR);
                FL_CANFD_Global_WriteGAFLID(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].filterID);
                FL_CANFD_Global_SetGAFLIDEMask(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].filterIDEMask);
                FL_CANFD_Global_SetGAFLRTRMask(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].filterRTRMask);
                FL_CANFD_Global_WriteGAFLIDMask(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].filterIDMask);
                FL_CANFD_Global_SetGAFLRxMBValid(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].rxmbValid);
                FL_CANFD_Global_WriteGAFLRxMBDirectionPointer(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].rxmbNum);
                FL_CANFD_Global_WriteGAFLDLC(CANFDx, index, CANFD_InitStruct->acceptanceFilterCfg[i].dlc);
                if (CANFD_InitStruct->acceptanceFilterCfg[i].commonfifoRecvEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_GAFLCommonFIFOReception(CANFDx, index);
                }
                else
                {
                    FL_CANFD_Global_Disable_GAFLCommonFIFOReception(CANFDx, index);
                }
                if (CANFD_InitStruct->acceptanceFilterCfg[i].rxfifo0RecvEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_GAFLRxFIFOReception(CANFDx, index, FL_CANFD_RXFIFO0);
                }
                else
                {
                    FL_CANFD_Global_Disable_GAFLRxFIFOReception(CANFDx, index, FL_CANFD_RXFIFO0);
                }
                if (CANFD_InitStruct->acceptanceFilterCfg[i].rxfifo1RecvEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_GAFLRxFIFOReception(CANFDx, index, FL_CANFD_RXFIFO1);
                }
                else
                {
                    FL_CANFD_Global_Disable_GAFLRxFIFOReception(CANFDx, index, FL_CANFD_RXFIFO1);
                }                
            }
        }
		/* 25.16.17 CFDGAFLECTR[8]:接收滤波器列表配置进入控制寄存器，关闭后保护滤波器参数不被修改*/
        FL_CANFD_Global_Disable_GAFLDataAccess(CANFDx);
		
		
		/* Pretended Network Filter List 配置 */ 
		/* 25.16.58 CFDGPFLECTR[8]:接收滤波器列表配置进入控制寄存器，使能后才可以配置滤波器参数*/
        FL_CANFD_Global_Enable_GPFLDataAccess(CANFDx);			
		/* 25.16.59 CFDGPFLCFG[25:24]:接收规则数目 1~2 */
        FL_CANFD_Global_WriteGPFLRuleNumber(CANFDx, CANFD_InitStruct->pflFilterRuleNum);
		/*   i改为已经定义的rule num*/
        for (i = 0; i < 2; i++)	/* 共2个PFL元素,	0:filter0,	1:filter1 */
        {
            if (CANFD_InitStruct->pflFilterCfg[i].PFL_Enable == FL_ENABLE)		
            { 
                FL_CANFD_Global_SetGPFLIDE(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_IDE);
                FL_CANFD_Global_SetGPFLRTR(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_RTR);
                FL_CANFD_Global_WriteGPFLID(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_ID);
                FL_CANFD_Global_SetGPFLIDEMask(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_IDEMask);
                FL_CANFD_Global_SetGPFLRTRMask(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_RTRMask);
                FL_CANFD_Global_WriteGPFLIDMask(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_IDMask);
                FL_CANFD_Global_SetGPFLRxMBValid(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_rxmbValid);
                FL_CANFD_Global_WriteGPFLRxMBDirectionPointer(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_rxmbNum);
                FL_CANFD_Global_WriteGPFLDLC(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_dlc);
                if (CANFD_InitStruct->pflFilterCfg[i].PFL_commonfifoRecvEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_GPFLCommonFIFOReception(CANFDx, i);
                }
                else
                {
                    FL_CANFD_Global_Disable_GPFLCommonFIFOReception(CANFDx, i);
                }
                if (CANFD_InitStruct->pflFilterCfg[i].PFL_rxfifo0RecvEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_GPFLRxFIFOReception(CANFDx, i, FL_CANFD_RXFIFO0);
                }
                else
                {
                    FL_CANFD_Global_Disable_GPFLRxFIFOReception(CANFDx, i, FL_CANFD_RXFIFO0);
                }
                if (CANFD_InitStruct->pflFilterCfg[i].PFL_rxfifo1RecvEnable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_GPFLRxFIFOReception(CANFDx, i, FL_CANFD_RXFIFO1);
                }
                else
                {
                    FL_CANFD_Global_Disable_GPFLRxFIFOReception(CANFDx, i, FL_CANFD_RXFIFO1);
                } 

				/* PFL Payload filter config */
				FL_CANFD_Global_SetGPFLConditionsOfFilters(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_AND_OR);
				FL_CANFD_Global_SetGPFLComparisonConditionsOfFilter0(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLRANG0);
				FL_CANFD_Global_SetGPFLLimitConditionsOfFilter0(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLOUT0);
				FL_CANFD_Global_WriteGPFLOffsetOfFilter0(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_OFFSET0);
				FL_CANFD_Global_SetGPFLComparisonConditionsOfFilter1(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLRANG1);
				FL_CANFD_Global_SetGPFLLimitConditionsOfFilter1(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_GPFLOUT1);
				FL_CANFD_Global_WriteGPFLOffsetOfFilter1(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_OFFSET1);
				FL_CANFD_Global_WriteGPFLFilterData0(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_Payload_Data0);
				FL_CANFD_Global_WriteGPFLFilterData0Mask(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_Payload_Data0Mask);
				FL_CANFD_Global_WriteGPFLFilterData1(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_Payload_Data1);
				FL_CANFD_Global_WriteGPFLFilterData1Mask(CANFDx, i, CANFD_InitStruct->pflFilterCfg[i].PFL_Payload_Data1Mask);
				
            }
        }
		/* 25.16.58 CFDGPFLECTR[8]:接收滤波器列表配置进入控制寄存器，关闭后保护滤波器参数不被修改*/
        FL_CANFD_Global_Disable_GPFLDataAccess(CANFDx);
		
		/* 接收滤波器配置一，AFL和PFL组合用法
		 * CANFD0->CFDC0FDCFG.RPNMD,该寄存器只能在Reset和Halt模式下改写
		 * CANFD0->CFDC0FDCTR.PNMDC,需要退出Reset模式后改写 */
		FL_CANFD_CH0_FD_SetReturnPNFMode(CANFDx, CANFD_InitStruct->filterRPNMD);

	
        /* 特殊参数配置 */
        FL_CANFD_Global_SetTxPriority(CANFDx, CANFD_InitStruct->txPriority);
        FL_CANFD_CH0_FD_SetErrorStateIndicationConfiguration(CANFDx, CANFD_InitStruct->esiCfg);
        FL_CANFD_CH0_FD_Write_TransceiverDelayCompensationOffset(CANFDx, CANFD_InitStruct->tdco);
        FL_CANFD_CH0_FD_SetTransceiverDelayCompensationOffsetConfiguration(CANFDx, CANFD_InitStruct->tdcoCfg);        
        if (CANFD_InitStruct->tdcoEnable == FL_ENABLE)
        {
            FL_CANFD_CH0_FD_Enable_TransceiverDelayCompensation(CANFDx);
        }
        else
        {
            FL_CANFD_CH0_FD_Disable_TransceiverDelayCompensation(CANFDx);
        }
        
        if (CANFD_InitStruct->respedEnable == FL_ENABLE)
        {
            FL_CANFD_Global_Disable_RESProtocolExceptionDisabled(CANFDx);
        }
        else
        {
            FL_CANFD_Global_Enable_RESProtocolExceptionDisabled(CANFDx);
        }
        
        if (CANFD_InitStruct->dlcCheckEnable == FL_ENABLE)
        {
            FL_CANFD_Global_Enable_DLCCheck(CANFDx);
            if (CANFD_InitStruct->dlcReplacementEnable == FL_ENABLE)
            {
                FL_CANFD_Global_Enable_DLCReplacement(CANFDx);
            }
            else
            {
                FL_CANFD_Global_Disable_DLCReplacement(CANFDx);
            }
        }
        else
        {
            FL_CANFD_Global_Disable_DLCCheck(CANFDx);
            FL_CANFD_Global_Disable_DLCReplacement(CANFDx);
        }
        
        if (CANFD_InitStruct->messageIsRejected == FL_ENABLE)
        {
            FL_CANFD_Global_SetCANFDMessagePayloadOverflowConfiguration(CANFDx, FL_CANFD_CMPOC_CONFIGURATION_REJECTED);
        }
        else
        {
            FL_CANFD_Global_SetCANFDMessagePayloadOverflowConfiguration(CANFDx, FL_CANFD_CMPOC_CONFIGURATION_CUTOFF);
        }
        
        /*配置全局和通道的工作模式为Operation Mode*/
        if (CANFD_InitStruct->globalMode == FL_CANFD_GLOBAL_MODE_OPERATION)
        {
            FL_CANFD_Global_SetGlobalMode(CANFDx, FL_CANFD_GLOBAL_MODE_OPERATION);            
        }
        else if (CANFD_InitStruct->globalMode == FL_CANFD_GLOBAL_MODE_RESET)
        {
            FL_CANFD_Global_SetGlobalMode(CANFDx, FL_CANFD_GLOBAL_MODE_RESET);            
        }
        else if (CANFD_InitStruct->globalMode == FL_CANFD_GLOBAL_MODE_HALT)
        {
            FL_CANFD_Global_SetGlobalMode(CANFDx, FL_CANFD_GLOBAL_MODE_HALT);            
        }
        
        if (CANFD_InitStruct->channelMode == FL_CANFD_CHANNEL_MODE_OPERATION)
        {
            FL_CANFD_CH0_SetChannelMode(CANFDx, FL_CANFD_CHANNEL_MODE_OPERATION);
            while (FL_CANFD_CH0_IsActiveFlag_SleepStatus(CANFDx) || 
                   FL_CANFD_CH0_IsActiveFlag_HaltStatus(CANFDx)  || 
                   FL_CANFD_CH0_IsActiveFlag_ResetStatus(CANFDx))
            {
                /* 直至看门狗溢出复位 */
            }
        }
        else if (CANFD_InitStruct->channelMode == FL_CANFD_CHANNEL_MODE_RESET)
        {
            FL_CANFD_CH0_SetChannelMode(CANFDx, FL_CANFD_CHANNEL_MODE_RESET);
            while (!FL_CANFD_CH0_IsActiveFlag_ResetStatus(CANFDx))
            {
                /* 直至看门狗溢出复位 */
            }
        }
        else if (CANFD_InitStruct->channelMode == FL_CANFD_CHANNEL_MODE_HALT)
        {
            FL_CANFD_CH0_SetChannelMode(CANFDx, FL_CANFD_CHANNEL_MODE_HALT);
            while (!FL_CANFD_CH0_IsActiveFlag_HaltStatus(CANFDx))
            {
                /* 直至看门狗溢出复位 */
            }
        }        
        
        /*使能Buffer*/
        if (!FL_CANFD_CH0_IsActiveFlag_SleepStatus(CANFDx) &&
            !FL_CANFD_CH0_IsActiveFlag_ResetStatus(CANFDx))
        {
            /* 配置RxFIFO */
            for (i = 0; i < 2; i++)
            {
                if (CANFD_InitStruct->rxfifoCfg[i].Enable == FL_ENABLE)
                {
                    FL_CANFD_Global_Enable_RxFIFO(CANFDx, i);
                }
            }

            /* 配置CommonFIFO */
            if (CANFD_InitStruct->commonfifoCfg.Enable == FL_ENABLE)
            {
                FL_CANFD_Global_Enable_CommonFIFO(CANFDx);
            }

            /* 配置TxQueue */
            if (CANFD_InitStruct->txqueueCfg.Enable == FL_ENABLE)
            {
                FL_CANFD_Global_Enable_TxQueue(CANFDx);
            }
            
            /* 配置TxHistoryList */
            if (CANFD_InitStruct->txHistoryCfg.Enable == FL_ENABLE)
            {
                FL_CANFD_Global_Enable_TxHistoryList(CANFDx);
            }            
        }
        
		
		/* 接收滤波器配置二，AFL和PFL组合用法
		 * CANFD0->CFDC0FDCTR.PNMDC,需要退出Reset模式后改写
		 * 在写入时，需要将KEY和PNMDC参数一次性写入 */
//		CANFD0->CFDC0FDCTR = 0xC4030000;		/* [31:24]:KEY_C4h, [17:16]:PNMDC */
		FL_CANFD_CH0_FD_SetPNFMode(CANFDx,CANFD_InitStruct->filterPNMDC);

		
		
        status = FL_PASS;
    }
    
    return status;
}


/**
  * @}
  */

/* Private functions ---------------------------------------------------------*/
/** @defgroup CANFD_FL_Private_Functions CAN Private Functions
  * @{
  */

/**
  * @}
  */

#endif /* FL_FDCAN_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/

