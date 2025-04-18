 /**   
  *******************************************************************************************************
  * @file    fm33ld5xx_fl_flash.c
  * @author  FMSH Application Team
  * @brief   Source file of FLASH FL Module 
  ******************************************************************************************************* 
  * @attention
  * Copyright 2025 SHANGHAI FUDAN MICROELECTRONICS GROUP CO., LTD.(FUDAN MICRO.)
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met: 
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its contributors
  *    may be used to endorse or promote products derived from this software without
  *    specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS"AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  *******************************************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "fm33ld5xx_fl.h"

/** @addtogroup FM33LD5xx_FL_Driver
  * @{
  */

/** @addtogroup FLASH
  * @{
  */

#ifdef FL_FLASH_DRIVER_ENABLED
/* Private macros ------------------------------------------------------------*/
/** @addtogroup FLASH_FL_Private_Macros
  * @{
  */

#define    IS_FLASH_ALL_INSTANCE(INTENCE)               ((INTENCE) == FLASH)

#define    IS_FL_FLASH_CODE_MAX_ADDR(__VALUE__)         ((uint32_t)(__VALUE__) <= (FL_FLASH_CODE_ADDR_MAXPROGRAM))
           
#define    IS_FL_FLASH_CODE_MAX_PAGE(__VALUE__)         ((uint32_t)(__VALUE__) < (FL_FLASH_CODE_MAX_PAGE_NUM))
           
#define    IS_FL_FLASH_CODE_MAX_SECTOR(__VALUE__)       ((uint32_t)(__VALUE__) < (FL_FLASH_CODE_MAX_SECTOR_NUM))


#define    IS_FL_FLASH_INFO_MAX_ADDR(__VALUE__)         ((((uint32_t)(FL_FLASH_INFO_ADDR_MINPROGRAM)) <= (__VALUE__)) &&\
                                                         ((__VALUE__) <= (FL_FLASH_INFO_ADDR_MAXPROGRAM)))
           
#define    IS_FL_FLASH_INFO_MAX_PAGE(__VALUE__)         ((uint32_t)(__VALUE__) < (FL_FLASH_INFO_MAX_PAGE_NUM))
           
#define    IS_FL_FLASH_INFO_MAX_SECTOR(__VALUE__)       ((uint32_t)(__VALUE__) < (FL_FLASH_IFFO_MAX_SECTOR_NUM))


#define    IS_FL_FLASH_DATA_MAX_ADDR(__VALUE__)         ((((uint32_t)(FL_FLASH_DATA_ADDR_MINPROGRAM)) <= (__VALUE__)) &&\
                                                         ((__VALUE__) <= (FL_FLASH_DATA_ADDR_MAXPROGRAM)))
                                                         
#define    IS_FL_FLASH_DATA_MAX_PAGE(__VALUE__)         ((__VALUE__) < (FL_FLASH_DATA_MAX_PAGE_NUM))
           
#define    IS_FL_FLASH_DATA_MAX_SECTOR(__VALUE__)       ((__VALUE__) < (FL_FLASH_DATA_MAX_SECTOR_NUM))
                                                     

/**
  * @}
  */


/* Private function prototypes -----------------------------------------------*/
/** @defgroup FLASH_FL_Private_Functions FLASH Private Functions
  * @{
  */



/** @addtogroup FLASH_FL_EF_Init
  * @{
  */

/**
  * @brief  Flash 页擦除函数，一个页为512byte.
  * @param  FLASHx FLASH Port
  * @param  address 为需要擦除的页内任意地址，推荐使用页开始的首地址（字对齐）

  *         .
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 擦写发生错误
  *         -FL_PASS 擦写成功
  */
FL_ErrorStatus FL_FLASH_PageErase(FLASH_Type *FLASHx, uint32_t address)
{
    uint32_t timeout = 0;
    uint32_t primask = 0;
    uint32_t ClockError, KeyError, AuthenticationError;
    FL_ErrorStatus ret = FL_FAIL;
    /* 入口参数检查 */
    assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
    assert_param((IS_FL_FLASH_CODE_MAX_ADDR((uint32_t)address)) ||\
                 (IS_FL_FLASH_DATA_MAX_ADDR((uint32_t)address)));
    /*时钟使能*/
    FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
    FL_FLASH_ClearFlag_KeyError(FLASHx);	
	
    /* 擦写时钟使能*/
    FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
    /* 配置擦写类型*/
    FL_FLASH_SetFlashEraseType(FLASHx, FL_FLASH_ERASE_TYPE_PAGE);
		
		 /* 清除中断标志*/
    FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
    FL_FLASH_ClearFlag_ClockError(FLASHx);
    FL_FLASH_ClearFlag_KeyError(FLASHx);
    FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
		FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
    /* 开始擦除页*/
    FL_FLASH_EnableErase(FLASHx);
    /* Key 序列*/
    primask = __get_PRIMASK();
    __disable_irq();
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_ERASE_KEY);
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PAGE_ERASE_KEY);
    
    /* 擦请求 */
    *((uint32_t *)address) = FL_FLASH_ERASE_REQUEST;
    while(1)
    {
        timeout++;
        ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
        KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
        AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);
        if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01UL) || (KeyError == 0X01UL) || (AuthenticationError == 0X01UL))
        {
            /* 超时或出现错误 */
            ret = FL_FAIL;
            break;
        }
        else
        {
            if(FL_FLASH_IsActiveFlag_EraseComplete(FLASHx) == 0X01U)
            {
                /*编程成功*/
                FL_FLASH_ClearFlag_EraseComplete(FLASHx);
                ret =  FL_PASS;
                break;
            }
        }
    }
    FL_FLASH_LockFlash(FLASHx);
    FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
    FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
    __set_PRIMASK(primask);
    return ret;
}

/**
  * @brief  Code Flash 扇区擦除函数，一个扇区为2k byte.
  * @param  FLASHx FLASH Port
  * @param  address 为需要擦除的扇区内任意地址，推荐使用扇区开始的首地址（字对齐）

  *         .
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 擦写发生错误
  *         -FL_PASS 擦写成功
  */
FL_ErrorStatus FL_FLASH_CodeSectorErase(FLASH_Type *FLASHx, uint32_t address)
{
    uint32_t timeout = 0;
    uint32_t primask = 0;
    uint32_t ClockError, KeyError, AuthenticationError;
    FL_ErrorStatus ret = FL_FAIL;
    /* 入口参数检查 */
    assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
    assert_param(IS_FL_FLASH_CODE_MAX_ADDR((uint32_t)address));

    /*时钟使能*/
    FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
	  FL_FLASH_ClearFlag_KeyError(FLASHx);
		
    /* 擦写时钟使能*/
    FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
    /* 配置擦写类型*/
    FL_FLASH_SetFlashEraseType(FLASHx, FL_FLASH_ERASE_TYPE_SECTOR);
		
		/* 清除中断标志*/
    FL_FLASH_ClearFlag_EraseComplete(FLASHx);
    FL_FLASH_ClearFlag_ClockError(FLASHx);
    FL_FLASH_ClearFlag_KeyError(FLASHx);
    FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
		FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
		
    /* 开始擦除*/
    FL_FLASH_EnableErase(FLASHx);
    /* Key 序列*/
    primask = __get_PRIMASK();
    __disable_irq();
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_ERASE_KEY);
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_SECTOR_ERASE_KEY);
    
    /* 擦请求 */
    *((uint32_t *)address) = FL_FLASH_ERASE_REQUEST;
    while(1)
    {
        timeout++;
        ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
        KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
        AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);
        if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01UL) || (KeyError == 0X01UL) || (AuthenticationError == 0X01UL))
        {
            /* 超时或出现错误 */
            ret = FL_FAIL;
            break;
        }
        else
        {
            if(FL_FLASH_IsActiveFlag_EraseComplete(FLASHx) == 0X01U)
            {
                /*擦除标志*/
                FL_FLASH_ClearFlag_EraseComplete(FLASHx);
                ret =  FL_PASS;
                break;
            }
        }
    }
    FL_FLASH_LockFlash(FLASHx);
    FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
    FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
    __set_PRIMASK(primask);
    return ret;
}

/**
  * @brief  Code Flash 单次字编程函数，编程地址必须对齐到字边界.
  * @param  FLASHx FLASH Port
  * @param  address 为需要编程的已经擦除过的扇区内任意地址，非对齐地址编程将触发fault。

  *
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 编程发生错误
  *         -FL_PASS 编程成功
  */
FL_ErrorStatus FL_FLASH_CodeProgram_DoubleWord(FLASH_Type *FLASHx, uint32_t address, uint32_t data1,uint32_t data2)
{
    uint32_t timeout = 0;
    uint32_t primask = 0;
    uint32_t ClockError, KeyError, AuthenticationError;
    FL_ErrorStatus ret = FL_PASS;
    /* 入口参数检查 */
    assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
    assert_param(IS_FL_FLASH_CODE_MAX_ADDR((uint32_t)address));
    /*时钟使能*/
    FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
	  FL_FLASH_ClearFlag_KeyError(FLASHx);
    
	  /* 地址对齐检查 */
    if((address % FL_CODEFLASH_ADDRS_ALIGN) != 0)
    {
        /*地址未对齐*/
        FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        return FL_FAIL;
    }
		
    /* 擦写时钟使能*/
    FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
		
		 /* 清除中断标志*/
    FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
    FL_FLASH_ClearFlag_ClockError(FLASHx);
    FL_FLASH_ClearFlag_KeyError(FLASHx);
    FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
		FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
		
    /* 开始编程*/
    FL_FLASH_EnableProgram(FLASHx);
    /* Key 序列*/
    primask = __get_PRIMASK();
    __disable_irq();
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY1);
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY2);
    
    /* flash编程 Word*/
    *((uint32_t *)address) = data1;
		__DMB();
		*((uint32_t *)(address+4)) = data2;
    while(1)
    {
        timeout++;
        ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
        KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
        AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);
        if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01U) || (KeyError == 0X01U) || (AuthenticationError == 0X01U))
        {
            /* 超时或出现错误 */
            ret = FL_FAIL;
            break;
        }
        else
        {
            if(FL_FLASH_IsActiveFlag_ProgramComplete(FLASHx) == 0X01U)
            {
                /*编程成功*/
                FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
                ret = FL_PASS;
                break;
            }
        }
    }
    FL_FLASH_LockFlash(FLASHx);
    FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
    FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
    __set_PRIMASK(primask);
    return ret;
}

/**
  * @brief  Flash 页编程函数，编程地址必须对齐到页边界.
  * @param  FLASHx FLASH Port
  * @param  PageNum 为需要编程的已经擦除过的页号，非对齐地址编程将触发fault。

  *
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 编程发生错误
  *         -FL_PASS 编程成功
  */
FL_ErrorStatus FL_FLASH_CodeProgram_Page(FLASH_Type *FLASHx, uint32_t pageNum, uint32_t *data)
{
    uint32_t count;
    uint32_t primask;
    uint32_t address;
    uint32_t timeout;
    uint32_t ClockError, KeyError, AuthenticationError;
    FL_ErrorStatus status = FL_FAIL;
    if(data != NULL)
    {
        /* 入口参数检查 */
        assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
        assert_param(IS_FL_FLASH_CODE_MAX_PAGE((uint32_t)pageNum)); 
			
        address = pageNum * FL_FLASH_CODE_PAGE_SIZE_BYTE;  
       
        assert_param(IS_FL_FLASH_CODE_MAX_ADDR((uint32_t)address));
        /* 时钟使能*/
        FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        /* 页地址对齐检查 */
        if((address % FL_FLASH_CODE_PAGE_SIZE_BYTE) != 0)
        {
            /*地址未对齐*/
            FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
            return FL_FAIL;
        }
        /* 擦写时钟使能*/
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
				
				/* 清除中断标志*/
				FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
        FL_FLASH_ClearFlag_ClockError(FLASHx);
        FL_FLASH_ClearFlag_KeyError(FLASHx);
        FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
				FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
				
        /* 开始编程*/
        FL_FLASH_EnableProgram(FLASHx);  
        /* Key 序列*/
        primask = __get_PRIMASK();
        __disable_irq();
        FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY1);
        FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY2);
     
        for(count = 0; count < FL_FLASH_CODE_PAGE_SIZE_BYTE; count += 8)
        {
            timeout = 0;
            FL_FLASH_EnableProgram(FLASHx);         				  
					 /* flash编程 Word*/
            *((uint32_t *)address) = *data ;
					  data++;
					  __DMB();
		        *((uint32_t *)(address+4)) =*data ;
				  	address += 8;
					  data++;
            while(1)
            {
                timeout++;
                ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
                KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
                AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);
                if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01U) || (KeyError == 0X01U) || (AuthenticationError == 0X01U))
                {
                    /* 超时或出现错误 */
                    status = FL_FAIL;
                    break;
                }
                else
                {
                    if(FL_FLASH_IsActiveFlag_ProgramComplete(FLASHx) == 0X01U)
                    {
                        /*编程成功*/
                        FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
                        status = FL_PASS;
                        break;
                    }
                }
            }
        }
        FL_FLASH_LockFlash(FLASHx);
        FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
        FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        __set_PRIMASK(primask);
    }
    return status;
}

/**
  * @brief  Code Flash 扇区编程函数，编程地址必须对齐到扇区边界.
  * @param  FLASHx FLASH Port
  * @param  sectorNum 为需要编程的已经擦除过的扇区号，非对齐地址编程将触发fault。

  *
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 编程发生错误
  *         -FL_PASS 编程成功
  */
FL_ErrorStatus FL_FLASH_CodeProgram_Sector(FLASH_Type *FLASHx, uint32_t sectorNum, uint32_t *data)
{
    uint32_t count;
    uint32_t primask;
    uint32_t address;
    uint32_t timeout;
    uint32_t ClockError, KeyError, AuthenticationError;
    FL_ErrorStatus status = FL_FAIL;
    if(data != NULL)
    {
        /* 入口参数检查 */
        assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
        assert_param(IS_FL_FLASH_CODE_MAX_SECTOR((uint32_t)sectorNum));
        address = sectorNum * FL_FLASH_CODE_SECTOR_SIZE_BYTE;
        
        assert_param(IS_FL_FLASH_CODE_MAX_ADDR((uint32_t)address));
        /*时钟使能*/
        FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        /* 地址对齐检查 */
        if((address % FL_FLASH_CODE_SECTOR_SIZE_BYTE) != 0)
        {
            /*地址未对齐*/
            FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
            return FL_FAIL;
        }
        /* 擦写时钟使能*/
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
				
				/* 清除中断标志*/
        FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
        FL_FLASH_ClearFlag_ClockError(FLASHx);
        FL_FLASH_ClearFlag_KeyError(FLASHx);
        FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
				FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
				
        /* 开始编程*/
        FL_FLASH_EnableProgram(FLASHx);  
        /* Key 序列*/
        primask = __get_PRIMASK();
        __disable_irq();
        FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY1);
        FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY2);
        
        for(count = 0; count < FL_FLASH_CODE_SECTOR_SIZE_BYTE; count += 8)
        {
            timeout = 0;
            FL_FLASH_EnableProgram(FLASHx);
            /* flash编程 Word*/
            *((uint32_t *)address) = *data ;
					  data++;
					  __DMB();
		        *((uint32_t *)(address+4)) =*data ;
				  	address += 8;
					  data++;
            while(1)
            {
                timeout++;
                ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
                KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
                AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);
                if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01U) || (KeyError == 0X01U) || (AuthenticationError == 0X01U))
                {
                    /* 超时或出现错误 */
                    status = FL_FAIL;
                    break;
                }
                else
                {
                    if(FL_FLASH_IsActiveFlag_ProgramComplete(FLASHx) == 0X01U)
                    {
                          /*编程成功*/
                          FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
                          status = FL_PASS;
                          break;
                    }
                }
            }
        }
        FL_FLASH_LockFlash(FLASHx);
        FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
        FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        __set_PRIMASK(primask);
    }
    return status;
}

/**
  * @brief  Data Flash 单次字编程函数，编程地址必须对齐到字边界.
  * @param  FLASHx FLASH Port
  * @param  address 为需要编程的已经擦除过的扇区内任意地址，非对齐地址编程将触发fault。

  *
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 编程发生错误
  *         -FL_PASS 编程成功
  */
FL_ErrorStatus FL_FLASH_DataProgram_Word(FLASH_Type *FLASHx, uint32_t address, uint32_t data)
{
    uint32_t timeout = 0;
    uint32_t primask = 0;
    uint32_t ClockError, KeyError, AuthenticationError,ParityError;
    FL_ErrorStatus ret = FL_PASS;
    /* 入口参数检查 */
    assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
    assert_param(IS_FL_FLASH_DATA_MAX_ADDR((uint32_t)address));
    /*时钟使能*/
    FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
	  FL_FLASH_ClearFlag_KeyError(FLASHx);
    
	  /* 地址对齐检查 */
    if((address % FL_DATAFLASH_ADDRS_ALIGN) != 0)
    {
        /*地址未对齐*/
        FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        return FL_FAIL;
    }
		
    /* 擦写时钟使能*/
    FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
		
		 /* 清除中断标志*/
    FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
    FL_FLASH_ClearFlag_ClockError(FLASHx);
    FL_FLASH_ClearFlag_KeyError(FLASHx);
    FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
		FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
		
    /* 开始编程*/
    FL_FLASH_EnableProgram(FLASHx);
    /* Key 序列*/
    primask = __get_PRIMASK();
    __disable_irq();
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY1);
    FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY2);
    
    /* flash编程 Word*/
    *((uint32_t *)address) = data;
    while(1)
    {
        timeout++;
        ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
        KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
        AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);
			  ParityError=FL_FLASH_IsActiveFlag_ParityCorrectionInDataFLASH(FLASHx);
        if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01U) || (KeyError == 0X01U) || (AuthenticationError == 0X01U)|| (ParityError == 0X01U))
        {
            /* 超时或出现错误 */
            ret = FL_FAIL;
            break;
        }
        else
        {
            if(FL_FLASH_IsActiveFlag_ProgramComplete(FLASHx) == 0X01U)
            {
                /*编程成功*/
                FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
                ret = FL_PASS;
                break;
            }
        }
    }
    FL_FLASH_LockFlash(FLASHx);
    FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
    FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
    __set_PRIMASK(primask);
    return ret;
}

/**
  * @brief  DataFlash 页编程函数，编程地址必须对齐到页边界.
  * @param  FLASHx FLASH Port
  * @param  PageNum 为需要编程的已经擦除过的页号，非对齐地址编程将触发fault。

  *
  * @retval ErrorStatus枚举值

  *         -FL_FAIL 编程发生错误
  *         -FL_PASS 编程成功
  */
FL_ErrorStatus FL_FLASH_DataProgram_Page(FLASH_Type *FLASHx, uint32_t pageNum, uint32_t *data)
{
    uint32_t count;
    uint32_t primask;
    uint32_t address;
    uint32_t timeout;
    uint32_t ClockError, KeyError, AuthenticationError,ParityError;
    FL_ErrorStatus status = FL_FAIL;
    if(data != NULL)
    {
        /* 入口参数检查 */
        assert_param(IS_FLASH_ALL_INSTANCE(FLASHx));
        assert_param(IS_FL_FLASH_DATA_MAX_PAGE((uint32_t)pageNum)); 
			
        address = pageNum * FL_FLASH_DATA_PAGE_SIZE_BYTE;  
       
        assert_param(IS_FL_FLASH_DATA_MAX_ADDR((uint32_t)address));
        /* 时钟使能*/
        FL_CMU_EnableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        /* 页地址对齐检查 */
        if((address % FL_FLASH_DATA_PAGE_SIZE_BYTE) != 0)
        {
            /*地址未对齐*/
            FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
            return FL_FAIL;
        }
        /* 擦写时钟使能*/
        FL_CMU_EnableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
				
				/* 清除中断标志*/
				FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
        FL_FLASH_ClearFlag_ClockError(FLASHx);
        FL_FLASH_ClearFlag_KeyError(FLASHx);
        FL_FLASH_ClearFlag_AuthenticationError(FLASHx);
				FL_FLASH_ClearFlag_ParityCorrectionInDataFLASH(FLASHx);
				
        /* 开始编程*/
        FL_FLASH_EnableProgram(FLASHx);  
        /* Key 序列*/
        primask = __get_PRIMASK();
        __disable_irq();
        FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY1);
        FL_FLASH_UnlockFlash(FLASHx, FL_FLASH_PROGRAM_KEY2);
     
        for(count = 0; count < FL_FLASH_CODE_PAGE_SIZE_BYTE; count += 4)
        {
            timeout = 0;
            FL_FLASH_EnableProgram(FLASHx);         				  
					 /* flash编程 Word*/
            *((uint32_t *)address) = *data ;
				  	address += 4;
					  data++;
            while(1)
            {
                timeout++;
                ClockError = FL_FLASH_IsActiveFlag_ClockError(FLASHx);
                KeyError = FL_FLASH_IsActiveFlag_KeyError(FLASHx);
                AuthenticationError = FL_FLASH_IsActiveFlag_AuthenticationError(FLASHx);							
                ParityError=FL_FLASH_IsActiveFlag_ParityCorrectionInDataFLASH(FLASHx);
                if((timeout > FL_FLASH_ERASE_TIMEOUT) || (ClockError == 0X01U) || (KeyError == 0X01U) || (AuthenticationError == 0X01U)|| (ParityError == 0X01U))
                {
                    /* 超时或出现错误 */
                    status = FL_FAIL;
                    break;
                }
                else
                {
                    if(FL_FLASH_IsActiveFlag_ProgramComplete(FLASHx) == 0X01U)
                    {
                        /*编程成功*/
                        FL_FLASH_ClearFlag_ProgramComplete(FLASHx);
                        status = FL_PASS;
                        break;
                    }
                }
            }
        }
        FL_FLASH_LockFlash(FLASHx);
        FL_CMU_DisableOperationClock(FL_CMU_GROUP1_OPCLKEN_FLASH);
        FL_CMU_DisableGroup2BusClock(FL_CMU_GROUP2_BUSCLK_FLASH);
        __set_PRIMASK(primask);
    }
    return status;
}
/**
  * @}
  */

#endif /* FL_FLASH_DRIVER_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/*************************(C) COPYRIGHT Fudan Microelectronics **** END OF FILE*************************/
