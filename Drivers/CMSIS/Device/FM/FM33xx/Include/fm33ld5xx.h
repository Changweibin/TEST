/**
  ****************************************************************************************************//**
  * @file     FM33LD5XX.h
  *
  * @brief    Starmc1 Peripheral Access Layer Header File for
  *           FM33LD5XX from Keil.
  *
  * @version  V1.0
  * @date     17. February 2025
  *******************************************************************************************************
  * @attention    
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

/** @addtogroup Keil
  * @{
  */

/** @addtogroup FM33LD5XX
  * @{
  */

#ifndef FM33LD5XX_H
#define FM33LD5XX_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunState;
typedef enum {FAIL = 0, PASS = !FAIL} ErrorStatus;

#define __RCHF_INITIAL_CLOCK        (16000000)        /* Value of the Internal RC HIGH oscillator in Hz */
#define __RCLP_CLOCK                (32768)           /* Value of the Internal RC LOW oscillator in Hz */
#define __XTHF_CLOCK                (8000000)         /* Value of the EXTERNAL oscillator in Hz */


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum
{

    /* -------------------  FM33LD5XX Processor Exceptions Numbers  -------------------*/
	  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset           */
    NMI_IRQn                      = -14,              /*!<   2  不可屏蔽中断 */
    Hardfault_IRQn                = -13,              /*!<   3  处理器内部错误 */
    MemManage_IRQn                = -12,              /*!<   4  MPU错误 */
    BusFault_IRQn                 = -11,              /*!<   5  总线错误 */
    UsageFault_IRQn               = -10,              /*!<   6  指令执行错误 */
    SVCall_IRQn                   = -5,               /*!<  11  SupervisorCall */
    DebugMonitor_IRQn             = -4,               /*!<  11  DebugMonitor */
    PendSV_IRQn                   = -2,               /*!<  14  系统服务请求 */
    SysTick_IRQn                  = -1,               /*!<  15  内核定时器中断 */

    /* --------------------  Cortex-M33 Specific Interrupt Numbers  --------------------*/
    WWDT_IRQn                     = 0,                /*!<   0  WWDT */
    SVD_IRQn                      = 1,                /*!<   1  SVD  */
    IWDT_IRQn                     = 3,                /*!<   3  IWDT */
    FLASH_IRQn                    = 4,                /*!<   4  FLASH */
    CMU_IRQn                      = 5,                /*!<   5  CMU */
    MAP_ERR_IRQn                  = 7,                /*!<   7  MAP_ERR */
    ADC0_IRQn                     = 8,                /*!<   8  ADC0 */
    ADC1_IRQn                     = 9,                /*!<   9  ADC1 */
    DAC0_IRQn                     = 10,               /*!<  10  DAC0 */
    SPI0_IRQn                     = 11,               /*!<  11  SPI0 */
    SPI1_IRQn                     = 12,               /*!<  12  SPI1 */
    SPI2_IRQn                     = 13,               /*!<  13  SPI2 */
    DAC1_IRQn                     = 16,               /*!<  16  DAC1 */
    QSPI_IRQn                     = 17,               /*!<  17  QSPI */
    UART0_IRQn                    = 18,               /*!<  18  UART0 */
    UART1_IRQn                    = 19,               /*!<  19  UART1 */
    UART2_IRQn                    = 20,               /*!<  20  UART2 */
    UART3_IRQn                    = 21,               /*!<  21  UART3 */
    UART4_IRQn                    = 22,               /*!<  22  UART4 */
    UART5_IRQn                    = 23,               /*!<  23  UART5 */
    ADC2_IRQn                     = 24,               /*!<  24  ADC2 */
    ADC3_IRQn                     = 25,               /*!<  25  ADC3 */
    EXTIA_IRQn                    = 26,               /*!<  26  GPIOA */
    EXTIB_IRQn                    = 27,               /*!<  27  GPIOB */
    EXTIC_IRQn                    = 28,               /*!<  28  GPIOC */
    EXTID_IRQn                    = 29,               /*!<  29  GPIOD */
    EXTIE_IRQn                    = 30,               /*!<  30  GPIOE */
    EXTIF_IRQn                    = 31,               /*!<  31  GPIOF */
    EXTIG_IRQn                    = 32,               /*!<  32  GPIOG */
    PMU_IRQn                      = 37,               /*!<  37  PMU */
    I2C0_IRQn                     = 38,               /*!<  38  I2C0 */
    I2C1_IRQn                     = 39,               /*!<  39  I2C1 */
    CLM0_IRQn                     = 43,               /*!<  43  CLM0 */
    CLM1_IRQn                     = 44,               /*!<  44  CLM1 */
    CLM2_IRQn                     = 45,               /*!<  45  CLM2 */
    FPU_IRQn                      = 48,               /*!<  48  FPU */
    DMA_IRQn                      = 50,               /*!<  50  DMA */
    WKUPx_IRQn                    = 51,               /*!<  51  WKUP */
    PGL_IRQn                      = 52,               /*!<  52  PGL */
    COMPx_IRQn                    = 53,               /*!<  53  COMP */
    CANFD0_IRQn                   = 56,               /*!<  56  CANFD0 */
    LPTIM16_IRQn                  = 60,               /*!<  60  LPTIM16 */
    BSTIM16_IRQn                  = 62,               /*!<  62  BSTIM16 */
    ATIM0_IRQn                    = 64,               /*!<  64  ATIM0 */
    ATIM1_IRQn                    = 65,               /*!<  65  ATIM1 */
    ATIM2_IRQn                    = 66,               /*!<  66  ATIM2 */
    GPTIM0_IRQn                   = 68,               /*!<  68  GPTIM0 */
    GPTIM1_IRQn                   = 69,               /*!<  69  GPTIM1 */
    GPTIM5_IRQn                   = 71,               /*!<  71  GPTIM5 */
    TAU0_IRQn                     = 73,               /*!<  73  TAU0 */
    CORDIC_IRQn                   = 76,               /*!<  76  CORDIC */
} IRQn_Type;

/** @addtogroup Configuration_of_CMSIS
  * @{
  */

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* --------  Configuration of the Cortex-M33 Processor and Core Peripherals  ------ */
#define __SAUREGION_PRESENT       0U        /* SAU regions present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __NVIC_PRIO_BITS          3U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1U        /* FPU present */
#define __DSP_PRESENT             1U        /* DSP extension present */
#define __ICACHE_PRESENT          1U        /* Define if an ICACHE is present or not */

/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_starmc1.h"                   /*!< Star-MC1 processor and core peripherals */
#include "system_fm33ld5xx.h"               /*!< FM33LD5XX System */

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/** @addtogroup Device_Peripheral_Registers
  * @{
  */

/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#elif defined(__ICCARM__)
#pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
#pragma warning 586
#else
#warning Not supported compiler type
#endif

/* ================================================================================ */
/* ================                       FLS                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t RDCR;                    /*!<  FlashReadControlRegister,                             Address offset: 0x00 */
    __IO uint32_t RSV1[4];                /*!<  RESERVED REGISTER,                                    Address offset: 0x04 */
    __IO uint32_t EPCR;                   /*!<  FlashErase/ProgramControlRegister,                    Address offset: 0x14 */
    __O  uint32_t KEY;                    /*!<  FlashKeyRegister,                                     Address offset: 0x18 */
    __IO uint32_t IER;                    /*!<  FlashInterruptEnableRegister,                         Address offset: 0x1C */
    __IO uint32_t ISR;	                  /*!<  FlashInterruptStatusRegister,                         Address offset: 0x20 */
    __IO uint32_t RSV2;	                  /*!<  FlashInterruptStatusRegister,                         Address offset: 0x24 */ 
	  __IO uint32_t CECCSR;	                /*!<  Flash Code ECC Status Register,                       Address offset: 0x28 */
	  __IO uint32_t DECCSR;	                /*!<  Flash Data ECC Status Register,                       Address offset: 0x2C */
	  __IO uint32_t RSV3[2];	                /*!<  FlashInterruptStatusRegister,                         Address offset: 0x30 */
	  __IO uint32_t ECCCR;	                /*!<  Flash ECC tControl Register,                          Address offset: 0x38 */ 
	  
} FLASH_Type;

/* ================================================================================ */
/* ================                       PMU                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR;                     /*!<  PowerManagementControlRegister,                       Address offset: 0x00 */
    __IO uint32_t WKTR;                   /*!<  WakeupTimeRegister,                                   Address offset: 0x04 */
    __IO uint32_t WKFR0;                  /*!<  WakeupSourceFlagsRegister1,                           Address offset: 0x08 */
    __IO uint32_t WKFR1;                  /*!<  WakeupSourceFlagsRegister2,                           Address offset: 0x0C */
    __IO uint32_t IER;                    /*!<  IER REGISTER,                                         Address offset: 0x10 */
    __IO uint32_t ISR;                    /*!<  ISR REGISTER,                                         Address offset: 0x14 */
    __IO uint32_t RAMCR;                  /*!<  RAM CONTROL REGISTER,                                 Address offset: 0x18 */
    __IO uint32_t RSV[8];                 /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
    __IO uint32_t BUFCR;                  /*!<  BUFFER CONTROL REGISTER,                              Address offset: 0x3C */
    __IO uint32_t RSV1[4];                /*!<  RESERVED REGISTER,                                    Address offset: 0x40 */
    __IO uint32_t PTATCR;                 /*!<  PAPTControlRegister,                                  Address offset: 0x50 */
    __IO uint32_t AUXEN;                  /*!<  AUXControlRegister,                                   Address offset: 0x54 */
} PMU_Type;

/* ================================================================================ */
/* ================                       CLM                      ================ */
/* ================================================================================ */
typedef struct
{
    __IO uint32_t CR;                     /*!< Address offset: 0x00  */
    __IO uint32_t CFGR;                   /*!< Address offset: 0x04  */
    __IO uint32_t CNT;                    /*!< Address offset: 0x08  */
    __IO uint32_t ISR;                    /*!< Address offset: 0x0C  */
    __IO uint32_t CMPH;                   /*!< Address offset: 0x10  */
    __IO uint32_t CMPL;                   /*!< Address offset: 0x14  */
	__IO uint32_t TOCFGR;                 /*!< Address offset: 0x18  */
} CLM_Type;

/* ================================================================================ */
/* ================                       RMU                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t PDRCR;                    /*!<  PDR Control Register,                                Address offset: 0x00 */
    __IO uint32_t RSV;                      /*!<  RESERVED REGISTER,                                    Address offset: 0x04 */
    __IO uint32_t RSTCR;                    /*!<  ResetConfigRegister,                                  Address offset: 0x08 */
    __O  uint32_t SOFTRST;                  /*!<  SoftwareResetRegister,                                Address offset: 0x0C */
    __IO uint32_t RSTFR;                    /*!<  ResetFlagRegister,                                    Address offset: 0x10 */
    __O  uint32_t PRSTEN;                   /*!<  PeripheralResetEnableRegister,                        Address offset: 0x14 */
    __IO uint32_t AHBRSTCR;                 /*!<  AHBPeripheralsResetRegister,                          Address offset: 0x18 */
    __IO uint32_t APBRSTCR1;                /*!<  APBPeripheralsResetRegister1,                         Address offset: 0x1C */
    __IO uint32_t APBRSTCR2;                /*!<  APBPeripheralsResetRegister2,                         Address offset: 0x20 */
} RMU_Type;

/* ================================================================================ */
/* ================                      IWDT                      ================ */
/* ================================================================================ */

typedef struct
{
    __O  uint32_t SERV;                   /*!<  IWDTServiceRegister,                                  Address offset: 0x00 */
    __IO uint32_t CR;                     /*!<  IWDTConfigRegister,                                   Address offset: 0x04 */
    __I  uint32_t CNT;                    /*!<  IWDTCounterRegister,                                  Address offset: 0x08 */
    __IO uint32_t WIN;                    /*!<  IWDTWindowRegister,                                   Address offset: 0x0C */
    __IO uint32_t IER;                    /*!<  IWDTInterruptEnableRegister,                          Address offset: 0x10 */
    __IO uint32_t ISR;                    /*!<  IWDTInterruptStatusRegister,                          Address offset: 0x14 */
} IWDT_Type;

/* ================================================================================ */
/* ================                      WWDT                      ================ */
/* ================================================================================ */

typedef struct
{
    __O  uint32_t CR;                     /*!<  WWDTControlRegister,                                  Address offset: 0x00 */
    __IO uint32_t CFGR;                   /*!<  WWDTConfigRegister,                                   Address offset: 0x04 */
    __I  uint32_t CNT;                    /*!<  WWDTCounterRegister,                                  Address offset: 0x08 */
    __IO uint32_t IER;                    /*!<  WWDTInterruptEnableRegister,                          Address offset: 0x0C */
    __IO uint32_t ISR;                    /*!<  WWDTInterruptStatusRegister,                          Address offset: 0x10 */
    __I  uint32_t PSC;                    /*!<  WWDTPrescalerRegister,                                Address offset: 0x14 */
} WWDT_Type;

/* ================================================================================ */
/* ================                       CMU                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t SYSCLKCR;               /*!<  SystemClockControlRegister,                           Address offset: 0x00 */
    __IO uint32_t RCHFCR;                 /*!<  RCHFControlRegister,                                  Address offset: 0x04 */
    __IO uint32_t RCHFTR;                 /*!<  RCHFTrimRegister,                                     Address offset: 0x08 */
    __IO uint32_t PLLCR;                  /*!<  PLLControlRegister,                                   Address offset: 0x0C */
    __IO uint32_t RCLPCR;                 /*!<  RCLPControlRegister,                                  Address offset: 0x10 */
    __IO uint32_t RCLPTR;                 /*!<  RCLPTrimRegister,                                     Address offset: 0x14 */
    __IO uint32_t RSV1[2];                /*!<  RESERVED REGISTER,                                    Address offset: 0x18 */
    __IO uint32_t XTHFCR;                 /*!<  XTHFControlRegister,                                  Address offset: 0x20 */
    __IO uint32_t RSV2[2];                /*!<  RESERVED REGISTER,                                    Address offset: 0x24 */
    __IO uint32_t IER;                    /*!<  InterruptEnableRegister,                              Address offset: 0x2C */
    __IO uint32_t ISR;                    /*!<  InterruptStatusRegister,                              Address offset: 0x30 */
    __IO uint32_t PCLKCR1;                /*!<  PeripheralbusClockControlRegister1,                   Address offset: 0x34 */
    __IO uint32_t PCLKCR2;                /*!<  PeripheralbusClockControlRegister2,                   Address offset: 0x38 */
    __IO uint32_t PCLKCR3;                /*!<  PeripheralbusClockControlRegister3,                   Address offset: 0x3C */
    __IO uint32_t PCLKCR4;                /*!<  PeripheralbusClockControlRegister4,                   Address offset: 0x40 */
    __IO uint32_t OPCCR1;                 /*!<  PeripheralClockConfigRegister1,                       Address offset: 0x44 */
    __IO uint32_t RSV3;                   /*!<  PeripheralClockConfigRegister2,                       Address offset: 0x48 */
    __IO uint32_t OPCER1;                 /*!<  PeripheralClockEnableRegister1,                       Address offset: 0x4C */
    __IO uint32_t AMCR;                   /*!<  AHB Master Control Register ,                         Address offset: 0x50 */
    __IO uint32_t RSV4;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x54 */
    __IO uint32_t RCHF16CR;               /*!<  RCHF16CR Register,                                    Address offset: 0x58 */
    __IO uint32_t RSV5[9];                /*!<  ClockCalibrationConfigRegister,                       Address offset: 0x5C */
    __IO  uint32_t CFDCR;                  /*!<  CFDCR Register,                                       Address offset: 0x80 */
    __IO uint32_t CFDER;                  /*!<  CFDER Register,                                       Address offset: 0x84 */
} CMU_Type;

/* ================================================================================ */
/* ================                       CORDIC                      ================ */
/* ================================================================================ */
typedef struct
{
    __IO uint32_t INA;               /*!< Address offset: 0x00 */
    __IO uint32_t INB;               /*!< Address offset: 0x04 */
    __IO uint32_t CR;                /*!< Address offset: 0x08 */
    __IO uint32_t OUT;               /*!< Address offset: 0x0C */
    __IO uint32_t ISR;               /*!< Address offset: 0x10 */
    __IO uint32_t IER;               /*!< Address offset: 0x14 */
} CORDIC_Type;

/* ================================================================================ */
/* ================                       SVD                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CFGR;                   /*!<  SVDConfigRegister,                                    Address offset: 0x00 */
    __IO uint32_t CR;                     /*!<  SVDControlRegister,                                   Address offset: 0x04 */
    __IO uint32_t IER;                    /*!<  SVDInterruptEnableRegister,                           Address offset: 0x08 */
    __IO uint32_t ISR;                    /*!<  SVDInterruptStatusRegister,                           Address offset: 0x0C */
    __IO uint32_t VSR;                    /*!<  SVDreferenceVoltageSelectRegister,                    Address offset: 0x10 */
} SVD_Type;

/* ================================================================================ */
/* ================                      COMP                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR;                     /*!< ComparatorControl Register 1,                        Address offset: 0x00 */

} COMP_Type;

typedef struct
{
    __IO uint32_t ICR;                    /*!< Comparator Interrupt Config Register,                Address offset: 0x00 */
    __IO uint32_t ISR;                    /*!< Comparator Interrupt Status Register,                Address offset: 0x04 */
    __IO uint32_t RSV0;                   /*!< RESERVED REGISTER0,                                  Address offset: 0x08 */
    __IO uint32_t BUFCR;                  /*!< Comparator Interrupt Status Register,                Address offset: 0x0C */
} COMP_COMMON_Type;

/* ================================================================================ */
/* ================                      I2Cx                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t MSPCFGR;                /*!<  I2CMasterConfigRegister,                              Address offset: 0x00 */
    __IO uint32_t MSPCR;                  /*!<  I2CMasterControlRegister,                             Address offset: 0x04 */
    __IO uint32_t MSPIER;                 /*!<  I2CMasterIntteruptEnableRegister,                     Address offset: 0x08 */
    __IO uint32_t MSPISR;                 /*!<  I2CMasterInterruptStatusRegister,                     Address offset: 0x0C */
    __IO uint32_t MSPSR;                  /*!<  I2CMasterStatusRegister,                              Address offset: 0x10 */
    __IO uint32_t MSPBGR;                 /*!<  I2CMasterBaudrateGeneratorRegister,                   Address offset: 0x14 */
    __IO uint32_t MSPBUF;                 /*!<  I2CMastertransferBuffer,                              Address offset: 0x18 */
    __IO uint32_t MSPTCR;                 /*!<  I2CMasterTimingControlRegister,                       Address offset: 0x1C */
    __IO uint32_t MSPTOR;                 /*!<  I2CMasterTime-OutRegister,                            Address offset: 0x20 */
} I2C_Type;

/* ================================================================================ */
/* ================                      UART                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t IRCR;                   /*!<  InfraredmodulationControlRegister,                    Address offset: 0x00 */
} UART_COMMON_Type;

/* ================================================================================ */
/* ================                      UARTx                     ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CSR;                    /*!<  UARTxControlStatusRegister,                           Address offset: 0x00 */
    __IO uint32_t IER;                    /*!<  UARTxInterruptEnableRegister,                         Address offset: 0x04 */
    __IO uint32_t ISR;                    /*!<  UARTxInterruptStatusRegister,                         Address offset: 0x08 */
    __IO uint32_t TODR;                   /*!<  UARTxTime-OutandDelayRegister,                        Address offset: 0x0C */
    __I  uint32_t RXBUF;                  /*!<  UARTxReceiveBuffer,                                   Address offset: 0x10 */
    __O  uint32_t TXBUF;                  /*!<  UARTxTransmitBuffer,                                  Address offset: 0x14 */
    __IO uint32_t BGR;                    /*!<  UARTxBaudrateGeneratorRegister,                       Address offset: 0x18 */
    __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
    __IO uint32_t MCR;                    /*!<  UARTxModeControlRegister,                             Address offset: 0x20 */
    __IO uint32_t LINCR;                  /*!<  UARTxLINControlRegister,                              Address offset: 0x24 */
    __I  uint32_t LINBSR;                 /*!<  UARTxLINBaudSyncRegister,                             Address offset: 0x28 */
    __IO uint32_t LINFTR;                 /*!<  UARTxLINFrameTimeoutRegister,                         Address offset: 0x2C */
    __IO uint32_t LINTTR;                 /*!<  UARTxLINTransmitTimingRegister,                       Address offset: 0x30 */
    __I  uint32_t LINPSR;                 /*!<  UARTLINPre-syncbaudregister,                          Address offset: 0x34 */
    __IO uint32_t LINBKR;                 /*!<  ,                                                     Address offset: 0x38 */
    __IO uint32_t RSV2;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x3C */
    __IO uint32_t SCISR;                  /*!<  UARTxSmartCardInterruptStatusRegister,                Address offset: 0x40 */
    __IO uint32_t SCIER;                  /*!<  UARTxSmartCardInterruptEnableRegister,                Address offset: 0x44 */
    __IO uint32_t FFCR;                   /*!<  UARTxSmartCardFrameFormatControlRegister,             Address offset: 0x48 */
    __IO uint32_t EGTR;                   /*!<  UARTxSmartCardExtraGuardTimeRegister,                 Address offset: 0x4C */
    __IO uint32_t CODR;                   /*!<  UARTxSmartCardClockOutputRegister,                    Address offset: 0x50 */
    __IO uint32_t RSV3[11];               /*!<  RESERVED REGISTER,                                    Address offset: 0x54 */
    __IO uint32_t LINSCCR;                /*!<  LINSelf-testCwitchControlRegister,                    Address offset: 0x80 */
} UART_Type;

/* ================================================================================ */
/* ================                      SPIx                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR1;                    /*!<  SPIxControlRegister1,                                 Address offset: 0x00 */
    __IO uint32_t CR2;                    /*!<  SPIxControlRegister2,                                 Address offset: 0x04 */
    __IO uint32_t CR3;                    /*!<  ControlRegister3,                                     Address offset: 0x08 */
    __IO uint32_t IER;                    /*!<  SPIxInterruptEnableRegister,                          Address offset: 0x0C */
    __IO uint32_t ISR;                    /*!<  SPIxStatusRegister,                                   Address offset: 0x10 */
    __O  uint32_t TXBUF;                  /*!<  SPIxTransmitBuffer,                                   Address offset: 0x14 */
    __I  uint32_t RXBUF;                  /*!<  SPIxReceiveBuffer,                                    Address offset: 0x18 */
    __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
    __IO uint32_t I2SCR;                  /*!<  I2SControlRegister,                                   Address offset: 0x20 */
    __IO uint32_t I2SPR;                  /*!<  SPIxI2SPrescalerRegister,                             Address offset: 0x24 */
} SPI_Type;

/* ================================================================================ */
/* ================                      CANFD                     ================ */
/* ================================================================================ */


/**
  * @brief CANFD module information (CANFD)
  */

typedef struct
{
    __IO uint32_t CFDC0NCFG;               /*!<  Channel 0 Nominal Bitrate Configuration Register                                             Address offset: 0000h                                     */
    __IO uint32_t CFDC0CTR;                /*!<  Channel 0 Control Register                                                                   Address offset: 0004h                                     */
    __IO uint32_t CFDC0STS;                /*!<  Channel 0 Status Register                                                                    Address offset: 0008h                                     */
    __IO uint32_t CFDC0ERFL;               /*!<  Channel 0 Error Flag Register                                                                Address offset: 000Ch                                     */
    __IO  uint32_t CFDGIPV;                /*!<  Global IP Version Register                                                                   Address offset: 0010h                                     */
    __IO uint32_t CFDGCFG;                 /*!<  Global Configuration Register                                                                Address offset: 0014h                                     */
    __IO  uint32_t CFDGCTR;                /*!<  Global Control Register                                                                      Address offset: 0018h                                     */
    __IO uint32_t CFDGSTS;                 /*!<  Global Status Register                                                                       Address offset: 001Ch                                     */
    __IO uint32_t CFDGERFL;                /*!<  Global Error Flag Register                                                                   Address offset: 0020h                                     */
    __IO uint32_t CFDGTSC;                 /*!<  Global Timestamp Counter Register                                                            Address offset: 0024h                                     */
    __IO uint32_t CFDGAFLECTR;             /*!<  Global Acceptance Filter List Entry Control Register                                         Address offset: 0028h                                     */
    __IO uint32_t CFDGAFLCFG;              /*!<  Global Acceptance Filter List Configuration Register                                         Address offset: 002Ch                                     */
    __IO  uint32_t CFDRMNB;                /*!<  RX Message Buffer Number Register                                                            Address offset: 0030h                                     */
    __IO  uint32_t CFDRMND;                /*!<  RX Message Buffer New Data Register                                                          Address offset: 0034h                                     */
    __IO  uint32_t CFDRMIEC;               /*!<  RX Message Buffer Interrupt Enable Configuration Register                                    Address offset: 0038h                                     */
    __IO  uint32_t CFDRFCC[2];             /*!<  RX FIFO Configuration / Control Registers a = [0:1]                                          Address offset: 003Ch + a*0004h                           */
    __IO  uint32_t CFDRFSTS[2];            /*!<  RX FIFO Status Registers a = [0:1]                                                           Address offset: 0044h + a*0004h                           */
    __IO  uint32_t CFDRFPCTR[2];           /*!<  RX FIFO Pointer Control Registers a = [0:1]                                                  Address offset: 004Ch + a*0004h                           */
    __IO  uint32_t CFDCFCC;                /*!<  Common FIFO Configuration / Control Register                                                 Address offset: 0054h                                     */
    __IO  uint32_t CFDCFSTS;               /*!<  Common FIFO Status Register                                                                  Address offset: 0058h                                     */
    __IO  uint32_t CFDCFPCTR;              /*!<  Common FIFO Pointer Control Register                                                         Address offset: 005Ch                                     */
    __IO  uint32_t CFDFESTS;               /*!<  FIFO Empty Status Register                                                                   Address offset: 0060h                                     */
    __IO  uint32_t CFDFFSTS;               /*!<  FIFO Full Status Register                                                                    Address offset: 0064h                                     */
    __IO  uint32_t CFDFMSTS;               /*!<  FIFO Message Lost Status Register                                                            Address offset: 0068h                                     */
    __IO uint32_t CFDRFISTS;               /*!<  RX FIFO Interrupt Flag Status Register                                                       Address offset: 006Ch                                     */
    __IO uint8_t  CFDTMC[4];               /*!<  TX Message Buffer Control Registers I = [0:3]                                                Address offset: 0070h + i*0001h                           */
    __IO uint8_t  CFDTMSTS[4];             /*!<  TX Message Buffer Status Registers j = [0:3]                                                 Address offset: 0074h + j*0001h                           */
    __IO uint32_t CFDTMTRSTS;              /*!<  TX Message Buffer Transmission Request Status Register                                       Address offset: 0078h                                     */
    __IO uint32_t CFDTMTARSTS;             /*!<  TX Message Buffer Transmission Abort Request Status Register                                 Address offset: 007Ch                                     */
    __IO uint32_t CFDTMTCSTS;              /*!<  TX Message Buffer Transmission Completion Status Register                                    Address offset: 0080h                                     */
    __IO uint32_t CFDTMTASTS;              /*!<  TX Message Buffer Transmission Abort Status Register                                         Address offset: 0084h                                     */
    __IO uint32_t CFDTMIEC;                /*!<  TX Message Buffer Interrupt Enable Configuration Register                                    Address offset: 0088h                                     */
    __IO uint32_t CFDTXQCC;                /*!<  TX Queue Configuration / Control Register                                                    Address offset: 008Ch                                     */
    __IO uint32_t CFDTXQSTS;               /*!<  TX Queue Status Register                                                                     Address offset: 0090h                                     */
    __IO uint32_t CFDTXQPCTR;              /*!<  TX Queue Pointer Control Register                                                            Address offset: 0094h                                     */
    __IO uint32_t CFDTHLCC;                /*!<  TX History List Configuration / Control Register                                             Address offset: 0098h                                     */
    __IO uint32_t CFDTHLSTS;               /*!<  TX History List Status Register                                                              Address offset: 009Ch                                     */
    __IO uint32_t CFDTHLPCTR;              /*!<  TX History List Pointer Control Register                                                     Address offset: 00A0h                                     */
    __IO uint32_t CFDGTINTSTS;             /*!<  Global TX Interrupt Status Register                                                          Address offset: 00A4h                                     */
    __IO uint32_t CFDGTSTCFG;              /*!<  Global Test Configuration Register                                                           Address offset: 00A8h                                     */
    __IO uint32_t CFDGTSTCTR;              /*!<  Global Test Control Register                                                                 Address offset: 00ACh                                     */
    __IO uint32_t CFDGFDCFG;               /*!<  Global FD Configuration register                                                             Address offset: 00B0h                                     */
    __IO uint32_t RSVD;                    /*!<  RSVD                                                                                         Address offset: 00B4h                                     */
    __IO uint32_t CFDGLOCKK;               /*!<  Global Lock Key Register                                                                     Address offset: 00B8h                                     */
    __IO uint32_t RSVD1;                   /*!<  RSVD                                                                                         Address offset: 00BCh                                     */
    __IO uint32_t CFDGAFLIGNENT;           /*!<  Global AFL Ignore Entry Register                                                             Address offset: 00C0h                                     */
    __IO uint32_t CFDGAFLIGNCTR;           /*!<  Global AFL Ignore Control Register                                                           Address offset: 00C4h                                     */
    __IO uint32_t CFDCDTCT;                /*!<  DMA Transfer Control Register                                                                Address offset: 00C8h                                     */
    __IO uint32_t CFDCDTSTS;               /*!<  DMA Transfer Status Register                                                                 Address offset: 00CCh                                     */
    __IO uint32_t CFDGPFLECTR;             /*!<  Pretended Network Filter List Entry control Register                                         Address offset: 00D0h                                     */
    __IO uint32_t CFDGPFLCFG;              /*!<  Pretended Network Filter List Entry Configuration Register                                   Address offset: 00D4h                                     */
    __IO uint32_t CFDGRSTC;                /*!<  Global SW reset Register                                                                     Address offset: 00D8h                                     */
    __IO uint32_t RSVD2;                   /*!<  RSVD                                                                                         Address offset: 00DCh                                     */
    __IO uint32_t CFDWKCR;                 /*!<  Global SW reset Register                                                                     Address offset: 00E0h                                     */
    __IO uint32_t CFDWKSR;                 /*!<  Global SW reset Register                                                                     Address offset: 00E4h                                     */
    __IO uint32_t RSVD3[6];                /*!<  RSVD                                                                                         Address offset: 00E8h                                     */
    __IO uint32_t CFDC0DCFG;               /*!<  Channel 0 Data Bitrate Configuration Register                                                Address offset: 0100h                                     */
    __IO uint32_t CFDC0FDCFG;              /*!<  Channel 0 CAN-FD Configuration Register                                                      Address offset: 0104h                                     */
    __IO uint32_t CFDC0FDCTR;              /*!<  Channel 0 CAN-FD Control Register                                                            Address offset: 0108h                                     */
    __IO uint32_t CFDC0FDSTS;              /*!<  Channel 0 CAN-FD Status Register                                                             Address offset: 010Ch                                     */
    __IO uint32_t CFDC0FDCRC;              /*!<  Channel 0 CAN-FD CRC Register                                                                Address offset: 0110h                                     */
    __IO uint32_t RSVD4[3];	               /*!<  RSVD                                                                                         Address offset: 0114h                                     */
    struct
    {
    __IO uint32_t CFDGAFLIDr;              /*!<  Global Acceptance Filter List ID Registers r = [1…10]h                                       Address offset: 0120h + (r-1)*0010h                      */
    __IO uint32_t CFDGAFLMr;               /*!<  Global Acceptance Filter List Mask Registers r = [1…10]h                                     Address offset: 0124h + (r-1)*0010h                      */
    __IO uint32_t CFDGAFLP0r;              /*!<  Global Acceptance Filter List Pointer 0 Registers r = [1…10]h                                Address offset: 0128h + (r-1)*0010h                      */
    __IO uint32_t CFDGAFLP1r;              /*!<  Global Acceptance Filter List Pointer 1 Registers r = [1…10]h                                Address offset: 012Ch + (r-1)*0010h                      */
    }CFDGAFL[16];
    struct
    {
    __IO uint32_t CFDGPFLIDs;              /*!<  Global Pretended Network Filter List ID Registers s = [1…2]h                                 Address offset: 0220h + (s-1) * 0024h                    */
    __IO uint32_t CFDGPFLMs;               /*!<  Global Pretended Network Filter List Mask Registers s = [1…2]h                               Address offset: 0224h + (s-1) * 0024h                    */
    __IO uint32_t CFDGPFLP0s;              /*!<  Global Pretended Network Filter List Pointer 0 Registers s = [1…2]h                          Address offset: 0228h + (s-1) * 0024h                    */
    __IO uint32_t CFDGPFLP1s;              /*!<  Global Pretended Network Filter List Pointer 1 Registers s = [1…2]h                          Address offset: 022Ch + (s-1) * 0024h                    */ 
    __IO uint32_t CFDGPFLPTs;              /*!<  Global Pretended Network Filter List Filter Payload Type Registers s = [1…2]h                Address offset: 0230h + (s-1) * 0024h                    */        
    __IO uint32_t CFDGPFLPD0s;             /*!<  Global Pretended Network Filter List Payload Data 0 Registers s = [1…2]                      Address offset: 0234h + (s-1) * 0024h                    */        
    __IO uint32_t CFDGPFLPM0s;             /*!<  Global Pretended Network Filter List Payload Mask 0 Registers s = [1…2]h                     Address offset: 0238h + (s-1) * 0024h                    */        
    __IO uint32_t CFDGPFLPD1s;             /*!<  Global Pretended Network Filter List Payload Data 1 Registers s = [1…2]h                     Address offset: 023Ch + (s-1) * 0024h                    */        
    __IO uint32_t CFDGPFLPM1s;             /*!<  Global Pretended Network Filter List Payload Mask 1 Registers s = [1…2]h                     Address offset: 0240h + (s-1) * 0024h                    */        
    }CFDGPFL[2];
    __IO uint32_t RSVD5[6];                /*!<  RSVD                                                                                         Address offset: 0268h                                    */
    __IO uint32_t CFDRPGACC[64];           /*!<  RAM Test Page Access Registers k = [0…3F]h                                                   Address offset: 0280h + k * 0004h                        */
}CANFD_COMMON_Type;


/* ================================================================================ */
/* ================                     CANFDx_RX_FIFO             ================ */
/* ================================================================================ */

typedef struct
{
  struct
  {
    __IO uint32_t CFDRFIDb;             /*!<  RX FIFO Access ID Registers b = [0＃1]h,                                         Address offset: 0520h + b * 004Ch             */
    __IO uint32_t CFDRFPTRb;            /*!<  RX FIFO Access Pointer Registers b = [0＃1]h,                                    Address offset: 0524h + b * 004Ch             */
    __IO uint32_t CFDRFFDSTSb;          /*!<  RX FIFO Access CAN-FD Status Registers b = [0＃1]h,                              Address offset: 0528h + b * 004Ch             */
    __IO uint32_t CFDRFDFbp[16];        /*!<  RX FIFO Access Data Field p Registers b = [0＃1]h p = [0＃F]h,                   Address offset: 052Ch + p * 0004h + b * 004Ch */
  }CFDRF[2];
}CANFD_RX_FIFO_Type;

/* ================================================================================ */
/* ================                     CANFDx_Common_FIFO             ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CFDCFID;                /*!<  Common FIFO Access ID Register,                                                  Address offset: 05B8h                         */
  __IO uint32_t CFDCFPTR;               /*!<  Common FIFO Access Pointer Register,                                             Address offset: 05BCh                         */
  __IO uint32_t CFDCFFDCSTS;            /*!<  Common FIFO Access CAN-FD Control/Status Register,                               Address offset: 05C0h                         */
  __IO uint32_t CFDCFDFp[16];           /*!<  Common FIFO Access Data Field p Registers p = [0＃F]h,                           Address offset: 05C4h + p * 0004h             */
}CANFD_COMMON_FIFO_Type;

/* ================================================================================ */
/* ================                     CANFDx_TX_MESSAGE          ================ */
/* ================================================================================ */

typedef struct
{
   struct
   {
     __IO uint32_t CFDTMIDb;             /*!<  TX Message Buffer ID Registers b = [0＃3]h,                                      Address offset: 0604h + b * 004Ch             */
     __IO uint32_t CFDTMPTRb;            /*!<  TX Message Buffer Pointer Registers b = [0＃3]h,                                 Address offset: 0608h + b * 004Ch             */
     __IO uint32_t CFDTMFDCTRb;          /*!<  TX Message Buffer CAN-FD Control Registers b = [0＃3]h,                          Address offset: 060Ch + b * 004Ch             */
     __IO uint32_t CFDTMDFbp[16];        /*!<  TX Message Buffer Data Field p Registers b = [0＃3]h p = [0＃F]h,                Address offset: 0610h + p * 0004h + b * 004Ch */
   }CFDTM[4];
}CANFD_TX_MESSAGE_Type;

/* ================================================================================ */
/* ================                     CANFDx_TX_HISTORY_LIST          ================ */
/* ================================================================================ */

typedef struct
{
  __IO uint32_t CFDTHLACC0;             /*!<  Channel 0 TX History List Access Registers 0,                                    Address offset: 0740h                         */
  __IO uint32_t CFDTHLACC1;             /*!<  Channel 0 TX History List Access Registers 1,                                    Address offset: 0744h                         */
}CANFD_TX_HISTORY_LIST_Type;


/* ================================================================================ */
/* ================                     CANFDx_RX_MESSAGE          ================ */
/* ================================================================================ */

typedef struct
{
  struct
  {
    __IO uint32_t CFDRMIDb;             /*!<  RX Message Buffer ID Registers b = [0＃7]h,                                      Address offset: 0920h + b * 004Ch             */
    __IO uint32_t CFDRMPTRb;            /*!<  RX Message Buffer Pointer Registers b = [0＃7]h,                                 Address offset: 0924h + b * 004Ch             */
    __IO uint32_t CFDRMFDSTSb;          /*!<  RX Message Buffer CAN-FD Status Registers b = [0＃7]h,                           Address offset: 0928h + b * 004Ch             */
    __IO uint32_t CFDRMDFbp[16];        /*!<  RX Message Buffer Data Field p Registers b = [0＃7]h p = [0＃F]h,                Address offset: 092Ch + p * 0004h + b * 004Ch */
  }CFDRM[8];
}CANFD_RX_MESSAGE_Type;

/* ================================================================================ */
/* ================                       DMA                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CH0CR;                  /*!<  Channel0ControlRegister,                              Address offset: 0x00 */
    __IO uint32_t CH0CFGR;                /*!<  Channel0ConfigRegister,                               Address offset: 0x04 */
    __IO uint32_t CH0PAR;                 /*!<  Channel0PeripheralAddressRegister,                    Address offset: 0x08 */
    __IO uint32_t CH0MAR0;                /*!<  Channel0MemoryAddressRegister0,                       Address offset: 0x0C */
    __IO uint32_t CH0MAR1;                /*!<  Channel0MemoryAddressRegister1,                       Address offset: 0x10 */
    __IO uint32_t RSV1[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x14 */
    __IO uint32_t CH1CR;                  /*!<  Channel1ControlRegister,                              Address offset: 0x20 */
    __IO uint32_t CH1CFGR;                /*!<  Channel1ConfigRegister,                               Address offset: 0x24 */
    __IO uint32_t CH1PAR;                 /*!<  Channel1PeripheralAddressRegister,                    Address offset: 0x28 */
    __IO uint32_t CH1MAR0;                /*!<  Channel1MemoryAddressRegister0,                       Address offset: 0x2C */
    __IO uint32_t CH1MAR1;                /*!<  Channel1MemoryAddressRegister1,                       Address offset: 0x30 */
    __IO uint32_t RSV2[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x34 */
    __IO uint32_t CH2CR;                  /*!<  Channel2ControlRegister,                              Address offset: 0x40 */
    __IO uint32_t CH2CFGR;                /*!<  Channel2ConfigRegister,                               Address offset: 0x44 */
    __IO uint32_t CH2PAR;                 /*!<  Channel2PeripheralAddressRegister,                    Address offset: 0x48 */
    __IO uint32_t CH2MAR0;                /*!<  Channel2MemoryAddressRegister0,                       Address offset: 0x4C */
    __IO uint32_t CH2MAR1;                /*!<  Channel2MemoryAddressRegister1,                       Address offset: 0x50 */
    __IO uint32_t RSV3[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x54 */
    __IO uint32_t CH3CR;                  /*!<  Channel3ControlRegister,                              Address offset: 0x60 */
    __IO uint32_t CH3CFGR;                /*!<  Channel3ConfigRegister,                               Address offset: 0x64 */
    __IO uint32_t CH3PAR;                 /*!<  Channel3PeripheralAddressRegister,                    Address offset: 0x68 */
    __IO uint32_t CH3MAR0;                /*!<  Channel3MemoryAddressRegister0,                       Address offset: 0x6C */
    __IO uint32_t CH3MAR1;                /*!<  Channel3MemoryAddressRegister1,                       Address offset: 0x70 */
    __IO uint32_t RSV4[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x74 */
    __IO uint32_t CH4CR;                  /*!<  Channel4ControlRegister,                              Address offset: 0x80 */
    __IO uint32_t CH4CFGR;                /*!<  Channel4ConfigRegister,                               Address offset: 0x84 */
    __IO uint32_t CH4PAR;                 /*!<  Channel4PeripheralAddressRegister,                    Address offset: 0x88 */
    __IO uint32_t CH4MAR0;                /*!<  Channel4MemoryAddressRegister0,                       Address offset: 0x8C */
    __IO uint32_t CH4MAR1;                /*!<  Channel4MemoryAddressRegister1,                       Address offset: 0x90 */
    __IO uint32_t RSV5[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x94 */
    __IO uint32_t CH5CR;                  /*!<  Channel5ControlRegister,                              Address offset: 0xA0 */
    __IO uint32_t CH5CFGR;                /*!<  Channel5ConfigRegister,                               Address offset: 0xA4 */
    __IO uint32_t CH5PAR;                 /*!<  Channel5PeripheralAddressRegister,                    Address offset: 0xA8 */
    __IO uint32_t CH5MAR0;                /*!<  Channel5MemoryAddressRegister0,                       Address offset: 0xAC */
    __IO uint32_t CH5MAR1;                /*!<  Channel5MemoryAddressRegister1,                       Address offset: 0xB0 */
    __IO uint32_t RSV6[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0xB4 */
    __IO uint32_t CH6CR;                  /*!<  Channel6ControlRegister,                              Address offset: 0xC0 */
    __IO uint32_t CH6CFGR;                /*!<  Channel6ConfigRegister,                               Address offset: 0xC4 */
    __IO uint32_t CH6PAR;                 /*!<  Channel6PeripheralAddressRegister,                    Address offset: 0xC8 */
    __IO uint32_t CH6MAR0;                /*!<  Channel6MemoryAddressRegister0,                       Address offset: 0xCC */
    __IO uint32_t CH6MAR1;                /*!<  Channel6MemoryAddressRegister1,                       Address offset: 0xD0 */
    __IO uint32_t RSV7[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0xD4 */
    __IO uint32_t CH7CR;                  /*!<  Channel7ControlRegister,                              Address offset: 0xE0 */
    __IO uint32_t CH7FAR;                 /*!<  Channel7FlashAddressRegister,                         Address offset: 0xE4 */
    __IO uint32_t CH7RAR;                 /*!<  Channel7RAMAddressRegister,                           Address offset: 0xE8 */
    __IO uint32_t RSV8[69];               /*!<  RESERVED REGISTER,                                    Address offset: 0xEC */
    __IO uint32_t GCR;                    /*!<  DMAGlobalControlRegister,                             Address offset: 0x200 */
    __IO uint32_t SWRR;                   /*!<  DMASoftwareRequestRegister,                           Address offset: 0x204 */
    __IO uint32_t RSV9[62];               /*!<  RESERVED REGISTER,                                    Address offset: 0x208 */
    __IO uint32_t ISR;                    /*!<  DMAInterruptStatusRegister,                           Address offset: 0x300 */
    __I  uint32_t CH0TFSADDR;             /*!<  ,                                                     Address offset: 0x304 */
    __I  uint32_t CH1TFSADDR;             /*!<  ,                                                     Address offset: 0x308 */
    __I  uint32_t CH2TFSADDR;             /*!<  ,                                                     Address offset: 0x30C */
    __I  uint32_t CH3TFSADDR;             /*!<  ,                                                     Address offset: 0x310 */
    __I  uint32_t CH4TFSADDR;             /*!<  ,                                                     Address offset: 0x314 */
    __I  uint32_t CH5TFSADDR;             /*!<  ,                                                     Address offset: 0x318 */
    __I  uint32_t CH6TFSADDR;             /*!<  ,                                                     Address offset: 0x31C */
} DMA_Type;

/* ================================================================================ */
/* ================                       CRC                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t DR;                     /*!<  CRCDataRegister,                                      Address offset: 0x00 */
    __IO uint32_t CR;                     /*!<  CRCControlRegister,                                   Address offset: 0x04 */
    __IO uint32_t LFSR;                   /*!<  CRCLinearFeedbackShiftRegister,                       Address offset: 0x08 */
    __IO uint32_t XOR;                    /*!<  CRCoutputXORRegister,                                 Address offset: 0x0C */
    __IO uint32_t RSV1[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x10 */
    __IO uint32_t POLY;                   /*!<  CRCPolynominalRegister,                               Address offset: 0x1C */
} CRC_Type;

/* ================================================================================ */
/* ================                      ATIMx                     ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR1;                    /*!<  ,                                                     Address offset: 0x00 */
    __IO uint32_t CR2;                    /*!<  ,                                                     Address offset: 0x04 */
    __IO uint32_t SMCR;                   /*!<  ,                                                     Address offset: 0x08 */
    __IO uint32_t DIER;                   /*!<  ,                                                     Address offset: 0x0C */
    __IO uint32_t ISR;                    /*!<  ,                                                     Address offset: 0x10 */
    __O  uint32_t EGR;                    /*!<  ,                                                     Address offset: 0x14 */
    __IO uint32_t CCMR1;                  /*!<  ,                                                     Address offset: 0x18 */
    __IO uint32_t CCMR2;                  /*!<  ,                                                     Address offset: 0x1C */
    __IO uint32_t CCER;                   /*!<  ,                                                     Address offset: 0x20 */
    __IO uint32_t CNT;                    /*!<  ,                                                     Address offset: 0x24 */
    __IO uint32_t PSC;                    /*!<  ,                                                     Address offset: 0x28 */
    __IO uint32_t ARR;                    /*!<  ,                                                     Address offset: 0x2C */
    __IO uint32_t RCR;                    /*!<  ,                                                     Address offset: 0x30 */
    __IO uint32_t CCR1;                   /*!<  ,                                                     Address offset: 0x34 */
    __IO uint32_t CCR2;                   /*!<  ,                                                     Address offset: 0x38 */
    __IO uint32_t CCR3;                   /*!<  ,                                                     Address offset: 0x3C */
    __IO uint32_t CCR4;                   /*!<  ,                                                     Address offset: 0x40 */
    __IO uint32_t BDTR;                   /*!<  ,                                                     Address offset: 0x44 */
    __IO uint32_t CCMR3;                  /*!<  ,                                                     Address offset: 0x48 */
    __IO uint32_t CCR5;                   /*!<  ,                                                     Address offset: 0x4C */
    __IO uint32_t CCR6;                   /*!<  ,                                                     Address offset: 0x50 */
    __IO uint32_t ECR;                    /*!<  ,                                                     Address offset: 0x54 */
    __IO uint32_t TISEL;                  /*!<  ,                                                     Address offset: 0x58 */
    __IO uint32_t AFR;                    /*!<  ,                                                     Address offset: 0x5C */
    __IO uint32_t BRKFLAG;                /*!<  ,                                                     Address offset: 0x60 */
    __IO uint32_t RSV1[37];               /*!<  RESERVED REGISTER,                                    Address offset: 0x64 */
    __IO uint32_t DCR;                    /*!<  ,                                                     Address offset: 0xF8 */
    __IO uint32_t DMAR;                   /*!<  ,                                                     Address offset: 0xFC */
} ATIM_Type;

/* ================================================================================ */
/* ================               GPTIMx                           ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR1;                    /*!<  ,                                                     Address offset: 0x00 */
    __IO uint32_t CR2;                    /*!<  ,                                                     Address offset: 0x04 */
    __IO uint32_t SMCR;                   /*!<  ,                                                     Address offset: 0x08 */
    __IO uint32_t DIER;                   /*!<  ,                                                     Address offset: 0x0C */
    __IO uint32_t ISR;                    /*!<  ,                                                     Address offset: 0x10 */
    __O  uint32_t EGR;                    /*!<  ,                                                     Address offset: 0x14 */
    __IO uint32_t CCMR1;                    /*!<  ,                                                   Address offset: 0x18 */
    __IO uint32_t CCMR2;                      /*!<  ,                                                 Address offset: 0x1C */
    __IO uint32_t CCER;                   /*!<  ,                                                     Address offset: 0x20 */
    __IO uint32_t CNT;                    /*!<  ,                                                     Address offset: 0x24 */
    __IO uint32_t PSC;                    /*!<  ,                                                     Address offset: 0x28 */
    __IO uint32_t ARR;                    /*!<  ,                                                     Address offset: 0x2C */
    __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x30 */
    __IO uint32_t CCR1;                   /*!<  ,                                                     Address offset: 0x34 */
    __IO uint32_t CCR2;                   /*!<  ,                                                     Address offset: 0x38 */
    __IO uint32_t CCR3;                   /*!<  ,                                                     Address offset: 0x3C */
    __IO uint32_t CCR4;                   /*!<  ,                                                     Address offset: 0x40 */
    __IO uint32_t RSV2[4];                /*!<  RESERVED REGISTER,                                    Address offset: 0x44 */
    __IO uint32_t ECR;                    /*!<  ,                                                     Address offset: 0x54 */
    __IO uint32_t TISEL;                  /*!<  ,                                                     Address offset: 0x58 */
    __IO uint32_t AFR;                    /*!<  ,                                                     Address offset: 0x5C */
    __IO uint32_t RSV3[38];               /*!<  RESERVED REGISTER,                                    Address offset: 0x60 */
    __IO uint32_t DCR;                    /*!<  ,                                                     Address offset: 0xF8 */
    __IO uint32_t DMAR;                   /*!<  ,                                                     Address offset: 0xFC */
} GPTIM_Type;

/* ================================================================================ */
/* ================                     BSTIM16                    ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR1;                    /*!<  BSTIM16ControlRegister1,                              Address offset: 0x00 */
    __IO uint32_t CR2;                    /*!<  BSTIM16ControlRegister2,                              Address offset: 0x04 */
    __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x08 */
    __IO uint32_t IER;                    /*!<  BSTIM16InterruptEnableRegister,                       Address offset: 0x0C */
    __IO uint32_t ISR;                    /*!<  BSTIM16InterruptStatusRegister,                       Address offset: 0x10 */
    __O  uint32_t EGR;                    /*!<  BSTIM16EventGenerationRegister,                       Address offset: 0x14 */
    __IO uint32_t RSV2[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x18 */
    __IO uint32_t CNT;                    /*!<  BSTIM16CounterRegister,                               Address offset: 0x24 */
    __IO uint32_t PSC;                    /*!<  BSTIM16PrescalerRegister,                             Address offset: 0x28 */
    __IO uint32_t ARR;                    /*!<  BSTIM16Auto-ReloadRegister,                           Address offset: 0x2C */
} BSTIM16_Type;

/* ================================================================================ */
/* ================                     LPTIM16                    ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CFGR;                   /*!<  LPTIM16ConfigRegister,                                Address offset: 0x00 */
    __I  uint32_t CNT;                    /*!<  LPTIM16CounterRegister,                               Address offset: 0x04 */
    __IO uint32_t CCSR;                   /*!<  LPTIM16Capture/CompareControlandStatusRegister,       Address offset: 0x08 */
    __IO uint32_t ARR;                    /*!<  LPTIM16Auto-ReloadRegister,                           Address offset: 0x0C */
    __IO uint32_t IER;                    /*!<  LPTIM16InterruptEnableRegister,                       Address offset: 0x10 */
    __IO uint32_t ISR;                    /*!<  LPTIM16InterruptStatusRegister,                       Address offset: 0x14 */
    __IO uint32_t CR;                     /*!<  LPTIM16ControlRegister,                               Address offset: 0x18 */
    __IO uint32_t RSV1;                   /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
    __IO uint32_t CCR1;                   /*!<  LPTIM16Capture/CompareRegister1,                      Address offset: 0x20 */
    __IO uint32_t CCR2;                   /*!<  LPTIM16Capture/CompareRegister2,                      Address offset: 0x24 */
} LPTIM16_Type;

/* ================================================================================ */
/* ================                     TAU0                       ================ */
/* ================================================================================ */
typedef struct
{
  __IO uint32_t T0CR;                   /*!<  ,                                                     Address offset: 0x00 */
  __IO uint32_t RSV0;                   /*!<  ,                                                     Address offset: 0x04 */
  __IO uint32_t T0EGR;                  /*!<  ,                                                     Address offset: 0x08 */
  __IO uint32_t RSV1;                   /*!<  ,                                                     Address offset: 0x04 */
  __IO uint32_t T0CFGR;                 /*!<  ,                                                     Address offset: 0x10 */    
  __IO uint32_t T1CFGR;                 /*!<  ,                                                     Address offset: 0x14 */
  __IO uint32_t T2CFGR;                 /*!<  ,                                                     Address offset: 0x18 */
  __IO uint32_t T3CFGR;                 /*!<  ,                                                     Address offset: 0x1C */
  __IO uint32_t T4CFGR;                 /*!<  ,                                                     Address offset: 0x20 */
  __IO uint32_t T5CFGR;                 /*!<  ,                                                     Address offset: 0x24 */
  __IO uint32_t T6CFGR;                 /*!<  ,                                                     Address offset: 0x28 */
  __IO uint32_t T7CFGR;                 /*!<  ,                                                     Address offset: 0x2C */
  __IO uint32_t T0MDR;                  /*!<  ,                                                     Address offset: 0x30 */
  __IO uint32_t T1MDR;                  /*!<  ,                                                     Address offset: 0x34 */
  __IO uint32_t T2MDR;                  /*!<  ,                                                     Address offset: 0x38 */
  __IO uint32_t T3MDR;                  /*!<  ,                                                     Address offset: 0x3C */
  __IO uint32_t T4MDR;                  /*!<  ,                                                     Address offset: 0x40 */
  __IO uint32_t T5MDR;                  /*!<  ,                                                     Address offset: 0x44 */
  __IO uint32_t T6MDR;                  /*!<  ,                                                     Address offset: 0x48 */
  __IO uint32_t T7MDR;                  /*!<  ,                                                     Address offset: 0x4C */
  __IO uint32_t T0ARR;                  /*!<  ,                                                     Address offset: 0x50 */
  __IO uint32_t T1ARR;                  /*!<  ,                                                     Address offset: 0x54 */
  __IO uint32_t T2ARR;                  /*!<  ,                                                     Address offset: 0x58 */
  __IO uint32_t T3ARR;                  /*!<  ,                                                     Address offset: 0x5C */
  __IO uint32_t T4ARR;                  /*!<  ,                                                     Address offset: 0x60 */
  __IO uint32_t T5ARR;                  /*!<  ,                                                     Address offset: 0x64 */
  __IO uint32_t T6ARR;                  /*!<  ,                                                     Address offset: 0x68 */
  __IO uint32_t T7ARR;                  /*!<  ,                                                     Address offset: 0x6C */
  __IO uint32_t T0CCR;                  /*!<  ,                                                     Address offset: 0x70 */
  __IO uint32_t T1CCR;                  /*!<  ,                                                     Address offset: 0x74 */
  __IO uint32_t T2CCR;                  /*!<  ,                                                     Address offset: 0x78 */
  __IO uint32_t T3CCR;                  /*!<  ,                                                     Address offset: 0x7C */
  __IO uint32_t T4CCR;                  /*!<  ,                                                     Address offset: 0x80 */
  __IO uint32_t T5CCR;                  /*!<  ,                                                     Address offset: 0x84 */
  __IO uint32_t T6CCR;                  /*!<  ,                                                     Address offset: 0x88 */
  __IO uint32_t T7CCR;                  /*!<  ,                                                     Address offset: 0x8C */
  __IO uint32_t T0IER;                  /*!<  ,                                                     Address offset: 0x90 */
  __IO uint32_t T1IER;                  /*!<  ,                                                     Address offset: 0x94 */
  __IO uint32_t T2IER;                  /*!<  ,                                                     Address offset: 0x98 */
  __IO uint32_t T3IER;                  /*!<  ,                                                     Address offset: 0x9C */
  __IO uint32_t T4IER;                  /*!<  ,                                                     Address offset: 0xA0 */
  __IO uint32_t T5IER;                  /*!<  ,                                                     Address offset: 0xA4 */
  __IO uint32_t T6IER;                  /*!<  ,                                                     Address offset: 0xA8 */
  __IO uint32_t T7IER;                  /*!<  ,                                                     Address offset: 0xAC */
  __IO uint32_t T0ISR;                  /*!<  ,                                                     Address offset: 0xB0 */
  __IO uint32_t T1ISR;                  /*!<  ,                                                     Address offset: 0xB4 */
  __IO uint32_t T2ISR;                  /*!<  ,                                                     Address offset: 0xB8 */
  __IO uint32_t T3ISR;                  /*!<  ,                                                     Address offset: 0xBC */
  __IO uint32_t T4ISR;                  /*!<  ,                                                     Address offset: 0xC0 */
  __IO uint32_t T5ISR;                  /*!<  ,                                                     Address offset: 0xC4 */
  __IO uint32_t T6ISR;                  /*!<  ,                                                     Address offset: 0xC8 */
  __IO uint32_t T7ISR;                  /*!<  ,                                                     Address offset: 0xCC */
  __I  uint32_t T0CNTR;                 /*!<  ,                                                     Address offset: 0xD0 */
  __I  uint32_t T1CNTR;                 /*!<  ,                                                     Address offset: 0xD4 */
  __I  uint32_t T2CNTR;                 /*!<  ,                                                     Address offset: 0xD8 */
  __I  uint32_t T3CNTR;                 /*!<  ,                                                     Address offset: 0xDC */
  __I  uint32_t T4CNTR;                 /*!<  ,                                                     Address offset: 0xE0 */
  __I  uint32_t T5CNTR;                 /*!<  ,                                                     Address offset: 0xE4 */
  __I  uint32_t T6CNTR;                 /*!<  ,                                                     Address offset: 0xE8 */
  __I  uint32_t T7CNTR;                 /*!<  ,                                                     Address offset: 0xEC */
  __IO uint32_t RSV2[20];               /*!<  ,                                                     Address offset: 0x18 */
}TAU0_Type;

/* ================================================================================ */
/* ================                       ADC                      ================ */
/* ================================================================================ */


/**
  * @brief ADC module information (ADC)
  */

typedef struct {                                    /*!< ADC Structure                                     */
  __IO uint32_t  GISR;                              /*!< GISR Register,          Address offset: 0x00 */
  __IO uint32_t  GIER;                              /*!< GIER Register,          Address offset: 0x04 */
  __IO uint32_t  CR1;                               /*!< CR1 Register,           Address offset: 0x08 */
  __IO uint32_t  CR2;                               /*!< CR2 Register,           Address offset: 0x0C */
  __I  uint32_t  RESERVED;                          /*!< RESERVED REGISTER,      Address offset: 0x10 */
  __IO uint32_t  CFGR1;                             /*!< CFGR1 Register ,        Address offset: 0x14 */
  __IO uint32_t  CFGR2;                             /*!< CFGR2 Register ,        Address offset: 0x18 */
  __I  uint32_t  RESERVED1[2];                      /*!< RESERVED REGISTER,      Address offset: 0x1C */
  __IO uint32_t  OSR;                               /*!< OSR Register,           Address offset: 0x24 */
  __IO uint32_t  ACQ_SR;                            /*!< ACQ_SR Register ,       Address offset: 0x28 */
  __IO uint32_t  ACQ_SACR;                          /*!< ACQ_SACR Register,      Address offset: 0x2C */
  __IO uint32_t  HLTR;                              /*!< HLTR Register,          Address offset: 0x30 */
  __IO uint32_t  AWDCR;                             /*!< AWDCR Register,         Address offset: 0x34 */
  __IO uint32_t  ACQ_ISR;                           /*!< ACQ_ISR Register,       Address offset: 0x38 */
  __I  uint32_t  RESERVED2;                          /*!< RESERVED REGISTER,      Address offset: 0x3C */
  __IO uint32_t  ACQ_IER;                           /*!< ACQ_IER Register,       Address offset: 0x40 */
  __I  uint32_t  RESERVED3[7];                      /*!< RESERVED REGISTER,      Address offset: 0x44 */
  __IO uint32_t  DMACOMB_CR;                        /*!< GISR Register,          Address offset: 0x60 */
  __IO uint32_t  DMACOMB_DR;                        /*!< GISR Register,          Address offset: 0x64 */
  __I  uint32_t  RESERVED4[6];                      /*!< RESERVED REGISTER,      Address offset: 0x68 */
  __IO uint32_t  ACQ0_CR;                           /*!< ACQ0_CR Register,       Address offset: 0x80 */
  __IO uint32_t  ACQ1_CR;                           /*!< ACQ1_CR Register,       Address offset: 0x84 */
  __IO uint32_t  ACQ2_CR;                           /*!< ACQ2_CR Register,       Address offset: 0x88 */
  __IO uint32_t  ACQ3_CR;                           /*!< ACQ3_CR Register,       Address offset: 0x8C */
  __IO uint32_t  ACQ4_CR;                           /*!< ACQ4_CR Register,       Address offset: 0x90 */
  __I  uint32_t  RESERVED5[11];                     /*!< RESERVED REGISTER,      Address offset: 0x94 */
  __IO uint32_t  ACQ0_DR;                           /*!< ACQ0_DR Register,       Address offset: 0xC0 */
  __IO uint32_t  ACQ1_DR;                           /*!< ACQ1_DR Register,       Address offset: 0xC4 */
  __IO uint32_t  ACQ2_DR;                           /*!< ACQ2_DR Register,       Address offset: 0xC8 */
  __IO uint32_t  ACQ3_DR;                           /*!< ACQ3_DR Register,       Address offset: 0xCC */
  __IO uint32_t  ACQ4_DR;                           /*!< ACQ4_DR Register,       Address offset: 0xD0 */
  __I  uint32_t  RESERVED6[201];                    /*!< RESERVED REGISTER,      Address offset: 0xD4 */
  __IO uint32_t  DBG_DR;                            /*!< ACQ0_DR Register,       Address offset: 0x3F8 */
  __IO uint32_t  DBG_CR;                            /*!< ACQ0_DR Register,       Address offset: 0x3FC */
} ADC_Type;

/* ================================================================================ */
/* ================                      DACx                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR1;                    /*!<  DACxControlRegister,                                  Address offset: 0x00 */
    __IO uint32_t CR2;                    /*!<  DACxControlRegister,                                  Address offset: 0x04 */
    __IO uint32_t CFGR;                   /*!<  DACxConfigRegister,                                   Address offset: 0x08 */
    __O  uint32_t SWTRGR;                 /*!<  DACxSoftwareTriggerRegister,                          Address offset: 0x0C */
    __IO uint32_t DHR;                    /*!<  DACxDataHoldingRegister,                              Address offset: 0x10 */
    __IO uint32_t ISR;                    /*!<  DACxInterruptStatusRegister,                          Address offset: 0x14 */
    __IO uint32_t IER;                    /*!<  DACxInterruptEnableRegister,                          Address offset: 0x18 */
    __IO uint32_t SHTR;                   /*!<  DACxSampleHoldTimeRegister,                           Address offset: 0x1C */
} DAC_Type;

/* ================================================================================ */
/* ================                       PGL                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR;                     /*!<  ,                                                     Address offset: 0x00 */
    __IO uint32_t CFGR0;                  /*!<  ,                                                     Address offset: 0x04 */
    __IO uint32_t CFGR1;                  /*!<  ,                                                     Address offset: 0x08 */
    __IO uint32_t CFGR2;                  /*!<  ,                                                     Address offset: 0x0C */
    __IO uint32_t CFGR3;                  /*!<  ,                                                     Address offset: 0x10 */
    __IO uint32_t IER;                    /*!<  ,                                                     Address offset: 0x14 */
    __IO uint32_t ISR;                    /*!<  ,                                                     Address offset: 0x18 */
    __IO uint32_t LUT0;                   /*!<  ,                                                     Address offset: 0x1C */
    __IO uint32_t LUT1;                   /*!<  ,                                                     Address offset: 0x20 */
    __IO uint32_t LUT2;                   /*!<  ,                                                     Address offset: 0x24 */
    __IO uint32_t LUT3;                   /*!<  ,                                                     Address offset: 0x28 */
} PGL_Type;

/* ================================================================================ */
/* ================                      GPIO                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t EXTIEDSA;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x00 */
    __IO uint32_t EXTIEDSB;               /*!<  ExternalInterruptInputSelectRegister1,                Address offset: 0x04 */
    __IO uint32_t EXTIEDSC;               /*!<  ExternalInterruptEdgeSelectandEnableRegister0,        Address offset: 0x08 */
    __IO uint32_t EXTIEDSD;               /*!<  ExternalInterruptEdgeSelectandEnableRegister1,        Address offset: 0x0C */
    __IO uint32_t EXTIEDSE;               /*!<  ExternalInterruptDigitalFilterRegister,               Address offset: 0x10 */
    __IO uint32_t EXTIEDSF;               /*!<  ExternalInterruptandStatusRegister,                   Address offset: 0x14 */
    __IO uint32_t EXTIEDSG;               /*!<  ExternalInterruptDataInputRegister,                   Address offset: 0x18 */
    __IO uint32_t RSV1[2];                /*!<  RESERVED REGISTER,                                    Address offset: 0x1C */
  	__IO uint32_t EXTIDFAB;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x24 */
 	  __IO uint32_t EXTIDFCD;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x28 */
	  __IO uint32_t EXTIDFEF;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x2C */
	  __IO uint32_t EXTIDFGH;                /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x30 */
  	__IO uint32_t RSV2[1];                /*!<  RESERVED REGISTER,                                    Address offset: 0x34 */
  	__IO uint32_t EXTIISRAB;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x38 */
  	__IO uint32_t EXTIISRCD;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x3C */
  	__IO uint32_t EXTIISREF;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x40 */
  	__IO uint32_t EXTIISRGH;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x44 */
  	__IO uint32_t RSV3[1];                /*!<  RESERVED REGISTER,                                    Address offset: 0x48 */
  	__IO uint32_t EXTIDIAB;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x4C */
   	__IO uint32_t EXTIDICD;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x50 */
	  __IO uint32_t EXTIDIEF;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x54 */
	  __IO uint32_t EXTIDIGH;               /*!<  ExternalInterruptInputSelectRegister0,                Address offset: 0x58 */
	  __IO uint32_t RSV4[9];                /*!<  RESERVED REGISTER,                                    Address offset: 0x5C */
    __IO uint32_t FOUTSEL;                /*!<  FrequencyOutputSelectRegister,                        Address offset: 0x60 */
    __IO uint32_t RSV5[22];               /*!<  RESERVED REGISTER,                                    Address offset: 0x64 */
    __IO uint32_t PINWKEN;                /*!<  WakeupEnableRegister,                                 Address offset: 0xBC */
} GPIO_COMMON_Type;

typedef struct
{
    __IO uint32_t INEN;                   /*!<  GPIOxInputEnableRegister,                             Address offset: 0x00 */
    __IO uint32_t PUDEN;                  /*!<  GPIOxPull-UpPull-DownEnableRegister,                  Address offset: 0x04 */
    __IO uint32_t ODEN;                   /*!<  GPIOxOpen-DrainEnableRegister,                        Address offset: 0x08 */
    __IO uint32_t FCR;                    /*!<  GPIOxFunctionControlRegister,                         Address offset: 0x0C */
    __IO uint32_t DO;                     /*!<  GPIOxDataOutputRegister,                              Address offset: 0x10 */
    __O  uint32_t DSET;                   /*!<  GPIOxDataSetRegister,                                 Address offset: 0x14 */
    __O  uint32_t DRST;                   /*!<  GPIOxDataResetRegister,                               Address offset: 0x18 */
    __I  uint32_t DIN;                    /*!<  GPIOxDataInputRegister,                               Address offset: 0x1C */
    __IO uint32_t DFS;                    /*!<  GPIOxDigitalFunctionSelect,                           Address offset: 0x20 */
    __IO uint32_t RSV1[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x24 */
    __IO uint32_t DSR;                    /*!<  GPIOxAnalogchannelEnableRegister,                     Address offset: 0x30 */ 
    __IO uint32_t RSV2[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x24 */	
} GPIO_Type;

/* ================================================================================ */
/* ================                       SFU                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t RSV1[2];                 /*!< , Address offset: 0x00 */
	__IO uint32_t MAP_CR;                  /*!< , Address offset: 0x08 */
    __IO uint32_t MAP_SR;                  /*!< , Address offset: 0x0C */
} SFU_Type;

/* ================================================================================ */
/* ================                      QSPI                      ================ */
/* ================================================================================ */

typedef struct
{
    __IO uint32_t CR;                      /*!<  , Address offset: 0x00 */
	__IO uint32_t CFGR;                    /*!<  , Address offset: 0x04 */
    __IO uint32_t SR;                      /*!<  , Address offset: 0x08 */
    __IO uint32_t DATALEN;                 /*!<  , Address offset: 0x0C */
    __IO uint32_t CCR;                     /*!<  , Address offset: 0x10 */
    __IO uint32_t ADDR;                    /*!<  , Address offset: 0x14 */
    __IO uint32_t ABR;                     /*!<  , Address offset: 0x18 */
    __IO uint32_t DATA;                    /*!<  , Address offset: 0x1C */
    __IO uint32_t SMSK;                    /*!<  , Address offset: 0x20 */
    __IO uint32_t SMAT;                    /*!<  , Address offset: 0x24 */
    __IO uint32_t PITV;                    /*!<  , Address offset: 0x28 */
	__IO uint32_t TOR;                     /*!<  , Address offset: 0x2C */
} QSPI_Type;

/* ================================================================================ */
/* ================                       SCU                      ================ */
/* ================================================================================ */

typedef struct
{
    __I  uint32_t SYSCFG;                 /*!<  ,                                                     Address offset: 0x00 */
    __I  uint32_t USRCFG;                 /*!<  ,                                                     Address offset: 0x04 */
    __IO uint32_t DBGCFG;                 /*!<  ,                                                     Address offset: 0x08 */
    __IO uint32_t RAMCFG;                 /*!<  ,                                                     Address offset: 0x0C */
    __IO uint32_t FLS_ACLOCK1;            /*!<  ,                                                     Address offset: 0x10 */
    __IO uint32_t FLS_ACLOCK2;            /*!<  ,                                                     Address offset: 0x14 */
    __I  uint32_t RAMRP00;                /*!<  ,                                                     Address offset: 0x18 */
    __I  uint32_t RAMRP01;                /*!<  ,                                                     Address offset: 0x1C */
    __I  uint32_t RAMRP10;                /*!<  ,                                                     Address offset: 0x20 */
    __I  uint32_t RAMRP11;                /*!<  ,                                                     Address offset: 0x24 */
    __I  uint32_t RAMRP20;                /*!<  ,                                                     Address offset: 0x2C */
    __I  uint32_t RAMRP21;                /*!<  ,                                                     Address offset: 0x30 */
	  __I  uint32_t RAMRP30;                /*!<  ,                                                     Address offset: 0x34 */
    __I  uint32_t RAMRP31;                /*!<  ,                                                     Address offset: 0x38 */
    __I  uint32_t FLS_RED;                /*!<  ,                                                     Address offset: 0x3C */
    __IO uint32_t RSV1[5];                /*!<  RESERVED REGISTER,                                    Address offset: 0x40 */
    __I  uint32_t FLS_TSTEN;              /*!<  ,                                                     Address offset: 0x50 */
    __IO uint32_t RSV2[3];                /*!<  RESERVED REGISTER,                                    Address offset: 0x54 */
    __I  uint32_t SCU_VRSR;               /*!<  ,                                                     Address offset: 0x60 */
} SCU_Type;



/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
#pragma pop
#elif defined(__ICCARM__)
/* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
#pragma warning restore
#else
#warning Not supported compiler type
#endif

/* ================================================================================ */
/* ================              CPU memory map                    ================ */
/* ================================================================================ */

/* Peripheral and SRAM base address */

#define FLASH_BASE            ((     uint32_t)0x00000000)
#define SRAM_BASE             ((     uint32_t)0x20000000)
#define PERIPH_BASE           ((     uint32_t)0x40000000)

/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

/* Peripheral memory map */
#define SCU_BASE                        (PERIPH_BASE        +0x00000000)
#define DMA_BASE                        (PERIPH_BASE        +0x00000400)
#define GPIOA_BASE                      (PERIPH_BASE        +0x00000C00)
#define GPIOB_BASE                      (PERIPH_BASE        +0x00000C40)
#define GPIOC_BASE                      (PERIPH_BASE        +0x00000C80)
#define GPIOD_BASE                      (PERIPH_BASE        +0x00000CC0)
#define GPIOE_BASE                      (PERIPH_BASE        +0x00000D00)
#define GPIOF_BASE                      (PERIPH_BASE        +0x00000D40)
#define GPIOG_BASE                      (PERIPH_BASE        +0x00000D80)
#define GPIO_COMMON_BASE                (PERIPH_BASE        +0x00000E40)
#define FLS_BASE                        (PERIPH_BASE        +0x00001000)
#define QSPI_BASE                       (PERIPH_BASE        +0x00001400)
#define PMU_BASE                        (PERIPH_BASE        +0x00002000)
#define CMU_BASE                        (PERIPH_BASE        +0x00002400)
#define RMU_BASE                        (PERIPH_BASE        +0x00002800)
#define SFU_BASE                        (PERIPH_BASE        +0x00002C00)
#define RAMBIST_BASE                    (PERIPH_BASE        +0x0000FC00)
#define SPI0_BASE                       (PERIPH_BASE        +0x00019000)
#define SPI1_BASE                       (PERIPH_BASE        +0x00018C00)
#define SPI2_BASE                       (PERIPH_BASE        +0x00010800)
#define IWDT_BASE                       (PERIPH_BASE        +0x00011400)
#define WWDT_BASE                       (PERIPH_BASE        +0x00011800)
#define UART0_BASE                      (PERIPH_BASE        +0x00011C00)
#define UART1_BASE                      (PERIPH_BASE        +0x00012000)
#define UART2_BASE                      (PERIPH_BASE        +0x00017400)
#define UART3_BASE                      (PERIPH_BASE        +0x00017800)
#define UART4_BASE                      (PERIPH_BASE        +0x00016800)
#define UART5_BASE                      (PERIPH_BASE        +0x00016C00)
#define UART_COMMON_BASE                (PERIPH_BASE        +0x00017C00)
#define I2C0_BASE                       (PERIPH_BASE        +0x00012400)
#define I2C1_BASE                       (PERIPH_BASE        +0x00012800)
#define SVD_BASE                        (PERIPH_BASE        +0x00012C00)
#define LPTIM16_BASE                    (PERIPH_BASE        +0x00013400)
#define GPTIM0_BASE                     (PERIPH_BASE        +0x00013800)
#define GPTIM1_BASE                     (PERIPH_BASE        +0x00013C00)
#define GPTIM5_BASE                     (PERIPH_BASE        +0x00014000)
#define TAU0_BASE                       (PERIPH_BASE        +0x00014400)
#define CLM0_BASE                       (PERIPH_BASE        +0x00015000)
#define CLM1_BASE                       (PERIPH_BASE        +0x00015400)
#define CLM2_BASE                       (PERIPH_BASE        +0x00015800)
#define CRC_BASE                        (PERIPH_BASE        +0x00018000)
#define ATIM0_BASE                      (PERIPH_BASE        +0x00018400)
#define ATIM1_BASE                      (PERIPH_BASE        +0x0001B000) 
#define ATIM2_BASE                      (PERIPH_BASE        +0x00018800) 
#define DAC0_BASE                       (PERIPH_BASE        +0x0001A800)
#define DAC1_BASE                       (PERIPH_BASE        +0x0001A400)
#define ADC0_BASE                       (PERIPH_BASE        +0x0001AC00)
#define ADC1_BASE                       (PERIPH_BASE        +0x0001B800)
#define ADC2_BASE                       (PERIPH_BASE        +0x0001BC00)
#define ADC3_BASE                       (PERIPH_BASE        +0x0001E000)
#define BSTIM16_BASE                    (PERIPH_BASE        +0x0001B400)
#define PGL_BASE                        (PERIPH_BASE        +0x0001DC00)
#define CORDIC_BASE                     (PERIPH_BASE        +0x0001EC00)
#define COMP0_BASE                      (PERIPH_BASE        +0x0001F000)
#define COMP1_BASE                      (PERIPH_BASE        +0x0001F004)
#define COMP2_BASE                      (PERIPH_BASE        +0x0001F008)
#define COMP3_BASE                      (PERIPH_BASE        +0x0001F00C)
#define COMP_COMMON_BASE                (PERIPH_BASE        +0x0001F010)
#define CANFD0_BASE                     (PERIPH_BASE        +0x00028000)
#define CANFD0_RX_FIFO_BASE             (CANFD0_BASE        +0x00000520)
#define CANFD0_COMMON_FIFO_BASE         (CANFD0_BASE        +0x000005B8)
#define CANFD0_TX_MESSAGE_BASE          (CANFD0_BASE        +0x00000604)
#define CANFD0_TX_HISTORY_LIST_BASE     (CANFD0_BASE        +0x00000740)
#define CANFD0_RX_MESSAGE0_BASE         (CANFD0_BASE        +0x00000920)
#define CANFD0_RX_MESSAGE1_BASE         (CANFD0_BASE        +0x00000D20)
#define CANFD0_RX_MESSAGE2_BASE         (CANFD0_BASE        +0x00001120)
#define CANFD0_RX_MESSAGE3_BASE         (CANFD0_BASE        +0x00001520)

/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */
#define ADC0                             ((ADC_Type          *) ADC0_BASE        )
#define ADC1                             ((ADC_Type          *) ADC1_BASE        )
#define ADC2                             ((ADC_Type          *) ADC2_BASE        )
#define ADC3                             ((ADC_Type          *) ADC3_BASE        )
#define ATIM0                            ((ATIM_Type         *) ATIM0_BASE       )
#define ATIM1                            ((ATIM_Type         *) ATIM1_BASE       )
#define ATIM2                            ((ATIM_Type         *) ATIM2_BASE       )
#define BSTIM16                          ((BSTIM16_Type      *) BSTIM16_BASE     )
#define CANFD0                           ((CANFD_COMMON_Type *) CANFD0_BASE      )
#define CANFD0_RX_FIFO                   ((CANFD_RX_FIFO_Type         *) CANFD0_RX_FIFO_BASE         )
#define CANFD0_COMMON_FIFO               ((CANFD_COMMON_FIFO_Type     *) CANFD0_COMMON_FIFO_BASE     )
#define CANFD0_TX_MESSAGE                ((CANFD_TX_MESSAGE_Type      *) CANFD0_TX_MESSAGE_BASE      )
#define CANFD0_TX_HISTORY_LIST           ((CANFD_TX_HISTORY_LIST_Type *) CANFD0_TX_HISTORY_LIST_BASE )
#define CANFD0_RX_MESSAGE0               ((CANFD_RX_MESSAGE_Type      *) CANFD0_RX_MESSAGE0_BASE     )
#define CANFD0_RX_MESSAGE1               ((CANFD_RX_MESSAGE_Type      *) CANFD0_RX_MESSAGE1_BASE     )
#define CANFD0_RX_MESSAGE2               ((CANFD_RX_MESSAGE_Type      *) CANFD0_RX_MESSAGE2_BASE     )
#define CANFD0_RX_MESSAGE3               ((CANFD_RX_MESSAGE_Type      *) CANFD0_RX_MESSAGE3_BASE     )
#define CLM0                             ((CLM_Type          *) CLM0_BASE        )
#define CLM1                             ((CLM_Type          *) CLM1_BASE        )
#define CLM2                             ((CLM_Type          *) CLM2_BASE        )
#define CMU                              ((CMU_Type          *) CMU_BASE         )
#define COMP0                            ((COMP_Type         *) COMP0_BASE       )
#define COMP1                            ((COMP_Type         *) COMP1_BASE       )
#define COMP2                            ((COMP_Type         *) COMP2_BASE       )
#define COMP3                            ((COMP_Type         *) COMP3_BASE       )
#define COMP                             ((COMP_COMMON_Type  *) COMP_COMMON_BASE )
#define CORDIC                           ((CORDIC_Type       *) CORDIC_BASE      )
#define CRC                              ((CRC_Type          *) CRC_BASE         )
#define DAC0                             ((DAC_Type          *) DAC0_BASE        )
#define DAC1                             ((DAC_Type          *) DAC1_BASE        )
#define DMA                              ((DMA_Type          *) DMA_BASE         )
#define FLASH                            ((FLASH_Type        *) FLS_BASE         )
#define GPIOA                            ((GPIO_Type         *) GPIOA_BASE       )
#define GPIOB                            ((GPIO_Type         *) GPIOB_BASE       )
#define GPIOC                            ((GPIO_Type         *) GPIOC_BASE       )
#define GPIOD                            ((GPIO_Type         *) GPIOD_BASE       )
#define GPIOE                            ((GPIO_Type         *) GPIOE_BASE       )
#define GPIOF                            ((GPIO_Type         *) GPIOF_BASE       )
#define GPIOG                            ((GPIO_Type         *) GPIOG_BASE       )
#define GPIO                             ((GPIO_COMMON_Type  *) GPIO_COMMON_BASE )
#define GPTIM0                           ((GPTIM_Type        *) GPTIM0_BASE      )
#define GPTIM1                           ((GPTIM_Type        *) GPTIM1_BASE      )
#define GPTIM5                           ((GPTIM_Type        *) GPTIM5_BASE      )
#define I2C0                             ((I2C_Type          *) I2C0_BASE        )
#define I2C1                             ((I2C_Type          *) I2C1_BASE        )
#define IWDT                             ((IWDT_Type         *) IWDT_BASE        )
#define LPTIM16                          ((LPTIM16_Type      *) LPTIM16_BASE     )
#define PGL                              ((PGL_Type          *) PGL_BASE         )
#define PMU                              ((PMU_Type          *) PMU_BASE         )
#define QSPI                             ((QSPI_Type         *) QSPI_BASE        )	
#define RAMBIST                          ((RAMBIST_Type      *) RAMBIST_BASE     )
#define RMU                              ((RMU_Type          *) RMU_BASE         )
#define SPI0                             ((SPI_Type          *) SPI0_BASE        )
#define SPI1                             ((SPI_Type          *) SPI1_BASE        )
#define SPI2                             ((SPI_Type          *) SPI2_BASE        )
#define TAU0                             ((TAU0_Type         *) TAU0_BASE      )
#define SVD                              ((SVD_Type          *) SVD_BASE         )
#define UART                             ((UART_COMMON_Type  *) UART_COMMON_BASE )
#define UART0                            ((UART_Type         *) UART0_BASE       )
#define UART1                            ((UART_Type         *) UART1_BASE       )
#define UART2                            ((UART_Type         *) UART2_BASE       )
#define UART3                            ((UART_Type         *) UART3_BASE       )
#define UART4                            ((UART_Type         *) UART4_BASE       )
#define UART5                            ((UART_Type         *) UART5_BASE       )
#define SCU                              ((SCU_Type          *) SCU_BASE         )
#define SFU                              ((SFU_Type          *) SFU_BASE         )
#define WWDT                             ((WWDT_Type         *) WWDT_BASE        )


/* ================================================================================ */
/* ================             Peripheral include                 ================ */
/* ================================================================================ */

/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group FM33LD5XX */
/** @} */ /* End of group Keil */

#ifdef __cplusplus
}
#endif

#endif  /* FM33LD5XX_H */
