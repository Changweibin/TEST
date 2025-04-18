/******************************************************************************
 * @file     startup_ARMv8MML.S
 * @brief    CMSIS-Core Device Startup File for Cortex-ARMv8MML Device
 * @version  V2.0.0
 * @date     18. February 2025
 ******************************************************************************/

                .syntax  unified
                .arch    armv8-m.main

                #define __INITIAL_SP     Image$$ARM_LIB_STACK$$ZI$$Limit
                #define __STACK_LIMIT    Image$$ARM_LIB_STACK$$ZI$$Base
                #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
                #define __STACK_SEAL     Image$$STACKSEAL$$ZI$$Base
                #endif

                .section RESET
                .align   2
                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    __INITIAL_SP                       /*     Initial Stack Pointer */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVCall_Handler                     /*  -5 SVC Handler */
                .long    DebugMonitor_Handler               /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

                /* Interrupts */
                .long    WWDT_IRQHandler                    /* 0   WDT */
                .long    SVD_IRQHandler                     /* 1   SVD */
                .long    0                                  /* 2   0 */
                .long    IWDT_IRQHandler                    /* 3   IWDT */
                .long    FLASH_IRQHandler                   /* 4   FLASH */
                .long    CMU_IRQHandler                     /* 5   CMU */
                .long    0                                  /* 6   Reserved */
                .long    MAP_ERR_IRQHandler                 /* 7   MAP_ERR */
                .long    ADC0_IRQHandler                    /* 8   ADC0 */
                .long    ADC1_IRQHandler                    /* 9   ADC1 */
                .long    DAC0_IRQHandler                    /* 10  DAC0 */
                .long    SPI0_IRQHandler                    /* 11  SPI0 */
                .long    SPI1_IRQHandler                    /* 12  SPI1 */
                .long    SPI2_IRQHandler                    /* 13  SPI2 */
                .long    0                                  /* 14  Reserved */
                .long    0                                  /* 15  Reserved */
                .long    DAC1_IRQHandler                    /* 16  DAC1 */
                .long    QSPI_IRQHandler                    /* 17  QSPI */
                .long    UART0_IRQHandler                   /* 18  UART0 */
                .long    UART1_IRQHandler                   /* 19  UART1 */
                .long    UART2_IRQHandler                   /* 20  UART2 */
                .long    UART3_IRQHandler                   /* 21  UART3 */
                .long    UART4_IRQHandler                   /* 22  UART4 */
                .long    UART5_IRQHandler                   /* 23  UART5 */
                .long    ADC2_IRQHandler                    /* 24  ADC2 */
                .long    ADC3_IRQHandler                    /* 25  ADC3 */
                .long    EXTIA_IRQHandler                   /* 26  GPIOA */
                .long    EXTIB_IRQHandler                   /* 27  GPIOB */
                .long    EXTIC_IRQHandler                   /* 28  GPIOC */
                .long    EXTID_IRQHandler                   /* 29  GPIOD */
                .long    EXTIE_IRQHandler                   /* 30  GPIOE */
                .long    EXTIF_IRQHandler                   /* 31  GPIOF */
                .long    EXTIG_IRQHandler                   /* 32  GPIOG */
                .long    0                                  /* 33  Reserved */
                .long    0                                  /* 34  Reserved */
                .long    0                                  /* 35  Reserved */
                .long    0                                  /* 36  Reserved */
                .long    PMU_IRQHandler                     /* 37  PMU */
                .long    I2C0_IRQHandler                    /* 38  I2C0 */
                .long    I2C1_IRQHandler                    /* 39  I2C1 */
                .long    0                                  /* 40  Reserved */
                .long    0                                  /* 41  Reserved  */
                .long    0                                  /* 42  Reserved */
                .long    CLM0_IRQHandler                    /* 43  CLM0   */
                .long    CLM1_IRQHandler                    /* 44  CLM1   */
                .long    CLM2_IRQHandler                    /* 45  CLM2   */
                .long    0                                  /* 46  Reserved */
                .long    0                                  /* 47  Reserved */
                .long    FPU_IRQHandler                     /* 48  FPU */
                .long    0                                  /* 49  Reserved */
                .long    DMA_IRQHandler                     /* 50  DMA */
                .long    WKUPx_IRQHandler                   /* 51  WKUPX */
                .long    PGL_IRQHandler                     /* 52  PGL */
                .long    COMPx_IRQHandler                   /* 53  COMPX */
                .long    0                                  /* 54  Reserved */
                .long    0                                  /* 55  Reserved */
                .long    CANFD0_IRQHandler                  /* 56  CANFD0 */
                .long    0                                  /* 57  Reserved */
                .long    0                                  /* 58  Reserved */
                .long    0                                  /* 59  Reserved */
                .long    LPTIM16_IRQHandler                 /* 60  LPTIM16 */
                .long    0                                  /* 61  Reserved */
                .long    BSTIM16_IRQHandler                 /* 62  BSTIM16 */
                .long    0                                  /* 63  Reserved */
                .long    ATIM0_IRQHandler                   /* 64  ATIM0 */
                .long    ATIM1_IRQHandler                   /* 65  ATIM1 */
                .long    ATIM2_IRQHandler                   /* 66  ATIM2 */
                .long    0                                  /* 67  Reserved */
                .long    GPTIM0_IRQHandler                  /* 68  GPTIM0 */
                .long    GPTIM1_IRQHandler                  /* 69  GPTIM1 */
                .long    0                                  /* 70  Reserved */
                .long    GPTIM5_IRQHandler                  /* 71  GPTIM5 */
                .long    0                                  /* 72  Reserved */
                .long    TAU0_IRQHandler                    /* 73  TAU0 */
                .long    0                                  /* 74  EPWM2 */
                .long    0                                  /* 75  EPWM2_TZ */
                .long    CORDIC_IRQHandler                  /* 76  CORDIC */
                .space   (403 * 4)                          /* Interrupts 77 .. 480 are left out */
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
Reset_Handler:
                ldr      r0, =__INITIAL_SP
                msr      psp, r0

                ldr      r0, =__STACK_LIMIT
                msr      msplim, r0
                msr      psplim, r0

                #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
                ldr      r0, =__STACK_SEAL
                ldr      r1, =0xFEF5EDA5U
                strd     r1,r1,[r0,#0]
                #endif

                bl       SystemInit

                bl       __main

                .fnend
                .size    Reset_Handler, . - Reset_Handler


/* The default macro is not used for HardFault_Handler
 * because this results in a poor debug illusion.
 */
                .thumb_func
                .type    HardFault_Handler, %function
                .weak    HardFault_Handler
                .fnstart
HardFault_Handler:
                b        .
                .fnend
                .size    HardFault_Handler, . - HardFault_Handler

                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
				Set_Default_Handler  Hardfault_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SVCall_Handler
                Set_Default_Handler  DebugMonitor_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                Set_Default_Handler  WWDT_IRQHandler
                Set_Default_Handler  SVD_IRQHandler
                Set_Default_Handler  IWDT_IRQHandler
                Set_Default_Handler  FLASH_IRQHandler
                Set_Default_Handler  CMU_IRQHandler
                Set_Default_Handler  MAP_ERR_IRQHandler
                Set_Default_Handler  ADC0_IRQHandler
				Set_Default_Handler  ADC1_IRQHandler
                Set_Default_Handler  DAC0_IRQHandler
				Set_Default_Handler  SPI0_IRQHandler
                Set_Default_Handler  SPI1_IRQHandler
                Set_Default_Handler  SPI2_IRQHandler
                Set_Default_Handler  DAC1_IRQHandler
				Set_Default_Handler  QSPI_IRQHandler
                Set_Default_Handler  UART0_IRQHandler
                Set_Default_Handler  UART1_IRQHandler
                Set_Default_Handler  UART2_IRQHandler
                Set_Default_Handler  UART3_IRQHandler
                Set_Default_Handler  UART4_IRQHandler
                Set_Default_Handler  UART5_IRQHandler
                Set_Default_Handler  ADC2_IRQHandler
				Set_Default_Handler  ADC3_IRQHandler
                Set_Default_Handler  EXTIA_IRQHandler
                Set_Default_Handler  EXTIB_IRQHandler
                Set_Default_Handler  EXTIC_IRQHandler
                Set_Default_Handler  EXTID_IRQHandler
                Set_Default_Handler  EXTIE_IRQHandler
                Set_Default_Handler  EXTIF_IRQHandler
				Set_Default_Handler  EXTIG_IRQHandler
				Set_Default_Handler  PMU_IRQHandler
                Set_Default_Handler  I2C0_IRQHandler
                Set_Default_Handler  I2C1_IRQHandler
                Set_Default_Handler  CLM0_IRQHandler
				Set_Default_Handler  CLM1_IRQHandler
				Set_Default_Handler  CLM2_IRQHandler
                Set_Default_Handler  FPU_IRQHandler
				Set_Default_Handler  DMA_IRQHandler
				Set_Default_Handler  WKUPx_IRQHandler
				Set_Default_Handler  PGL_IRQHandler
				Set_Default_Handler  COMPx_IRQHandler
				Set_Default_Handler  CANFD0_IRQHandler
				Set_Default_Handler  LPTIM16_IRQHandler
				Set_Default_Handler  BSTIM16_IRQHandler
				Set_Default_Handler  ATIM0_IRQHandler
                Set_Default_Handler  ATIM1_IRQHandler
				Set_Default_Handler  ATIM2_IRQHandler
				Set_Default_Handler  GPTIM0_IRQHandler
                Set_Default_Handler  GPTIM1_IRQHandler
				Set_Default_Handler  GPTIM5_IRQHandler
				Set_Default_Handler  TAU0_IRQHandler
				Set_Default_Handler  CORDIC_IRQHandler
                .end
