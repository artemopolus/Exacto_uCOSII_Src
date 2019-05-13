/*******************************************************************************************************\
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2006; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
\*******************************************************************************************************/

/*******************************************************************************************************\
*
*                                      APPLICATION CONFIGURATION
*
*                                     ST Microelectronics STM32
*                                              with the
*                                   STM3210B-EVAL Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.10
\*******************************************************************************************************/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__

/*******************************************************************************************************\
*                                       MODULE ENABLE / DISABLE
\*******************************************************************************************************/

#define  APP_OS_PROBE_EN                         DEF_DISABLED
#define  APP_PROBE_COM_EN                        DEF_DISABLED


/*******************************************************************************************************\
*                                            TASK PRIORITIES
\*******************************************************************************************************/
#define  APP_TASK_START_PRIO             3
#define  APP_TASK_MEAS1_PRIO             5      
#define  APP_TASK_MEAS2_PRIO             6      
#define  APP_TASK_MEAS3_PRIO             7  

#define		APP_LSM303_MEAS_PRIO					7
#define		APP_BMP280_MEAS_PRIO					3
#define		APP_ISM330_MEAS_PRIO					5

#define  APP_TASK_SEND_PRIO             11
#define  APP_TASK_SEND1_PRIO            11
#define  APP_TASK_UART_SEND_PRIO            13
#define APP_TASK_STM32_PRIO             15
#define APP_TASK_BUFFER_PRIO            14                 


/*******************************************************************************************************\
*                                            TASK STACK SIZES
*                             Size of the task stacks (# of OS_STK entries)
\*******************************************************************************************************/
#define  APP_TASK_START_STK_SIZE        64
#define  APP_TASK_STK_SIZE              64


/*******************************************************************************************************\
*                                                  LIB
\*******************************************************************************************************/
#define  uC_CFG_OPTIMIZE_ASM_EN                 DEF_ENABLED
#define  LIB_STR_CFG_FP_EN                      DEF_DISABLED

/*******************************************************************************************************\
*                                                 PROBE
\*******************************************************************************************************/
#define  OS_PROBE_TASK                                     0    /* Task will be created for uC/Probe OS Plug-In.        */
#define  OS_PROBE_TMR_32_BITS                              0    /* uC/Probe OS Plugin timer is a 16-bit timer.          */
#define  OS_PROBE_TIMER_SEL                                2    /* Select timer 2.                                      */
#define  OS_PROBE_HOOKS_EN                                 1    /* Hooks to update OS_TCB profiling members included.   */
#define  OS_PROBE_USE_FP                                   1


#endif
