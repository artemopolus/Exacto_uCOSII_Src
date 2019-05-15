
/******************************************************************************************************\
*  Проект-прототип (пример) программы, написанной под ОС РВ uC/OS-II v.2.86
* Пример написан на базе демонстрационной программы разработчиков uC/OS  для архитектуры Cortex-M3
*
*  Filename      : uCOS_3Ch-MQ-Irq.c
* Как можно на базе одной функции создать несколько Задач 
* 
*
\******************************************************************************************************/

#include "includes.h"
#include <stm32f10x.h>  

#include "exacto_commander.h"
#include "exacto_buffer.h"
#include "lsm303ah_reglib.h"
#include "bmp280_reglib.h"
#include "ism330_reglib.h"

#include "exacto_struct.h"
#include "exacto_defs.h"

#define ECHO_ALL
//#define PING_MODE
#define UARTMAXWORDLEN   80






//  На базе этих массивов будут созданы стеки Задач 
OS_STK         App_TaskStartStk[APP_TASK_STK_SIZE];

//OS_STK         App_TaskMeas1Stk[APP_TASK_STK_SIZE];   //  Стек измерителя1
//OS_STK         App_TaskMeas2Stk[APP_TASK_STK_SIZE];   //  Стек измерителя2
//OS_STK         App_TaskMeas3Stk[APP_TASK_STK_SIZE];   //  Стек измерителя3

//OS_STK         App_TaskSendStk[APP_TASK_STK_SIZE];    //  Стек передатчика 
extern      CPU_FNCT_VOID BSP_IntVectTbl[];           //  для отладки

//  Объявления функций, на базе которых в программе будут созданы Задачи
//  Определения этих функций смотрите далее в этом файле 
static  void  App_TaskStart    (void *p_arg);

ExactoLBIdata buffer;

const uint8_t CntExactoStm32States =   10;
uint8_t * ExactoStm32States;
void SetInitExactoStm32States(void)
{
    ExactoStm32States = (uint8_t *)calloc(CntExactoStm32States, sizeof(uint8_t));
}



//TASKS
ExactoSensorSet lsm303;
OS_STK  Stk_App_lsm303[APP_TASK_STK_SIZE];
static void App_lsm303(void * p_arg);
OS_FLAG_GRP * pFlgSensors;

ExactoSensorSet bmp280;
OS_STK  Stk_App_bmp280[APP_TASK_STK_SIZE];
static void App_bmp280(void * p_arg);

ExactoSensorSet ism330;
OS_STK	Stk_App_ism330[APP_TASK_STK_SIZE];
static void App_ism330(void * p_arg);


OS_STK  Stk_App_Messager[APP_TASK_STK_SIZE];
static void App_Messager(void *p_arg);

OS_STK  Stk_App_UartRxBuffParser[APP_TASK_STK_SIZE];
static void App_UartRxBuffParser(void *p_arg);

OS_STK  Stk_App_stm32[APP_TASK_STK_SIZE];
void App_stm32(void * p_arg);

OS_STK  Stk_App_buffer[APP_TASK_STK_SIZE];
void        App_buffer(void * p_arg);

unsigned char cTicks;   //  Служебная для наблюдения "тиков"
u8 ucSend=0, ucSend1=0, ucSend2=0;  //  Длит.передачи в UART

s8    cBuf[16];         //  Буфер для строки результата
s8*   pTx;              //  Указатель для передачи в обработчик адреса строки
//EVENTS
OS_EVENT * pMailStm32;
CmdToStm32 bMailStm32;

#define   QL    10
OS_EVENT *pQue;				  //  Указатель на Очередь
void *MsgQue[QL];			  //  Массив для Очереди
OS_Q_DATA qData; 			  //  Структура для информации об Очереди
OS_EVENT* pUart;        //  Указатель для Семафора (UART)
u16 cQn;                //  Элементов в Очереди

#define MAXCOUNTUARTRECEIVER 10
OS_EVENT 	* pEvUartRxBuff;
void			* pArUartRxBuff[MAXCOUNTUARTRECEIVER];
OS_Q_DATA 	infUartRxBuff;
//OS_EVENT 	* flgUartRxBuff;
uint16_t		cntUartRxBuff;
#define MAXUARTRXBUFFER	EXACTO_BUFLEN_16

#define CNTSENSORBUFFER   30
OS_EVENT * pEvSensorBuff;
void *pSensorBuff[CNTSENSORBUFFER];
OS_Q_DATA infSensorBuff;
uint16_t cntSensorBuff;



//  Объявления вспомогательных функций (Функции находятся в файле Addit_Funcs.c)
s8* Dec_Convert(s8* buf, s32 value);
u16 Write1_Poll(s8* ptr);
void Periph_Init(void);
u16 FreeStkSpace(OS_STK * x);
void SendStr(s8* ptr);          //  Функция запуска передачи

void UART_init(void);
void USART2_IRQHandler(void);

uint8_t Exacto_init_lsm303ah(void);
uint8_t Exacto_slftst_lsm303ah(void);
uint8_t Exacto_setfrq_lsm303ah(uint8_t mode);

uint8_t Exacto_init_bmp280(void);
uint8_t Exacto_setfrq_bmp280(uint8_t mode);

uint8_t Exacto_init_ism330(void);
uint8_t Exacto_setfrq_ism330(uint8_t mode);

uint8_t Exacto_saferead_lsm303(uint8_t RegAdr);
uint8_t Exacto_saferead_bmp280(uint8_t RegAdr);
uint8_t Exacto_saferead_ism330(uint8_t RegAdr);
void Exacto_safewrite_lsm303(uint8_t RegAdr, uint8_t RegVal);
void Exacto_safewrite_bmp280(uint8_t RegAdr, uint8_t RegVal);
void Exacto_safewrite_ism330(uint8_t RegAdr, uint8_t RegVal);

uint8_t Exacto_sensor_read  (uint8_t TrgModule, uint8_t RegAdr, uint8_t * RegValue );
uint8_t Exacto_sensor_write    (uint8_t TrgModule, uint8_t RegAdr, uint8_t RegValue );

uint8_t ExactoLBIdata2arrayUint8(ExactoLBIdata * src, uint8_t * dst);





void setInitExactoSensorSet(ExactoSensorSet * dst, char * name, uint8_t flg, uint8_t tdiscr)
{
    dst->Whoami = 0x00;
    strcpy(dst->Name,name);
    dst->flgSens = flg;
    dst->TDiscr = tdiscr;
}
void FlagPendError_Callback(uint8_t src, OS_FLAGS flag)
{
    if(flag != OS_ERR_NONE)
    {
        switch(src)
        {
            case FLG_LSM303:
                SendStr((int8_t*)"FLGERR:lsm303:");
                __NOP();
                break; 
            case FLG_BMP280:
                SendStr((int8_t*)"FLGERR:bmp280:");
                __NOP();
                break; 
            case FLG_ISM330:
                SendStr((int8_t*)"FLGERR:ism330:");
                __NOP();
                break; 
        }
    }
    switch(flag)
    {
        case OS_ERR_NONE:
            __NOP();
            break;            
        case OS_ERR_PEND_ISR :
            __NOP();
            SendStr((int8_t*)"FLGERR:OS_ERR_PEND_ISR\n");
            break; 
        case OS_ERR_FLAG_INVALID_PGRP:
            __NOP();
            SendStr((int8_t*)"FLGERR:OS_ERR_FLAG_INVALID_PGRP\n");
            break; 
        case OS_ERR_EVENT_TYPE:
            __NOP();
            SendStr((int8_t*)"FLGERR:OS_ERR_EVENT_TYPE\n");
            break; 
        case OS_ERR_TIMEOUT:
            __NOP();
            SendStr((int8_t*)"FLGERR:OS_ERR_TIMEOUT\n");
            break; 
        case OS_ERR_PEND_ABORT: 
            __NOP();
            SendStr((int8_t*)"FLGERR:OS_ERR_PEND_ABORT\n");
            break;             
        case OS_ERR_FLAG_WAIT_TYPE:
            __NOP();
            SendStr((int8_t*)"FLGERR:OS_ERR_FLAG_WAIT_TYPE\n");
            break; 
    }
}

/*******************************************************************************************************\
*                                                main()
\*******************************************************************************************************/
int  main (void)
{
  BSP_IntDisAll();       // Disable all ints until we are ready to accept them.

  OSInit();              // Initialize "uC/OS-II, The Real-Time Kernel"
  OSTaskCreate((void (*)(void *)) App_TaskStart,     // Create the start task.                               */
               (void          * ) 0,
               (OS_STK        * )&App_TaskStartStk[APP_TASK_START_STK_SIZE - 1],
               (INT8U           ) APP_TASK_START_PRIO
              );
  OSStart();              // Start multitasking (i.e. give control to uC/OS-II).
        //  Возврата из функции OSStart() не произойдет никогда.
  return (0);   //  Сюда управление не попадет никогда
}


/*******************************************************************************************************\
*        App_TaskStart()
*
* Description : Стартовая (начальная) Задача. Она должна получить управление первой. Для этого ей
*   следует присваивать наивысший приоритет (в случае, если в main() вы создадите несколько Задач
*   В стартовой Задаче (не раньше!!!) следует запустить системный таймер и разрешить от него прерывание
* 
*   Стартовая Задача - самое подходящее место для настроек периферийных подсистем 
   и для создания Задач прикладной программы
\*******************************************************************************************************/

static  void  App_TaskStart (void *p_arg)
{
//u8 err;                 //  Для кода ошибки

  BSP_Init();   //  Фактически - только настройка RCC - 72 (или 24) МГц от ФАПЧ 
  //BSP_IntVectSet(BSP_INT_ID_USART1, USART1_IRQHandler);
  BSP_IntVectSet(BSP_INT_ID_USART2, USART2_IRQHandler);
                //  Устанка псевдовектора, имя обработчика задал прикладной программист
  //Periph_Init();    //  В этой функции делаются начальные настройки  
	UART_init();
    setInitExactoSensorSet(&lsm303,"lsm303",FLG_LSM303,OS_TIME_10mS);
    setInitExactoSensorSet(&bmp280,"bmp280",FLG_BMP280,OS_TIME_10mS);
    setInitExactoSensorSet(&ism330,"ism330",FLG_ISM330,OS_TIME_10mS);
	
	SetInitExactoStm32States();



  OS_CPU_SysTickInit();
  
    uint8_t errorParser;
		pEvUartRxBuff = OSQCreate(&pArUartRxBuff[0],MAXCOUNTUARTRECEIVER);
    pEvSensorBuff = OSQCreate(&pSensorBuff[0],CNTSENSORBUFFER);
    pUart = OSSemCreate(1);             //  Создадим семафор для UART
    pFlgSensors = OSFlagCreate(0x00,&errorParser);
    pMailStm32 = OSMboxCreate((void*)0);
  
  
  OSTaskCreate((void (*)(void *)) App_Messager,  
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_Messager[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_SEND_PRIO   
              );
  OSTaskCreate((void (*)(void *)) App_stm32,  
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_stm32[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_STM32_PRIO   
              );
  OSTaskCreate((void (*)(void *)) App_buffer,  
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_buffer[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_BUFFER_PRIO   
              );
  OSTaskCreate((void (*)(void *)) App_UartRxBuffParser,        //  Создадим Задачу для измерения1
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_UartRxBuffParser[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_TASK_UART_SEND_PRIO   // Уровень приоритета для этой Задачи =5
              );
	
  OSTaskCreate((void (*)(void *)) App_lsm303,        //  Создадим Задачу для измерения1
               (void          * ) &lsm303,
               (OS_STK        * ) &Stk_App_lsm303[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_LSM303_MEAS_PRIO   // Уровень приоритета для этой Задачи =5
              );
  OSTaskCreate((void (*)(void *)) App_bmp280,        //  Создадим Задачу для измерения1
               (void          * ) &bmp280,
               (OS_STK        * ) &Stk_App_bmp280[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_BMP280_MEAS_PRIO   // Уровень приоритета для этой Задачи =5
              );
  OSTaskCreate((void (*)(void *)) App_ism330,        //  Создадим Задачу для измерения1
               (void          * ) &ism330,
               (OS_STK        * ) &Stk_App_ism330[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_ISM330_MEAS_PRIO   // Уровень приоритета для этой Задачи =5
              );
  
  OSTaskDel(OS_PRIO_SELF);   //  
}   

/*--------------------------------------------------------------------------------*\
*                              App_TaskMeas()
* Функция, на базе которой создаются Измерители
*     
\*--------------------------------------------------------------------------------*/

static void App_lsm303(void * p_arg)
{
    ExactoSensorSet * Parameters = (ExactoSensorSet*)p_arg;
    OS_CPU_SR cpu_sr = 0;
    uint8_t error;
    OS_FLAGS flValue;
	
    //SensorParameters splsm330;
	Parameters->Whoami = Exacto_init_lsm303ah();
    strcpy(Parameters->Name,"lsm330");
    SensorData Val_lsm303;
    Val_lsm303.pSensor = FLG_LSM303;
    if(!Exacto_setfrq_lsm303ah(0))
    {
        __NOP();
        SendStr((int8_t*)"ERRSET:lsm303 set freq\n");
    }
    uint8_t ready = 0;
    while(DEF_TRUE)
    {
        flValue = OSFlagPend(pFlgSensors,FLG_LSM303,OS_FLAG_WAIT_SET_ALL,0,&error);
        if(!flValue)    SendStr((int8_t*)"RTNFLGPendERR:lsm303\n");
        FlagPendError_Callback(FLG_LSM303, error);
        OS_ENTER_CRITICAL()
		ready =	GetXLallDataUint8_lsm303ah(Val_lsm303.s1);
        OS_EXIT_CRITICAL()
        if(ready)
        {
            if (OSQPost(pEvSensorBuff, ((void*)(&Val_lsm303)))==OS_Q_FULL)
            {
                __NOP();
                SendStr((int8_t*)"OS_Q_FULL:lsm303\n");
            }
            OSQQuery(pEvSensorBuff, &infSensorBuff);
            cntSensorBuff = infSensorBuff.OSNMsgs;
            if(CNTSENSORBUFFER==cntSensorBuff)
            {
                __NOP();
                SendStr((int8_t*)"OS_Q_CNTWRN:lsm303\n");
            }
            OSTimeDly(Parameters->TDiscr);
        }
        else OSTimeDly(OS_TIME_1mS);
    }
}
static void App_bmp280(void * p_arg)
{
    ExactoSensorSet * Parameters = (ExactoSensorSet*)p_arg;
    OS_CPU_SR cpu_sr = 0;
    uint8_t error;
    OS_FLAGS flValue;
    SensorData Val_bmp280;
    Parameters->Whoami = Exacto_init_bmp280();
    Val_bmp280.pSensor = FLG_BMP280;
    if(!Exacto_setfrq_bmp280(0))
    {
        __NOP();
        SendStr((int8_t*)"ERRSET:bmp280 set freq\n");
    }
    uint8_t ready = 0;
		while(DEF_TRUE)
		{
			flValue = OSFlagPend(pFlgSensors,FLG_BMP280,OS_FLAG_WAIT_SET_ALL,0,&error);
            if(!flValue)    SendStr((int8_t*)"RTNFLGPendERR:bmp280\n");
        FlagPendError_Callback(FLG_BMP280, error);
        OS_ENTER_CRITICAL()
            ready = GetPresTempValuesUint8_bmp280(Val_bmp280.s1);
        OS_EXIT_CRITICAL()
            if(ready)
            {
				if (OSQPost(pEvSensorBuff, ((void*)(&Val_bmp280)))==OS_Q_FULL)
                {
                    __NOP();
                    SendStr((int8_t*)"OS_Q_FULL:bmp280\n");
                }
                OSQQuery(pEvSensorBuff, &infSensorBuff);
                cntSensorBuff = infSensorBuff.OSNMsgs;
                if(CNTSENSORBUFFER==cntSensorBuff)
                {
                    __NOP();
                    SendStr((int8_t*)"OS_Q_CNTWRN:bmp280\n");
                }
                OSTimeDly(Parameters->TDiscr);
            }
            else OSTimeDly(OS_TIME_1mS);
		}
}
static void App_ism330(void * p_arg)
{
		ExactoSensorSet * Parameters = (ExactoSensorSet*)p_arg;
    OS_CPU_SR cpu_sr = 0;
    uint8_t error;
    uint8_t ready = 0;
    OS_FLAGS flValue;
    SensorData  Val_ism330;
    Parameters->Whoami = Exacto_init_ism330();
    Val_ism330.pSensor = FLG_ISM330;
    if(!Exacto_setfrq_ism330(0))
    {
        __NOP();
        SendStr((int8_t*)"ERRSET:ism330 set freq\n");
    }
		while(DEF_TRUE)
		{
			flValue = OSFlagPend(pFlgSensors,FLG_ISM330,OS_FLAG_WAIT_SET_ALL,0,&error);
            if(!flValue)    SendStr((int8_t*)"RTNFLGPendERR:ism330\n");
            FlagPendError_Callback(FLG_ISM330, error);
        OS_ENTER_CRITICAL()
            ready = GetGXLData_ism330(Val_ism330.s1);
        OS_EXIT_CRITICAL()
            if(ready)
            {
				if (OSQPost(pEvSensorBuff, ((void*)(&Val_ism330)))==OS_Q_FULL)
                {
                    __NOP();
                    SendStr((int8_t*)"OS_Q_FULL:ism330\n");
                }
                OSQQuery(pEvSensorBuff, &infSensorBuff);
                cntSensorBuff = infSensorBuff.OSNMsgs;
                if(CNTSENSORBUFFER==cntSensorBuff)
                {
                    __NOP();
                    SendStr((int8_t*)"OS_Q_CNTWRN:ism330\n");
                }
                OSTimeDly(Parameters->TDiscr);
            }
        else OSTimeDly(OS_TIME_1mS);
		}
}


static void App_Messager(void * p_arg)
{

    INT16U BaseDelay = OS_TICKS_PER_SEC;
    uint32_t CounterDelay = 0;
    OS_FLAGS flags;
    INT8U err;
    uint8_t ExactoLBIdata2send[EXACTOLBIDATASIZE*3 + 3];
    uint8_t Cnt_ExactoLBIdata2send = 0;
    while(DEF_TRUE)
    {
        CounterDelay++;
        flags = OSFlagQuery(pFlgSensors, &err);
        switch(err)
        {
            case OS_ERR_NONE:
                break;
            case OS_ERR_FLAG_INVALID_PGRP:
                SendStr((int8_t*)"APP_MSG:OS_ERR_FLAG_INVALID_PGRP\n");
                break;
            case OS_ERR_EVENT_TYPE:
                SendStr((int8_t*)"APP_MSG:OS_ERR_EVENT_TYPE\n");
                break;
        }
        if(flags)
        {
            Cnt_ExactoLBIdata2send = ExactoLBIdata2arrayUint8(& buffer, ExactoLBIdata2send);
            if(Cnt_ExactoLBIdata2send > 3)
            {
                ExactoLBIdata2send[Cnt_ExactoLBIdata2send] = '\0';
                SendStr((int8_t*)"h");
                SendStr((s8*)ExactoLBIdata2send);
                SendStr((int8_t*)"\n");
            }
            else
            {
                SendStr((int8_t*)"No data in buffer\n");
            }
        }
        else
        {
            Dec_Convert((s8*)cBuf, CounterDelay); 
            SendStr((int8_t*)"Timer=");
            SendStr((int8_t*)&cBuf[6]);
            SendStr((int8_t*)"s\n");
            OSTimeDly(BaseDelay);
        }
    }
}
static void App_UartRxBuffParser(void *p_arg)
{
    uint8_t error;
    uint8_t UartRxValue;
//		FIFO(64) UARTfifo;
	
    ExactoBufferUint8Type UartRxBuffer;
    setini_exbu8(&UartRxBuffer,MAXUARTRXBUFFER);
    uint8_t pData[MAXUARTRXBUFFER + 1];
    uint8_t TrgSens = 0, RegAdr = 0, RegVal = 0, SensVal = 0;
    
    uint8_t Merr;
    

    while(DEF_TRUE)
    {
        UartRxValue=(int8_t)OSQPend(pEvUartRxBuff,0,&error);
			switch(error)
			{
				case OS_ERR_TIMEOUT:
                    SendStr((int8_t*)"UART_RX:OS_ERR_TIMEOUT\n");
					break;
				case OS_ERR_PEND_ABORT:
                    SendStr((int8_t*)"UART_RX:OS_ERR_PEND_ABORT\n");
					break;
                case OS_ERR_EVENT_TYPE:
                    SendStr((int8_t*)"UART_RX:OS_ERR_EVENT_TYPE\n");
					break;
                case OS_ERR_PEVENT_NULL:
                    SendStr((int8_t*)"UART_RX:OS_ERR_PEVENT_NULL\n");
					break;
                case OS_ERR_PEND_ISR:
                    SendStr((int8_t*)"UART_RX:OS_ERR_PEND_ISR\n");
					break;
                case OS_ERR_PEND_LOCKED:
                    SendStr((int8_t*)"UART_RX:OS_ERR_PEND_LOCKED\n");
					break;
			}
        pshfrc_exbu8(&UartRxBuffer,UartRxValue);
        if(grball_exbu8(&UartRxBuffer, pData))
        {
        pData[getlen_exbu8(&UartRxBuffer)+ 1] = '\0';

    #ifdef ECHO_ALL
        SendStr((int8_t*)"Get Data:");
        SendStr((int8_t*)&pData[0]);
    #endif
        }
        int pointer2cmd = checkCmdType((char*)pData, getlen_exbu8(&UartRxBuffer));
        if(pointer2cmd != -1)
        {
            uint8_t flgchck = getDataCmd((char*)(pData + pointer2cmd), & TrgSens, & RegAdr, & RegVal );
            clrsvr_exbu8(&UartRxBuffer, 8);
            #ifdef ECHO_ALL
            SendStr((int8_t*)"Clr Data:");
            SendStr((int8_t*)&pData[0]);
            #endif
            switch( flgchck)
            {
                case 1:
                    if(TrgSens == 0)
                    {
                        SendStr((int8_t*)"\nExAdr=");
                        Dec_Convert((s8*)cBuf, RegAdr);
                        SendStr((s8*)&cBuf[6]);
                        SendStr((int8_t*)"value=");
                        Dec_Convert((s8*)cBuf, ExactoStm32States[RegAdr]);
                        SendStr((s8*)&cBuf[6]);
                    }
                    else
                    {
                        SendStr((int8_t*)"\nRead value:");
                        if(Exacto_sensor_read(TrgSens,RegAdr,&SensVal))
                            {
                                    Dec_Convert((s8*)cBuf, SensVal); 
                                    SendStr((s8*)&cBuf[6]);
                                    setclr_exbu8(&UartRxBuffer,(pointer2cmd + 4));
                            }
                            else
                            {
                                    SendStr((int8_t*)"can't read");
                            }
                    }
                    break;
                case 2:
                    if(TrgSens == 0)
                    {
                        bMailStm32.actiontype = 1;
                        bMailStm32.adr = RegAdr;
                        bMailStm32.val = RegVal;
                        Merr = OSMboxPost(pMailStm32, (void *)&bMailStm32);
                        if(Merr == OS_ERR_NONE)
                        {
                            __NOP();
                        }
                        else
                        {
                            __NOP();
                        }
                    }
                    else
					{
                        SendStr((int8_t*)"\nWrite value:");
                        if(Exacto_sensor_write(TrgSens,RegAdr,RegVal))
                        {
                            Dec_Convert((s8*)cBuf, RegVal); 
                            SendStr((s8*)&cBuf[6]);
                            setclr_exbu8(&UartRxBuffer,(pointer2cmd + 4));
                        }
                        else
                        {
                            SendStr((int8_t*)"can't write");
                        }
					}
                    break;
                case 0:
                    break;
            }
            
        }
        else
        {
            pointer2cmd = checkSrvType((char*)pData, getlen_exbu8(&UartRxBuffer));
            if(pointer2cmd != -1)
            {
                SendStr((int8_t*)"\nHelp str\n");
                setclr_exbu8(&UartRxBuffer,(pointer2cmd + 4));
            }
        }
        
        SendStr((int8_t*)"\n");
    }
}


// ------------ Функция запуска передачи ----------------------------
void SendStr(s8* ptr) { //  Неплохо бы проверять параметр на !=0
u8 err;        //  Для кода завершения
  OSSemPend(pUart,0,&err);  //  Свободен ли UART?  
  pTx=ptr;                  //  Да, передача адреса строки символов 
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //  разрешить запрос от Тх
}


// --------------- Обработчик прерывания от USART1 -------------------
void USART2_IRQHandler(void) {
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        if(*pTx!=0)
            USART_SendData(USART2, *pTx++);
        else 
        {   
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
            pTx = 0;
            OSSemPost(pUart);
        }
    }
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t RxByte = USART_ReceiveData(USART2);
        if (OSQPost(pEvUartRxBuff, ((void*)(RxByte)))==OS_Q_FULL)
        {
            __NOP();
        }
        OSQQuery(pEvUartRxBuff, &infUartRxBuff);
        cntUartRxBuff = infUartRxBuff.OSNMsgs;
        if(MAXCOUNTUARTRECEIVER==cntUartRxBuff)
        {
            __NOP();
        }
    }
}
uint8_t Exacto_sensor_read  (uint8_t TrgModule, uint8_t RegAdr, uint8_t * RegValue )
{
    switch(TrgModule)
    {
        case 0:
            break;
        case FLG_LSM303:
            if (lsm303.Whoami)
            {
                * RegValue = Exacto_saferead_lsm303(RegAdr);
                return 1;
            }
            else
                return 0;
        case FLG_BMP280:
            if(bmp280.Whoami)
            {
                * RegValue = Exacto_saferead_bmp280(RegAdr);
                return 1;
            }
            else
                return 0;
        case FLG_ISM330:
            if(ism330.Whoami)
            {
                * RegValue = Exacto_saferead_ism330(RegAdr);
                return 1;
            }
            else
                return 0;
    }
    return 0;
}
uint8_t Exacto_sensor_write    (uint8_t TrgModule, uint8_t RegAdr, uint8_t RegValue )
{
    switch(TrgModule)
    {
        case 0:
            break;
        case FLG_LSM303:
            if(lsm303.Whoami)
            {
                Exacto_safewrite_lsm303(RegAdr,RegValue);
                return 1;
            }
            else	return 0;
        case FLG_BMP280:
            if(bmp280.Whoami)
            {
                Exacto_safewrite_bmp280(RegAdr,RegValue);
                return 1;
            }
            else	return 0;
        case FLG_ISM330:
            if(ism330.Whoami)
            {
                Exacto_safewrite_ism330(RegAdr,RegValue);
                return 1;
            }
            else	return 0;
    }
		return 0;
}
uint8_t Exacto_saferead_lsm303(uint8_t RegAdr)
{
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    uint8_t result = read_lsm303ah(RegAdr);
    OS_EXIT_CRITICAL()
    return result;
}
uint8_t Exacto_saferead_bmp280(uint8_t RegAdr)
{
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    uint8_t result = read_bmp280(RegAdr);
    OS_EXIT_CRITICAL()
    return result;
}
uint8_t Exacto_saferead_ism330(uint8_t RegAdr)
{
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    uint8_t result = read_ism330(RegAdr);
    OS_EXIT_CRITICAL()
    return result;
}
void Exacto_safewrite_lsm303(uint8_t RegAdr, uint8_t RegVal)
{
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    write_lsm303ah(RegAdr, RegVal);
    OS_EXIT_CRITICAL()
}
void Exacto_safewrite_bmp280(uint8_t RegAdr, uint8_t RegVal)
{
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    write_bmp280(RegAdr, RegVal);
    OS_EXIT_CRITICAL()
}
void Exacto_safewrite_ism330(uint8_t RegAdr, uint8_t RegVal)
{
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    write_ism330(RegAdr, RegVal);
    OS_EXIT_CRITICAL()
}
