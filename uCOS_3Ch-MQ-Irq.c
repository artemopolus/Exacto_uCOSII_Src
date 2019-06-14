
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




#define UARTMAXWORDLEN          80
#define   QL                    10
#define MAXCOUNTUARTRECEIVER    10
#define MAXUARTRXBUFFER	        EXACTO_BUFLEN_16
#define CNTSENSORBUFFER         30

ExactoBufferUint8Type ExBufLSM303;
ExactoBufferUint8Type ExBufBMP280;
ExactoBufferUint8Type ExBufISM330;

volatile ExactoSensorSet lsm303;
volatile ExactoSensorSet bmp280;
volatile ExactoSensorSet ism330;

#ifdef ENABLE_TIME_MEAS

uint32_t ClkCnt;

#endif

INT16U BaseDelay = OS_TICKS_PER_SEC;

volatile uint32_t CounterAppMessager = 0;

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

ExactoLBIdata ExactoBuffer;


const uint8_t CntExactoStm32States =   10;
uint8_t * ExactoStm32States;



//EVENTS
OS_EVENT * pMailStm32;
OS_EVENT *pQue;				  //  Указатель на Очередь
OS_EVENT* pUart;        //  Указатель для Семафора (UART)
OS_EVENT 	* pEvUartRxBuff;
OS_EVENT * pEvSensorBuff;
//5
OS_FLAG_GRP * pFlgSensors;
#ifdef ENABLE_SAFE_CP2BUFFER
OS_EVENT * pBuffRdy;
#endif
OS_EVENT * pI2C;

//10

//TASKS
static void App_lsm303(void * p_arg);
static void App_bmp280(void * p_arg);
static void App_ism330(void * p_arg);
static void App_Messager(void *p_arg);
static void App_UartRxBuffParser(void *p_arg);
//5
void App_stm32(void * p_arg);
void        App_buffer(void * p_arg);


OS_STK  Stk_App_lsm303[APP_TASK_STK_SIZE];
OS_STK  Stk_App_bmp280[APP_TASK_STK_SIZE];
OS_STK	Stk_App_ism330[APP_TASK_STK_SIZE];
OS_STK  Stk_App_Messager[APP_TASK_STK_SIZE];
OS_STK  Stk_App_UartRxBuffParser[APP_TASK_STK_SIZE];
//5
OS_STK  Stk_App_stm32[APP_TASK_STK_SIZE];
OS_STK  Stk_App_buffer[APP_TASK_STK_SIZE];


unsigned char cTicks;   //  Служебная для наблюдения "тиков"
u8 ucSend=0, ucSend1=0, ucSend2=0;  //  Длит.передачи в UART

s8    cBuf[16];         //  Буфер для строки результата
s8*   pTx;              //  Указатель для передачи в обработчик адреса строки

uint8_t * pTxFixLength;
uint8_t pTxFixLengthCnt = 0;
uint8_t pTxFixLength_i = 0;
uint8_t ModeTx = 0;

uint8_t SilenceMode = 1;

CmdToStm32 bMailStm32;




void *MsgQue[QL];			  //  Массив для Очереди
OS_Q_DATA qData; 			  //  Структура для информации об Очереди

u16 cQn;                //  Элементов в Очереди



void			* pArUartRxBuff[MAXCOUNTUARTRECEIVER];
OS_Q_DATA 	infUartRxBuff;
//OS_EVENT 	* flgUartRxBuff;
uint16_t		cntUartRxBuff;




void *pSensorBuff[CNTSENSORBUFFER];
OS_Q_DATA infSensorBuff;
uint16_t cntSensorBuff;



//  Объявления вспомогательных функций (Функции находятся в файле Addit_Funcs.c)
s8* Dec_Convert(s8* buf, s32 value);
u16 Write1_Poll(s8* ptr);
void Periph_Init(void);
u16 FreeStkSpace(OS_STK * x);
void SendStr(s8* ptr);          //  Функция запуска передачи

void SendStrFixLen(uint8_t * ptr, uint8_t cnt);

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

uint32_t ExactoLBIdata2arrayUint8(ExactoLBIdata * src, uint8_t * dst, const uint32_t dstlen);

void ExactoStm32StatesChanged_Callback(uint8_t RegAdr, uint8_t RegVal, uint8_t * perr);

#ifdef ENABLE_I2C_SLAVE
void Exacto_init_i2c_slave(void);
void DMA1_Channel7_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void SetInitTransmitData(void);
uint8_t CheckTransmitBuffer(void);
void ReleaseTransmitBuffer(void);
void SetData2transmit(uint8_t * pData, const uint32_t datalen);
#endif

uint8_t FakeEx_lsm303(uint8_t * src1, uint8_t *src2) {for(uint8_t i = 1; i < 7;i++) {src1[i] = i;src2[i] = i;}return 1;}
uint8_t FakeEx_bmp280(uint8_t * src1 ){for(uint8_t i = 1; i < 7;i++) {src1[i] = i;}return 1;}
uint8_t FakeEx_ism330(uint8_t * src1 ){for(uint8_t i = 1; i < 15;i++) {src1[i] = i;}return 1;}


void SetInitExactoStm32States(void)
{
    ExactoStm32States = (uint8_t *)calloc(CntExactoStm32States, sizeof(uint8_t));
}
void setInitExactoSensorSet( volatile ExactoSensorSet * dst, char * name, uint8_t flg, uint8_t tdiscr)
{
    dst->Whoami = 0x00;
		dst->initFreq = 0x00;
    //strcpy(dst->Name,name);
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
    #ifdef ENABLE_I2C_SLAVE
    BSP_IntVectSet(BSP_INT_ID_I2C1_EV, I2C1_EV_IRQHandler);
    BSP_IntVectSet(BSP_INT_ID_I2C1_ER, I2C1_ER_IRQHandler);
    BSP_IntVectSet(BSP_INT_ID_DMA1_CH7, DMA1_Channel7_IRQHandler);
    BSP_IntVectSet(BSP_INT_ID_DMA1_CH6, DMA1_Channel6_IRQHandler);
    #endif
    
                //  Устанка псевдовектора, имя обработчика задал прикладной программист
  //Periph_Init();    //  В этой функции делаются начальные настройки  
	UART_init();
    #ifdef ENABLE_I2C_SLAVE
    Exacto_init_i2c_slave();
		SetInitTransmitData();
    #endif
    setInitExactoSensorSet(&lsm303,"lsm303",FLG_LSM303,OS_TIME_10mS);
    setInitExactoSensorSet(&bmp280,"bmp280",FLG_BMP280,OS_TIME_10mS);
    setInitExactoSensorSet(&ism330,"ism330",FLG_ISM330,OS_TIME_10mS);
	
	SetInitExactoStm32States();



    OS_CPU_SysTickInit();
  
    uint8_t errorParser;
	pEvUartRxBuff = OSQCreate(&pArUartRxBuff[0],MAXCOUNTUARTRECEIVER);
    if(pEvUartRxBuff == (OS_EVENT *)0 )
    {
        __NOP();
    }
    pEvSensorBuff = OSQCreate(&pSensorBuff[0],CNTSENSORBUFFER);
    if(pEvSensorBuff == (OS_EVENT *)0 )
    {
        __NOP();
    }
    pUart = OSSemCreate(1);             //  Создадим семафор для UART
    if(pUart == (OS_EVENT *)0 )
    {
        __NOP();
    }
		pI2C = OSSemCreate(1);
		if(pI2C == (OS_EVENT*)0)
		{
				__NOP();
		}
    #ifdef ENABLE_SAFE_CP2BUFFER
    pBuffRdy = OSMutexCreate(19,&errorParser);
    switch(errorParser)
    {
        case OS_ERR_NONE:
            __NOP();
            break;
        case OS_ERR_CREATE_ISR:
            __NOP();
            break;
        case OS_ERR_PRIO_EXIST:
            __NOP();
            break;				
        case OS_ERR_PEVENT_NULL:  
            __NOP();
            break;
        case OS_ERR_PRIO_INVALID:
            __NOP();
            break;
    }
    #endif
    pFlgSensors = OSFlagCreate(0x00,&errorParser);
    if(errorParser != OS_ERR_NONE)
    {
        __NOP();
    }
    pMailStm32 = OSMboxCreate((void*)0);
    if(pMailStm32 == (OS_EVENT *)0 )
    {
        __NOP();
    }

    
     uint8_t crflg_lsm303 = 0, crflg_bmp280 = 0, crflg_ism330 = 0; 
    if(Exacto_init_lsm303ah())
    { 
        lsm303.Whoami = 0x01;        
                OSTimeDly(OS_TIME_1mS);
        if(!Exacto_setfrq_lsm303ah(0))
        {
            __NOP();					
        }
        else
        {
            lsm303.initFreq = 0x01;
            crflg_lsm303 = 1;
        }
    }
    if(Exacto_init_bmp280())
    {
        bmp280.Whoami = 1;
        OSTimeDly(OS_TIME_10mS);
        if(!Exacto_setfrq_bmp280(0))
        {
            __NOP();
        }
        else
        {
            OSTimeDly(OS_TIME_10mS);
            bmp280.initFreq = 0x01;
            crflg_bmp280 = 1;
        }
    }
    if( Exacto_init_ism330())
    {
        ism330.Whoami = 1;
        if(!Exacto_setfrq_ism330(0))
        {
            __NOP();

        }
        else
        {
            ism330.initFreq = 0x01;
            crflg_ism330 = 1;
        }
    }
    
    setini_exbu8(&ExBufLSM303, EXACTO_BUFLEN_256);
    setini_exbu8(&ExBufBMP280, EXACTO_BUFLEN_256);
    setini_exbu8(&ExBufISM330, EXACTO_BUFLEN_256);
  
  switch(OSTaskCreate((void (*)(void *)) App_Messager,  
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_Messager[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_SEND_PRIO   
              ))
    {
        case OS_ERR_NONE:
            __NOP();
            break;
        case OS_ERR_PRIO_EXIST:
            __NOP();
            break;
        case OS_ERR_PRIO_INVALID:
            __NOP();
            break;
        case OS_ERR_TASK_CREATE_ISR:
            __NOP();
            break;
    }
  switch(OSTaskCreate((void (*)(void *)) App_stm32,  
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_stm32[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_STM32_PRIO   
              ))
    {
        case OS_ERR_NONE:
            __NOP();
            break;
        case OS_ERR_PRIO_EXIST:
            __NOP();
            break;
        case OS_ERR_PRIO_INVALID:
            __NOP();
            break;
        case OS_ERR_TASK_CREATE_ISR:
            __NOP();
            break;
    }
  switch(OSTaskCreate((void (*)(void *)) App_buffer,  
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_buffer[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_BUFFER_PRIO   
              ))
    {
        case OS_ERR_NONE:
            __NOP();
            break;
        case OS_ERR_PRIO_EXIST:
            __NOP();
            break;
        case OS_ERR_PRIO_INVALID:
            __NOP();
            break;
        case OS_ERR_TASK_CREATE_ISR:
            __NOP();
            break;
    }
  switch(OSTaskCreate((void (*)(void *)) App_UartRxBuffParser,        //  Создадим Задачу для измерения1
               (void          * ) 0,
               (OS_STK        * ) &Stk_App_UartRxBuffParser[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_TASK_UART_SEND_PRIO   // Уровень приоритета для этой Задачи =5
              ))
    {
        case OS_ERR_NONE:
            __NOP();
            break;
        case OS_ERR_PRIO_EXIST:
            __NOP();
            break;
        case OS_ERR_PRIO_INVALID:
            __NOP();
            break;
        case OS_ERR_TASK_CREATE_ISR:
            __NOP();
            break;
    }      

   

    if(crflg_lsm303)
    {
      switch(OSTaskCreate((void (*)(void *)) App_lsm303,        //  Создадим Задачу для измерения1
                   (void          * ) &lsm303,
                   (OS_STK        * ) &Stk_App_lsm303[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
                   (INT8U           ) APP_LSM303_MEAS_PRIO   // Уровень приоритета для этой Задачи =5
                  ))
        {
            case OS_ERR_NONE:
                __NOP();
                break;
            case OS_ERR_PRIO_EXIST:
                __NOP();
                break;
            case OS_ERR_PRIO_INVALID:
                __NOP();
                break;
            case OS_ERR_TASK_CREATE_ISR:
                __NOP();
                break;
        }
    }
    if(crflg_bmp280)
    {
      switch(OSTaskCreate((void (*)(void *)) App_bmp280,        //  Создадим Задачу для измерения1
                   (void          * ) &bmp280,
                   (OS_STK        * ) &Stk_App_bmp280[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
                   (INT8U           ) APP_BMP280_MEAS_PRIO   // Уровень приоритета для этой Задачи =5
                  ) )
        {
            case OS_ERR_NONE:
                __NOP();
                break;
            case OS_ERR_PRIO_EXIST:
                __NOP();
                break;
            case OS_ERR_PRIO_INVALID:
                __NOP();
                break;
            case OS_ERR_TASK_CREATE_ISR:
                __NOP();
                break;
        }
    }

    if(crflg_ism330)
    {
      switch(OSTaskCreate((void (*)(void *)) App_ism330,        //  Создадим Задачу для измерения1
                   (void          * ) &ism330,
                   (OS_STK        * ) &Stk_App_ism330[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
                   (INT8U           ) APP_ISM330_MEAS_PRIO   // Уровень приоритета для этой Задачи =5
                  ))
        {
            case OS_ERR_NONE:
                __NOP();
                break;
            case OS_ERR_PRIO_EXIST:
                __NOP();
                break;
            case OS_ERR_PRIO_INVALID:
                __NOP();
                break;
            case OS_ERR_TASK_CREATE_ISR:
                __NOP();
                break;
        }
    }
  SilenceMode = 0;
  OSTaskDel(OS_PRIO_SELF);   //  
}   

/*--------------------------------------------------------------------------------*\
*                              App_TaskMeas()
* Функция, на базе которой создаются Измерители
*     
\*--------------------------------------------------------------------------------*/

static void App_lsm303(void * p_arg)
{
    OS_CPU_SR cpu_sr = 0;
    uint8_t error;
    OS_FLAGS flValue;    
	
		uint8_t flagSensDataRdy = 0x00;
		uint16_t mltCnt_XL = 1;
		uint16_t mltCnt_M = 1;
	
    SensorData Val_lsm303;
    Val_lsm303.pSensor = FLG_LSM303;
	
    while(DEF_TRUE)
    {
        flValue = OSFlagPend(pFlgSensors,FLG_LSM303,OS_FLAG_WAIT_SET_ALL,0,&error);
        if(!flValue)    SendStr((int8_t*)"RTNFLGPendERR:lsm303\n");
        FlagPendError_Callback(FLG_LSM303, error);

				flagSensDataRdy = 0x00;
				//check XL
			#ifdef ENABLE_LSM303_XL
				if(lsm303.MultSens1){
					if(lsm303.MultSens1 == mltCnt_XL){
						mltCnt_XL ++;
					}
					else{
						mltCnt_XL = 1;
						flagSensDataRdy |= 0x01;
					}
				}
			#endif
			#ifdef ENABLE_LSM303_M
				//check M
				if(lsm303.MultSens2){
					if(lsm303.MultSens2 == mltCnt_M){
						mltCnt_M ++;
					}
					else{
						mltCnt_M = 1;
						flagSensDataRdy |= 0x02;
					}
				}
			#endif
				//check available data
				if(flagSensDataRdy&0x01){
					OS_ENTER_CRITICAL()
					if(read_lsm303ah_fst(LSM303AH_STATUS_A)&0x01)
						flagSensDataRdy |= 0x04;
					OS_EXIT_CRITICAL()
				}
				if(flagSensDataRdy&0x04){
					OS_ENTER_CRITICAL()
					Val_lsm303.s1_status = multiread_lsm303ah_fst(LSM303AH_OUT_X_L_A,Val_lsm303.s1,6);
					OS_EXIT_CRITICAL()
				}
				if(flagSensDataRdy&0x02){
					OS_ENTER_CRITICAL()
					if(read_lsm303ah_fst(LSM303AH_STATUS_REG_M)&0x08)
						flagSensDataRdy |= 0x08;
					OS_EXIT_CRITICAL()
				}
				//read neccesary data
				if(flagSensDataRdy&0x08){
					OS_ENTER_CRITICAL()
					Val_lsm303.s2_status = multiread_lsm303ah_fst(LSM303AH_OUTX_L_REG_M,Val_lsm303.s2,6);
					OS_EXIT_CRITICAL()
				}
      #ifdef FAKE_LSM303
        ready = FakeEx_lsm303(Val_lsm303.s1,Val_lsm303.s2);
        Val_lsm303.s1_status = 1;
        Val_lsm303.s2_status = 1;
      #endif
        if(flagSensDataRdy)
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
        }
		OSTimeDly(lsm303.TDiscr);
    }
}
static void App_bmp280(void * p_arg)
{
    ExactoSensorSet * Parameters = (ExactoSensorSet*)p_arg;
    OS_CPU_SR cpu_sr = 0;
    uint8_t error;
    uint8_t flValue;
    SensorData Val_bmp280;
    Val_bmp280.pSensor = FLG_BMP280;

    uint8_t ready = 0;
    //SendStr((int8_t*)"TSKCRTD:bmp280\n");
    while(DEF_TRUE)
    {
        flValue = OSFlagPend(pFlgSensors,FLG_BMP280,OS_FLAG_WAIT_SET_ALL,0,&error);
        if(!flValue)    SendStr((int8_t*)"RTNFLGPendERR:bmp280\n");
        if(error != OS_ERR_NONE)	FlagPendError_Callback(FLG_BMP280, error);
			for(uint8_t i =0; i <3;i++)
			{
        OS_ENTER_CRITICAL()
				//OSTimeDly(OS_TIME_1mS);
			#ifdef FAKE_BMP280
				ready = FakeEx_bmp280(Val_bmp280.s1);
			#else
        //ready = GetPresTempValuesUint8_bmp280(Val_bmp280.s1);
				ready = getval_bmp280(BMP280_STATUS_ADDR);
				if(ready )
					getMultiVal_bmp280(BMP280_PRES_MSB_ADDR, Val_bmp280.s1, 6);
			#endif
        OS_EXIT_CRITICAL()
				if(ready) break;
			}
        if(ready)
        {
            Val_bmp280.s1_status = 1;
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
            //OSTimeDly(Parameters->TDiscr);
        }
        //else OSTimeDly(OS_TIME_1mS);
				OSTimeDly(Parameters->TDiscr);
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
    Val_ism330.pSensor = FLG_ISM330;
    //SendStr((int8_t*)"TSKCRTD:ism330\n");
    while(DEF_TRUE)
    {
//        flValue = OSFlagPend(pFlgSensors,0x04,OS_FLAG_WAIT_SET_ALL,0,&error);
//        if(!flValue)    SendStr((int8_t*)"RTNFLGPendERR:ism330\n");

		flValue = OSFlagPend(pFlgSensors,FLG_ISM330,OS_FLAG_WAIT_SET_ANY,0,&error);
		if(error == OS_ERR_NONE)
		{
        //FlagPendError_Callback(FLG_ISM330, error);
		for(uint8_t i = 0; i < 3; i++)
		{
        OS_ENTER_CRITICAL()
        //    ready = GetGXLData_ism330(Val_ism330.s1);
			#ifdef FAKE_ISM330
				ready = FakeEx_ism330(Val_ism330.sL);
			#else		
				ready = Get_T_G_XL_uint8_ism330(Val_ism330.sL);
			#endif
        OS_EXIT_CRITICAL()
			if(ready) break;
		}
        if(ready)
        {
            Val_ism330.sL_status = 1;
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
            //OSTimeDly(Parameters->TDiscr);
        }
    //else OSTimeDly(OS_TIME_1mS);
		OSTimeDly(Parameters->TDiscr);
		}
		OSTimeDly(Parameters->TDiscr);
    }
}


static void App_Messager(void * p_arg)
{
		 
    
    uint32_t CounterDelay = 0;
    OS_FLAGS flags;
    #ifdef ENABLE_SAFE_CP2BUFFER
		uint8_t errB;
		#endif
    INT8U err;
    #ifdef ENABLE_FIFO_BUFFER
		uint8_t str2send[3*EXACTO_BUFLEN_256+5] = {0}; 
    const int str2send_len = 3*EXACTO_BUFLEN_256+5;
    str2send[0] = 'h';
    str2send[5] = 0;
    str2send[6] = 0;
    str2send[7] = 0;
    uint8_t flag2send = 0;
    uint8_t tmplen = 0;
    uint32_t ptr2nextdata = 0;
    #else
    OS_CPU_SR cpu_sr = 0;
    uint8_t ExactoLBIdata2send[EXACTOLSM303SZ + EXACTOBMP280SZ + EXACTOISM330SZ + 7];
		const uint32_t Max_ExactoLBIdata2send = EXACTOLSM303SZ + EXACTOBMP280SZ + EXACTOISM330SZ + 3;
    uint8_t Cnt_ExactoLBIdata2send = 0;


    #endif
    
    
    SendStr((int8_t*)"APP_MSG:start messaging\n");
    if(lsm303.Whoami)
    {
        if(lsm303.initFreq)
            SendStr((int8_t*)"APP_MSG:lsm303 init and config\n");
        else
            SendStr((int8_t*)"APP_MSG:lsm303 init and config error\n");
    }
    else    SendStr((int8_t*)"APP_MSG:lsm303 init error and config error\n");
    if(bmp280.Whoami)
    {
        if(bmp280.initFreq)
            SendStr((int8_t*)"APP_MSG:bmp280 init and config\n");
        else
            SendStr((int8_t*)"APP_MSG:bmp280 init and config error\n");
    }
    else    SendStr((int8_t*)"APP_MSG:bmp280 init error and config error\n");
    if(ism330.Whoami)
    {
        if(ism330.initFreq)
            SendStr((int8_t*)"APP_MSG:ism330 init and config\n");
        else
            SendStr((int8_t*)"APP_MSG:ism330 init and config error\n");
    }
    else    SendStr((int8_t*)"APP_MSG:ism330 init error and config error\n");
    if(ExBufLSM303.isExist) SendStr((int8_t*)"APP_MSG:buffer for lsm303 created\n");
    else SendStr((int8_t*)"APP_MSG:buffer for lsm303 error\n");
    if(ExBufBMP280.isExist) SendStr((int8_t*)"APP_MSG:buffer for bmp280 created\n");
    else SendStr((int8_t*)"APP_MSG:buffer for bmp280 error\n");
    if(ExBufISM330.isExist) SendStr((int8_t*)"APP_MSG:buffer for ism330 created\n");
    else SendStr((int8_t*)"APP_MSG:buffer for ism330 error\n");
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
						if(CounterAppMessager > 1)
						{
							OS_ENTER_CRITICAL()
							CounterAppMessager--;
							OS_EXIT_CRITICAL()
						} else if (CounterAppMessager == 1)
						{
							//stop all
							ExactoStm32StatesChanged_Callback(0, 0, &err);
							OS_ENTER_CRITICAL()
							CounterAppMessager = 0;
							OS_EXIT_CRITICAL()
						}
            #ifdef ENABLE_FIFO_BUFFER
                tmplen = ExBufLSM303.datalen;
                if(grball_exbu8(&ExBufLSM303,&str2send[8]))
                {
                    ptr2nextdata = 8 + tmplen;
                    str2send[5] = tmplen;
                    flag2send = 1;
                }
                else
                {
                    ptr2nextdata = 8;
                    str2send[5] = 0;
                }
                tmplen = ExBufBMP280.datalen;
                if(grball_exbu8(&ExBufBMP280,&str2send[ptr2nextdata]))
                {
                    ptr2nextdata += tmplen;
                    str2send[6] = ptr2nextdata;
                    flag2send = 1;
                }
                else
                {
                    str2send[6] = 0;
                }
                tmplen = ExBufISM330.datalen;
                if(grball_exbu8(&ExBufISM330,&str2send[ptr2nextdata]))
                {
                    ptr2nextdata += tmplen;
                    str2send[7] = ptr2nextdata;
                    flag2send = 1;
                }
                else
                {
                    str2send[7] = 0;
                }
                if(flag2send)
                {
                    str2send[1] = (uint8_t)CounterDelay << 24;
                    str2send[2] = (uint8_t)CounterDelay << 16;
                    str2send[3] = (uint8_t)CounterDelay << 8;
                    str2send[4] = (uint8_t)CounterDelay;
                }
                else
                    SendStr((int8_t*)"No data in buffer\n");
            #endif
						
						ExactoLBIdata2send[0] = (uint8_t)CounterDelay << 24;
						ExactoLBIdata2send[1] = (uint8_t)CounterDelay << 16;
						ExactoLBIdata2send[2] = (uint8_t)CounterDelay << 8;
						ExactoLBIdata2send[3] = (uint8_t)CounterDelay;

						if(flags&FLG_TEST)
						{
							//Cnt_ExactoLBIdata2send = 0;
							#ifdef TEST_I2C_SENDING
							if( CheckTransmitBuffer())
							{
								SetInitTransmitData();
								ReleaseTransmitBuffer();
							}
							#endif
						}
						else{
							#ifdef ENABLE_SAFE_CP2BUFFER
							OSMutexPend(pBuffRdy,0,&errB);
							Cnt_ExactoLBIdata2send = ExactoLBIdata2arrayUint8(& ExactoBuffer, ExactoLBIdata2send, Max_ExactoLBIdata2send);
							errB = OSMutexPost(pBuffRdy);
							#endif
							#ifdef ENABLE_CRITSEC_CP2BUFFER
							OS_ENTER_CRITICAL()
							Cnt_ExactoLBIdata2send = ExactoLBIdata2arrayUint8(& ExactoBuffer, &ExactoLBIdata2send[4], Max_ExactoLBIdata2send);
							OS_EXIT_CRITICAL()
							#endif
						}
						
						
						
						if(!(flags&FLG_UART))
						{
							#ifdef USART_SEND_MSG
							if(Cnt_ExactoLBIdata2send > 3){
								#ifdef ENABLE_TIME_MEAS
									OSTimeSet(0L);
								#endif
									ExactoLBIdata2send[Cnt_ExactoLBIdata2send] = '\0';
									SendStr((s8*)"h");
									SendStrFixLen(ExactoLBIdata2send,Cnt_ExactoLBIdata2send +4);
									SendStr((int8_t*)"\n");
							}
							else{
									SendStr((int8_t*)"No data in buffer\n");
							}
							#endif
						}
						OSTimeDly(BaseDelay);
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
        UartRxValue=(int8_t)OSQPend(pEvUartRxBuff,OS_TIME_2S,&error);
				switch(error)
				{
					case OS_ERR_TIMEOUT:
						//					SendStr((int8_t*)"UART_RX:OS_ERR_TIMEOUT\n");
						__NOP();
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
				if(error == OS_ERR_NONE)
				{
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
                            SendStr((int8_t*)"MBOX_ERR:bMailStm32 READ\n");
                        }
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
                        bMailStm32.actiontype = 2;
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
                            SendStr((int8_t*)"MBOX_ERR:bMailStm32 WRITE\n");
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
}


// ------------ Функция запуска передачи ----------------------------
void SendStr(s8* ptr) { //  Неплохо бы проверять параметр на !=0
  if(SilenceMode)
      return;
  u8 err;        //  Для кода завершения
  OSSemPend(pUart,0,&err);  //  Свободен ли UART?  
  pTx=ptr;                  //  Да, передача адреса строки символов 
	ModeTx = 0;
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //  разрешить запрос от Тх
}
void SendStrFixLen(uint8_t * ptr, uint8_t cnt)
{
    if(SilenceMode)
      return;
	uint8_t err;
	OSSemPend(pUart,0,&err);
    if(err != OS_ERR_NONE)
    {
        __NOP();
    }
	pTxFixLength = ptr;
	pTxFixLengthCnt = cnt;
	pTxFixLength_i = 0;
	ModeTx = 1;
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

// --------------- Обработчик прерывания от USART1 -------------------
void USART2_IRQHandler(void) {
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        switch(ModeTx)
        {
            case 0:
                if(*pTx!=0)
                    USART_SendData(USART2, *pTx++);
                else 
                {
                    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
                    pTx = 0;
                    OSSemPost(pUart);
                }
				break;
            case 1:
                if(pTxFixLengthCnt != pTxFixLength_i)
                        USART_SendData(USART2, pTxFixLength[pTxFixLength_i++]);
                else
                {
                        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
                        pTxFixLength = 0;
									#ifdef ENABLE_TIME_MEAS
												ClkCnt = OSTimeGet();
									#endif
                        OSSemPost(pUart);
                }
                break;
		}
    }
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t RxByte = USART_ReceiveData(USART2);
        switch (OSQPost(pEvUartRxBuff, ((void*)(RxByte))))
        {
					case OS_Q_FULL:
						__NOP();
					break;
					case OS_ERR_NONE:
						__NOP();
					break;
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
