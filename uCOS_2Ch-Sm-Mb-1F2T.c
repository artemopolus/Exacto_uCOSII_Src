
/******************************************************************************************************\
*  Проект-прототип (пример) программы, написанной под ОС РВ uC/OS-II v.2.86
* Пример написан на базе демонстрационной программы разработчиков uC/OS  для архитектуры Cortex-M3
*
*  Filename      : uCOS_2Ch-Sm-Mb-1F2T.c
* Как можно на базе одной функции создать несколько Задач 
* 
*
\******************************************************************************************************/

#include "includes.h"
#include <stm32f10x.h>

#define   IZM   400

  //  На базе этих массивов будут (при работе системной функции OSTaskCreate(...)) созданы стеки Задач 
OS_STK         App_TaskStartStk[64];

OS_STK         App_TaskMeas1Stk[APP_TASK_STK_SIZE];    //  Стек измерителя1
OS_STK         App_TaskSend1Stk[APP_TASK_STK_SIZE];    //  Стек измерителя2

OS_STK         App_TaskMeas2Stk[APP_TASK_STK_SIZE];   //  Стек передатчика1
OS_STK         App_TaskSend2Stk[APP_TASK_STK_SIZE];   //  Стек передатчика2

extern      CPU_FNCT_VOID BSP_IntVectTbl[];           //  для отладки

//  Объявления функций, на базе которых в программе будут созданы Задачи
//  Определения этих функций смотрите далее в этом файле 

static  void  App_TaskStart    (void        *p_arg);

static  void  App_TaskMeas1    (void        *p_arg);
//static  void  App_TaskMeas2    (void        *p_arg);  Теперь не нужна!!

static  void  App_TaskSend1    (void        *p_arg);
static  void  App_TaskSend2    (void        *p_arg);

u8 err;               //  Для кода ошибки
unsigned char cTicks;    //  Служебная для наблюдения "тиков"
u8 ucSend=0, ucSend1=0, ucSend2=0;  //  Длит.передачи в UART

u8    cBuf1[20];        //  Буфер для строки результата1
u8    cBuf2[20];        //  Буфер для строки результата2

OS_EVENT *pUartS, *pMb1, *pMb2;			//  Указатель на ECB (блок управления событием)
u8 err = 0;		    // переменная для кода ошибки 


//  Объявления вспомогательных функций (Функции находятся в файле Addit_Funcs.c)
s8* Dec_Convert(s8* buf, s32 value);
u16 Write1_Poll(s8* ptr);
void Periph_Init(void);
u16 FreeStkSpace(OS_STK * x);
u32 uiRez1, uiRez2;

//  Структура для передачи параметров в Задачи
typedef struct 
{                       //                        (-1-)
  OS_EVENT* pMbox;      // 
  u16       TDiscr;
  u16       NSer;
  u8        ADCh;
  u8        SampleT;
} MeasTaskParamSet;

MeasTaskParamSet ParData1;        //                (-2-)
MeasTaskParamSet ParData2;

/*******************************************************************************************************\
*                                                main()
\*******************************************************************************************************/
int  main (void)
{
//    CPU_INT08U  os_err;
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
//u8 err;               //  Для кода ошибки
  BSP_Init();   //  Фактически - только настройка RCC - 72 (или 24) МГц от ФАПЧ 
  OS_CPU_SysTickInit();  // Запуск системного таймера и прерывания от его переполнения
        //  После этого SysTick работает, и многозадачность функционирует полностью

  Periph_Init();
//  Здесь самое подходящее место для создания пользовательских Задач, 
// объектов синхронизации-коммуникации и т.п.

	if(0==(pUartS = OSSemCreate(1)))			//  Создадим семафор, проверим возвр. знач.
	{ __asm("nop"); }

	if(0==(pMb1 = OSMboxCreate(0)))			//  Создадим семафор, проверим возвр. знач.
	{ __asm("nop"); }
	if(0==(pMb2 = OSMboxCreate(0)))			//  Создадим семафор, проверим возвр. знач.
	{ __asm("nop"); }
  
  //  Заполнение двух (глобальных) структур для параметров двух Задач-Измерителей,
  // которые создаются на базе одной и той же функции
  //  Если структуры с параметрами объявлены глобально, то параметры легко перестраивать
  // при выполнении программы из любой другой Задачи.  (??? А можно ли объявить их локально? 
  ParData1.pMbox    = pMb1;                   //                  (-3-)
  ParData1.NSer     = 19;         //  измерений в серии
  ParData1.ADCh     = ADC_Channel_1;
  ParData1.SampleT  = ADC_SampleTime_13Cycles5;
  ParData1.TDiscr   = 11;         //  тиков

  ParData2.pMbox    = pMb2;
  ParData2.NSer     = 23;         //  измерений в серии
  ParData2.ADCh     = ADC_Channel_2;
  ParData2.SampleT  = ADC_SampleTime_28Cycles5;
  ParData2.TDiscr   = 13;         //  тиков
  
  

  OSTaskCreate((void (*)(void *)) App_TaskMeas1,        //  Создадим Задачу для измерения1
               (void          * ) &ParData1,      //!! Указатель на блок параметров1
               (OS_STK        * ) &App_TaskMeas1Stk[APP_TASK_STK_SIZE - 1],  //  Стек Задачи1
               (INT8U           ) APP_TASK_MEAS1_PRIO   // Уровень приоритета для этой Задачи =5
              );

//OSTaskCreate((void (*)(void *)) App_TaskMeas2,        //  Создадим Задачу для измерения2
  OSTaskCreate((void (*)(void *)) App_TaskMeas1,  //!! Создадим Задачу для измер2 на базе App_TaskMeas1
               (void          * ) &ParData2,      //!! Указатель на блок параметров2
               (OS_STK        * ) &App_TaskMeas2Stk[APP_TASK_STK_SIZE - 1],  //  Стек Задачи2
               (INT8U           ) APP_TASK_MEAS2_PRIO   // Уровень приоритета для этой Задачи =6
              );

  OSTaskCreate((void (*)(void *)) App_TaskSend1,          //  Создадим Задачу для опроса клавиатуры
               (void          * ) 0,
               (OS_STK        * ) &App_TaskSend1Stk[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_SEND1_PRIO   // Уровень приоритета для этой Задачи =9
              );

  if((   //  Проверяйте возвращаемое значение, когда есть сомнение
    OSTaskCreate((void (*)(void *)) App_TaskSend2,          //  Создадим Задачу-передатчик
               (void          * ) 0,
               (OS_STK        * ) &App_TaskSend2Stk[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_SEND2_PRIO   // Уровень приоритета для этой Задачи =11
              )) !=OS_ERR_NONE)
  {__asm("nop");}     //  Смотрим значение кода ошибки в файле ucos_ii.h, со строки 245

/*   Фрагмент для контроля переполнения стека
  u16 volatile usStkFree=APP_TASK_START_STK_SIZE;
  usStkFree=FreeStkSpace(App_TaskStartStk);   //  Проверка переполнения стека Задачи
  {__asm("nop");} 
  // */

    //  Можно удалить Задачу, если она больше не нужна, освободится TCB и стек
  OSTaskDel(OS_PRIO_SELF);   //  
}   


/*--------------------------------------------------------------------------------*\
*                         App_TaskMeas1()
* 
\*--------------------------------------------------------------------------------*/
//  Параметры регистрации для сигнала1 (переменные глобальные) - теперь не используются
// Параметры передаются в каждую Задачу-измеритель через индивидуальную структуру
//u16   TDiscr1 =11;    //  Период дискретизации сигнала1 (10 - 1 мс)
//u16   NSer1   =19;    //  Длина серии для сигнала1
//u16   usCtr1  =0;     //  счетчик отсчетов в серии1

static void App_TaskMeas1(void *p_arg) {
//  u16 volatile usStkFree=128;   //  Для контроля стека
//  usStkFree=FreeStkSpace(App_TaskStartStk);
MeasTaskParamSet* pPar; //  Указатель на блок параметров              (-5-)
OS_CPU_SR cpu_sr=0;   //  Для сохранения регистра состояний   
u32   uiSum  =0;
u16   usCtr  =0;      //  счетчик отсчетов в серии
u32		uiRez;	
                      //  Адрес структуры с параметрами - в лок.указатель
  pPar=(MeasTaskParamSet*)p_arg;            //                        (-6-)
  while (DEF_TRUE) {    //  В Задаче должен быть бесконечный цикл

    OS_ENTER_CRITICAL();      //  Начало критической секции
    //  Выбор канала, задание времени выборки
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_13Cycles5); // (3) и (4)
    ADC_RegularChannelConfig(ADC1, pPar->ADCh, 1, pPar->SampleT); //  (-7-)
    //  Запустить АЦП
    ADC_Cmd(ADC1, ENABLE);    //  Запустить АЦП
    while (RESET== ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))   // Готовность результата
    {  //  Ожидание результата АЦ преобразования
      __asm("nop");
    }
    uiSum+=ADC_GetConversionValue(ADC1);
    OS_EXIT_CRITICAL();       //  Конец критической секции
//    if((usCtr++)==NSer1) 
    if((usCtr++)==pPar->NSer)                        // (-7-)   (-8-)
    { usCtr=0;
//    uiRez1=uiRez=(u16)uiSum*3300/4096/NSer1; 
      uiRez=(u16)uiSum*3300/4096/pPar->NSer;                // (-7-)
      uiSum=0;
      //  Выполнить передачу
//    if (OSMboxPost(pMb1, &uiRez)==OS_MBOX_FULL)  //  Передается адрес  
      if (OSMboxPost(pPar->pMbox, &uiRez)==OS_MBOX_FULL)    // (-7-)
			{ 
        __asm("nop");		//	Если передатчик не успел прочитать
			}
    }
//  OSTimeDly(TDiscr1);  //  Запустить тайм-аут на период дискретизации           
    OSTimeDly(pPar->TDiscr);  //                               (-7-)  
  }
}

/*--------------------------------------------------------------------------------*\
*                           App_TaskMeas2()
* Задача выполняет измерения по второму каналу с периодом дискретизации TDiscr2
* Уникальный уровень приоритета для этой Задачи = 6 
\*--------------------------------------------------------------------------------*/
       // === Теперь код Задачи App_TaskMeas2() не используется ==== //

/*  Параметры регистрации для сигнала2  (переменные глобальные)
u16   TDiscr2 =13;            //  Период дискретизации сигнала2 (10->1 мс)
u16   NSer2   =23;            //  Длина серии для сигнала2
u16   usCtr2  =0;             //  счетчик отсчетов в серии2
u32   uiRez2  =0;             //  Для результата

static void App_TaskMeas2(void *p_arg) {
u32 uiSum=0;
u16 usRez;

  while (DEF_TRUE) {    //  В Задаче должен быть бесконечный цикл
      //  Включить канал АЦП
    //  Выбор канала 2, задание времени выборки
    //  Времена выборки: 1, 7, 13, 28, 41, 55, 71, 239
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_13Cycles5);

      //  Запустить АЦП
    ADC_Cmd(ADC1, ENABLE);    //  Запустить АЦП
    while((ADC1->SR&2)==0) {  //  Ожидание результата АЦ преобразования
      __asm("nop");
    }
    uiSum+=ADC1->DR;
    if((usCtr2++)==NSer2)
    { usCtr2=0;
      uiRez2=usRez=uiSum*3300/4096/NSer2;
			uiSum=0;
      if(OSMboxPost(pMb2, (u32*)usRez)==OS_MBOX_FULL)  //  Передается результат
			{__asm("nop");}		//	Если передатчик не успел прочитать
    } 
      //  Запустить тайм-аут на период дискретизации
    OSTimeDly(TDiscr2);      
  }
} // */

/*---------------------------------------------------------------------------*\
*                          App_TaskSend1()
* Description : Задача выполняет передачу отсчетов первого канала
* Уникальный уровень приоритета для этой Задачи = 9 
\*---------------------------------------------------------------------------*/

static void App_TaskSend1(void *p_arg) {
  while (DEF_TRUE) {        //  В Задаче должен быть бесконечный цикл
//  OSTaskSuspend(OS_PRIO_SELF);    	//  Без Mbox было так
      //  Задача App_TaskSend1() будет продолжена, когда будут готовы данные для передачи      
			//	Теперь ожидание отсчета внутри OSMboxPend    
    uiRez1=*((u32*)OSMboxPend(pMb1,0,&err));  //  Принимается адрес
    Dec_Convert((s8*)cBuf1, uiRez1); 
    ucSend1=100;
    ucSend=ucSend1+ucSend2;           //  Для наблюдения времени передачи
    OSSemPend(pUartS,0,&err);         // 
		Write1_Poll((s8*)"Sig_1 = ");
    Write1_Poll((s8*)&cBuf1[6]);
    Write1_Poll((s8*)" mV\n");
    OSSemPost(pUartS);
    ucSend1=0;
      ucSend=ucSend1+ucSend2;         //  Для наблюдения времени передачи
    __asm("nop");
  }   //  В Задаче должен быть бесконечный цикл, здесь его конец
}     //  Здесь заканчивается тело функции, на базе которой создается Задача

/*--------------------------------------------------------------------------------*\
*                                            App_TaskSend2()
* Description : Задача выполняет передачу отсчетов второго канала
* Уникальный уровень приоритета для этой Задачи = 11 
\*--------------------------------------------------------------------------------*/

static void App_TaskSend2(void *p_arg) {
  while (DEF_TRUE) {                //  В Задаче должен быть бесконечный цикл
      //  Теперь ожидание отсчета внутри OSMboxPend
    uiRez2=*((u32*)OSMboxPend(pMb2,0,&err));  //  Принимается адрес
    Dec_Convert((s8*)cBuf2, uiRez2); //  Принимается адрес
    ucSend2=60;
      ucSend=ucSend1+ucSend2;       //  Для наблюдения времени передачи
    OSSemPend(pUartS,0,&err);
    Write1_Poll((s8*)"       Sig_2 = ");
    Write1_Poll((s8*)&cBuf2[6]);
    Write1_Poll((s8*)" mV\n");
		OSSemPost(pUartS);
      ucSend2=0;
      ucSend=ucSend1+ucSend2;       //  Для наблюдения времени передачи
    __asm("nop");
  }   //  В Задаче должен быть бесконечный цикл, здесь его конец
}     //  Здесь заканчивается тело функции, на базе которой создается Задача


