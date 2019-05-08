
/******************************************************************************************************\
*  Проект-прототип (пример) программы, написанной под ОС РВ uC/OS-II v.2.86
* Пример написан на базе демонстрационной программы разработчиков uC/OS  для архитектуры Cortex-M3
*
*  Filename      : uCOS_3Ch-MQ-1F3T.c
* Три Задачи-Измерителя на базе одной функции 
* Используется ОчередьСообщений
*
\******************************************************************************************************/

#include "includes.h"
#include <stm32f10x.h>

//#define   IZM   400
#define   QL    10

//  На базе этих массивов будут созданы стеки Задач 
OS_STK         App_TaskStartStk[APP_TASK_STK_SIZE];

OS_STK         App_TaskMeas1Stk[APP_TASK_STK_SIZE];   //  Стек измерителя1
OS_STK         App_TaskMeas2Stk[APP_TASK_STK_SIZE];   //  Стек измерителя2
OS_STK         App_TaskMeas3Stk[APP_TASK_STK_SIZE];   //  Стек измерителя3

OS_STK         App_TaskSendStk[APP_TASK_STK_SIZE];    //  Стек передатчика 

//  Объявления функций, на базе которых в программе будут созданы Задачи
//  Определения этих функций смотрите далее в этом файле 

static  void  App_TaskStart    (void *p_arg);

static  void  App_TaskMeas     (void *p_arg);
static  void  App_TaskSend     (void *p_arg);

u8 err;                 //  Для кода ошибки
u8 cTicks;   //  Служебная для наблюдения "тиков"
u16 cQl;
u8 ucSend=0, ucSend1=0, ucSend2=0;  //  Длит.передачи в UART

u8     cBuf[16];        //  Буфер для строки результата

OS_EVENT *pQue;				  //  Указатель на Очередь  pQue, qData
void *MsgQue[QL];			  //  Массив для Очереди
OS_Q_DATA qData; 			  //  Структура для информации об Очереди qData->OSNMsgs


//  Объявления вспомогательных функций (Функции находятся в файле Addit_Funcs.c)
s8* Dec_Convert(s8* buf, s32 value);
u16 Write1_Poll(s8* ptr);
void Periph_Init(void);
u16 FreeStkSpace(OS_STK * x);

//  Структура для передачи параметров в Задачи-Измерители
typedef struct {
  u16       TDiscr;     //  Период дискретизации (в тиках)
  u16       NSer;       //  Длина серии усреднения
  u8        ADCh;       //  Номер канала АЦП
  u8        SampleT;    //  Время выборки
} MeasTaskParamSet;

MeasTaskParamSet ParData1;
MeasTaskParamSet ParData2;
MeasTaskParamSet ParData3;

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
  BSP_Init();   //  Фактически - только настройка RCC - 72 (или 24) МГц от ФАПЧ 
  OS_CPU_SysTickInit();  // Запуск системного таймера и прерывания от его переполнения
        //  После этого SysTick работает, и многозадачность функционирует полностью

  Periph_Init();    //  Функция находится в файле Addit_Funcs.c
//  Здесь самое подходящее место для создания пользовательских Задач, 
// объектов синхронизации-коммуникации и т.п.

  pQue = OSQCreate(&MsgQue[0], QL);		//  Создадим Очередь
  
  //  Заполнение трёх глобальных структур для настройки параметров Задач-Измерителей,
  // которые создаются на базе одной и той же функции
  //  Если структуры с параметрами объявлены глобально, то параметры легко перестраивать
  // при выполнении программы из любой другой Задачи. 
  //  В стравнении с проектом Mb-1F2T в данном в перечне параметров нет Mbox
  
  ParData1.NSer     = 17;
  ParData1.ADCh     = ADC_Channel_1;
  ParData1.SampleT  = ADC_SampleTime_7Cycles5;
  ParData1.TDiscr   = 9;

  ParData2.NSer     = 23;
  ParData2.ADCh     = ADC_Channel_2;
  ParData2.SampleT  = ADC_SampleTime_13Cycles5;
  ParData2.TDiscr   = 13;
  
  ParData3.NSer     = 27;
  ParData3.ADCh     = ADC_Channel_7;
  ParData3.SampleT  = ADC_SampleTime_1Cycles5;
  ParData3.TDiscr   = 17;

  OSTaskCreate((void (*)(void *)) App_TaskMeas,        //  Создадим Задачу для измерения1
               (void          * ) &ParData1,
               (OS_STK        * ) &App_TaskMeas1Stk[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_TASK_MEAS1_PRIO   // Уровень приоритета для этой Задачи =5
              );

  OSTaskCreate((void (*)(void *)) App_TaskMeas,        //  Создадим Задачу для измерения2
               (void          * ) &ParData2,
               (OS_STK        * ) &App_TaskMeas2Stk[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_TASK_MEAS2_PRIO   // Уровень приоритета для этой Задачи =6
              );

  OSTaskCreate((void (*)(void *)) App_TaskMeas,        //  Создадим Задачу для измерения2
               (void          * ) &ParData3,
               (OS_STK        * ) &App_TaskMeas3Stk[APP_TASK_STK_SIZE - 1],  //  Стек Задачи
               (INT8U           ) APP_TASK_MEAS3_PRIO   // Уровень приоритета для этой Задачи =7
              );

  if((err=    //  Проверяйте возвращаемое значение, когда есть сомнение
    OSTaskCreate((void (*)(void *)) App_TaskSend,          //  Создадим Задачу-передатчик
               (void          * ) 0,
               (OS_STK        * ) &App_TaskSendStk[APP_TASK_STK_SIZE - 1],
               (INT8U           ) APP_TASK_SEND_PRIO   // Уровень приоритета для этой Задачи =11
              )) !=OS_ERR_NONE)
  {
    __NOP();
  } //  Смотрим значение кода ошибки в файле ucos_ii.h, со строки 245
    //  Имена для кодов завершения были изменены в версии 2.84,
    // в данном проекте использованы новые

    //  Можно удалить Задачу, если она больше не нужна, освободится TCB
  OSTaskDel(OS_PRIO_SELF);   //  
}   


/*--------------------------------------------------------------------------------*\
*                                            App_TaskMeas()
*     
* 
\*--------------------------------------------------------------------------------*/

//  Параметры регистрации для сигнала (переменные глобальные) - теперь не используются
// Параметры передаются в каждую Задачу=измеритель через индивидуальную структуру

static void App_TaskMeas(void *p_arg) {
MeasTaskParamSet* pPar;
OS_CPU_SR cpu_sr=0;   //  Для сохранения регистра состояний   
u32   uiSum  =0;      //  Для накопления суммы
u16   usCtr  =0;      //  Счетчик отсчетов в серии
u16		usRez;          //  Усредненный результат для передачи в Очередь

  pPar=(MeasTaskParamSet*)p_arg;  //  Адрес структуры с параметрами - в лок.указатель
  while (DEF_TRUE) {    //  В Задаче должен быть бесконечный цикл
    OS_ENTER_CRITICAL();      //  Начало критической секции
    //  Выбор канала, задание времени выборки
    ADC_RegularChannelConfig(ADC1, pPar->ADCh, 1, pPar->SampleT);
    //  Запустить АЦП
    ADC_Cmd(ADC1, ENABLE);    //  Запустить АЦП
    while (RESET== ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))   // Готовность результата
    {  //  Ожидание результата АЦ преобразования
      __NOP();
    }
    uiSum+=ADC_GetConversionValue(ADC1);
    OS_EXIT_CRITICAL();       //  Конец критической секции
    if((usCtr++)==pPar->NSer) //  Проверка конца серии
    { 
      usCtr=0;                //  Сброс счетчика точек              
      usRez=(u16)(uiSum*3300/(4096*pPar->NSer));  //  Усреднение
      uiSum=0;
      //  Усредненный отсчет вместе с  номером канала -> в Очередь
			if (OSQPost(pQue, (void*)(((pPar->ADCh)<<16)+usRez))==OS_Q_FULL)
			{	
        __NOP();  //  Очередь переполнилась
      }         
      OSQQuery(pQue, &qData);
    }
    OSTimeDly(pPar->TDiscr);  //  Запуск тайм-аута на период дискретизации    // (1)   
  }
}


/*--------------------------------------------------------------------------------*\
*                                            App_TaskSend()
*
* Description : Задача выполняет передачу отсчетов  из Очереди,
*  Каналов может быть сколько угодно. pQue->
* Уникальный уровень приоритета для этой Задачи = 11 
\*--------------------------------------------------------------------------------*/

static void App_TaskSend(void *p_arg) {
u32 uiRez;
u8  ucCh;

  while (DEF_TRUE) {        //  В Задаче должен быть бесконечный цикл
//    Опрос очереди
    uiRez=(int)OSQPend(pQue,0,&err);
//    cQl=qData.OSNMsgs;
    Dec_Convert((s8*)cBuf, uiRez&0xFFF); 
      ucSend=100;           //  Для наблюдения времени передачи
		ucCh=(uiRez>>16)&0x1F;  //  Канал
    if(ucCh==ParData1.ADCh)     // ====>>  Подумайте, нельзя ли сделать изящнее ???
    {  
      Write1_Poll((s8*)"Sig_1 = ");
    }
    if(ucCh==ParData2.ADCh)
    {  
      Write1_Poll((s8*)"  Sig_2 = ");
    }
    if(ucCh==ParData3.ADCh)
    {  
      Write1_Poll((s8*)"    Sig_3 = ");
    }

    Write1_Poll((s8*)&cBuf[6]);
    Write1_Poll((s8*)" mV\n");
      ucSend1=0;
      ucSend=ucSend1+ucSend2;       //  Для наблюдения времени передачи
  }   //  В Задаче должен быть бесконечный цикл, здесь его конец
}     //  Здесь заканчивается тело функции, на базе которой создается Задача

