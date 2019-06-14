
/*************************************************************************************************\
*  Filename      : Addit_Funcs.c 
*  
* Содержит вспомогательные функции для примеров, демонстрирующих использование ОСРВ UCOS-II
*
\*************************************************************************************************/


#include "includes.h"
#include <stm32f10x.h>

#include "exacto_commander.h"
#include "exacto_buffer.h"
#include "lsm303ah_reglib.h"
#include "bmp280_reglib.h"
#include "ism330_reglib.h"

#include "exacto_struct.h"
#include "exacto_defs.h"

extern volatile ExactoSensorSet lsm303;
extern ExactoSensorSet bmp280;
extern ExactoSensorSet ism330;

extern void SendStr(s8* ptr);

//------------------------------------------------------------------------------------
uint8_t Exacto_getfrq_ism330_G(void)
{
    if(!ism330.Whoami) return 0;
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    uint8_t ctrl1 = read_ism330(ISM330DLC_CTRL2_G);
    OS_EXIT_CRITICAL()
    return ctrl1;
}
uint8_t Exacto_setfrq_ism330(uint8_t mode)
{
  if(!ism330.Whoami) return 0;
  OS_CPU_SR cpu_sr = 0;
	uint8_t trgXL = 0x00;
	uint8_t trgG = 0x00;
	switch(mode)
	{
		case 0:
			// 0100 01 0 0
			trgXL = 0x44;
			// 0100 11 0 0
			trgG = 0x4c;
			break;
		default:
			return 0;
	}
	OS_ENTER_CRITICAL()
	write_ism330(ISM330DLC_CTRL1_XL,	trgXL);
	write_ism330(ISM330DLC_CTRL2_G,	trgG);
	uint8_t ctrl1 = read_ism330(ISM330DLC_CTRL1_XL);
	uint8_t ctrl2 = read_ism330(ISM330DLC_CTRL2_G);
	OS_EXIT_CRITICAL()
	if((ctrl1 == trgXL)&&(ctrl2 == trgG))
	{
		switch(mode)
		{
				case 0:
						OS_ENTER_CRITICAL()
						ism330.TDiscr = OS_TIME_10mS;
						ism330.MultSens1 = 1;
						ism330.MultSens2 = 1;
						ism330.MultSens3 = 1;
						OS_EXIT_CRITICAL()
						SendStr((int8_t*)"SETFREQ:ism330:10ms\n");
						break;
		}
		return 1;
	}
	else
		return 0;
}
uint8_t Exacto_getfrq_bmp280(void)
{
    if(!bmp280.Whoami) return 0;
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    uint8_t ctrl1 = read_bmp280(BMP280_CTRL_MEAS_ADDR);
    OS_EXIT_CRITICAL()
    return ctrl1;
}
uint8_t Exacto_setfrq_bmp280(uint8_t mode)
{
    if(!bmp280.Whoami) return 0;
	OS_CPU_SR cpu_sr = 0;
	uint8_t trg1 = 0;
	uint8_t trg2 = 0;
	switch(mode)
	{
		case 0:		
			// 101 010 11
			trg1 = 0xab;
			// tsb 000 fltr 0000 spi3 0
			trg2 = 0x00;
			break;
		default:
			return 0;
	}
	OS_ENTER_CRITICAL()
	
//	uint8_t cnt = 0;
	uint8_t ctrl;
	
//	EnReset_bmp280();
	
//DsReset_bmp280();
	
	setval_bmp280(BMP280_CTRL_MEAS_ADDR,trg1);

	ctrl = getval_bmp280(BMP280_CTRL_MEAS_ADDR);

	write_bmp280(BMP280_CONFIG_ADDR,trg2);
	
	uint8_t ctrl2 = getval_bmp280(BMP280_CONFIG_ADDR);
	OS_EXIT_CRITICAL()
	if((ctrl == trg1)&&(ctrl2 == trg2))
	{
		switch(mode)
		{
			case 0:
				OS_ENTER_CRITICAL()
				bmp280.TDiscr = OS_TIME_1mS*38;
				OS_EXIT_CRITICAL()
				SendStr((int8_t*)"SETFREQ:bmp280:38ms\n");
				break;
		}
		return 1;
	}
	else
		return 0;
}
uint8_t Exacto_getfrq_lsm303ah(void)
{
    if(!lsm303.Whoami) return 0;
    OS_CPU_SR cpu_sr = 0;
    OS_ENTER_CRITICAL()
    uint8_t ctrl1 = read_lsm303ah(LSM303AH_CTRL1_A);
    OS_EXIT_CRITICAL()
    return ctrl1;
}
uint8_t Exacto_setfrq_lsm303ah(uint8_t mode)
{
    if(!lsm303.Whoami) return 0;
	OS_CPU_SR cpu_sr = 0;
  uint8_t trg = 0;
	uint8_t trgM = 0;
	switch(mode)
	{
		case 0:
            //0100 0100 -- 100 Hz
            trg = 0x44;
						//0000 1100 -- 100 Hz
						trgM = 0x0C;
            break;
        case 1:
            // 800 Hz 1111 01 00
            trg = 0xF4;
						//0000 1100 -- 100 Hz
						trgM = 0x0C;
            break;
        default:
            return 0;
	}
    OS_ENTER_CRITICAL()
    write_lsm303ah(LSM303AH_CTRL1_A,trg);
		write_lsm303ah(LSM303AH_CFG_REG_A_M,trgM);
    uint8_t ctrl1 = read_lsm303ah(LSM303AH_CTRL1_A);
		uint8_t ctrl2 = read_lsm303ah(LSM303AH_CFG_REG_A_M);
    OS_EXIT_CRITICAL()
    if((ctrl1 == trg)&&(ctrl2 == trgM))
    {
        switch(mode)
        {
            case 0:
                OS_ENTER_CRITICAL()
                lsm303.TDiscr = OS_TIME_10mS;
								lsm303.MultSens1 = 1;
								lsm303.MultSens2 = 1;
                OS_EXIT_CRITICAL()
                SendStr((int8_t*)"SETFREQ:lsm303:10ms\n");
                break;
            case 1:
                OS_ENTER_CRITICAL()
                lsm303.TDiscr = (INT16U)12; 
								lsm303.MultSens1 = 1;
								lsm303.MultSens2 = 8;
                OS_EXIT_CRITICAL()
                SendStr((int8_t*)"SETFREQ:lsm303:1.2ms\n");
                break;
        }
        return 1;
    }
    else
        return 0;
}
uint8_t Exacto_slftst_lsm303ah(void)
{
    if(!lsm303.Whoami) return 0;
//	uint8_t data[6];
	int16_t data_BUFF[] = {0,0,0};
	int16_t data_NOST[] = {0,0,0};
	int16_t data_ST[] = {0,0,0};
	/* Инициализируем датчик */
//    writeSPI_lsm303(LSM303AH_SPI, LSM303AH_GPIO, LSM303AH_CS_PIN, LSM303AH_CTRL1_A_ADR, LSM303AH_CTRL1_A_SELFTEST_SETUP);
//	uint8_t ctrl1 = readWordSPI_lsm303(LSM303AH_SPI, GPIOA, LL_GPIO_PIN_4, LSM303AH_CTRL1_A_ADR);
//	uint8_t ctrl2 = readWordSPI_lsm303(LSM303AH_SPI, GPIOA, LL_GPIO_PIN_4, LSM303AH_CTRL2_A_ADR);
	OS_CPU_SR cpu_sr = 0;
  OS_ENTER_CRITICAL()
	write_lsm303ah(LSM303AH_CTRL1_A,0x39);
	uint8_t ctrl1 = read_lsm303ah(LSM303AH_CTRL1_A);
	OS_EXIT_CRITICAL()
	if (ctrl1 != 0x39)
		return 0;
	/* Ждем 20 мс для стабильных данных*/
//    LL_mDelay(20);
	OSTimeDly((INT16U)((INT32U)OS_TICKS_PER_SEC * 20L / 1000L));
	/* Читаем данные для очитки бита готовности данных */
//	while(!lsm303ah_readXLcont(data));
	OS_ENTER_CRITICAL()
	while(!GetXLallData_lsm303ah(data_BUFF)){ __NOP();}
	OS_EXIT_CRITICAL()
	/* Читаем данные пять раз */
	//		while(!lsm303ah_readXLcont(data));
//		for (uint8_t j = 0; j <3;j++)
//		{
//			int16_t value = lsm303ah_genval(data[j*2],data[j*2 + 1]);
//			data_NOST[j] += value;
//		}
	for (uint8_t i = 0; i < 5; i++)
	{
		OS_ENTER_CRITICAL()
		while(!GetXLallData_lsm303ah(data_BUFF)){ __NOP();}
		OS_EXIT_CRITICAL()
		for (uint8_t j = 0; j <3;j++)	data_NOST[j] += data_BUFF[j];
	}
	for (uint8_t j = 0; j <3;j++)
	{
		data_NOST[j] *= 0.061/5;
	}
	/* Включаем самотест */
//    writeSPI_lsm303(LSM303AH_SPI, LSM303AH_GPIO, LSM303AH_CS_PIN, LSM303AH_CTRL3_A_ADR, LSM303AH_CTRL3_A_SELFTEST_ENABLE);
//	uint8_t ctrl3  = readWordSPI_lsm303(LSM303AH_SPI, GPIOA, LL_GPIO_PIN_4, LSM303AH_CTRL3_A_ADR);
//	if (ctrl3 != LSM303AH_CTRL3_A_SELFTEST_ENABLE)
//		return 0;
	OS_ENTER_CRITICAL()
	write_lsm303ah(LSM303AH_CTRL3_A,0x40);
	uint8_t ctrl3 = read_lsm303ah(LSM303AH_CTRL3_A);
	OS_EXIT_CRITICAL()
	if(ctrl3 != 0x40)	return 0;
	/* Ждем 60 мс */
    //LL_mDelay(60);
	OSTimeDly((INT16U)((INT32U)OS_TICKS_PER_SEC * 60L / 1000L));
	/* Читаем данные для очитки бита готовности данных */
	//while(!lsm303ah_readXLcont(data));
	/* Читаем данные пять раз */
//	for (uint8_t i = 0; i < 5; i++)
//	{
//		//while(!lsm303ah_readXLcont(data));
//		//for (uint8_t j = 0; j <3;j++)	data_ST[j] += lsm303ah_genval(data[j*2],data[j*2 + 1]);
//	}
	for (uint8_t i = 0; i < 5; i++)
	{
		OS_ENTER_CRITICAL()
		while(!GetXLallData_lsm303ah(data_BUFF)){ __NOP();}
		OS_EXIT_CRITICAL()
		for (uint8_t j = 0; j <3;j++)	data_ST[j] += data_BUFF[j];
	}
	for (uint8_t j = 0; j <3;j++)
	{
		data_ST[j] *= 0.061/5;
	}
	/* Сравнеиваем результаты измерения */
	uint8_t flag = 0;
	int16_t diff[] = {-1,-1,-1};
	for (uint8_t i = 0; i < 3;  ++i)
	{
		int16_t val = (data_ST[i] - data_NOST[i]);
		diff[i] = val;
		if (val < 0)
			val *= -1;
		if ((val >= LSM303AH_SELFTEST_ST_MIN)&&(val <= LSM303AH_SELFTEST_ST_MAX))
			flag ++;
	}

	for (uint8_t i = 0; i < 3;  ++i)
	{
		if(diff[i] == -1)
			return 0;
	}
	/* Отключаем самотест */
    //writeSPI_lsm303(LSM303AH_SPI, LSM303AH_GPIO, LSM303AH_CS_PIN, LSM303AH_CTRL3_A_ADR, 0x00);
	/* Отключаем сенсор */
    //writeSPI_lsm303(LSM303AH_SPI, LSM303AH_GPIO, LSM303AH_CS_PIN, LSM303AH_CTRL1_A_ADR, 0x00);
	OS_ENTER_CRITICAL()
	write_lsm303ah(LSM303AH_CTRL1_A,0x00);
	write_lsm303ah(LSM303AH_CTRL3_A,0x00);
	OS_EXIT_CRITICAL()
	if (flag == 3)
		return 1;
	else
        return 0;
}

uint8_t Exacto_init_lsm303ah(void)
{
    ClockEnPins_lsm303ah();
    ConfigurePins_lsm303ah();
    ConfigureSPI3_lsm303ah();
    ActivateSPI3_lsm303ah();
    return (Set3wireAndGetWhoami_lsm303ah());
}
uint8_t Exacto_init_bmp280(void)
{
    OS_CPU_SR cpu_sr = 0;
  OS_ENTER_CRITICAL()
	ClockEnPinsI2C_bmp280();
    
    ConfigurePinsI2C_bmp280();
    
    ConfigureI2C_bmp280();
    
    
    ActivateI2C_bmp280();
    OS_EXIT_CRITICAL()
    return (GetWhoami_bmp280());
}
uint8_t Exacto_init_ism330(void)
{
	ConfigureSPI3_ism330();
    ActivateSPI3_ism330();
    return (Set3wireAndGetWhoami_ism330());
}

//  Функция начальной настройки периферийных подсистем
void Periph_Init(void)
{
  //  Разрешить тактирование нужных подсистем
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |  //  Порт PA - два входа аналоговых
                          RCC_APB2Periph_ADC1 |   //  АЦП
                          RCC_APB2Periph_USART1,  //  UART
                            ENABLE);   

  //  Настроить выводы порта PA1, PA2, PA7 как аналоговые входы
    GPIO_InitTypeDef gpioA;
  gpioA.GPIO_Pin    = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;    //  для подключения датчиков
  gpioA.GPIO_Mode   = GPIO_Mode_AIN;   
    GPIO_Init(GPIOA, &gpioA);
  
  //  Выход PA9 - Tx передатчик UART
  gpioA.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;         //  Выходы Tx|Rx USART1
  gpioA.GPIO_Mode = GPIO_Mode_AF_PP;
  gpioA.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioA);

    //  Настройка АЦП общая
    ADC_InitTypeDef ADC_InitStructure;
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    //  Если два или более АЦП
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;         //  Режима сканирования нет
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;   //  Непрер.режима нет
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //  Внеш.запуск ОТКЛ
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //  Рез. выровнен вправо
  ADC_InitStructure.ADC_NbrOfChannel = 1;               //  Число каналов 1
    ADC_Init(ADC1, &ADC_InitStructure);           //  Настройка
    
  ADC_Cmd(ADC1, ENABLE);      //  Разрешение работы АЦП

    //  Калибровка АЦП
  ADC_ResetCalibration(ADC1); //  Сброс встроенной калибровки
  while(ADC_GetResetCalibrationStatus(ADC1)) {} //  Ожидание конца сброса
  ADC_StartCalibration(ADC1); //  Запуск калибровки
  while(ADC_GetCalibrationStatus(ADC1)){}   //  Ожидание конца калибровки

    //  Настройка UART
    USART_InitTypeDef USART_InitStructure;      //  Задание настроечных параметров для USART1
  USART_InitStructure.USART_BaudRate = 115200;   //  Скорость 19200 бит/с 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx; //  Нужeн Tx 
    USART_Init(USART1, &USART_InitStructure);

    //  После включения все прерывания от USART запрещены
    //  Настраиваем группинг прерываний  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);     //  Кол.уровней прио.=8
    
  NVIC_SetPriority (SysTick_IRQn, 0xC);  //  Прио=6, СубПрио=0
    //  Настраиваем контроллер прерываний  
    NVIC_InitTypeDef NVIC_InitStructure;    //  Для настройки контроллера прерываний
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   //  Прер.от USART1
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //  Разрешаем прерывание от USART1
    NVIC_Init(&NVIC_InitStructure);                   // Собственно настройка
    

  USART_Cmd(USART1, ENABLE);    //  Включили UART
}

//----------------------------------------------------------------------------------------
  //  Функция для контроля размера стеков при отладке. Ей надо давать в качестве параметра
  // имя массива, на базе которой организован стек Задачи, например, вот так:
  // Var=FreeStkSpace(App_TaskStartStk);  
  //   В переменную Var вернется размер свободного места в Задаче TaskStart
u16 FreeStkSpace(OS_STK * x) 
{ OS_STK * pS  = x;
  OS_CPU_SR cpu_sr = 0; 
  u16 StkCtr   = 0;
    OS_ENTER_CRITICAL();
    while (*pS==0) { StkCtr++; pS++;}
    OS_EXIT_CRITICAL();
    return StkCtr;
}

//-------------------------------------------------------------------------
//  Преобразование 32-битовой величины со знаком в 10-чную ASCII-строку
#define FALSE 0
#define TRUE !FALSE

s8* Dec_Convert(s8* buf, s32 value) {
                //0123456789
	int divider = 1000000000;     
	unsigned char bNZflag=FALSE, minus=FALSE;		//  Флаги левых нулей и минуса
	unsigned char current_digit;

	if (value < 0) {		//    Если число value отрицательное 
		minus=TRUE;
		value = -value;
	}
	while (divider) {
		current_digit = value / divider;
		if (current_digit || bNZflag) { //  Как только получили ненулевую цифру,
		  	if (minus) { 	      //  Если число отрицательное, то поставим -
		    	buf--;
				*buf++ = '-';
				minus=FALSE; 
		  	} 
			value %= divider;
			*buf++ = current_digit + '0';
			bNZflag = TRUE;				// это значит, что левые нули закончились
		} else {  			//  Вместо левых нулей - пробелы, чтобы выровнять вправо
		    *buf++ = ' ';
		}
		divider /= 10;
    }
	if (!bNZflag)
		*buf++ = '0';
	*buf = 0;				//  Это нуль-терминатор (признак окончания строки)
  return buf;
}


//-------------------------------------------------------------------------------
//  Функция передачи строки через USART1 с поллингом без проверок параметров
u16 Write1_Poll(s8* ptr) {
  int32_t len = 0;
  while (*ptr) {
    USART_SendData(USART1, *ptr);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
    ptr++;  
		len++;
//    while (GetChar1() != ' ') { }    //  Ожидание приема пробела
  }
  return len;
}

void UART_init(void)
{
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  GPIO_InitTypeDef gpioA;
    gpioA.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;         
    gpioA.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioA.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpioA);
  USART_InitTypeDef USART_InitStructure;     
    USART_InitStructure.USART_BaudRate = 115200;   
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);      
  NVIC_SetPriority (SysTick_IRQn, 0xC); 
    //   
    NVIC_InitTypeDef NVIC_InitStructure;    
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;   
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
    NVIC_Init(&NVIC_InitStructure);                   
  USART_Cmd(USART2, ENABLE);    
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
}


