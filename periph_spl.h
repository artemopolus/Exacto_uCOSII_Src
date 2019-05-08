#ifndef PERIPH_SPL_H_
#define PERIPH_SPL_H_

#include "includes.h"
#include <stm32f10x.h>

uint8_t * pTxUARTbuffer;


void UARTsendEnable(uint8_t * pData)
{
  if(!pData)
        return;
  u8 err;        
  OSSemPend(pUart,0,&err);   
  pTxUARTbuffer = pData;                  
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
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
    USART_InitStructure.USART_BaudRate = 19200;   
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

#endif /* PERIPH_SPL_H_ */