#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "stm32f10x.h"
#include <stdio.h>

#define USART1_DR_Base    0x40013804		//USART1数据寄存器基地址： 0x40013800 + 0x04 = 0x40013804
#define USART3_DR_Base    0x40004804		//USART3数据寄存器基地址： 0x40004800 + 0x04 = 0x40004804
#define SENDBUFF_SIZE     500     //最大65535
#define RECEIVEBUFF_SIZE  1000    //最大65535

///////////////////////////////////////////////////////////////////////////////
//声明结构体
struct DMA_USART3_RecevConfig 
{
	int  CurrentDataStartNum;     //当前接收数据开始位置
	int  CurrentDataEndNum;       //当前接收数据结束位置
	int  NextDataStartNum;        //下一次接收数据开始位置
	
};


void USART3_DMA_Config(void);
void USART1_Config(void);
void USART2_Config(void);
void USART3_Config(void);
void USART4_Config(void);
void UART_NVIC_Configuration(void);
int  fputc(int ch, FILE *f);
int  fgetc(FILE *f);
#define  DIR485_Receive()  GPIO_ResetBits(GPIOC,GPIO_Pin_9)
#define  DIR485_Send()     GPIO_SetBits(GPIOC,GPIO_Pin_9)

#endif /* __USART1_H */
