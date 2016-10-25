/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <stdio.h>
#include "bsp_TiMbase.h" 
#include "bsp_usart.h"

extern uint16_t Noise_Count;
extern  uint16_t      Usart1_recev_buff[300];
extern  uint16_t  Usart1_recev_count;
extern  uint32_t  time ;                       // ms 计时变量  
extern  char      Usart2_recev_buff[50];         //USART2接收缓存
extern  uint8_t   Usart2_recev_count;            //USART2接收计数器
extern  char      Usart3_recev_buff[1000];
extern  uint16_t  Usart3_recev_count;

extern  uint32_t  TimeDisplay;                   //RTC定时标志变量
extern  u8        WWDG_CNT;                      //保存WWDG计数器的设置值,默认为最大
extern  uint16_t  WWDOG_Feed ;                   //窗口看门狗复位周期为：255*1.8s = 7.6min
extern  u8        Uart4_send_buff[100]; 
extern  u8        Uart4_rev_buff[100];
extern  char      Usart4_recev_buff[1000];
extern  uint16_t  Usart4_recev_count;
extern  vs8       Uart4_send_counter;
extern  vu8       Uart4_rev_count;               //RS485串口接收计数器
extern  vu8       Uart4_rev_comflag;
extern  vu8       Crc_counter;                   //RS485校验计数器
extern  vu8*      Uart4_send_pointer ;           //RS485串口发送指针
extern  uint8_t   DMA_UART3_RECEV_FLAG;          //USART3 DMA接收标志变量
extern char Usart2_recev_count1;
//extern  char  Usart1_send_buff[];
//extern  uint8_t  Usart1_send_count;
//extern  uint32_t  Tic_IWDG;
//extern  char  Usart3_send_buff[];
//extern  uint8_t  Usart3_send_count;
//extern void Delay(uint32_t nCount);

extern void TimingDelay_Decrement(void);

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	printf("\r\nWarnning!! NMI Occured\r\n");// 测试使用
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */

	while (1)
  {
		printf("\r\nWarnning!! Hard Fault Occured\r\n");// 测试使用
		break;
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
		printf("\r\nWarnning!! MemManage Occured\r\n");// 测试使用
		break;
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
		printf("\r\nWarnning!! BusFault Occured\r\n");// 测试使用
		break;
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
		printf("\r\nWarnning!! UsageFault Occured\r\n");// 测试使用
		break;
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	printf("\r\nWarnning!! SVCall exception Occured\r\n");// 测试使用
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	printf("\r\nWarnning!! Debug Monitor exception Occured\r\n");// 测试使用
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	printf("\r\nWarnning!! PendSVC exception Occured\r\n");// 测试使用
}


/**
  * @brief  This function handles WWDG Handler.
  * @param  None
  * @retval None
  */
//窗口看门狗中断服务程序 
void WWDG_IRQHandler(void)
{
	if(WWDOG_Feed>0)
	{
		WWDG_SetCounter(WWDG_CNT);   //重装计数值
		WWDG_ClearFlag();//清除提前唤醒中断标志位
		WWDOG_Feed--;
	}
	
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)  //当前设置，10us中断一次
{
	TimingDelay_Decrement();	
	
//	Tic_IWDG++;                //看门狗喂狗时间间隔计数器
//  if(Tic_IWDG>=50000)        //喂狗时间为500ms
//  {  
//    
//	 IWDG_ReloadCounter();     //喂狗
//   Tic_IWDG=0;               //复位计数器
//  }
}

/**
  * @brief  This function handles RTCAlarm interrupt request.
  * @param  None
  * @retval None
  */
void RTCAlarm_IRQHandler(void)
{				
 //等待RTC_CTL寄存器中的RSF位(寄存器同步标志)被硬件置1
 RTC_WaitForSynchro();
 if (RTC_GetITStatus(RTC_IT_ALR) != RESET)
 {
   printf("Wakeup!!");
   EXTI_ClearITPendingBit(EXTI_Line17);          // 清EXTI_Line17挂起位
   if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)   // 检查唤醒标志是否设置
   {   
       PWR_ClearFlag(PWR_FLAG_WU);               // 清除唤醒标志
   }

   /* Clear the RTC Second interrupt */
////  RTC_SetCounter(0);
////  RTC_WaitForLastTask();
   RTC_ClearITPendingBit(RTC_IT_ALR);
   RTC_WaitForLastTask();
////    RTC_SetAlarm(RTC_GetCounter()+10);
////    RTC_WaitForLastTask();
 }
 //Noise_Count = 0;
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles RTC interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
	  //Noise_Count = 10;
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET)       //秒中断发生
	  {
	    /* Clear the RTC Second interrupt */
	    RTC_ClearITPendingBit(RTC_IT_SEC);
	
	    /* Enable time update */
	    TimeDisplay = 1;
	
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
	  }
		// Noise_Count = 0;
}


/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)           //1ms定时计数器
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);  	
    time++;		
	}		 	
}

/**
  * @brief  This function handles TIM3 interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)         //20ms定时计数器
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
    	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);    
		  TIM_Cmd(TIM3, DISABLE);                      //关闭TIM3 
		  TIM_DeInit(TIM3);                            //复位TIM3定时器
		  Uart4_rev_comflag = 1;                       //RS485接收完成标志
    	Crc_counter = Uart4_rev_count;               //crc校验计数器赋值
    	Uart4_rev_count = 0;                         //RS485接收计数器清零
		  Usart4_recev_count =0;

	}	
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void USART1_IRQHandler(void)
{   
	
	  if(Usart1_recev_count == (sizeof(Usart1_recev_buff)-1))                         //USART1接收Buff满，重置接收计数器，数据从Usart1_recev_buff[0]开始存，防止发生越界访问
		{
      printf("\r\nUSART1 Receive Buff Full!!\r\n");                      //测试使用
			Usart1_recev_count =0;
    }
		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)                //USART1接收缓冲区有数据
		{ 
			 USART_ClearITPendingBit(USART1,USART_IT_RXNE);
			 Usart1_recev_buff[Usart1_recev_count++] = USART_ReceiveData(USART1);
       while (USART_GetITStatus(USART1, USART_FLAG_RXNE) == SET){; }     //等待接收完成	
		} 
		if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)               //接收缓冲区溢出，放弃接收到的数据
		{				
			USART_ClearFlag(USART1, USART_FLAG_ORE);
			USART_ReceiveData(USART1);
		}
	
}	
void USART2_IRQHandler(void)
{
	  if(Usart2_recev_count == (sizeof(Usart2_recev_buff)-1))              //USART2接收Buff满，重置接收计数器，数据从Usart2_recev_buff[0]开始存，防止发生越界访问
		{
      printf("\r\nUSART2 Receive Buff Full!!\r\n");                      //测试使用
			Usart2_recev_count =0;
    }
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)                 //USART2接收缓冲区有数据
		{ 
			 USART_ClearITPendingBit(USART2,USART_IT_RXNE);
			 Usart2_recev_buff[Usart2_recev_count++] =USART_ReceiveData(USART2);
       while (USART_GetITStatus(USART2, USART_FLAG_RXNE) == SET){; }      //等待接收完成	
			 Usart2_recev_count1 = 1;
		} 
		if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) == SET)                //接收缓冲区溢出，放弃接收到的数据
		{				
			USART_ClearFlag(USART2, USART_FLAG_ORE);
			USART_ReceiveData(USART2);
		}
	
}	
//void USART3_IRQHandler(void)
//{
//    u16 i =0;  
//	
//	  printf("\r\nEnter USART3_IRQ!! \r\n");     //测试使用
//    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)   //如果为空闲总线中断
//    {  
//       /* IDLE bit is cleared by a software sequence(an read to the USART_SR register followed by a read to the USART_DR register)*/ 
//		  	i = USART1->SR;  
//        i = USART1->DR;  
//        USART_ClearITPendingBit(USART3, USART_IT_IDLE);  
//			  USART_ITConfig(USART3, USART_IT_IDLE, DISABLE); 
////        DMA_Cmd(DMA1_Channel3, DISABLE);           //关闭DMA防止处理期间有数据
//			  DMA_UART3_RECEV_FLAG =1;                  //设置标志变量
//			  printf("\r\nOK!! \r\n");     //测试使用
////			DMA_Cmd(DMA1_Channel3, ENABLE);          //开启DMA 
//		}

//}




void USART3_IRQHandler(void)
{
		 /* TIM2 重新开时钟，开始计时 */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)   //USART3接收缓冲区有数据
		{ 
			 USART_ClearITPendingBit(USART3,USART_IT_RXNE);
			 Usart3_recev_buff[Usart3_recev_count++] =USART_ReceiveData(USART3);       
       while (USART_GetITStatus(USART3, USART_FLAG_RXNE) == SET){; }       //等待接收完成	
			 DMA_UART3_RECEV_FLAG =1;                  //设置标志变量
		} 
		if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) == SET)                 //接收缓冲区溢出，放弃接收到的数据
		{				
			USART_ClearFlag(USART3, USART_FLAG_ORE);
			USART_ReceiveData(USART3);
		}
		time=0;
}

//void USART4_IRQHandler(void)
//{
//		 /* TIM2 重新开时钟，开始计时 */
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
//		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)   //USART3接收缓冲区有数据
//		{ 
//			 USART_ClearITPendingBit(UART4,USART_IT_RXNE);
//			 Usart4_recev_buff[Usart4_recev_count++] =USART_ReceiveData(UART4);       
//       while (USART_GetITStatus(UART4, USART_FLAG_RXNE) == SET){; }       //等待接收完成	
//			 DMA_UART3_RECEV_FLAG =1;                  //设置标志变量
//		} 
//		if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) == SET)                 //接收缓冲区溢出，放弃接收到的数据
//		{				
//			USART_ClearFlag(UART4, USART_FLAG_ORE);
//			USART_ReceiveData(UART4);
//		}
//		time=0;
//}

//void USART3_IRQHandler(void)
//{
//		 /* TIM2 重新开时钟，开始计时 */
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
//	
//	  if(Usart3_recev_count == (sizeof(Usart3_recev_buff)-1))              //USART3接收Buff满，重置接收计数器，数据从Usart3_recev_buff[0]开始存，防止发生越界访问
//		{
//      printf("\r\nUSART1 Receive Buff Full!!\r\n");                      //测试使用
//			Usart3_recev_count =0;
//    }
//		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)                  //USART3接收缓冲区有数据
//		{ 
//			 USART_ClearITPendingBit(USART3,USART_IT_RXNE);
//			 Usart3_recev_buff[Usart3_recev_count++] =USART_ReceiveData(USART3);       
//       while (USART_GetITStatus(USART3, USART_FLAG_RXNE) == SET){; }       //等待接收完成	
//			 
//		} 
//		if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) == SET)                 //接收缓冲区溢出，放弃接收到的数据
//		{				
//			USART_ClearFlag(USART3, USART_FLAG_ORE);
//			USART_ReceiveData(USART3);
//		}
//		time=0;
//}

//void UART4_IRQHandler(void)
//{
//		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
//	  {
//		   USART_ClearITPendingBit(UART4,USART_IT_RXNE);
//    	 if(Uart4_rev_comflag != 1)       
//    	 {
//        	if(Uart4_rev_count == (sizeof(Uart4_rev_buff)-1))              //UART4接收Buff满，重置接收计数器，数据从Uart4_recev_buff[0]开始存，防止发生越界访问
//					{
//						printf("\r\nUSART4 Receive Buff Full!!\r\n");                      //测试使用
//						Uart4_rev_count =0;
//					}			 
//				  else
//        	{       
//						 Uart4_rev_buff[Uart4_rev_count++]= USART_ReceiveData(UART4);	 
//						 while (USART_GetITStatus(UART4, USART_FLAG_RXNE) == SET){; }    //RS485等待接收完成	
//			 	 	   TIM3_Configuration();                                           //打开接收定时器
//        	}   
//    	 }    
//     	 else
//    	 {
//        	USART_ReceiveData(UART4);   //如果接收没有处理，放弃接收的数据
//    	 }
//   }  
//	 if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) == SET)   //接收缓冲区溢出，放弃接收到的数据
//	 {			
//			USART_ClearFlag(UART4, USART_FLAG_ORE);
//			USART_ReceiveData(UART4);
//	 }		
//}		
		
void UART4_IRQHandler(void)
{
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	  {
		   USART_ClearITPendingBit(UART4,USART_IT_RXNE);
    	 if(Uart4_rev_comflag != 1)       
    	 {
        	if(Usart4_recev_count == (sizeof(Usart4_recev_buff)-1))              //UART4接收Buff满，重置接收计数器，数据从Uart4_recev_buff[0]开始存，防止发生越界访问
					{
						printf("\r\nUSART4 Receive Buff Full!!\r\n");                      //测试使用
						Usart4_recev_count =0;
					}			 
				  else
        	{       
						 Usart4_recev_buff[Usart4_recev_count++]= USART_ReceiveData(UART4);	 
						 while (USART_GetITStatus(UART4, USART_FLAG_RXNE) == SET){; }    //RS485等待接收完成
             DMA_UART3_RECEV_FLAG =1;                  //设置标志变量						 
			 	 	   TIM3_Configuration();                                           //打开接收定时器
        	}   
    	 }    
     	 else
    	 {
        	USART_ReceiveData(UART4);   //如果接收没有处理，放弃接收的数据
    	 }
   }  
	 if(USART_GetFlagStatus(UART4, USART_FLAG_ORE) == SET)   //接收缓冲区溢出，放弃接收到的数据
	 {			
			USART_ClearFlag(UART4, USART_FLAG_ORE);
			USART_ReceiveData(UART4);
	 }		
}	
	
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
 */ 
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
