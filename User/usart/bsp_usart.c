/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   usart应用bsp
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO-MINI STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include "stm32f10x.h"
#include "bsp_usart.h"
extern void Delay(uint32_t nCount);

uint8_t SendBuff[SENDBUFF_SIZE];
uint8_t DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
struct  DMA_USART3_RecevConfig   DMA_USART3_RecevIndicator; 
/**
  * @brief  USART1 GPIO 配置,工作模式配置。9600 8-N-1
  * @param  无
  * @retval 无
  */
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* USART1 GPIO config */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Configure PowerEN_433M(PA.07) as output push-pull */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure PowerEN_12V(PA.08) as output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Configure PowerEN_485(PA.08) as output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 9600;    //液位计通信波特率默认9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	/* 使能串口1接收中断 */
 	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 	USART_ClearITPendingBit(USART1,USART_IT_TC);
	USART_Cmd(USART1, ENABLE);
}

/**
  * @brief  USART2 GPIO 配置,工作模式配置。115200 8-N-1
  * @param  无
  * @retval 无
  */
void USART2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* config GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
	/* WKUP GPIO config */
	/* Configure WKUP (PA.00) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART2 GPIO config */
	/* Configure USART2 Tx (PA.02) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART2 Rx (PA.03) as input floating */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 9600;    //通信波特率默认9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	/* 使能串口2接收中断 */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ClearITPendingBit(USART2,USART_IT_TC);
	USART_Cmd(USART2, ENABLE);
	
	/* 用PA3控制5v电压输出给噪声模拟电路*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  USART3 GPIO 配置,工作模式配置。9600 8-N-1
  * @param  无
  * @retval 无
  */
void USART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	
	/* USART3 GPIO config */
	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	/* Configure USART3 Rx (PB.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Configure SIM5216 POWER_ON (PC.07) as output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* USART3 mode config */
	USART_InitStructure.USART_BaudRate = 9600;         //3G模块通信波特率默认9600
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	/* 使能串口3接收中断 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ClearITPendingBit(USART3,USART_IT_TC);
//  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  
	USART_Cmd(USART3, ENABLE);

}

 /**
  * @brief  UART4 GPIO 配置,工作模式配置。9600 8-N-1
  * @param  无
  * @retval 无
  */
void USART4_Config(void)     
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
		/* config USART4 clock */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO , ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* UART4&485 GPIO config */
	/* Configure 485 DIR (PC.07) as output push-pull  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);   
	/* Configure USART4 Tx (PC.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);    
	/* Configure USART4 Rx (PC.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	
	/* USART4 mode config */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	
	/* 使能串口4接收中断 */
 	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  USART_ClearITPendingBit(UART4,USART_IT_TC);
	USART_Cmd(UART4, ENABLE);
//  USART_GetFlagStatus(UART4, USART_FLAG_TC);          //串口硬件复位之后，发送首字节之前，先读一下USART_SR，防止数据发送时首字节被覆盖
//	USART_ClearFlag(UART4, USART_FLAG_TC);
}


///// 配置UART接收中断
//void UART_NVIC_Configuration(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure; 
//	/* Configure the NVIC Preemption Priority Bits */  
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	
//	/* Enable the USART1 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
// /* Enable the USART2 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	/* Enable the USART3 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	/* Enable the UART4 Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;	     
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //测试通过以后，在多串口通信应用中应该有所区别
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

//}


/// 配置UART接收中断
void UART_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
 /* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the UART4 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;	     
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //测试通过以后，在多串口通信应用中应该有所区别
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/**
  * @brief  USART1 TX DMA 配置，内存到外设(USART1->DR)
  * @param  无
  * @retval 无
  */
void USART3_DMA_Config(void)
{
		DMA_InitTypeDef DMA_InitStructure;
	
		/*开启DMA时钟*/
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
		//NVIC_Config();	   			//配置DMA中断
		/*设置DMA源：串口数据寄存器地址*/
		DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;	   
		/*内存地址(要传输的变量的指针)*/
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_USART3_RecevBuff;
		/*方向：从外设到内存*/		
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	    
//		/*方向：从内存到外设*/		
//		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	        
//		/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
//		DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;
	  /*传输大小DMA_BufferSize = RECEIVEBUFF_SIZE*/	
		DMA_InitStructure.DMA_BufferSize = RECEIVEBUFF_SIZE;
		/*外设地址不增*/	    
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
		/*内存地址自增*/
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
		/*外设数据单位*/	
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		/*内存数据单位 8bit*/
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
		/*DMA模式：不断循环*/
		//DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	    
//		/*优先级：中*/	
//		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
		/*优先级：非常高*/	
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;       //DMA_Priority_High
		/*禁止内存到内存的传输	*/
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		/*配置DMA1的3通道*/		   
		DMA_Init(DMA1_Channel3, &DMA_InitStructure); 	              //USART3接收通道为：DMA1_Channel3
		/*使能DMA*/
		DMA_Cmd (DMA1_Channel3,ENABLE);					
		//DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
		DMA_USART3_RecevIndicator.CurrentDataStartNum =0;  //初始化当前接收数据开始位置
		DMA_USART3_RecevIndicator.CurrentDataEndNum =0;    //初始化当前接收数据结束位置
		DMA_USART3_RecevIndicator.NextDataStartNum =0;     //初始化下一次接收数据开始位置
		
}





////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///**
//  * @brief  Configures the DMA.
//  * @param  None
//  * @retval None
//  */
//void DMA_Configuration(void)
//{
//  DMA_InitTypeDef DMA_InitStructure;

//  /* USART1_Tx_DMA_Channel (triggered by USART1 Tx event) Config */
//  DMA_DeInit(USART1_Tx_DMA_Channel);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBufferUart1;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//  DMA_InitStructure.DMA_BufferSize = 0;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(USART1_Tx_DMA_Channel, &DMA_InitStructure);
//        
//  /* USART1_Rx_DMA_Channel (triggered by USART1 Rx event) Config */
//  DMA_DeInit(USART1_Rx_DMA_Channel);
//  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBufferUart1;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//  DMA_InitStructure.DMA_BufferSize =64;//?????????????
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//  DMA_Init(USART1_Rx_DMA_Channel, &DMA_InitStructure);
//  DMA_Cmd(USART1_Rx_DMA_Channel, ENABLE);              
//        
//}




//void USART1_DMA_Config(void)  
//{  
//    DMA_InitTypeDef DMA_InitStructure;  

//        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//??DMA??  
//        DMA_InitStructure.DMA_Channel = DMA_Channel_4;//??4   
//        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART1_DR_Address; //?????[#define USART1_DR_Address    (0x40011000+0x04)]  
//        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Data_Buffer; //????  
//        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//?????  
//        DMA_InitStructure.DMA_BufferSize = 65535; //????  
//        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//??????  
//        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//?????  
//        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//??????1byte  
//        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//??????1byte  
//        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//????  
//        DMA_InitStructure.DMA_Priority = DMA_Priority_High;//????  
//        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;// ??fifo  
//        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //  
//        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//  
//        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//  
//        DMA_Init(DMA2_Stream5, &DMA_InitStructure);//???dma2?5  
//        DMA_Cmd(DMA2_Stream5, ENABLE);//??dma2?5  
//}  





/// 重定向c库函数printf到USART1
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到USART1 */
	 
		USART_SendData(USART1, (uint8_t) ch);
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		{
        ;
    }			
		return (ch);
}

/// 重定向c库函数scanf到USART1
int fgetc(FILE *f)
{
		/* 等待串口1输入数据 */
	  int  temp;
	  
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
		{
			;
    }

    temp=(int)USART_ReceiveData(USART1);
		return temp;
}
/*********************************************END OF FILE**********************/
