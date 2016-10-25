/**
  ******************************************************************************
  * @file    WatchDog.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   系统看门狗   
  ******************************************************************************
  * @attention
  *
  * 论坛    :http://www.chuxue123.com
  *
  ******************************************************************************
  */
  
#include "WatchDog.h"



//保存WWDG计数器的设置值,默认为最大. 
u8 WWDG_CNT=0X7F;
//初始化窗口看门狗 	
//tr   :T[6:0],计数器值 
//wr   :W[6:0],窗口值 
//fprer:分频系数（WDGTB）,仅最低2位有效 
//Fwwdg=PCLK1/(4096*2^fprer). PCLK1=36Mhz
//看门狗复位周期为计数器从0x7F减到0x40的时间，即(4096*2^fprer)*32/36*1000    (ms)

void WWDG_Init(u8 tr,u8 wr,u32 fprer)                 //看门狗周期：1.8s
{
 
	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE); //使能窗口看门狗时钟
	
	WWDG_CNT=tr&WWDG_CNT;   //初始化WWDG_CNT. 
	WWDG_SetPrescaler(fprer); //设置分频值
	WWDG_SetWindowValue(wr); //设置窗口值
//	WWDG_SetCounter(WWDG_CNT);//设置计数值
	WWDG_Enable(WWDG_CNT);  //开启看门狗
	
	NVIC_InitStructure.NVIC_IRQChannel=WWDG_IRQn;  //窗口看门狗中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;  //抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;					//子优先级为3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  //使能窗口看门狗
	NVIC_Init(&NVIC_InitStructure);
	
	WWDG_ClearFlag();//清除提前唤醒中断标志位
  WWDG_EnableIT();//开启提前唤醒中断
}


///**
//  * @brief  窗口看门狗设置初始化：WWDG_Config
//  * @param  无
//  * @retval 无
//  * 溢出时间计算：Tout=(1/(PCLK1/4096/WWDG_Prer))*rlr
//  */

//void WWDG_Config(void)
//{  
//  WWDG_SetPrescaler(WWDG_Prescaler_8);          //时钟8分频4ms,(PCLK1/4096)/8=244Hz
//  WWDG_SetWindowValue(65);                      //计数器数值（向下计数），溢出时间为：4ms*64=262ms
//  WWDG_Enable(127);                             //启动计数器，设置喂狗时间
//	WWDG_ClearFlag();                             //清除标志位
//	WWDG_EnableIT();                              //启动中断
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
//}
//void WWDG_NVIC_Configuration(void)
//{
//	NVIC_InitTypeDef NVIC_InitStructure; 
//    
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	NVIC_InitStructure.NVIC_IRQChannel = WWDG_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}

/**
  * @brief  独立看门狗设置初始化：IWDG_Config
  * @param  无
  * @retval 无
  * 溢出时间计算：Tout=((4*IWDG_Prer)*rlr)/40
  */
//void IWDG_Config(void)
//{ 

//   IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //启动寄存器读写
//   IWDG_SetPrescaler(IWDG_Prescaler_32);          //40K时钟32分频
//   IWDG_SetReload(1200);                           //计数器数值（向下计数），溢出时间为3840ms
//   IWDG_ReloadCounter();                          //重启计数器
//   IWDG_Enable();                                 //启动看门狗
//}

///**
//  * @brief  独立看门狗初始化：WatchDog_Init
//  * @param  prer-分频，reld-计数器重装载值（向下计数）
//  * @retval 无
//	* 溢出时间计算：Tout=((4*2^prer)*rlr)/40
//  */
//void WatchDog_Init(u8 prer, u16 reld)
//{
//   IWDG->KR=0x5555; //允许访问PR和RLR寄存器
//   IWDG->PR=prer;   //设置分频
//   IWDG->RLR=reld;  //设定计数器初值，向下计数
//   IWDG->KR=0xaaaa; //初次装初值
//   IWDG->KR=0xcccc; //启动看门狗定时器
//}

///**
//  * @brief  独立看门狗喂狗：WatchDog_Feed
//  * @param  无
//  * @retval 无
//  */
//void WatchDog_Feed(void)  //等价于:IWDG_ReloadCounter
//{
// IWDG->KR=0xaaaa;
//}

/*********************************************END OF FILE**********************/
