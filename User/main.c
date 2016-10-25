/**
  ******************************************************************************
  * @file    main.c
  * @author  casic 203
  * @version V1.0
  * @date    2015-07-27
  * @brief   
  * @attention
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "modbus.h"
#include "bsp_SysTick.h"
#include "string.h"
#include "gprs.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "AiderProtocol.h"
#include "SPI_Flash.h"
#include "common.h"
#include "DS2780.h" 
#include "433_Wiminet.h"
#include "bsp_adc.h"
#define snap 6


// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue;

// 局部变量，用于保存转换计算后的电压值 	 
//float ADC_ConvertedValueLocal;        

struct rtc_time    systmtime;        //RTC时钟设置结构体
struct liquid_set  DeviceConfig;     //液位计配置信息结构体
struct SMS_Config_RegPara   ConfigData;     //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
uint16_t  WWDOG_Feed =0x1FFF;        //窗口看门狗复位周期为：XX*1.8s = 7.6min
char      SetRev_OK =0;              //成功接收服务器配置
char      DatRev_OK =0;              //成功正确接收液位数据
char      Alive =0;                  //不休眠标志变量,为1时说明不进行休眠处理，有待调整
uint8_t   PowerOffReset =0;          //掉电重启标志位
uint8_t   LiquidDataSend_Flag =0;
uint8_t   SLNoiseConfigFlag = 0;
//uint8_t   Batch_Num =0x01;                     //数据批次序号
//uint8_t   Batch_Sum =0x01;                     //数据批次总数
//uint8_t   DataCollectBkCount =0;               //备份数据采集计数器
//uint8_t   DataCollectCache[13][4]={'\0'};      //液位数据采集缓存，最多13组数据；浮点数HEX格式存储，低字节在前，高字节在后
float     LevelData_Float[FILTER_ORDER]={0.0};//采集到的临时液位数据，浮点型
extern    uint8_t  LevelDataCount ;              //液位数据计数器，标示当前采集到数据数量
uint8_t   DataCollectCount =1;                 //数据采集计数器
char      Usart1_recev_buff[300] ={'\0'};      //USART1接收缓存
uint16_t  Usart1_recev_count =0;               //USART1发送计数器
char      Usart2_recev_buff[4]={'\0'};        //USART2接收缓存
unsigned char 			Dense_Data[60] = {'\0'};
unsigned char 			Noise_Data[60] = {'\0'};
char      SLNoise_char[300]={'\0'};           //存放转换后的char格式数据
int       Length_Frame = 0;
uint8_t   Usart2_recev_count =0;               //USART2接收计数器
uint8_t   DMA_UART3_RECEV_FLAG =0;             //USART3 DMA接收标志变量
uint16_t  Noise_Count=0x0000;   
//uint16_t  BATT_Capacity =19000;                //一组电池当前剩余电量（满容量19Ah）

uint16_t  SampleTime = 60;                       //采集时间
uint16_t  SampleSpan = 60;                       //采集间隔
char			Dense_Num = 20;                        //采集次数
char			UploadCount = 1;                       //上传重试次数
uint16_t  WakeTime = 60;                       //采集时间
uint16_t  WakeSecond = 0;                       //采集时间

char DeviceId[6] = {0x21,0x20,0x16,0x09,0x99,0x01};   //噪声记录仪设备编号
u16 NodeAddr =0x0000;
char 			BkpCount;
char      NoiseSensorFlag=5;

extern  char      Usart3_recev_buff[1000];
extern  uint16_t  Usart3_recev_count;

extern  char      Usart4_recev_buff[1000];
extern  uint16_t  Usart4_recev_count;

extern  uint8_t   CSQ_OK ;                  //信号质量标志变量
extern  uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern  struct    DMA_USART3_RecevConfig   DMA_USART3_RecevIndicator; 
extern  int NetErrorCount;
SLNoiseSet Msg;
SLNoiseSet* pMsg = &Msg;
uint32_t  time=0 ;                   // ms 计时变量  
char      Usart1_send_buff[300]={'\0'};       //USART1发送缓存
uint8_t   Usart1_send_count=0;                 //USART1发送计数器
uint32_t  Tic_IWDG=0;                //独立看门狗喂狗时间设置
extern  char  Usart3_send_buff[];
extern  uint8_t  Usart3_send_count;                     
//extern  struct SMS_Config_RegPara   ConfigData;  //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
//extern  float  Data_Liquid_Level;
//extern void     Receive_Analysis_GPRS(void);	
//extern char     Receive_Monitor_GPRS(void);
//extern void     Receive_Deal_GPRS(void);
//extern uint8_t  NetStatus_Detection( void );
extern void     Delay(uint32_t nCount);
extern void     LSLIQUID_DataCollect(void);          
extern void     RecvBuffInit_USART3(void);

unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag);     //USART3接收数据监测与数据解析
void  PeripheralInit( void );          //初始化外围设备
void  LevelDataMessageSend(void);      // 通过短信发送液位数据
char Usart2_recev_count1 = 0;
char  Receive_Monitor_433(void);
char receive_flag = 0;

int i;
int j;

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void  RecvBuffInit_USART3(void)
//{
//  
//     memset(Usart3_recev_buff,'\0',500);	
//		 Usart3_recev_count=0;                           //清空USART3接收计数器
//		 time=0;	                                       //定时器复位
//   
//}
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : 从DMA接收存储器中提取有效数据，放入Usart3_recev_buff[],便于后续数据解析
* Input          : None
* Output         : None
* Return         : 接收数据长度
*******************************************************************************/
int  DMA_UART3_RecevDataGet(void)
{
   int i=0,j=0;
	 uint8_t VoidCount=5;
	 uint8_t FilterDepth=10;   //过滤深度为10
	
  
   printf("\r\nDMA_UART3_RecevDataGet start!!%d---%d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //测试使用
   memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));
	 DMA_USART3_RecevIndicator.CurrentDataStartNum =DMA_USART3_RecevIndicator.NextDataStartNum ;
	 i = DMA_USART3_RecevIndicator.CurrentDataStartNum;
	 if(i>=RECEIVEBUFF_SIZE)
	 {
     printf("\r\nCurrent Data Start Number Error!! \r\n");    //测试使用
		 DMA_USART3_RecevIndicator.NextDataStartNum =0 ;   //发生错误时复位
		 return 0;
   }
	 if(DMA_USART3_RecevBuff[i]==0x00)   
	 {
     while(FilterDepth!=0)   //滤除起始的空字节，过滤深度为10
		 {
       FilterDepth--;
			 if(i< RECEIVEBUFF_SIZE-1)
			 {
					i++;
			 }
			 else   
			 {
					i=0;      //检索到DMA数据存储器的最底端依然未出现无效数据，则闭环回到存储器顶端
			 }
			 if(DMA_USART3_RecevBuff[i]!=0x00) 
			 {
          break;
       }
     }
   }		
   if(DMA_USART3_RecevBuff[i]!=0x00)    //开始字节非空，表示接收数据有效
	 {
     while(DMA_USART3_RecevBuff[i]!=0x00)
		 {
			 while(DMA_USART3_RecevBuff[i]!=0x00)
			 {
				 if(i< RECEIVEBUFF_SIZE-1)
				 {
					 i++;
				 }
				 else   
				 {
					 i=0;      // 检索到DMA数据存储器的最底端依然未出现无效数据，则闭环回到存储器顶端
				 }
			 }
			 if(i==0)
			 {
          DMA_USART3_RecevIndicator.CurrentDataEndNum = RECEIVEBUFF_SIZE-1;
       }
			 else
			 {
          DMA_USART3_RecevIndicator.CurrentDataEndNum =i-1;  // DMA_USART3_RecevBuff[i]== 0x00, 标示结束位置
       }
			 while(VoidCount !=0)
			 {
				 VoidCount--;
				 if(i< RECEIVEBUFF_SIZE-1)
				 {
					 i++;
				 }
				 else   
				 {
					 i=0;      //检索到DMA数据存储器的最底端依然未出现无效数据，则闭环回到存储器顶端
				 }
				 if(DMA_USART3_RecevBuff[i]!=0x00)
				 {
					 VoidCount =5;      //不是接收序列结尾，重新置位
					 break;
				 }
			 }
			 if(VoidCount==0)     //检测到接收序列的结尾
			 {
				 if(DMA_USART3_RecevIndicator.CurrentDataEndNum ==RECEIVEBUFF_SIZE-1)
				 {
					 DMA_USART3_RecevIndicator.NextDataStartNum = 0;
         }
				 else
				 {
					  DMA_USART3_RecevIndicator.NextDataStartNum = DMA_USART3_RecevIndicator.CurrentDataEndNum + 1;
         }	
				 break;
			 }
     } 
///////////////////////////////////////找到数据起始与结束位置以后，将数据复制到指定数组，便于后续分析/////////////////////////////////////////
		 if(DMA_USART3_RecevIndicator.CurrentDataEndNum > DMA_USART3_RecevIndicator.CurrentDataStartNum)
		 {
        for(i=DMA_USART3_RecevIndicator.CurrentDataStartNum,j=0;i<=DMA_USART3_RecevIndicator.CurrentDataEndNum;i++,j++)	
        {
          Usart3_recev_buff[j] =DMA_USART3_RecevBuff[i] ;			
        }			
         	
     }
		 else if(DMA_USART3_RecevIndicator.CurrentDataEndNum < DMA_USART3_RecevIndicator.CurrentDataStartNum)
		 {
        for(i=DMA_USART3_RecevIndicator.CurrentDataStartNum,j=0;i<=(RECEIVEBUFF_SIZE-1);i++,j++)	
        {
          Usart3_recev_buff[j] =DMA_USART3_RecevBuff[i] ;
        }	
        for(i=0;i<=DMA_USART3_RecevIndicator.CurrentDataEndNum ;i++,j++)	
        {
          Usart3_recev_buff[j] =DMA_USART3_RecevBuff[i] ;
        }	
     }
		 printf("\r\nRecev OKK:%d\r\n",j);    //测试使用			
     return j;
   }
	 else
	 {
      printf("\r\nFirst Void\r\n");    //测试使用	 
   }
   return 0;
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag)
{
  
	int DataLength =0;
	int i=0;
	unsigned char  StateFlag =0;

	memset(DMA_USART3_RecevBuff, 0x00, RECEIVEBUFF_SIZE);
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //配置串口DMA1接收
	
  if(DMA_UART3_RECEV_FLAG ==1)
  {
		 DataLength = DMA_UART3_RecevDataGet();
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);           //测试使用
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);         //测试使用
			  for(i=0;i<DataLength;i++)
			  {
             printf("%x ", Usart3_recev_buff[i]);               //测试使用
        }
				if(RecevFlag == NETLOGIN)
				{
           StateFlag = GPRS_Receive_NetLogin();      //
        }
				else if(RecevFlag == TCPCONNECT)
				{
           StateFlag = GPRS_Receive_TcpConnect();    //
        }	
        else if(RecevFlag == DATARECEV)
				{
					 StateFlag = GPRS_Receive_DataAnalysis();  //
        }	 				
		 }
		 else
		 {
        printf("\r\nNo data\r\n");
     }
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //复位DMA数据接收BUFF
		 
		 USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);  //配置串口DMA1接收

     DMA_UART3_RECEV_FLAG =0;
		 
  }
	
	return StateFlag;
}


/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : 初始化端口及外设
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void PeripheralInit( void )
{  
		
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//首先开启窗口看门狗，计数器值为7f,窗口寄存器为5f,分频数为8	
	USART1_Config();      /* USART1 配置模式为 9600 8-N-1，  中断接收 */
	USART2_Config();      /* USART2 配置模式为 115200 8-N-1，中断接收 */
	USART3_Config();      /* USART3 配置模式为 115200 8-N-1，中断接收 */
 	USART4_Config();      /* UART4  配置模式为 9600 8-N-1，  中断接收 */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();	
	
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //配置串口DMA1接收
  TIM2_Configuration();       /* 定时器TIM2参数配置 */	
	TIM2_NVIC_Configuration();  /* 设置定时器TIM2的中断优先级 */
//	TIM3_Configuration();       /* 定时器TIM3参数配置 */	
//	TIM3_NVIC_Configuration();  /* 设置定时器TIM3的中断优先级 */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //暂时关闭定时器TIM3
	
	RTC_NVIC_Config();                 /* 配置RTC秒中断优先级 */
//	RTC_Configuration();       //
	RTC_CheckAndConfig(&systmtime);
 	Time_Show(&systmtime);             /* Display time in infinite loop */
  SysTick_Init();
//	SPI_FLASH_Init();
//	PowerON_GPRS();                    //打开GPRS模块电源
	PowerON_Flash(); 
//	Delay_ms(1000);  

	///////////////////必须将模块配置成接收状态，否则串口接收不到数据
	PowerON_433_SET();                      //433模块SET管脚拉高，切换到接收模式
	Delay_ms(100);
  PowerOFF_433_EN();                      //433模块EN管脚拉低，切换到高速发送模式
	Delay_ms(100);

	ConfigData_Init(&DeviceConfig);
	//BatteryMonitorInit();              //库仑计初始化	
		
//  Delay_ms(1000);
//	StartupRequest(2);                  //设备唤醒回复
//	Delay_ms(5000);

	if (PowerOffReset ==1)     
  {		
		printf("\r\n掉电重启，重新初始化库仑计\r\n");                 //测试使用
	 	Set_register_ds2780();    //掉电后对库仑计重新初始化
	  set_ACR(1000);           //掉电后对库仑计重新初始化
	  DS2780_CapacityInit();    //掉电后重新写电池容量
		DS2780_Test();
		SetNodeID( NodeAddr );    //设置节点ID号，从产品Device编号中提取
    
//		StartupRequest(PowerOffReset); //设备重启请求
		Delay_ms(1500);
  }	
}

/*******************************************************************************
* Function Name  : int main(void)
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main( void )
{
		
	#ifdef DEBUG
  debug();
  #endif

	NodeAddr =DeviceId[4]*256 +DeviceId[5];     //提取设备ID号最后面两个字节作为节点地址
		
	PeripheralInit();
	
	printf("\r\n.....!!\r\n");
	
	PowerON_NoiseSensor();
//  while(1)
//	{}
	
	/* enable adc1 and config adc1 to dma mode */
//	ADC1_Init();	
	
	if (SLNoiseConfigFlag)
	{		
		SampleTime = BKP_ReadBackupRegister(BKP_DR2);
		SampleSpan = BKP_ReadBackupRegister(BKP_DR3);
		Dense_Num = BKP_ReadBackupRegister(BKP_DR4);
		UploadCount = BKP_ReadBackupRegister(BKP_DR5);
		WakeSecond = DeviceId[5]-DeviceId[5]/16*6;
    WakeTime = SampleTime + DeviceId[5]-DeviceId[5]/16*6;		
		
//		UploadCount = 10;
//		Dense_Num = 20; 
		//SLNoiseConfigFlag = 0;
		printf("配置更新。。");
//		printf("\r\n The current BKP_ReadBackupRegister(BKP_DR4) value = %d \r\n",BKP_ReadBackupRegister(BKP_DR4));
	}	
  else
	{
    printf("配置未更新。。");
		SampleTime = 60;
		SampleSpan = 1;	 
		Dense_Num = 20;
    UploadCount = 3;		
	}
	while(1)
  {
		  WWDOG_Feed =0x1FFF;                          //窗口看门狗喂狗,定时4分20秒，第二字节约1秒变化一次，即0x09AF变到0x099F约耗时1秒
		  Usart2_recev_count1 = 0;	
		
		  for(i=0;i<10;i++)
		  {
			   Delay_ms(500);
				 printf("\r\n The current i value =%d \r\n",i);
         receive_flag = Receive_Monitor_433();				
			   if(receive_flag == 1)                      //查询数据接收情况
			   {
				   printf("\r\nUart4串口接收信息输出:");
		       for(j;j<Usart4_recev_count;j++)
		       {
			       printf(" %x",Usart4_recev_buff[j]);
		       }

					 GPRS_Receive_DataAnalysis();                                 //!!!设备唤醒以后第一次发送数据之前必须进行接收解析，不能删除，必须保留，否则容易出错!!!
					 memset(Usart4_recev_buff,'\0',300);	
					 Usart4_recev_count =0;                                       //清空USART4接收计数器
					 time=0;	                                                     //定时器复位    
			   }
      }
			
			Delay_ms(1000);				
			SLNoise_DataUpload(&DeviceConfig, 1); //数据上传
			Delay_ms(1500);

			printf("\r\n??????????????????????");
			Delay_ms(5000);         //等待短信到达
			gotoSleep(0);		 //长休眠
		
//			printf("\r\nPowerON!\r\n");
//			Delay_ms(5000);
//																		
//			ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*3.3; // 读取转换的AD值
//			printf("\r\n The current AD value = 0x%04X \r\n", ADC_ConvertedValue); 
//      printf("\r\n The current AD value = %f V \r\n", ADC_ConvertedValueLocal);
//							
//			Usart2_recev_buff[0] = ADC_ConvertedValue/100;// 将采样的噪声值赋给串口
//			
//			Noise_Data[Noise_Count] = Usart2_recev_buff[0];
//			
//			Noise_Count++;	
//			printf("\r\n %d \r\n",Usart2_recev_count);
//			printf("\r\n %x \r\n",Usart2_recev_buff[0]);
//			printf("\r\n %x \r\n",Usart2_recev_buff[1]);
//			printf("\r\n %x \r\n",Usart2_recev_buff[2]);
//			printf("\r\n %x \r\n",Usart2_recev_buff[3]);
//			printf("Noise_Count: %d \r\n",Noise_Count);
//																						
//			if (Noise_Count >= Dense_Num)//Dense_Num
//			{
//				printf("\r\n The current Dense_Num value = %d \r\n",Dense_Num);
//				Noise_Data[Dense_Num-1] = Usart2_recev_buff[0];
////				Dense_Data[Dense_Num*2-2] = Usart2_recev_buff[0];
//				//Noise_Count = 0;
//				Usart2_recev_count = 0;	
//				Delay_ms(500);	//NOP();
//				
////				PowerON_GPRS();
//				
//        Delay_ms(1000);				
//				SLNoise_DataUpload(&DeviceConfig); //数据上传
//				Delay_ms(1500);

//				printf("\r\n??????????????????????");
//				Delay_ms(5000);         //等待短信到达
//				gotoSleep(0);		 //长休眠
//			}
//			gotoSleep(1);	//短休眠
 	}
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number */
 
  printf("\n\r Wrong parameter value detected on\r\n");
  printf("       file  %s\r\n", file);
  printf("       line  %d\r\n", line);
    
  /* Infinite loop */
  /* while (1)
  {
  } */
}
#endif
/*********************************************END OF FILE**********************/

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
char  Receive_Monitor_433(void)
{
    char* pRecevBuff =NULL;  //测试使用
	  u8 i=0;                               //测试使用
		if(Usart4_recev_count!=0)
		{
				USART_ClearITPendingBit(UART4, USART_FLAG_RXNE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);    /* 关闭TIM2  */
			
				#if DEBUG_TEST
				printf("\r\nUART4:");               //测试使用
				for(i=0;i<50;i++)
				{
					printf(" %x ",Usart4_recev_buff[i]);      //测试使用
				}
				pRecevBuff =&Usart4_recev_buff[i];    //测试使用
				printf("%s\r\n",pRecevBuff);          //测试使用 
				#endif
				
				return 1;
		}	
		return 0;
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Receive_Deal_433(void)	
{
		 unsigned char     Recev_Flag2 =0;     //接收数据正确性标志变量
		 char*  pRecevBuff =NULL;
     char*  ptemp =NULL;
	   int   count =0;
		 char  Recev_OK[2]={0xAA,0x1E};       //433模块接收指示

		 pRecevBuff = Find_SpecialString(Usart4_recev_buff,Recev_OK,300,2);  //检查有无收到主站回复
		 if(pRecevBuff != NULL)
		 {	
			 
start:	   
 			  ptemp =pRecevBuff;
			  count =pRecevBuff -Usart4_recev_buff+2;            //下一次查找包头的范围
			  Recev_Flag2 = ReceiveMessageVerify( pRecevBuff );
				if(Recev_Flag2==0)                                 //当前查找到的接收数据有误
				{
						pRecevBuff =NULL;
						pRecevBuff = Find_SpecialString(ptemp-2,Recev_OK,(300-count),2);  //检查有无收到主站回复
						if(pRecevBuff !=NULL)
						{
							#if DEBUG_TEST	 
							printf("\r\n第二次查找，当前查找位置: %d\r\n",count);   //测试使用
						  #endif
							goto start;
						}
						else           //查找不到接收包头，直接丢弃数据
						{
							#if DEBUG_TEST	 
							printf("\r\nReceive  data not correct!!\r\n");   //测试使用
							#endif
							memset(Usart4_recev_buff,'\0',300);	
							Usart4_recev_count=0;                            //清空USART3接收计数器
							time=0;	                                         //定时器复位
						}
			   }
				 else           //接收数据正确
			 	 {
//				  	Receive_Analysis_433();            //解析接收数据
//					  GPRS_Receive_DataAnalysis();
//					  DMA_UART3_RecevDetect(DATARECEV);	                    //接收数据解析处理
				    memset(Usart4_recev_buff,'\0',300);	
			      Usart4_recev_count=0;                          //清空USART3接收计数器
		        time=0;	                                       //定时器复位
			      Delay_ms(1000);	
			   }
		  }		
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetRequest_433(void)	
{
//	char  Section_Req[53]="SLRESET:FF,02,0E,51,212015090053,212015090053,030D0A";		//设备重启请求
	char  section_GPRS[52] ="SLGPRS:FF,02,0E,61,212015090053,212015090053,030D0A";
	unsigned char     Recev_Flag1=0;     //USART3接收数据标志变量
	u8    SetReq_Count =3;               //请求下发配置尝试次数，暂时设为最多尝试三次
	u8    NetStatusFlag =1;                //网络状态标志变量

	SetRev_OK = 0;
	while(SetReq_Count !=0)
	{
		SetReq_Count--;
//		SendMessage(Section_Req,strlen(section_GPRS));
		NetStatusFlag = SendMessage(section_GPRS,strlen(section_GPRS));
		if(SetRev_OK ==1)         //成功接收配置，退出循环
		{
			SetRev_OK = 0;
			break;
    }
		Delay_ms(10000);	
		Recev_Flag1 = Receive_Monitor_433();
		if(Recev_Flag1==1)                                    //USART3接收到数据		
		{
			Receive_Deal_433();
			Recev_Flag1 =0;
			#if DEBUG_TEST	 
			printf("\r\n第二次接收查到数据!!\r\n");   //测试使用
			#endif
		}  		
	}
	
	if(NetStatusFlag==0)                         //网络出现异常时，设备进入休眠
	{
      gotoSleep(1);
  }		
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Receive(void)
{
		  Usart2_recev_count1 = 0;	
		
		  for(i=0;i<10;i++)
		  {
			   Delay_ms(500);
				 printf("\r\n The current i value =%d \r\n",i);
         receive_flag = Receive_Monitor_433();				
			   if(receive_flag == 1)                      //查询数据接收情况
			   {
				   printf("\r\nUart4串口接收信息输出:");
		       for(j=0;j<Usart4_recev_count;j++)
		       {
			       printf(" %x",Usart4_recev_buff[j]);
		       }

					 GPRS_Receive_DataAnalysis();                                 //!!!设备唤醒以后第一次发送数据之前必须进行接收解析，不能删除，必须保留，否则容易出错!!!
					 memset(Usart4_recev_buff,'\0',300);	
					 Usart4_recev_count =0;                                       //清空USART4接收计数器
					 time=0;	                                                     //定时器复位    
			   }
      }
}