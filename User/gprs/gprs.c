
// File Name: gprs.c
#include "string.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "AiderProtocol.h"
#include "bsp_rtc.h"
#include "433_Wiminet.h"
#include "common.h"
#include "math.h"
#include "SPI_Flash.h"

//char  Receive_Monitor_GPRS(void); //3G模块串口监听，当MCU检测到3G模块串口的数据时
//void  Receive_Deal_GPRS(void);    //3G模块接收数据解析，主要用于处理MCU与3G模块之间的交互数据
//void  Receive_Analysis_GPRS(void);//3G模块接收服务器数据解析，主要处理服务器下发的基于埃德尔协议的数据
//u8    NetStatus_Detection( void );//3G模块网络连接状态检测

char      Usart3_recev_buff[1000]={'\0'};     //USART3接收缓存
uint16_t  Usart3_recev_count=0;              //USART3接收计数器
char      Usart4_recev_buff[1000]={'\0'};     //USART3接收缓存
uint16_t  Usart4_recev_count=0;              //USART3接收计数器

uint8_t   CSQ_OK =0;                         //信号质量标志变量
//uint8_t  TCP_Connect_Flag =0;                //TCP连接标志变量
//uint8_t  TCP_Connect_Start=0;                //TCP连接启动标志变量
extern struct    SMS_Config_RegPara   ConfigData;     //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
//static char      RespRev_OK =0;              //成功接收服务器应答
//static uint8_t   HandFlag =1;
//static uint8_t   NetErrorCount =NETERRORCOUNT;           //网络异常时尝式建立连接的最大次数
//extern uint32_t  time ;                    //USART3接收定时器（ms）  
extern struct    liquid_set  DeviceConfig;   //液位计配置信息结构体
extern char      SetRev_OK;                  //成功接收服务器配置
extern char      Alive;                      //不休眠标志变量,为1时说明不进行休眠处理，有待调整
extern char      DatRev_OK ;                 //成功正确接收液位数据
extern uint8_t   DataCollectCount;           //数据采集计数器
extern uint8_t   LiquidDataSend_Flag;
extern uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern uint8_t   DMA_UART3_RECEV_FLAG ;             //USART3 DMA接收标志变量
extern char SLNoise_char[300];
extern int   Length_Frame; 

extern char  UploadCount;    //数据上传最大尝试次数
extern char DeviceId[6];     //噪声记录仪设备编号

//extern char NoiseSensorFlag;
//uint8_t  Usart3_send_buff[500]={'\0'};     //USART3发送缓存
//uint8_t   Usart3_send_count=0;             //USART3发送计数器

//extern char  Receive_Monitor_GPRS(void);
//extern void  Receive_Deal_GPRS(void);
//extern void  Receive_Analysis_GPRS(void);	
//extern void  RecvBuffInit_USART3(void);
extern  unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag);     //USART3接收数据监测与数据解析
extern  void      LevelDataMessageSend(void);      // 通过短信发送液位数据
extern  char     PowerOffReset ;
extern  uint16_t  Noise_Count;

extern int char_hextochar(char* dealbuf,unsigned char* databuf, u8 length);

void  SIM5216_PowerOn(void)            //打开SIM5216模块
{
   GPIO_SetBits(GPIOC,GPIO_Pin_3);	   //POWER_ON引脚拉低
   Delay_ms(120);                      //100ms延时     64ms<Ton<180ms
   GPIO_ResetBits(GPIOC,GPIO_Pin_3);   //POWER_ON引脚拉高
   Delay_ms(5500);                     //5s延时        Tuart>4.7s           
	
}

void  SIM5216_PowerOff(void)            //关闭SIM5216模块
{
   GPIO_SetBits(GPIOC,GPIO_Pin_3);	    //POWER_ON引脚拉低
   Delay_ms(2000);                      //1s延时       500ms<Ton<5s
   GPIO_ResetBits(GPIOC,GPIO_Pin_3);    //POWER_ON引脚拉高
   Delay_ms(6000);                             
}

void USART_DataBlock_Send(USART_TypeDef* USART_PORT,char* SendUartBuf,u16 SendLength)    //批量向串口发送数据
{
    u16 i;
        
    for(i=0;i<SendLength;i++)
    {
        USART_SendData(USART_PORT, *(SendUartBuf+i));
        while (USART_GetFlagStatus(USART_PORT, USART_FLAG_TC) == RESET);
    } 
}
void mput_mix(char *str,int length)
{
	printf("length:%d\r\n",length);             //测试使用
//	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);   //向USART3发送数据前，先打开USART3接收空闲中断，便于监测数据接收完成
	
	PowerOFF_433_SET();                              //433模块SET脚拉低，方便接收
	Delay_ms(100);
	PowerOFF_433_EN();                               //433模块EN脚拉低，方便接收
	Delay_ms(100);
	
	USART_GetFlagStatus(UART4, USART_FLAG_TC);          //串口硬件复位之后，发送首字节之前，先读一下USART_SR，防止数据发送时首字节被覆盖
	Delay_ms(500);
	USART_DataBlock_Send(UART4,str,length);
	USART_DataBlock_Send(USART1,str,length);
	USART_DataBlock_Send(USART1,"\r\n",2);
	
	Delay_ms(2000);
	PowerON_433_SET();                    //433模块SET管脚拉高，切换到接收模式
	Delay_ms(2000);
}

void mput(char* str)
{
//	printf("length:%d\r\n",strlen(str));     //测试使用
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //向USART3发送数据前，先打开USART3接收空闲中断，便于监测数据接收完成
	USART_DataBlock_Send(USART3,str,strlen(str));
	USART_DataBlock_Send(USART3,"\r\n",2);
	USART_DataBlock_Send(USART1,str,strlen(str));
	USART_DataBlock_Send(USART1,"\r\n",2);
}
/*******************************************************************************
* Function Name  : void GPRS_Config(void)
* Description    : GPRS
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPRS_Config(void)
{
	 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);
   USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //配置串口DMA1接收
   mput("AT^UARTRPT=1");  //设置通信接口为UART
	 Delay_ms(200);      
	 mput("AT+CMGF=1");     //短信格式设置为文本模式
	 Delay_ms(300); 
	 mput("AT+CPMS=\"ME\",\"ME\",\"ME\"");     //设置发送和接收短信存储器为ME消息存储器，否则使用AT+CMGL="REC UNREAD"命令查不到短信
	 Delay_ms(300); 
   mput("AT+CNMI=1,1");   //新消息指示设置为存储并送通知
	 Delay_ms(200);
//	 if(DMA_UART3_RECEV_FLAG==1)
//	 {
//	   DMA_UART3_RecevDetect();	                    //接收数据解析处理
//	 }
//	 mput("AT+CMGD=1,3");   //新消息指示设置为存储并送通知
//	 Delay_ms(500);
//	 mput("AT+CPMS?");   //新消息指示设置为存储并送通知
//	 Delay_ms(200);
}
/*******************************************************************************
* Function Name  : void GPRS_Init(void)
* Description    : GPRS
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPRS_Init(void)
{
  u8   NetSearchCount=6;           //搜寻网络最大尝试次数
	u8   CSQ_DetectCount=3;           //接收信号强度监测次数，用以稳定网络连接
//	char SendArry1[80] ={'\0'};
//  u8   i=0;
	
  memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //配置串口DMA1接收
	Delay_ms(3000);
	while(NetSearchCount!=0)
	{
	   NetSearchCount--;
		 printf("\r\nGPRS Net Searching...\r\n-------------NetSearchCount:%d\r\n",NetSearchCount);    //测试使用
		 mput("AT+CGREG=1");   //完成信号自动收索，当3G信号很弱时自动切换到3G信号
		 Delay_ms(800);
		 mput("AT+CGREG?");   //完成信号自动收索，当3G信号很弱时自动切换到3G信号
		 Delay_ms(800);
		 DMA_UART3_RecevDetect( NETLOGIN );
		 if(CSQ_OK ==1) 
		 {
        CSQ_OK =0;
				CSQ_DetectCount--;
				if(CSQ_DetectCount==0)
				{
           break;
        } 
     }  
  }
	if(NetSearchCount==0)
	{
     gotoSleep(0);      	//搜不到网络进入
  }
}
/*******************************************************************************
* Function Name  : void TCP_Connect(void)
* Description    : GPRS
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TCP_Connect(void)
{
    char SendArry[50] ={'\0'};
    unsigned char  TcpConnectFlag =0;  //TCP连接建立标志变量
    unsigned char  NetErrorCount =6;   //TCP建立连接尝试次数，下一版短信可配置


	  printf(" Multi GPRS Start!\r\n");
//  GPRS_Init();

     while(NetErrorCount>0)
		{
				 
			 mput("at+mipcall=1");
			 Delay_ms(800);		
			 snprintf(SendArry,sizeof(SendArry),"at+miptrans=1,\"%s\",%s",DeviceConfig.ServerIP,DeviceConfig.ServerPort);  // mput("at+miptrans=1,\"58.210.41.202\",2015");   //后续需要从寄存器中读取，有待进一步完善
       mput(SendArry);
			 Delay_ms(5000); 
						 TcpConnectFlag =DMA_UART3_RecevDetect( TCPCONNECT );    //对GPRS模块接收到的数据进行解析
						 NetErrorCount--;
				     if(TcpConnectFlag==1) 
						 {
                break;
             }
			 }
			 if(NetErrorCount == 0)
			 {
         GPRS_Init();
				 Delay_ms(6000);  
				 Sms_Consult();  
				 gotoSleep(0);     //多次尝试未连接到网络，直接进入休眠模式，用于监测模块唤醒以后第一次入网状态
       } 
		
}
/*******************************************************************************
* Function Name  : void TCP_Disconnect(void)
* Description    : GPRS
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TCP_Disconnect(void)
{
  mput("+++");
	Delay_ms(500);
//  TCP_Connect_Flag =0;    //关闭循环联网
}
/*******************************************************************************
* Function Name  : char* Find_String(char* Source, char* Object)
* Description    : 在目标字符串中发现一个指定的字符串
* Input          : 
* Output         : 
* Return         : 如果找到，则返回目标字符串在源字符串中的首地址
*******************************************************************************/
char* Find_String(char* Source, char* Object)
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   Length_Source =0;
	short   Length_Object =0;
	short   i=0,j=0;
	short   count=0;
	
	
	Length_Source = strlen(Source);
	Length_Object = strlen(Object);

	if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 return  &Ptemp1[i]; 
				 
        }
				return NULL;
     }	  
  }
}
/***********函数功能：在特定序列中发现一个指定的序列****************/
/***********如果找到，则返回目标序列在源序列中的首地址**************/
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object)  
{
	char*   Ptemp1 = Source;
	char*   Ptemp2 = Object;
	short   i=0,j=0;
	short   count=0;
	
	if((Length_Source < 0)||(Length_Object < 0))
	{
     return NULL;
  }
  if(Length_Source < Length_Object)
	{
     return NULL;
  }
  
	else if(Length_Source == Length_Object)
	{
		 if((Length_Source==0)&&(Length_Object==0))
		 {
			  return NULL;
     }  
		 else  
		 { 
			  for(i=0;i<Length_Source;i++)
		    {
				   if(Ptemp1[i] != Ptemp2[i])
					 return NULL;
        }
				return Ptemp1;
     }	  
  }
	else 
	{
		 if(Length_Object == 0)
		 {
			  return NULL;
     }  
		 else  
		 {  
			  count = Length_Source - Length_Object + 1;
			  for(i=0;i<count;i++)
		    {  for(j=0;j<Length_Object;j++)
					 {
               if(Ptemp1[i+j] != Ptemp2[j])
						   break;
           }
					 if(j==Length_Object)
					 {
							return  &Ptemp1[i]; 
					 }
				 
        }
				return NULL;
     }	  
  }
}
/********************函数功能：短信发送前预处理*********************/
/*************即在需要发送的数据最后增加一个结束符0x1a**************/
void SendPretreatment(char* pSend)
{  
   uint8_t  i=0;
	 char*    pTemp =NULL;
	
	 i= strlen(pSend);
	 pTemp =pSend+i;
   *(pTemp) =0x1a;
}

/***********************函数功能：发送短信**************************/
/*******************************************************************/
//发送短信流程：
//第一步：设置接收短信提醒格式：AT+CNMI=1,2,0,0,0    
//第二步：设定短信接收方号码：AT+CMGS="15116924685"
//第三步：发送短信正文，并以16进制0x1a作为结尾
void Sms_Send(char*  pSend)
{ 
//	uint8_t  PhoneNum[13]={0x00};    //后续更改为从寄存器中读取
	char  SendBuf[200]={0x00};                //短信发送缓存,根据最大可能的发送数据长度确定其大小

//	struct liquid_set* Para= &DeviceConfig;
	
//	memcpy(PhoneNum, DeviceConfig.AlarmPhoneNum,strlen((char*)DeviceConfig.AlarmPhoneNum));          
//	mput("AT+CNMI=1,2,0,0,0");
	
  memset(SendBuf,0,sizeof(SendBuf));
	snprintf(SendBuf,sizeof(SendBuf),"AT+CMGS=\"%s\"",ConfigData.CurrentPhoneNum); //发送短信号码为当前通信手机号码
	mput(SendBuf);
//	mput("AT+CMGS=\"861064617006426\"");  //测试使用，有待完善

	Delay_ms(500);
  memset(SendBuf,0,sizeof(SendBuf));
	snprintf(SendBuf, (sizeof(SendBuf)-1), pSend, strlen(pSend));
  SendPretreatment(SendBuf);
	mput(SendBuf);
  Delay_ms(1000);  
//	DMA_UART3_RecevDetect();  //等待接收信息

}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t Char2Hex(uint8_t* HexArry, char* CharArry, uint8_t Length_CharArry)
{
	uint8_t   i=0,j=0;         
	uint8_t   val=0;     
//	char*     pChar = CharArry;
	
  for(i=0; i<Length_CharArry; i++)
	{
		
//		val = *pChar;
		val = CharArry[i];
		if((val >= '0')&&(val <= '9'))
		{
			HexArry[j++] = val-'0';	
		}
		else if(val == '.')
		{
			HexArry[j++] = val;	
		}
//		pChar++;
	}	
  return  j;	
}



/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t  UploadFlash(char* pSetPara, uint8_t  InstructCode)
{
//  uint8_t   HexData[16] ={0x00}; 
//  char      CharTemp[16]={0x00};
//  uint16_t  i =0;
  uint8_t   j=0;
  uint8_t   Counter=0;
//  uint16_t  IntegerPart =0x0000;        //整数部分
//  uint16_t  DecimalPart =0x0000;        //小数部分
//  uint8_t   DataLength =0;  //用于指示有效数据转后的数据长度
//  uint8_t*  pData =NULL;
//  uint8_t*  pDot[3] ={NULL};
//  uint8_t*  pDot1 =NULL;
//  uint8_t*  pTemp =NULL;
//  float     AlarmData_F=0;
	
	switch(InstructCode)
	{
//		case 1:            //存储报警号码
//		{
//			 
//			 DataWrite_To_Flash(0,4,0,(uint8_t*)pSetPara,strlen(pSetPara));   //将报警号码写入Flash 
//       DeviceConfig.MessageSetFlag =1;                                  //标示当前液位仪参数通过短信修改		 
//       printf("\r\n Alarm Phone Number:%s\r\n",pSetPara);			          //测试使用
//			 return 1;
//		}
// 		case 2:            //存储报警阈值
// 		{
// 			 for(j=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=5
// 			 {
//           if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
// 					{
//              CharTemp[j] =pSetPara[j];
//           }
// 				  else
// 					{
// 						 break;
//           }
//        }
// 			 if(strlen(CharTemp)>2)
// 			 {
//           printf("\r\nInput Alarm Threshold ERROR!!\r\n");
//           return 0;
//        } 
// 			
// 			 for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
// 			 {
//           IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //计算报警阈值整数部分
//        }
// 			 
// 			 Counter =strlen(CharTemp)+1; //跳过小数点
// 			 memset(CharTemp,0x00,sizeof(CharTemp));
// 			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
// 			 {
//           if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
// 					{
//              CharTemp[i] =pSetPara[j];
//           }
// 				  else
// 					{
// 						 break;
//           }
//        }
// 			 if(strlen(CharTemp)>2)
// 			 {
//           printf("\r\nInput Alarm Threshold ERROR!!\r\n");
//           return 0;
//        }
//        for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
// 			 {
//          if(strlen(CharTemp)==1)
// 				 {
//            DecimalPart =(CharTemp[i-1]-'0')*10;
// 					 break;
//          }
// 				 else
// 				 {
//            DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //计算报警阈值小数部分
//          }
//        }
// 			 //DeviceConfig.AlarmThreshold =DecimalPart*0.01 + IntegerPart;
//        //ConfigData.threshold.Threshold_float = DecimalPart*0.01 + IntegerPart;
// //		   DataWrite_To_Flash(0,7,0, ConfigData.threshold.SMS_Set_Threshold,4);   //将报警阈值（Hex格式）写入Flash 
// 			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
// 			 return 1;
// 		}
		case 3:              //存储服务器IP
		{

			 for(j=0,Counter=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=15
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
//             CharTemp[j] =pSetPara[j];
						 ;
          }
				  else if(pSetPara[j]=='.')                                    //分隔符统计
					{
						 Counter++;
          }
					else
					{
              break;
          }
       }
			 if(Counter==3)
			 {
          DataWrite_To_Flash(0,5,0, (uint8_t*)pSetPara,strlen(pSetPara));   //将服务器IP写入Flash 
       }
			 else
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
       }
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 return 1;
			 
		}
		case 4:           //存储服务器端口号
		{
			 DataWrite_To_Flash(0,6,0,(uint8_t*)pSetPara,strlen(pSetPara));      //将服务器端口号写入Flash
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 return 1;
		}
//		case 5:          //存储液位监测仪探头安装高度
//		{
//			for(j=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=5
//			 {
//          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
//					{
//             CharTemp[j] =pSetPara[j];
//          }
//				  else
//					{
//						 break;
//          }
//       }
//			 if(strlen(CharTemp)>2)
//			 {
//          printf("\r\nInput Mounting Height ERROR!!\r\n");
//          return 0;
//       } 
//			
//			 for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
//			 {
//          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //计算探头安装高度整数部分
//       }
//			 
//			 Counter =strlen(CharTemp)+1; //跳过小数点
//			 memset(CharTemp,0x00,sizeof(CharTemp));
//			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
//			 {
//          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
//					{
//             CharTemp[i] =pSetPara[j];
//          }
//				  else
//					{
//						 break;
//          }
//       }
//			 if(strlen(CharTemp)>2)
//			 {
//          printf("\r\nInput Mounting Height ERROR!!\r\n");
//          return 0;
//       }
//       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
//			 {
//         if(strlen(CharTemp)==1)
//				 {
//           DecimalPart =(CharTemp[i-1]-'0')*10;
//					 break;
//         }
//				 else
//				 {
//           DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //计算探头安装高度小数部分
//         }
//       }
////			 DeviceConfig.MountingHeight =DecimalPart*0.01 + IntegerPart;
//     //  ConfigData.MountHeight.MountHeight_float = DecimalPart*0.01 + IntegerPart;
//		  // DataWrite_To_Flash(0,8,0, ConfigData.MountHeight.MountingHeigh,4);   //将探头安装高度（Hex格式）写入Flash 
//		}
		default:
		{
			 printf("\r\nInstruct Code ERROR !!\r\n");
		   return 0;
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
/*
uint8_t  UploadBKP(char* pSetPara, uint8_t  InstructCode)
{
  uint8_t   HexData[16] ={0x00}; 
  char      CharTemp[16]={0x00};
  uint16_t  i = 0;
  uint8_t   j = 0;
  uint8_t   Counter=0;
  uint16_t  IntegerPart =0x0000;        //整数部分
  uint16_t  DecimalPart =0x0000;        //小数部分
//  uint8_t   DataLength =0;  //用于指示有效数据转后的数据长度
//  uint8_t*  pData =NULL;
//  uint8_t*  pDot[3] ={NULL};
//  uint8_t*  pDot1 =NULL;
//  uint8_t*  pTemp =NULL;
//  float     AlarmData_F=0;
	
	switch(InstructCode)
	{
		case 1:       //手机号码合法性判断
		{
			 
			 if(strlen(pSetPara)<11)   //默认有效电话号码是11位
			 {
          printf("\r\nPhone Number ERROR_1!!\r\n");
					return 0;
       }
			 CharTemp[0] ='0';
			 memcpy(&(CharTemp[1]), pSetPara, strlen(pSetPara));   //调用函数已经做了防溢出处理
			 Char2Hex(HexData, CharTemp, 12);           
			 for(j=0;j<12;j++)
			 {
         printf("--%x-",HexData[j]);//测试使用
       }
			 printf("\r\n");//测试使用			 

			 for(j=0;j<12;j++)
			 {
          if(HexData[j]>9)
					{
						 break;
          }
       }
       if(j<12)  
			 {
          printf("\r\nPhone Number ERROR_2!!\r\n");
					return 0;
       }
		   i=0;
			 IntegerPart = (HexData[i]*4096) + (HexData[i+1]*256) + (HexData[i+2]*16) + (HexData[i+3]);
       ConfigData.PhoneNum[0] =IntegerPart;
			 i =i+4;
		   IntegerPart = (HexData[i]*4096) + (HexData[i+1]*256) + (HexData[i+2]*16) + (HexData[i+3]);
		   ConfigData.PhoneNum[1] =IntegerPart;
			 i =i+4;
		   IntegerPart = (HexData[i]*4096) + (HexData[i+1]*256) + (HexData[i+2]*16) + (HexData[i+3]);
//			 Temp = (HexData[i]<<12) + (HexData[i+1]<<8) + (HexData[i+2]<<4) + (HexData[i+3]);
		   ConfigData.PhoneNum[2] =IntegerPart;   
       DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
       printf("\r\n ConfigData.PhoneNum:--%4x--%4x--%4x--\r\n",ConfigData.PhoneNum[0],ConfigData.PhoneNum[1],ConfigData.PhoneNum[2]);			  //测试使用
			 return 1;
		}
		case 2:
		{
			 for(j=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=5
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[j] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>2)
			 {
          printf("\r\nInput Alarm Threshold ERROR!!\r\n");
          return 0;
       } 
			
			 for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //计算报警阈值整数部分
       }
			 
			 Counter =strlen(CharTemp)+1; //跳过小数点
			 memset(CharTemp,0x00,sizeof(CharTemp));
			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[i] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>2)
			 {
          printf("\r\nInput Alarm Threshold ERROR!!\r\n");
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
         if(strlen(CharTemp)==1)
				 {
           DecimalPart =(CharTemp[i-1]-'0')*10;
					 break;
         }
				 else
				 {
           DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //计算报警阈值小数部分
         }
       }
			 DeviceConfig.AlarmThreshold =DecimalPart*0.01 + IntegerPart;     //计算报警阈值整数部分 
			 ConfigData.Threshold = IntegerPart*256+DecimalPart;
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 return 1;
		}
		case 3:
		{

			 for(j=0;j<strlen(pSetPara);j++)                       //调用函数已经做了防溢出处理,strlen(pSetPara)<=15
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[j] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>3)
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       } 
			 for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //计算服务器IP地址第一个字段
       }
			 if((IntegerPart==0)||(IntegerPart>255))
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       }				 
			 Counter =strlen(CharTemp)+1;            //跳过分隔符
			 memset(CharTemp,0x00,sizeof(CharTemp));
			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[i] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>3)
			 {
          printf("\r\nInput Server IP ERROR !!\r\n");     //测试使用
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //计算服务器IP地址第二个字段
       }
			  if(DecimalPart>255)
			 {
          printf("\r\nInput Input Server IP ERROR!!\r\n");
          return 0;
       }	
			 ConfigData.ServerIP[0] = IntegerPart*256+DecimalPart;
			
			 IntegerPart =0x0000;     //复位
			 DecimalPart =0x0000;     //复位
			 j =strlen(CharTemp);           
			 Counter = Counter+j+1;     //跳过分隔符
			 memset(CharTemp,0x00,sizeof(CharTemp));
			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[i] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>3)
			 {
            printf("\r\nInput Server IP ERROR !!\r\n");     //测试使用
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));     //计算服务器IP地址第三个字段
       }
			  if(DecimalPart>255)
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       }	
			  
			 j =strlen(CharTemp);           
			 Counter = Counter+j+1;     //跳过分隔符
			 memset(CharTemp,0x00,sizeof(CharTemp));
			 for(i=0,j=Counter;j<strlen(pSetPara);j++,i++)                     
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
             CharTemp[i] =pSetPara[j];
          }
				  else
					{
						 break;
          }
       }
			 if(strlen(CharTemp)>3)
			 {
            printf("\r\nInput Server IP ERROR !!\r\n");     //测试使用
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));     //计算服务器IP地址第四个字段
       }
			  if(DecimalPart>255)
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       }	
			 ConfigData.ServerIP[1] = IntegerPart*256+DecimalPart;
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 return 1;
			 
		}
		case 4:
		{
			 if(strlen(pSetPara)!=4)   //默认有效端口号是4位
			 {
          printf("\r\nInput Server port ERROR !!\r\n");
					return 0;
       }
	
			 Char2Hex(HexData, pSetPara, strlen(pSetPara));    //调用函数已经做了防溢出处理        
			 for(j=0;j<strlen(pSetPara);j++)
			 {
          if(HexData[j]>9)
					{
						 break;
          }
       }
       if(j<strlen(pSetPara))  
			 {
          printf("\r\nInput Server port ERROR !!\r\n");
					return 0;
       }
			 for(i=0,Counter=0; i<j; i++,Counter++)
			 { 
				 if(Counter!=0)
				 {
           IntegerPart =IntegerPart<<4;
         }
				 IntegerPart= IntegerPart + HexData[i];
				 
       }
	     ConfigData.ServerPort =IntegerPart;
			 DeviceConfig.MessageSetFlag =1;     //标示当前液位仪参数通过短信修改		 
			 return 1;
		}
		default:
		{
			 printf("\r\nInstruct Code ERROR !!\r\n");
		   return 0;
		}
  }

}
*/
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void ParaUpdateCheck(void)
{
   printf("\r\nPhone Num Set OK!!%4x---%4x---%4x\r\n",ConfigData.PhoneNum[0],ConfigData.PhoneNum[1],ConfigData.PhoneNum[2]);
	 if((ConfigData.PhoneNum[0])&&(ConfigData.PhoneNum[1])&&(ConfigData.PhoneNum[2]))
		{
      BKP_WriteBackupRegister(BKP_DR4, ConfigData.PhoneNum[0]);  
		  RTC_WaitForLastTask();
			BKP_WriteBackupRegister(BKP_DR5, ConfigData.PhoneNum[1]);  
		  RTC_WaitForLastTask();
			BKP_WriteBackupRegister(BKP_DR6, ConfigData.PhoneNum[2]);  
		  RTC_WaitForLastTask();
    }
		if(ConfigData.Threshold != 0)
		{
      BKP_WriteBackupRegister(BKP_DR7, ConfigData.Threshold);  
		  RTC_WaitForLastTask();
    }
		if((ConfigData.ServerIP[0])&&(ConfigData.ServerIP[1]))
		{
      BKP_WriteBackupRegister(BKP_DR8, ConfigData.ServerIP[0]);  
		  RTC_WaitForLastTask();
			BKP_WriteBackupRegister(BKP_DR9, ConfigData.ServerIP[1]);  
		  RTC_WaitForLastTask();
    }
	  if(ConfigData.ServerPort != 0)
		{
      BKP_WriteBackupRegister(BKP_DR10, ConfigData.ServerPort);  
		  RTC_WaitForLastTask();
    }
	  RCC_ClearFlag();                             //Clear reset flags 
}
*/
/*******************************************************************************
* Function Name  : XX
* Description    : 查阅未读短信
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Sms_Consult(void)
{
  	
    mput("AT+CMGL=\"REC UNREAD\"");     //读取未读信息
		Delay_ms(1500); 
	  DMA_UART3_RecevDetect(DATARECEV);		
//		mput("AT+CMGD=,3");      //删除已读信息
		Delay_ms(2500);          //等待模块切换回数据传输状态
		
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/***********************函数功能：短信解析**************************/
/*******************************************************************/
//AT+CNMI=1,2,0,0,0                                //接收短信设置
//+CMT: "+8613121342836","","15/08/18,17:05:29+32" //接收到的短信报文格式，回车换行（0D 0A）之后为消息正文
//ok                                               //消息正文
//AT+CNMI=2,1                                      //接收短信设置
//+CMTI: "ME",1                                    //接收到新短信提醒
//AT+CMGL="REC UNREAD"                             //需发送查询未读短信命令
//+CMGL: 1,"REC UNREAD","+8613121342836","","15/08/18,17:12:19+32"   //接收到的短信报文格式，回车换行（0D 0A）之后为消息正文
//ok                                               //消息正文

void Sms_Analysis(char* pBuff)
{ 

//	char      MessageRead[11] ="REC UNREAD";                            //查询到未读短信存在
	char      MessageRecevIndicate[5] ="+32\"";                         //接收到短信指示性字符串
	//char      AlarmPhoneSet[23] ="casic_set_alarm_phone_";              //报警号码设置命令
	//char      AlarmThresholdSet[27] ="casic_set_alarm_threshold_";      //报警阈值设置命令
	char      ServerIpSet[21]   ="casic_set_server_ip_";                //服务器IP设置命令
	char      ServerPortSet[23] ="casic_set_server_port_";              //服务器端口号设置命令
	//char      DeviceMountHeight[27] ="casic_set_mounting_height_";      //探头安装高度设置
//	char      LevelDataInquire[33]  ="casic_inquire_current_level_data";//当前液位数据查询命令
	char      CurrentPhoneNum[16] ={0x00};                              //存储当前通信手机号码
//	char      AlarmThresholdTemp[6] ={0x00};                            //临时存放ASCII格式报警阈值
	char*     pSmsRecevBuff =NULL;          //查找特征数据位置指针
  char*     pSmsRecevData =NULL;          //查找目标数据位置指针
  uint8_t   UploadFlag =0;                //用于指示短信设置参数修改结果
	uint8_t   i=0;                          //循环变量

//	int       LocationFlag =0;      
//	uint8_t   AlarmPhoneNum[6]={0x00};     //存放配置号码信息
//  uint8_t   AlarmThreshold[2]={0x00};    //存放配置报警阈值
//  uint8_t   ServerIP[4]={0x00};          //存放服务器IP地址
//  uint8_t   ServerPort[2]={0x00};        //存放服务器端口地址
//	char      Recev_OK[2]="+CMT:";       //433模块接收指示
//  uint8_t   SmsRecev_Flag =0;     //接收数据正确性标志变量
//****sample:***+CMGL: 0,"REC UNREAD","+861064617006426",,"15/11/12,09:30:02+32"
//****sample****1245
//  pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, MessageRead, sizeof(Usart3_recev_buff),strlen(MessageRead));    //检查是否收到短信
	pSmsRecevBuff = Find_String(pBuff,"\"REC UNREAD\""); //检查是否收到短信
	if(pSmsRecevBuff !=NULL)
	{
		if(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-29)) //防止指针越界
		{
      
			 pSmsRecevBuff = pSmsRecevBuff+15;   //指针指向手机号码前缀86
			
			 printf("\r\n111:%s\r\n",pSmsRecevBuff);         //测试
			 for(i=0;i<16;i++)
			 {
          if(*pSmsRecevBuff == '\"')
					{
              break;
          }
					CurrentPhoneNum[i] = *pSmsRecevBuff; 
          pSmsRecevBuff++;					
       }
    }
		if(i>=13)
		{
        memcpy(ConfigData.CurrentPhoneNum,CurrentPhoneNum,i); //提取当前通信手机号码，有前缀86
			  printf("\r\nCurrentPhoneNum:%s\r\n",CurrentPhoneNum); //测试使用
    }
		
		 pSmsRecevBuff =NULL;
//		 pSmsRecevBuff = Find_String(pSmsRecevBuff,"+32\"");         //检查是否收到短信
	   pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, MessageRecevIndicate,sizeof(Usart3_recev_buff),strlen(MessageRecevIndicate));    //检查是否收到短信，过滤短信查询命令
		 if((pSmsRecevBuff !=NULL) &&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(MessageRecevIndicate))))   //防止指针越界
		 {
       pSmsRecevData =pSmsRecevBuff +strlen(MessageRecevIndicate);                          //测试使用
			 pSmsRecevBuff =NULL;   
			 printf("\r\nReceive Short Message is: %s\r\n",pSmsRecevData);    //测试使用
			  //pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, ServerIpSet,sizeof(Usart3_recev_buff),strlen(ServerIpSet));  //设置服务器IP
			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_server_ip_");    //设置服务器IP
			 if((pSmsRecevBuff != NULL)&&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(ServerIpSet)-16)))
			 {
          pSmsRecevData =pSmsRecevBuff +strlen(ServerIpSet);  
				  for(i=0;i<16;i++)           
				  {
             if(pSmsRecevData[i]==0x0D)
						 {
							  break;
             }
          }
				  if((i>15)||(i<7))  //输入数据不合法，直接丢弃，不更新配置信息，并给出错误提示
					{
						 printf("\r\nMessage set input ERROR!!\r\n");     //测试使用 	
//						 i=15;  //i表示有效IP地址长度，限制长度，防止溢出
          }	
          else
					{
             memset(DeviceConfig.ServerIP,0x00,sizeof(DeviceConfig.ServerIP));
						 memcpy(DeviceConfig.ServerIP, pSmsRecevData, i);		
						 UploadFlag = UploadFlash(DeviceConfig.ServerIP, 3);
          }						
				  if(UploadFlag ==1)
					{
              printf("\r\nServer IP upload success!!\r\n");    //测试使用
					  	Delay_ms(2000);  //短信发送缓冲
						  Sms_Send("\rServer IP upload success!!\r");
          }
				  else           //对非法输入值给出提示（负值不做非法处理，自动取绝对值）
				  {
             printf("\r\nInput server IP not correct!\r\nPlease check and retry.\r\n");    //测试使用
						 Delay_ms(2000);  //短信发送缓冲
						 Sms_Send("\rInput server IP not correct!\rPlease check and retry.\r");
          }
					UploadFlag =0;           //复位变量
				  pSmsRecevBuff = NULL;    //复位变量
       }
			// pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, ServerPortSet,sizeof(Usart3_recev_buff),strlen(ServerPortSet));   //设置服务器端口号
			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_server_port_");    //设置服务器端口号
			 if((pSmsRecevBuff != NULL)&&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(ServerPortSet)-6)))
			 {
          pSmsRecevData =pSmsRecevBuff +strlen(ServerPortSet);  
				  for(i=0;i<6;i++)
				  {
             if(pSmsRecevData[i]==0x0D)
						 {
							  break;
             }
          }
				  if((i<2)||(i>5))    //输入数据不合法，直接丢弃，不更新配置信息，并给出错误提示，端口号范围有待确认
					{
						 printf("\r\nMessage set input ERROR!!\r\n");     //测试使用 	
//						 i=5;  //i表示有效端口号长度，限制长度，防止溢出
          }	
   				else
					{
             memset(DeviceConfig.ServerPort,0x00,sizeof(DeviceConfig.ServerPort));
             memcpy(DeviceConfig.ServerPort, pSmsRecevData, i);				 
				     UploadFlag = UploadFlash(DeviceConfig.ServerPort, 4);
          }						
				  if(UploadFlag ==1)
					{
              printf("\r\nServer port upload success!!:%s\r\n",pSmsRecevData);    //测试使用
					  	Delay_ms(2000);  //短信发送缓冲
						  Sms_Send("\rServer port upload success!!\r");
          }
				  else           //对非法输入值给出提示（负值不做非法处理，自动取绝对值）
				  {
             printf("\r\nInput server port not correct!\r\nPlease check and retry.\r\n");    //测试使用
					   Delay_ms(2000);  //短信发送缓冲
						 Sms_Send("\rInput server port not correct!\rPlease check and retry.\r");
          }
					UploadFlag =0;           //复位变量
				  pSmsRecevBuff = NULL;    //复位变量
       }
	    }
  }
}		 
			 
			 
/*****************************************参数设置功能*************************************************************/		 			 
//			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_alarm_phone_");  //设置报警号码
// 			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, AlarmPhoneSet,sizeof(Usart3_recev_buff),strlen(AlarmPhoneSet));  
// 			 if((pSmsRecevBuff != NULL)&&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(AlarmPhoneSet)-16)))
// 			 {
//           pSmsRecevData =pSmsRecevBuff +strlen(AlarmPhoneSet);
//           for(i=0;i<16;i++)           
// 				  {
//              if(pSmsRecevData[i]==0x0D)
// 						 {
// 							  break;
//              }
//           }
// 				  if((i<11)||(i>16))    //输入数据不合法，直接丢弃，不更新配置信息，并给出错误提示
// 					{
// 						 printf("\r\nMessage set input ERROR!!\r\n");     //测试使用
// //						 i=16;  //i表示有效报警阈值长度，限制长度，防止溢出
//           }		
// 					else
// 					{
//              memset(DeviceConfig.AlarmPhoneNum,0x00,sizeof(DeviceConfig.AlarmPhoneNum));
// 				     memcpy(DeviceConfig.AlarmPhoneNum, pSmsRecevData, i);
// 				     UploadFlag = UploadFlash(DeviceConfig.AlarmPhoneNum, 1);
//           }
// 				  if(UploadFlag ==1)
// 					{
//               printf("\r\nAlarm phone number upload success!!\r\n");    //测试使用
// 							Delay_ms(2000);  //短信发送缓冲
// 						  Sms_Send("\rAlarm phone number upload success!!\r");
//           }
// 					else
// 					{
//               printf("\r\nAlarm phone number not correct!\r\nPlease check and retry.\r\n");    //测试使用
// 							Delay_ms(2000);  //短信发送缓冲
// 						  Sms_Send("\rAlarm phone number not correct!\rPlease check and retry.\r");
//           }
// 					UploadFlag =0;           //复位变量
// 				  pSmsRecevBuff = NULL;    //复位变量
//        }
// 			 
// //			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_alarm_threshold_");    //设置报警阈值
// 			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, AlarmThresholdSet,sizeof(Usart3_recev_buff),strlen(AlarmThresholdSet));   //设置报警阈值
// 			 if((pSmsRecevBuff != NULL)&&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(AlarmThresholdSet)-6)))
// 			 {
//           pSmsRecevData =pSmsRecevBuff +strlen(AlarmThresholdSet);   
// 				  for(i=0;i<6;i++)           
// 				  {
//              if(pSmsRecevData[i]==0x0D)
// 						 {
// 							  break;
//              }
//           }
// 					if(i==0)
// 					{
//              printf("\r\nMessage set input ERROR!!\r\n");     //测试使用
//           }
// 				  else 
// 					{
// 						 if(i>5)
// 						 {
//                i=5;  //i表示有效报警阈值长度，限制长度，防止溢出
//              }
// 						 memcpy(AlarmThresholdTemp, pSmsRecevData, i);		
// 				     UploadFlag = UploadFlash(AlarmThresholdTemp, 2);
//           }		
// 				  if(UploadFlag ==1)
// 					{
//               printf("\r\nAlarm threshold upload success!!\r\n");    //测试使用
// 						  Delay_ms(2000);  //短信发送缓冲
// 						  Sms_Send("\rAlarm threshold upload success!!\r");
//           }
// 				  else           //对非法输入值给出提示（负值不做非法处理，自动取绝对值）
// 				  {
//              printf("\r\nInput alarm threshold not correct!\r\nPlease check and retry.\r\n");    //测试使用
// 						 Delay_ms(2000);  //短信发送缓冲
// 						 Sms_Send("\rInput alarm threshold not correct!\rPlease check and retry.\r");
//           }
// 					UploadFlag =0;           //复位变量
// 				  pSmsRecevBuff = NULL;    //复位变量
//        }
			
			 
// 			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, DeviceMountHeight,sizeof(Usart3_recev_buff),strlen(DeviceMountHeight));   //设置探头安装高度
// 			 if((pSmsRecevBuff != NULL)&&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(DeviceMountHeight)-6)))
// 			 {
//           pSmsRecevData =pSmsRecevBuff +strlen(DeviceMountHeight);  
// 				  for(i=0;i<6;i++)
// 				  {
//              if(pSmsRecevData[i]==0x0D)
// 						 {
// 							  break;
//              }
//           }
// 					
// 					if(i==0)
// 					{
//              printf("\r\nMessage set input ERROR!!\r\n");     //测试使用
//           }
// 				  else 
// 					{
// 						 if(i>5)
// 						 {
//                i=5;  //i表示有效探头安装高度数据长度，限制长度，防止溢出
//              }
// 						 memset(AlarmThresholdTemp,0x00,sizeof(AlarmThresholdTemp));
// 						 memcpy(AlarmThresholdTemp, pSmsRecevData, i);		
// 				     UploadFlag = UploadFlash(AlarmThresholdTemp, 5);
//           }			
// 				  if(UploadFlag ==1)
// 					{
//               printf("\r\nMounting height upload success!!:%s\r\n",pSmsRecevData);    //测试使用
// 					  	Delay_ms(2000);  //短信发送缓冲
// 						  Sms_Send("\rMounting height upload success!!\r");
//           }
// 				  else           //对非法输入值给出提示（负值不做非法处理，自动取绝对值）
// 				  {
//              printf("\r\nInput mounting height not correct!\r\nPlease check and retry.\r\n");    //测试使用
// 					   Delay_ms(2000);  //短信发送缓冲
// 						 Sms_Send("\rInput mounting height not correct!\rPlease check and retry.\r");
//           }
// 					UploadFlag =0;           //复位变量
// 				  pSmsRecevBuff = NULL;    //复位变量
//        }
// 			 
/*****************************************查询功能****************************************************************/		 
////			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_inquire_current_level_data");    //查询液位信息
//			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, LevelDataInquire,sizeof(Usart3_recev_buff),strlen(LevelDataInquire));   //查询液位信息
//			 if(pSmsRecevBuff != NULL)           //有查询液位数据查询命令
//			 {
//				  if(DataCollectCount==0)          //数据采集完成则直接通过短信发送当前液位信息
//					{
//             LevelDataMessageSend();
//          }
//					else
//					{
//             ConfigData.LiquidDataInquireFlag =1;  //液位数据未进行采集时，收到查询命令，将标志变量置1，当液位数据采集完成时通过短信发送当前液位信息
//          }
//				  pSmsRecevBuff = NULL;            //复位变量
//       }	 
 

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char GPRS_Receive_NetLogin(void)
{

//   unsigned char    NetLoginFlag =0;     //模块注册基站状态标志变量
	 char*        pRecevBuff =NULL;
//   char   SendArry[50] ={'\0'};
//	 char   Recev_OK[2]={0xAA,0x1D};          //433模块接收指示
//   char   NewMessageIndicate[7] ="+CMGL:";  //收到未读短信指示
//   char   ModuleError[6] ="ERROR";          //GPRS模块故障指示


	 #if DEBUG_TEST
   printf("\r\nGPRS Net Login Receive Analysis ...\r\n");          //测试使用
	 #endif

	  ////////////////////////////////////////////////////////////////////////////////////////////////////////
    pRecevBuff = Find_SpecialString(Usart3_recev_buff, "+CGREG: 1,1", sizeof(Usart3_recev_buff), 11);  //检查模块入网状态
		if(pRecevBuff!=NULL)                                                
		{   
			 printf("\r\nNet Register OK!!\r\n");   
			 CSQ_OK =1;              //握手有响应时，标志变量置1			 
		   pRecevBuff =NULL;
			 return  1;
		}	 
		//////////////////////////////////////////////////////////////////////////////////////////////////////
    pRecevBuff = Find_SpecialString(Usart3_recev_buff, "+CGREG: 1,5", sizeof(Usart3_recev_buff), 11);  //检查模块入网状态
		if(pRecevBuff!=NULL)                                                
		{   
			 printf("\r\nNet Register OK!!Roaming on!!\r\n");   
			 CSQ_OK =1;              //握手有响应时，标志变量置1			 
		   pRecevBuff =NULL;
			 return  1;
		}	
    return  0;
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char GPRS_Receive_TcpConnect(void)
{
	 char*  pRecevBuff =NULL;

   #if DEBUG_TEST
   printf("\r\nTCP Connect is in process...\r\n");          //测试使用
	 #endif

		pRecevBuff = Find_SpecialString(Usart3_recev_buff, "CONNECT", sizeof(Usart3_recev_buff), 7);  //检查模块入网状态
		if(pRecevBuff != NULL)                                       //网络连接出现故障时，对3G模块复位
		{
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
unsigned char GPRS_Receive_DataAnalysis(void)	
{
	 unsigned char     Recev_Flag2 =0;     //接收数据正确性标志变量
	 char*  pRecevBuff =NULL;
	 char  i=0;
//   char   SendArry[50] ={'\0'};
//	 unsigned char   ServerNoiseSet[2] = {0xFF,0x02};
   unsigned char   ServerNoiseSet[2] = {0xA3,0x20};
   char   NewMessageIndicate[7] ="+CMGL:";  //收到未读短信指示
//   char   ModuleError[6] ="ERROR";          //GPRS模块故障指示

	 #if DEBUG_TEST
   printf("\r\n接收数据解析!!\r\n");          //测试使用
	 #endif
//////////////////////////////短信功能，暂时屏蔽////////////////////////////////////////////////////////////
	 pRecevBuff = Find_SpecialString(Usart4_recev_buff, NewMessageIndicate,sizeof(Usart4_recev_buff),strlen(NewMessageIndicate));    //检查是否收到短信
   {
			if(pRecevBuff !=NULL)
			{
         Sms_Analysis(pRecevBuff);          //接收短信解析
				 pRecevBuff =NULL;                  //复位查询指针
				 Delay_ms(3000);                    //等待时间不能太短，否则无法成功清空短信记录   
				 mput("AT+CMGD=1,3");               //删除全部已发、未发和已读短信
				 Delay_ms(500);           
//				 GPRS_Init();             //短信解析完成以后，需要重新建立网络连接
				 return 1;
		  }  
   }
	 
///////////////////////////////////////////////////////////////////////////////////////////////////
	 
//	 printf("\r\nUart3串口接收信息输出:%4x\r\n", Usart3_recev_buff);         //测试使用
	 printf("\r\nUart3串口接收信息输出:");
						 for(i=0;i<Usart4_recev_count;i++)
						 {
								printf(" %x",Usart4_recev_buff[i]);
						 }
	 
	 pRecevBuff = Find_SpecialString(Usart4_recev_buff,(char*)ServerNoiseSet,sizeof(Usart4_recev_buff),sizeof(ServerNoiseSet));  //检查有无收到主站回复
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart4_recev_buff+sizeof(Usart4_recev_buff)-1-24))  //防止指针越界 
//	 if(pRecevBuff != NULL)  //防止指针越界          		 
	 {	 
			
		  printf("\r\nReceive Server Config Data!\r\n");    //测试使用
		  for(i=0;i<60;i++)
		  {
         printf("\r\n--%x--\r\n",pRecevBuff[i]);         //测试使用
      }
		  Recev_Flag2 = ReceiveMessageVerify( pRecevBuff );
			if(Recev_Flag2==0)                                 //当前查找到的接收数据有误
			{
				pRecevBuff = NULL;
				#if DEBUG_TEST	 
				printf("\r\nReceive  data not correct!!\r\n");   //测试使用
				#endif
			}
			else                                               //接收数据正确
			{
				
				SetRev_OK = 1;
//				Receive_Analysis_GPRS();                         //测试使用 正式版删除
				Noise_Count = 0;
				Delay_ms(500);	
				pRecevBuff =NULL;
			  return 1;
			}
	 }
	
	 #if DEBUG_TEST	 
	 printf("\r\n接收数据解析完成!!\r\n");                 //调试使用
   #endif 
	 return 0;
	
}

/**********************************************END******************************************/













