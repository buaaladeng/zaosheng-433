#include "stm32f10x.h"
#include "gprs.h"
#include "bsp_SysTick.h"
#include "modbus.h"
#include "string.h"
#include "AiderProtocol.h"
#include "433_Wiminet.h"
#include "bsp_rtc.h"
#include "DS2780.h"
#include "SPI_Flash.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
1、由于数据帧中存在回车换行符，因此通过3G模块AT命令上传数据的长度有待确认。
2、关于一帧上传数据的数据采集方式有两种方案，分别是：
一、数据采集时间在两次数据上传时间之间平均分配，此时需要采集一次数据进行休眠，同时将采集到的数据存入外部存储区，
并用外存或者备份寄存器记录数据采集次数。
二、数据采集间隔远小于数据上传时间间隔，定时唤醒，唤醒之后立即采集数据，N组数据采集完成之后立即上传到服务器，
上传完成，并完成其他相关处理后，再次进入待机模式。
目前默认采用方案二
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern  struct liquid_set  DeviceConfig;        //液位计配置信息结构体
extern  struct SMS_Config_RegPara   ConfigData;    //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
extern  char  PowerOffReset ;

//extern u8  DataCollectCache[13][4];            //浮点数HEX格式存储，低字节在前，高字节在后
extern u8  DataCollectCount;                   //数据采集计数器
extern u8    Batch_Num ;                       //数据批次序号
extern u8    Batch_Sum ;                       //数据批次总数
extern char   SetRev_OK ;                      //成功接收服务器配置
extern char   DatRev_OK ;                      //成功正确接收液位数据
extern void  LSLIQUID_DataCollect(void);//N次液位数据采集，以及最后一次采集时间记录更新
extern void  RecvBuffInit_USART3(void);
extern char  Usart2_recev_buff[50];
extern uint16_t  Noise_Count;
extern char  Dense_Data[60];
extern char  Noise_Data[60];
extern char SLNoise_char[300];
extern int   Length_Frame; 
extern char	 Dense_Num;
extern char  UploadCount;    //数据上传最大尝试次数
extern char SLNoiseConfigFlag;
extern char DeviceId[6];     //噪声记录仪设备编号
char  LSLiquidSend[160] ={'\0'}; //
unsigned short CRC_temp;
uint16_t	RSampleTime;
extern uint16_t SampleSpan;

extern void  Receive(void);

// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_ConvertedValue;

// 局部变量，用于保存转换计算后的电压值 	 
float ADC_ConvertedValueLocal; 

extern struct rtc_time systmtime;        //RTC时钟设置结构体

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Hex2Char(char* CharArry, uint8_t* HexArry, uint8_t Length_HexArry)
{
	uint8_t   i=0,j=0;
	uint8_t   ch=0,val=0;
//  uint8_t*  pData =HexArry ;
	
	for(j=0;j<Length_HexArry;j++)
	{
		val = HexArry[j] & 0xf0;  //对高位进行转换
		val>>=4;
		if(val <= 9)
		{
			ch= val+'0';	
		}
		else  	
		{
			ch =val-10+'A';
		}
		CharArry[i++] = ch;
				 
		val = HexArry[j] & 0x0f; //对低位进行转换
		if(val <= 9)
		{
			ch= val+'0';	
		}
		else  	
		{
			ch =val-10+'A';
		}
		CharArry[i++] = ch;				  
//		pData++;
	}
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Hex2DecimalChar(char* CharArry, uint8_t* HexArry, uint8_t Length_HexArry)
{
	uint8_t   i=0,j=0;
	uint8_t   val=0;
//  uint8_t*  pData =HexArry ;
	
	for(j=0;j<Length_HexArry;j++)    
	{
		val =HexArry[j]/100;       //取数据的百位数字
		if(val != 0)
		{
      CharArry[i] = val+'0';
			i++;
    }
		
		val =HexArry[j]%100/10;   //取数据的十位数字
		if((val!=0)||(i!=0))    //当百位数不为零，或者十位数不为零时，对该位进行转变
		{
      CharArry[i] = val+'0';
			i++;
    }
		
		val =HexArry[j]%10;       //取数据的个位数字
		if((val!=0)||(i!=0))    //当百位数、十位数、个位数有一个不为零时，对该位进行转变
		{
      CharArry[i] = val+'0';
			i++;
    }
		if(j<3)                 //IP地址最后一个字段转换完成以后不加"."
		{
		  CharArry[i] = '.';	
			i++;
		}		  
//		pData++;
	}
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Init(struct liquid_set* Para)
{
	uint8_t  length =0;
	uint8_t  Temp =0;
	
  printf("\r\nConfigData_Init start...\r\n");  //测试使用
  BKP_TamperPinCmd(DISABLE);                   //

	
	//保存噪声参数
	
  Noise_Count = BKP_ReadBackupRegister(BKP_DR11);	
	if(Noise_Count > 500)
	{
			Noise_Count = 20;
	}

//	TempNoiseDataRead();
		
	DataRead_From_Flash(0,5,0, (u8*)ConfigData.SMS_Set_ServerIP,15);   //从Flash中读取预设服务器IP
	for(Temp=0;Temp<15;)
  {
    if(((ConfigData.SMS_Set_ServerIP[Temp] >='0')&&(ConfigData.SMS_Set_ServerIP[Temp] <='9'))||(ConfigData.SMS_Set_ServerIP[Temp] =='.'))
		{
       Temp++;
    }
		else
		{
			break;
    }
  }
	length =Temp;
	if(length>7)                            //对IP地址合法性做初步筛选，后续有待完善 
	{
     memcpy(Para->ServerIP, ConfigData.SMS_Set_ServerIP, length);        
  }
	else                                                     
	{
		 memcpy(Para->ServerIP,"58.210.41.202",13);                  //读取数据无效时，初始化服务器IP   
  }
	
	
	DataRead_From_Flash(0,6,0, (u8*)ConfigData.SMS_Set_ServerPort,5);   //从Flash中读取预设服务器端口号
	for(Temp=0;Temp<5;)
  {
    if((ConfigData.SMS_Set_ServerPort[Temp] >='0')&&(ConfigData.SMS_Set_ServerPort[Temp] <='9'))
		{
       Temp++;
    }
		else
		{
			break;
    }
  }
	length =Temp;
	if(length>0)                                              //对服务器端口号合法性做初步筛选，后续有待完善 
	{
     memcpy(Para->ServerPort, ConfigData.SMS_Set_ServerPort, length);        
  }
//	length = strlen(ConfigData.SMS_Set_ServerPort);
//	if((length>0)&&(length<=5))                            //对服务器端口号合法性做初步筛选，后续有待完善 
//	{
//     memcpy(Para->ServerPort, ConfigData.SMS_Set_ServerPort, length);        
//  }
	else                                                     
	{ 
		 memcpy(Para->ServerPort,"2014",4);                          //读取数据无效时，初始化服务器端口号 
  }
	
		SLNoiseConfigFlag = BKP_ReadBackupRegister(BKP_DR7); 
		 
}
///*******************************************************************************
//* Function Name  : XX
//* Description    : 将hex格式数据转换为char型，返回转换后的字符个数，主要用于上传液位数据转换
//* Input          : None
//* Output         : None
//* Return         : 转换后得到字符的数量
//*******************************************************************************/
//int char_hextochar(char* dealbuf,unsigned char* databuf, u8 length)
//{
//	int i;
//	u8 ch=0,val=0;
//	
//	unsigned char* pData = databuf;
//	
//	for(i=0;i<length;i++)
//  {
//     dealbuf[i] = databuf[i];
//  }	
////	char  head[9] ={'L','S','L','E','V','D','A','T','A',':'};
// unsigned char* pData = databuf + 8;
//	
//	for(i=0;i<8;i++)
//  {
//     dealbuf[i] = databuf[i];
//  }		
//	
//  while((pData-databuf)<length)  //最好使用数据长度计数器来判断
//	{

//		 if(((pData-databuf)==9)||((pData-databuf)==10)||((pData-databuf)==11)||((pData-databuf)==12)
//			 ||((pData-databuf)==18)||((pData-databuf)==24)||((pData-databuf)==36)||((pData-databuf)==41)
//		   ||((pData-databuf)==42)||((pData-databuf)==42+Dense_Num*2)||((pData-databuf)==43+Dense_Num*2)||((pData-databuf)==45+Dense_Num*2)) //138-82=56
//		 {
//        dealbuf[i++] =',';
//			  //pData++;
//     } 
//		 else
//		 {;} 
//				val =*pData&0xf0;
//				val>>=4;
//				if(val<=9) ch=val+'0';	
//				else  	ch=val-10+'A';
//				dealbuf[i++]=ch;
//			 
//				val =*pData&0x0f;
//				if(val<=9)  ch=val+'0';			
//				else  ch=val-10+'A';
//				dealbuf[i++]=ch;
//			  
//			  pData++;
//     
//  }
//   
////	 dealbuf[i++] =0x03;
//   dealbuf[i++] ='\r';
//	 dealbuf[i++] ='\n';
//   
//	return i;
//}


void Section_Handle(void)
{
	
/*   有待完善       */ 
	#if DEBUG_TEST	 
	printf("\r\nSection over!!\r\n");
	#endif	
}

/*******************************************************************************
* Function Name  : XX
* Description    : 噪声记录仪主动上报噪声数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SLNoise_DataUpload(struct liquid_set* Para, char noisesensorflag)
{
	struct rtc_time* ptm;
	struct rtc_time tm;	
	u32    OID_NoiseData;               //数据OID编码
  			
  char Preamble[1] = {0xA3};                                           //前导码
  char Version[1] = {0x20};                                            //版本号
	char Leng[2] = {0x00,0x00};                                          //数据长度
	
	char RouteMark[1] = {0x01};                                          //路由标志
	char RouteSite[2] = {0x00,0x00};                                     //节点地址
	char PDUType[2] = {0x04,0x82};                                       //PDUType
	char Sep[1] = {0x01};                                                //报文序列号

  char StateOid[4] = {0x60,0x00,0x02,0x00};                      //设备状态Oid
	char StateLen[2] = {0x00,0x01};                                //设备状态数值长度
	char StateValue[1] = {0x01};                                   //设备状态值

	char DateOid[4] = {0x10,0x00,0x00,0x50};                             //日期Oid
	char DateLen[2] = {0x00,0x06};                                       //日期长度
	
	char NoiseOid[4] = {0xC4,0x00,0x18,0x3C};                            //噪声值Oid
	char NoiseLen[2] = {0x00,0x30};                                      //噪声值长度

	char BatteryOid[4] = {0x60,0x00,0x00,0x20};                          //电量Oid
	char BatteryLen[2] = {0x00,0x01};                                    //电量值长度
	char BatteryValue[1] = {0xFF};                                       //电量值

  char  CRC1[2] = {0x00,0x00};                                         //CRC校验码
		
  char* pSend = NULL;

  int   Length_Frame = 0;
 
  u16   i=0;//,j=0;
	
	if(noisesensorflag == 2)            //设备唤醒回复
	{
     PDUType[0] = 0x0B;
  }

		/* enable adc1 and config adc1 to dma mode */
	ADC1_Init();
	
  ptm = &tm;
  pSend = LSLiquidSend;
	Leng[1] = 37 + Dense_Num*2; //长度

	(*pSend++) = Preamble[0];                                            //前导码
	(*pSend++) = Version[0];                                             //版本号
	for(i=0;i<2;i++)
	{
    (*pSend++) = Leng[i];                                             //数据长度
  }
	for(i=0;i<6;i++)
	{
    (*pSend++) = DeviceId[i];                                          //设备编号
  }
	(*pSend++) = RouteMark[0];                                          //路由标志
	for(i=0;i<2;i++)
	{
    (*pSend++) = DeviceId[i+4];                                       //节点地址，取设备编号后两位
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = PDUType[i];                                          //PDUType
  }
	(*pSend++) = Sep[0];                                                //报文序列号

	if(noisesensorflag == 2)            //设备唤醒回复
	{
		for(i=0;i<4;i++)
		{
			(*pSend++) = StateOid[i];                                      //设备状态Oid
		}
		for(i=0;i<2;i++)
		{
			(*pSend++) = StateLen[i];                                      //设备状态长度
		}
		(*pSend++) = StateValue[0];                                       //设备状态值
  }		
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = BatteryOid[i];                                      //电量Oid
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = BatteryLen[i];                                      //电量值长度
  }
	BatteryValue[0] = DS2780_Test();                                    //电量值
	printf("\r\n The current BatteryValue[0] value = %d \r\n", BatteryValue[0]);
	(*pSend++) = BatteryValue[0];                                       //电量值
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = DateOid[i];                                         //日期Oid
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = DateLen[i];                                         //日期长度
  }
	Time_Show(ptm);                                                    //系统日期
	(*pSend++) = ptm->tm_year-2000;
	(*pSend++) = ptm->tm_mon;
	(*pSend++) = ptm->tm_mday;
	(*pSend++) = ptm->tm_hour;
	(*pSend++) = ptm->tm_min;		
	(*pSend++) = ptm->tm_sec;		

	RSampleTime = ptm->tm_hour*60 + ptm->tm_min - SampleSpan*Dense_Num;
	OID_NoiseData = (SampleSpan<<11)+ RSampleTime +(0xC4<<24);          //噪声值Oid根据采样时间和采样间隔生成
	NoiseOid[0] = (OID_NoiseData&0xFF000000)>>24;
	NoiseOid[1] = (OID_NoiseData&0x00FF0000)>>16;
	NoiseOid[2] = (OID_NoiseData&0x0000FF00)>>8;
	NoiseOid[3] = OID_NoiseData&0x000000FF;
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = NoiseOid[i];                                       //噪声值Oid
  }
	NoiseLen[1] = (u16)Dense_Num*2;                                   //噪声值长度，为噪声组数的两倍
	for(i=0;i<2;i++)
	{
    (*pSend++) = NoiseLen[i];
  }
	printf("\r\nNoiseLen[1] is %x", NoiseLen[1]);
	for(i=0;i<5;i++)
	{
		Time_Show(&systmtime);             /* Display time in infinite loop */
		printf("\r\n-----BKP_DR2:%d----BKP_DR3:%d-----BKP_DR4:%d----BKP_DR5:%d----\r\n",BKP_ReadBackupRegister(BKP_DR2),BKP_ReadBackupRegister(BKP_DR3),BKP_ReadBackupRegister(BKP_DR4),BKP_ReadBackupRegister(BKP_DR5));    //测试使用
		Receive();
		
		ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*3.3; // 读取转换的AD值
		printf("Noise_Count: %d \r\n",i);
		printf("\r\n The current AD value = 0x%04X \r\n", ADC_ConvertedValue/100); 
    printf("\r\n The current AD value = %f V \r\n", ADC_ConvertedValueLocal);
							
//		Usart2_recev_buff[0] = ADC_ConvertedValue/100;// 将采样的噪声值赋给串口
		(*pSend++) = ADC_ConvertedValue/100;                                     //噪声值
		(*pSend++) = 0x00;
    Delay_ms(5000);		
  }
	
	CRC_temp = CRC16((unsigned char *)pSend, 41 + Dense_Num*2);	                       
  printf("\r\nCRC is %x", CRC_temp);	
	CRC1[1] = CRC_temp&0x00FF;
	CRC1[0] = (CRC_temp&0xFF00)>>8;
	for(i=0;i<2;i++)
	{
    (*pSend++) = CRC1[i];
  }

	Length_Frame =pSend - LSLiquidSend;     //计算上报数据帧长度，测试使用
	printf("\r\nLength_Frame: %d!!!!!!!!!!!!!",Length_Frame);
	Delay_ms(1000);
	
	//////////////////////////Test Part////////////////////////////////////////
	printf("\r\n");                     //测试使用
	for(i=0;i<Length_Frame;i++)
	{
		printf(" %x",LSLiquidSend[i]);  //测试使用
  }
	printf("\r\n");                     //测试使用
	
	
	#if DEBUG_TEST	
  printf("\r\nSEND Length is%d\r\n",Length_Frame);  //测试使用
	printf("\r\nSEND:%s\r\n",LSLiquid_char);  //测试使用
	#endif
	
//	UploadCount = BKP_ReadBackupRegister(BKP_DR5);
	UploadCount = 3;
	SetRev_OK = 0;
  printf("\r\nUploadCount is %d\r\n",UploadCount);  //测试使用	
	while(UploadCount!=0)
	{		
		UploadCount--;
		if(SetRev_OK ==1)                          //成功接收服务器配置标志变量
		{
			SetRev_OK = 0;
			break;                                   //成功接收服务器配置，退出循环
    }
			
		SendMessage(LSLiquidSend,Length_Frame);
		
	}
}

/*******************************************************************************
* Function Name  : XX
* Description    : 设备开机注册请求服务器下发配置信息
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StartupRequest(char noisesensorflag)	
{
	u8    NetStatusFlag =1;                //网络状态标志变量
	
	char Preamble[1] = {0xA3};
  char Version[1] = {0x20};
	char Leng[2] = {0x00,0x00};
	
	char RouteMark[1] = {0x01};
	char RouteSite[2] = {0x00,0x00};

	char PDUType[2] = {0x08,0x82};
	char Sep[1] = {0x01};

	char StateOid[4] = {0x60,0x00,0x01,0x00};                      //设备状态Oid
	char StateLen[2] = {0x00,0x01};                                //设备状态数值长度
	char StateValue[1] = {0x01};                                   //设备状态值
	
  char  CRC1[2] = {0x00,0x00};

	char* pSend = NULL;

  int   Length_Frame = 0;
  u16   i=0;//,j=0;

	if(noisesensorflag == 2)            //设备唤醒回复
	{
     PDUType[0] = 0x0B;
		 PDUType[1] = 0x82;
		 StateOid[0] = 0x60;
		 StateOid[1] = 0x00;
		 StateOid[2] = 0x02;
		 StateOid[3] = 0x00;
  }
	
  pSend = LSLiquidSend;
	Leng[1] = 19;             //长度

	(*pSend++) = Preamble[0];
	(*pSend++) = Version[0];
	for(i=0;i<2;i++)
	{
    (*pSend++) = Leng[i];
  }
	for(i=0;i<6;i++)
	{
    (*pSend++) = DeviceId[i];
  }
	(*pSend++) = RouteMark[0];
	for(i=0;i<2;i++)
	{
    (*pSend++) = DeviceId[i+4];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = PDUType[i];
  }
	(*pSend++) = Sep[0];
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = StateOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = StateLen[i];
  }
	(*pSend++) = StateValue[0];
	
	CRC_temp = CRC16((unsigned char *)pSend, 23);	
  printf("\r\nCRC is %x", CRC_temp);	
	CRC1[1] = CRC_temp&0x00FF;
	CRC1[0] = (CRC_temp&0xFF00)>>8;
	for(i=0;i<2;i++)
	{
    (*pSend++) = CRC1[i];
  }
	
	Length_Frame =pSend - LSLiquidSend;     //计算上报数据帧长度，测试使用
	printf("\r\nLength_Frame: %d!!!!!!!!!!!!!",Length_Frame);
	Delay_ms(1000);
	
		//////////////////////////Test Part////////////////////////////////////////
	printf("\r\n");                     //测试使用
	for(i=0;i<Length_Frame;i++)
	{
		printf(" %x",LSLiquidSend[i]);  //测试使用
  }
	printf("\r\n");                     //测试使用	
  UploadCount = BKP_ReadBackupRegister(BKP_DR5);
  SetRev_OK = 0;	
	while(UploadCount!=0)
	{
		UploadCount--;
		if(SetRev_OK ==1)                          //成功接收服务器配置标志变量
		{
			SetRev_OK =0;
			break;                                   //成功接收服务器配置，退出循环
    }
		
		NetStatusFlag = SendMessage(LSLiquidSend,Length_Frame);   //	
	}
	if(NetStatusFlag==0)                         //网络出现异常时，设备进入休眠
	{
      gotoSleep(1);
  }
}


/*******************************************************************************
* Function Name  : XX
* Description    : 发送设备配置信息到服务器
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Upload()	
{
	u8    NetStatusFlag =1;                //网络状态标志变量
	
	char Preamble[1] = {0xA3};
  char Version[1] = {0x20};
	char Leng[2] = {0x00,0x00};
	
	char RouteMark[1] = {0x01};
	char RouteSite[2] = {0x00,0x00};
	char PDUType[2] = {0x02,0x82};
	char Sep[1] = {0x01};

	char StartTimeOid[4] = {0x10,0x00,0x01,0x04};
	char StartTimeLen[2] = {0x00,0x02};
	char StartTimeValue[2] = {0x00,0x00};

	char SpanOid[4] = {0x10,0x00,0x01,0x05};
	char SpanLen[2] = {0x00,0x02};
	char SpanValue[2] = {0x00,0x00};
	
	char NumberOid[4] = {0x10,0x00,0x01,0x06};
	char NumberLen[2] = {0x00,0x02};
	char NumberValue[2] = {0x00,0x00};
	
	char UploadCountOid[4] = {0x10,0x00,0x00,0x0A};
	char UploadCountLen[2] = {0x00,0x01};
	char UploadCountValue[1] = {0x00};

  char  CRC1[2] = {0x00,0x00};

	char* pSend = NULL;

  int   Length_Frame = 0;
  u16   i=0;//,j=0;

  pSend = LSLiquidSend;
	Leng[1] = 43;             //长度

	(*pSend++) = Preamble[0];
	(*pSend++) = Version[0];
	for(i=0;i<2;i++)
	{
    (*pSend++) = Leng[i];
  }
	for(i=0;i<6;i++)
	{
    (*pSend++) = DeviceId[i];
  }
	(*pSend++) = RouteMark[0];
	for(i=0;i<2;i++)
	{
    (*pSend++) = DeviceId[i+4];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = PDUType[i];
  }
	(*pSend++) = Sep[0];
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = StartTimeOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = StartTimeLen[i];
  }
  (*pSend++) = (BKP_ReadBackupRegister(BKP_DR2)&0xFF00)>>8;              // 采样时间高位
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR2)&0x00FF;	                 // 采样时间低位
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = SpanOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = SpanLen[i];
  }
	(*pSend++) = (BKP_ReadBackupRegister(BKP_DR3)&0xFF00)>>8;              // 采样间隔高位
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR3)&0x00FF;                   // 采样间隔低位
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = NumberOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = NumberLen[i];
  }
  (*pSend++) = (BKP_ReadBackupRegister(BKP_DR4)&0xFF00)>>8;              // 采样次数高位
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR4)&0x00FF;	                 // 采样次数低位
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = UploadCountOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = UploadCountLen[i];
  }	
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR5);	                         // 上传次数
	
	CRC_temp = CRC16((unsigned char *)pSend, 47);	
  printf("\r\nCRC is %x", CRC_temp);	
	CRC1[1] = CRC_temp&0x00FF;
	CRC1[0] = (CRC_temp&0xFF00)>>8;
	for(i=0;i<2;i++)
	{
    (*pSend++) = CRC1[i];
  }
	
	Length_Frame =pSend - LSLiquidSend;     //计算上报数据帧长度，测试使用
	printf("\r\nLength_Frame: %d!!!!!!!!!!!!!",Length_Frame);
	Delay_ms(1000);
	
		//////////////////////////Test Part////////////////////////////////////////
	printf("\r\n");                     //测试使用
	for(i=0;i<Length_Frame;i++)
	{
		printf(" %x",LSLiquidSend[i]);  //测试使用
  }
	printf("\r\n");                     //测试使用		

  UploadCount = BKP_ReadBackupRegister(BKP_DR5);
  SetRev_OK = 0;	
	while(UploadCount !=0)
	{
		UploadCount--;
		if(SetRev_OK ==1)                          //成功接收服务器配置标志变量
		{
			SetRev_OK =0;
			break;                                   //成功接收服务器配置，退出循环
    }
		
		NetStatusFlag = SendMessage(LSLiquidSend,Length_Frame);   //	
	}
	if(NetStatusFlag==0)                         //网络出现异常时，设备进入休眠
	{
      gotoSleep(1);
  }
}


/**********************************************END******************************************/

