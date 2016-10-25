#ifndef _GPRS_H_
#define _GPRS_H_
#include "stm32f10x.h"
//#define  NETERRORCOUNT    10
#define  NETLOGIN         1     //设备注册基站
#define  TCPCONNECT       2     //设备建立TCP连接
#define  DATARECEV        3     //设备接收服务器数据



///////////////////////////////////////////////////////////////////////////////
//声明结构体
struct SMS_Config_RegPara 
{
	
	char     CurrentPhoneNum[16];      //当前通信手机号码，字符串格式
	char     SMS_Set_AlarmPhone[16];   //短信配置液位报警号码，字符串格式
	char     SMS_Set_ServerIP[16];     //短信配置服务器IP，字符串格式
	char     SMS_Set_ServerPort[6];    //短信配置服务器端口号，字符串格式

	uint8_t  CollectPeriod_Byte[2];    //数据采集间隔，使用数组存储，以方便Flash读写
	uint8_t  SendCount_Byte[2];        //一天上传液位数据次数，使用数组存储，以方便Flash读写
	uint8_t  LiquidDataInquireFlag;    //短信查询当前液位数据标志变量
	
};

//声明函数
void  GPRS_Init(void);          //433通信方式，该函数暂时无用，使用3G通信时才打开
void  TCP_Connect(void);        //建立TCP连接
void  TCP_Disconnect(void);     //断开TCP连接
void  mput(char *str);
void  mput_mix(char *str,int length);
char* Find_String(char *Source, char *Object);
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object);
void  SIM5216_PowerOn(void);    //433通信方式，该函数暂时无用，使用3G通信时才打开
void  SIM5216_PowerOff(void);   //433通信方式，该函数暂时无用，使用3G通信时才打开
void  Sms_Send(char*  pSend);   //短信发送函数
void Sms_Analysis(char* pBuff);       //短信接收解析函数
void  Sms_Consult(void);         //查阅未读短信
void  USART_DataBlock_Send(USART_TypeDef *USART_PORT,char *SendUartBuf,u16 SendLength);   //批量向串口发送数据
//void  ParaUpdateCheck(void);    //将通过短信下发的配置参数写入BKP寄存器

//char  Receive_Monitor_GPRS(void); //3G模块串口监听，当MCU检测到3G模块串口的数据时
//void  Receive_Deal_GPRS(void);    //3G模块接收数据解析，主要用于处理MCU与3G模块之间的交互数据
void  Receive_Analysis_GPRS(void);//3G模块接收服务器数据解析，主要处理服务器下发的基于埃德尔协议的数据
//u8    NetStatus_Detection( void );//3G模块网络连接状态检测
void  StartupRequest(char NoiseSensorFlag);      //设备开机请求服务器下发配置信息
void  GPRS_Config(void);
unsigned char GPRS_Receive_NetLogin(void);  //模块注册网络接收信息解析
unsigned char GPRS_Receive_TcpConnect(void);//建立TCP连接接收信息解析
unsigned char Receive_Deal_GPRS(void);    //3G模块接收数据解析，主要用于处理MCU与3G模块之间的交互数据
unsigned char GPRS_Receive_DataAnalysis(void);        //3G模块接收服务器数据解析，主要处理服务器下发的基于埃德尔协议的数据

#endif

