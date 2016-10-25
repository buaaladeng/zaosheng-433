#ifndef _GPRS_H_
#define _GPRS_H_
#include "stm32f10x.h"
//#define  NETERRORCOUNT    10
#define  NETLOGIN         1     //�豸ע���վ
#define  TCPCONNECT       2     //�豸����TCP����
#define  DATARECEV        3     //�豸���շ���������



///////////////////////////////////////////////////////////////////////////////
//�����ṹ��
struct SMS_Config_RegPara 
{
	
	char     CurrentPhoneNum[16];      //��ǰͨ���ֻ����룬�ַ�����ʽ
	char     SMS_Set_AlarmPhone[16];   //��������Һλ�������룬�ַ�����ʽ
	char     SMS_Set_ServerIP[16];     //�������÷�����IP���ַ�����ʽ
	char     SMS_Set_ServerPort[6];    //�������÷������˿ںţ��ַ�����ʽ

	uint8_t  CollectPeriod_Byte[2];    //���ݲɼ������ʹ������洢���Է���Flash��д
	uint8_t  SendCount_Byte[2];        //һ���ϴ�Һλ���ݴ�����ʹ������洢���Է���Flash��д
	uint8_t  LiquidDataInquireFlag;    //���Ų�ѯ��ǰҺλ���ݱ�־����
	
};

//��������
void  GPRS_Init(void);          //433ͨ�ŷ�ʽ���ú�����ʱ���ã�ʹ��3Gͨ��ʱ�Ŵ�
void  TCP_Connect(void);        //����TCP����
void  TCP_Disconnect(void);     //�Ͽ�TCP����
void  mput(char *str);
void  mput_mix(char *str,int length);
char* Find_String(char *Source, char *Object);
char* Find_SpecialString(char* Source, char* Object, short Length_Source, short Length_Object);
void  SIM5216_PowerOn(void);    //433ͨ�ŷ�ʽ���ú�����ʱ���ã�ʹ��3Gͨ��ʱ�Ŵ�
void  SIM5216_PowerOff(void);   //433ͨ�ŷ�ʽ���ú�����ʱ���ã�ʹ��3Gͨ��ʱ�Ŵ�
void  Sms_Send(char*  pSend);   //���ŷ��ͺ���
void Sms_Analysis(char* pBuff);       //���Ž��ս�������
void  Sms_Consult(void);         //����δ������
void  USART_DataBlock_Send(USART_TypeDef *USART_PORT,char *SendUartBuf,u16 SendLength);   //�����򴮿ڷ�������
//void  ParaUpdateCheck(void);    //��ͨ�������·������ò���д��BKP�Ĵ���

//char  Receive_Monitor_GPRS(void); //3Gģ�鴮�ڼ�������MCU��⵽3Gģ�鴮�ڵ�����ʱ
//void  Receive_Deal_GPRS(void);    //3Gģ��������ݽ�������Ҫ���ڴ���MCU��3Gģ��֮��Ľ�������
void  Receive_Analysis_GPRS(void);//3Gģ����շ��������ݽ�������Ҫ����������·��Ļ��ڰ��¶�Э�������
//u8    NetStatus_Detection( void );//3Gģ����������״̬���
void  StartupRequest(char NoiseSensorFlag);      //�豸��������������·�������Ϣ
void  GPRS_Config(void);
unsigned char GPRS_Receive_NetLogin(void);  //ģ��ע�����������Ϣ����
unsigned char GPRS_Receive_TcpConnect(void);//����TCP���ӽ�����Ϣ����
unsigned char Receive_Deal_GPRS(void);    //3Gģ��������ݽ�������Ҫ���ڴ���MCU��3Gģ��֮��Ľ�������
unsigned char GPRS_Receive_DataAnalysis(void);        //3Gģ����շ��������ݽ�������Ҫ����������·��Ļ��ڰ��¶�Э�������

#endif

