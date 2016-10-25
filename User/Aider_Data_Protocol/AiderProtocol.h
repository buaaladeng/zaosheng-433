#ifndef _AIDERPROTOCOL_H_
#define _AIDERPROTOCOL_H_

#include "stm32f10x.h"
#include "stdio.h" 
 
struct liquid_set 
{
	//uint16_t CollectPeriod;     //�ɼ����
	//uint16_t SendCount;         //һ���ϴ����ݴ���
//	u8  BatteryCapacity;        //���������⣬����xx%����Ϊ�ٷֺ�ǰ��Ĳ��֡�
	u8  Time_Sec;
	u8  Time_Min;
	u8  Time_Hour;
	u8  Time_Mday;
	u8  Time_Mon;
	u8  Time_Year;
//	u8 Time_Wday;               //���ֶ���ʱ���ԣ���������
	u8    MessageSetFlag;         //����ָʾ��ǰ�Ƿ�ͨ�������޸�Һλ�ǲ�������
	//char  AlarmPhoneNum[12];      //�����绰����
	char  ServerIP[16];           //������IP
	char  ServerPort[6];          //�������˿�
	//float AlarmThreshold;         //Һλ������ֵ
	
};

void ConfigData_Init(struct liquid_set* Para);
//void LSLIQUSET_Handle(char* pLiqudSet, struct liquid_set* Parameter);
//void LSLIQUSET_Response(struct liquid_set* Parameter);
//void LSTIMESET_Handle(char* pLiqudSet, struct liquid_set* Parameter);
void LSDataUpload_Finish(struct liquid_set* Parameter);
void DataUpload_TALK_OVER(struct liquid_set* Parameter);
void Float2Hex_Aider( float DataSmooth );

void SLNoise_DataUpload(struct liquid_set* Para, char noisesensorflag);
//void LSLIQUID_DataUpload(struct liquid_set* Para);
void Section_Request(void);
void Section_Handle(void);
//void mput_mix(char *str,u8 length);

#endif

