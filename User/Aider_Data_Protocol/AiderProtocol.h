#ifndef _AIDERPROTOCOL_H_
#define _AIDERPROTOCOL_H_

#include "stm32f10x.h"
#include "stdio.h" 
 
struct liquid_set 
{
	//uint16_t CollectPeriod;     //采集间隔
	//uint16_t SendCount;         //一天上传数据次数
//	u8  BatteryCapacity;        //电池容量检测，例如xx%，仅为百分号前面的部分。
	u8  Time_Sec;
	u8  Time_Min;
	u8  Time_Hour;
	u8  Time_Mday;
	u8  Time_Mon;
	u8  Time_Year;
//	u8 Time_Wday;               //本字段暂时忽略，不做处理
	u8    MessageSetFlag;         //用于指示当前是否通过短信修改液位仪参数配置
	//char  AlarmPhoneNum[12];      //报警电话号码
	char  ServerIP[16];           //服务器IP
	char  ServerPort[6];          //服务器端口
	//float AlarmThreshold;         //液位报警阈值
	
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

