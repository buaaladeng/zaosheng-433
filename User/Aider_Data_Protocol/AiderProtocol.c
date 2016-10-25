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
1����������֡�д��ڻس����з������ͨ��3Gģ��AT�����ϴ����ݵĳ����д�ȷ�ϡ�
2������һ֡�ϴ����ݵ����ݲɼ���ʽ�����ַ������ֱ��ǣ�
һ�����ݲɼ�ʱ�������������ϴ�ʱ��֮��ƽ�����䣬��ʱ��Ҫ�ɼ�һ�����ݽ������ߣ�ͬʱ���ɼ��������ݴ����ⲿ�洢����
���������߱��ݼĴ�����¼���ݲɼ�������
�������ݲɼ����ԶС�������ϴ�ʱ��������ʱ���ѣ�����֮�������ɼ����ݣ�N�����ݲɼ����֮�������ϴ�����������
�ϴ���ɣ������������ش�����ٴν������ģʽ��
ĿǰĬ�ϲ��÷�����
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern  struct liquid_set  DeviceConfig;        //Һλ��������Ϣ�ṹ��
extern  struct SMS_Config_RegPara   ConfigData;    //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
extern  char  PowerOffReset ;

//extern u8  DataCollectCache[13][4];            //������HEX��ʽ�洢�����ֽ���ǰ�����ֽ��ں�
extern u8  DataCollectCount;                   //���ݲɼ�������
extern u8    Batch_Num ;                       //�����������
extern u8    Batch_Sum ;                       //������������
extern char   SetRev_OK ;                      //�ɹ����շ���������
extern char   DatRev_OK ;                      //�ɹ���ȷ����Һλ����
extern void  LSLIQUID_DataCollect(void);//N��Һλ���ݲɼ����Լ����һ�βɼ�ʱ���¼����
extern void  RecvBuffInit_USART3(void);
extern char  Usart2_recev_buff[50];
extern uint16_t  Noise_Count;
extern char  Dense_Data[60];
extern char  Noise_Data[60];
extern char SLNoise_char[300];
extern int   Length_Frame; 
extern char	 Dense_Num;
extern char  UploadCount;    //�����ϴ�����Դ���
extern char SLNoiseConfigFlag;
extern char DeviceId[6];     //������¼���豸���
char  LSLiquidSend[160] ={'\0'}; //
unsigned short CRC_temp;
uint16_t	RSampleTime;
extern uint16_t SampleSpan;

extern void  Receive(void);

// ADC1ת���ĵ�ѹֵͨ��MDA��ʽ����SRAM
extern __IO uint16_t ADC_ConvertedValue;

// �ֲ����������ڱ���ת�������ĵ�ѹֵ 	 
float ADC_ConvertedValueLocal; 

extern struct rtc_time systmtime;        //RTCʱ�����ýṹ��

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
		val = HexArry[j] & 0xf0;  //�Ը�λ����ת��
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
				 
		val = HexArry[j] & 0x0f; //�Ե�λ����ת��
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
		val =HexArry[j]/100;       //ȡ���ݵİ�λ����
		if(val != 0)
		{
      CharArry[i] = val+'0';
			i++;
    }
		
		val =HexArry[j]%100/10;   //ȡ���ݵ�ʮλ����
		if((val!=0)||(i!=0))    //����λ����Ϊ�㣬����ʮλ����Ϊ��ʱ���Ը�λ����ת��
		{
      CharArry[i] = val+'0';
			i++;
    }
		
		val =HexArry[j]%10;       //ȡ���ݵĸ�λ����
		if((val!=0)||(i!=0))    //����λ����ʮλ������λ����һ����Ϊ��ʱ���Ը�λ����ת��
		{
      CharArry[i] = val+'0';
			i++;
    }
		if(j<3)                 //IP��ַ���һ���ֶ�ת������Ժ󲻼�"."
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
	
  printf("\r\nConfigData_Init start...\r\n");  //����ʹ��
  BKP_TamperPinCmd(DISABLE);                   //

	
	//������������
	
  Noise_Count = BKP_ReadBackupRegister(BKP_DR11);	
	if(Noise_Count > 500)
	{
			Noise_Count = 20;
	}

//	TempNoiseDataRead();
		
	DataRead_From_Flash(0,5,0, (u8*)ConfigData.SMS_Set_ServerIP,15);   //��Flash�ж�ȡԤ�������IP
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
	if(length>7)                            //��IP��ַ�Ϸ���������ɸѡ�������д����� 
	{
     memcpy(Para->ServerIP, ConfigData.SMS_Set_ServerIP, length);        
  }
	else                                                     
	{
		 memcpy(Para->ServerIP,"58.210.41.202",13);                  //��ȡ������Чʱ����ʼ��������IP   
  }
	
	
	DataRead_From_Flash(0,6,0, (u8*)ConfigData.SMS_Set_ServerPort,5);   //��Flash�ж�ȡԤ��������˿ں�
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
	if(length>0)                                              //�Է������˿ںźϷ���������ɸѡ�������д����� 
	{
     memcpy(Para->ServerPort, ConfigData.SMS_Set_ServerPort, length);        
  }
//	length = strlen(ConfigData.SMS_Set_ServerPort);
//	if((length>0)&&(length<=5))                            //�Է������˿ںźϷ���������ɸѡ�������д����� 
//	{
//     memcpy(Para->ServerPort, ConfigData.SMS_Set_ServerPort, length);        
//  }
	else                                                     
	{ 
		 memcpy(Para->ServerPort,"2014",4);                          //��ȡ������Чʱ����ʼ���������˿ں� 
  }
	
		SLNoiseConfigFlag = BKP_ReadBackupRegister(BKP_DR7); 
		 
}
///*******************************************************************************
//* Function Name  : XX
//* Description    : ��hex��ʽ����ת��Ϊchar�ͣ�����ת������ַ���������Ҫ�����ϴ�Һλ����ת��
//* Input          : None
//* Output         : None
//* Return         : ת����õ��ַ�������
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
//  while((pData-databuf)<length)  //���ʹ�����ݳ��ȼ��������ж�
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
	
/*   �д�����       */ 
	#if DEBUG_TEST	 
	printf("\r\nSection over!!\r\n");
	#endif	
}

/*******************************************************************************
* Function Name  : XX
* Description    : ������¼�������ϱ���������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SLNoise_DataUpload(struct liquid_set* Para, char noisesensorflag)
{
	struct rtc_time* ptm;
	struct rtc_time tm;	
	u32    OID_NoiseData;               //����OID����
  			
  char Preamble[1] = {0xA3};                                           //ǰ����
  char Version[1] = {0x20};                                            //�汾��
	char Leng[2] = {0x00,0x00};                                          //���ݳ���
	
	char RouteMark[1] = {0x01};                                          //·�ɱ�־
	char RouteSite[2] = {0x00,0x00};                                     //�ڵ��ַ
	char PDUType[2] = {0x04,0x82};                                       //PDUType
	char Sep[1] = {0x01};                                                //�������к�

  char StateOid[4] = {0x60,0x00,0x02,0x00};                      //�豸״̬Oid
	char StateLen[2] = {0x00,0x01};                                //�豸״̬��ֵ����
	char StateValue[1] = {0x01};                                   //�豸״ֵ̬

	char DateOid[4] = {0x10,0x00,0x00,0x50};                             //����Oid
	char DateLen[2] = {0x00,0x06};                                       //���ڳ���
	
	char NoiseOid[4] = {0xC4,0x00,0x18,0x3C};                            //����ֵOid
	char NoiseLen[2] = {0x00,0x30};                                      //����ֵ����

	char BatteryOid[4] = {0x60,0x00,0x00,0x20};                          //����Oid
	char BatteryLen[2] = {0x00,0x01};                                    //����ֵ����
	char BatteryValue[1] = {0xFF};                                       //����ֵ

  char  CRC1[2] = {0x00,0x00};                                         //CRCУ����
		
  char* pSend = NULL;

  int   Length_Frame = 0;
 
  u16   i=0;//,j=0;
	
	if(noisesensorflag == 2)            //�豸���ѻظ�
	{
     PDUType[0] = 0x0B;
  }

		/* enable adc1 and config adc1 to dma mode */
	ADC1_Init();
	
  ptm = &tm;
  pSend = LSLiquidSend;
	Leng[1] = 37 + Dense_Num*2; //����

	(*pSend++) = Preamble[0];                                            //ǰ����
	(*pSend++) = Version[0];                                             //�汾��
	for(i=0;i<2;i++)
	{
    (*pSend++) = Leng[i];                                             //���ݳ���
  }
	for(i=0;i<6;i++)
	{
    (*pSend++) = DeviceId[i];                                          //�豸���
  }
	(*pSend++) = RouteMark[0];                                          //·�ɱ�־
	for(i=0;i<2;i++)
	{
    (*pSend++) = DeviceId[i+4];                                       //�ڵ��ַ��ȡ�豸��ź���λ
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = PDUType[i];                                          //PDUType
  }
	(*pSend++) = Sep[0];                                                //�������к�

	if(noisesensorflag == 2)            //�豸���ѻظ�
	{
		for(i=0;i<4;i++)
		{
			(*pSend++) = StateOid[i];                                      //�豸״̬Oid
		}
		for(i=0;i<2;i++)
		{
			(*pSend++) = StateLen[i];                                      //�豸״̬����
		}
		(*pSend++) = StateValue[0];                                       //�豸״ֵ̬
  }		
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = BatteryOid[i];                                      //����Oid
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = BatteryLen[i];                                      //����ֵ����
  }
	BatteryValue[0] = DS2780_Test();                                    //����ֵ
	printf("\r\n The current BatteryValue[0] value = %d \r\n", BatteryValue[0]);
	(*pSend++) = BatteryValue[0];                                       //����ֵ
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = DateOid[i];                                         //����Oid
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = DateLen[i];                                         //���ڳ���
  }
	Time_Show(ptm);                                                    //ϵͳ����
	(*pSend++) = ptm->tm_year-2000;
	(*pSend++) = ptm->tm_mon;
	(*pSend++) = ptm->tm_mday;
	(*pSend++) = ptm->tm_hour;
	(*pSend++) = ptm->tm_min;		
	(*pSend++) = ptm->tm_sec;		

	RSampleTime = ptm->tm_hour*60 + ptm->tm_min - SampleSpan*Dense_Num;
	OID_NoiseData = (SampleSpan<<11)+ RSampleTime +(0xC4<<24);          //����ֵOid���ݲ���ʱ��Ͳ����������
	NoiseOid[0] = (OID_NoiseData&0xFF000000)>>24;
	NoiseOid[1] = (OID_NoiseData&0x00FF0000)>>16;
	NoiseOid[2] = (OID_NoiseData&0x0000FF00)>>8;
	NoiseOid[3] = OID_NoiseData&0x000000FF;
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = NoiseOid[i];                                       //����ֵOid
  }
	NoiseLen[1] = (u16)Dense_Num*2;                                   //����ֵ���ȣ�Ϊ��������������
	for(i=0;i<2;i++)
	{
    (*pSend++) = NoiseLen[i];
  }
	printf("\r\nNoiseLen[1] is %x", NoiseLen[1]);
	for(i=0;i<5;i++)
	{
		Time_Show(&systmtime);             /* Display time in infinite loop */
		printf("\r\n-----BKP_DR2:%d----BKP_DR3:%d-----BKP_DR4:%d----BKP_DR5:%d----\r\n",BKP_ReadBackupRegister(BKP_DR2),BKP_ReadBackupRegister(BKP_DR3),BKP_ReadBackupRegister(BKP_DR4),BKP_ReadBackupRegister(BKP_DR5));    //����ʹ��
		Receive();
		
		ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*3.3; // ��ȡת����ADֵ
		printf("Noise_Count: %d \r\n",i);
		printf("\r\n The current AD value = 0x%04X \r\n", ADC_ConvertedValue/100); 
    printf("\r\n The current AD value = %f V \r\n", ADC_ConvertedValueLocal);
							
//		Usart2_recev_buff[0] = ADC_ConvertedValue/100;// ������������ֵ��������
		(*pSend++) = ADC_ConvertedValue/100;                                     //����ֵ
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

	Length_Frame =pSend - LSLiquidSend;     //�����ϱ�����֡���ȣ�����ʹ��
	printf("\r\nLength_Frame: %d!!!!!!!!!!!!!",Length_Frame);
	Delay_ms(1000);
	
	//////////////////////////Test Part////////////////////////////////////////
	printf("\r\n");                     //����ʹ��
	for(i=0;i<Length_Frame;i++)
	{
		printf(" %x",LSLiquidSend[i]);  //����ʹ��
  }
	printf("\r\n");                     //����ʹ��
	
	
	#if DEBUG_TEST	
  printf("\r\nSEND Length is%d\r\n",Length_Frame);  //����ʹ��
	printf("\r\nSEND:%s\r\n",LSLiquid_char);  //����ʹ��
	#endif
	
//	UploadCount = BKP_ReadBackupRegister(BKP_DR5);
	UploadCount = 3;
	SetRev_OK = 0;
  printf("\r\nUploadCount is %d\r\n",UploadCount);  //����ʹ��	
	while(UploadCount!=0)
	{		
		UploadCount--;
		if(SetRev_OK ==1)                          //�ɹ����շ��������ñ�־����
		{
			SetRev_OK = 0;
			break;                                   //�ɹ����շ��������ã��˳�ѭ��
    }
			
		SendMessage(LSLiquidSend,Length_Frame);
		
	}
}

/*******************************************************************************
* Function Name  : XX
* Description    : �豸����ע������������·�������Ϣ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void StartupRequest(char noisesensorflag)	
{
	u8    NetStatusFlag =1;                //����״̬��־����
	
	char Preamble[1] = {0xA3};
  char Version[1] = {0x20};
	char Leng[2] = {0x00,0x00};
	
	char RouteMark[1] = {0x01};
	char RouteSite[2] = {0x00,0x00};

	char PDUType[2] = {0x08,0x82};
	char Sep[1] = {0x01};

	char StateOid[4] = {0x60,0x00,0x01,0x00};                      //�豸״̬Oid
	char StateLen[2] = {0x00,0x01};                                //�豸״̬��ֵ����
	char StateValue[1] = {0x01};                                   //�豸״ֵ̬
	
  char  CRC1[2] = {0x00,0x00};

	char* pSend = NULL;

  int   Length_Frame = 0;
  u16   i=0;//,j=0;

	if(noisesensorflag == 2)            //�豸���ѻظ�
	{
     PDUType[0] = 0x0B;
		 PDUType[1] = 0x82;
		 StateOid[0] = 0x60;
		 StateOid[1] = 0x00;
		 StateOid[2] = 0x02;
		 StateOid[3] = 0x00;
  }
	
  pSend = LSLiquidSend;
	Leng[1] = 19;             //����

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
	
	Length_Frame =pSend - LSLiquidSend;     //�����ϱ�����֡���ȣ�����ʹ��
	printf("\r\nLength_Frame: %d!!!!!!!!!!!!!",Length_Frame);
	Delay_ms(1000);
	
		//////////////////////////Test Part////////////////////////////////////////
	printf("\r\n");                     //����ʹ��
	for(i=0;i<Length_Frame;i++)
	{
		printf(" %x",LSLiquidSend[i]);  //����ʹ��
  }
	printf("\r\n");                     //����ʹ��	
  UploadCount = BKP_ReadBackupRegister(BKP_DR5);
  SetRev_OK = 0;	
	while(UploadCount!=0)
	{
		UploadCount--;
		if(SetRev_OK ==1)                          //�ɹ����շ��������ñ�־����
		{
			SetRev_OK =0;
			break;                                   //�ɹ����շ��������ã��˳�ѭ��
    }
		
		NetStatusFlag = SendMessage(LSLiquidSend,Length_Frame);   //	
	}
	if(NetStatusFlag==0)                         //��������쳣ʱ���豸��������
	{
      gotoSleep(1);
  }
}


/*******************************************************************************
* Function Name  : XX
* Description    : �����豸������Ϣ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ConfigData_Upload()	
{
	u8    NetStatusFlag =1;                //����״̬��־����
	
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
	Leng[1] = 43;             //����

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
  (*pSend++) = (BKP_ReadBackupRegister(BKP_DR2)&0xFF00)>>8;              // ����ʱ���λ
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR2)&0x00FF;	                 // ����ʱ���λ
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = SpanOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = SpanLen[i];
  }
	(*pSend++) = (BKP_ReadBackupRegister(BKP_DR3)&0xFF00)>>8;              // ���������λ
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR3)&0x00FF;                   // ���������λ
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = NumberOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = NumberLen[i];
  }
  (*pSend++) = (BKP_ReadBackupRegister(BKP_DR4)&0xFF00)>>8;              // ����������λ
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR4)&0x00FF;	                 // ����������λ
	
	for(i=0;i<4;i++)
	{
    (*pSend++) = UploadCountOid[i];
  }
	for(i=0;i<2;i++)
	{
    (*pSend++) = UploadCountLen[i];
  }	
	(*pSend++) = BKP_ReadBackupRegister(BKP_DR5);	                         // �ϴ�����
	
	CRC_temp = CRC16((unsigned char *)pSend, 47);	
  printf("\r\nCRC is %x", CRC_temp);	
	CRC1[1] = CRC_temp&0x00FF;
	CRC1[0] = (CRC_temp&0xFF00)>>8;
	for(i=0;i<2;i++)
	{
    (*pSend++) = CRC1[i];
  }
	
	Length_Frame =pSend - LSLiquidSend;     //�����ϱ�����֡���ȣ�����ʹ��
	printf("\r\nLength_Frame: %d!!!!!!!!!!!!!",Length_Frame);
	Delay_ms(1000);
	
		//////////////////////////Test Part////////////////////////////////////////
	printf("\r\n");                     //����ʹ��
	for(i=0;i<Length_Frame;i++)
	{
		printf(" %x",LSLiquidSend[i]);  //����ʹ��
  }
	printf("\r\n");                     //����ʹ��		

  UploadCount = BKP_ReadBackupRegister(BKP_DR5);
  SetRev_OK = 0;	
	while(UploadCount !=0)
	{
		UploadCount--;
		if(SetRev_OK ==1)                          //�ɹ����շ��������ñ�־����
		{
			SetRev_OK =0;
			break;                                   //�ɹ����շ��������ã��˳�ѭ��
    }
		
		NetStatusFlag = SendMessage(LSLiquidSend,Length_Frame);   //	
	}
	if(NetStatusFlag==0)                         //��������쳣ʱ���豸��������
	{
      gotoSleep(1);
  }
}


/**********************************************END******************************************/

