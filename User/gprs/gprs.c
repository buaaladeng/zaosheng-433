
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

//char  Receive_Monitor_GPRS(void); //3Gģ�鴮�ڼ�������MCU��⵽3Gģ�鴮�ڵ�����ʱ
//void  Receive_Deal_GPRS(void);    //3Gģ��������ݽ�������Ҫ���ڴ���MCU��3Gģ��֮��Ľ�������
//void  Receive_Analysis_GPRS(void);//3Gģ����շ��������ݽ�������Ҫ����������·��Ļ��ڰ��¶�Э�������
//u8    NetStatus_Detection( void );//3Gģ����������״̬���

char      Usart3_recev_buff[1000]={'\0'};     //USART3���ջ���
uint16_t  Usart3_recev_count=0;              //USART3���ռ�����
char      Usart4_recev_buff[1000]={'\0'};     //USART3���ջ���
uint16_t  Usart4_recev_count=0;              //USART3���ռ�����

uint8_t   CSQ_OK =0;                         //�ź�������־����
//uint8_t  TCP_Connect_Flag =0;                //TCP���ӱ�־����
//uint8_t  TCP_Connect_Start=0;                //TCP����������־����
extern struct    SMS_Config_RegPara   ConfigData;     //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
//static char      RespRev_OK =0;              //�ɹ����շ�����Ӧ��
//static uint8_t   HandFlag =1;
//static uint8_t   NetErrorCount =NETERRORCOUNT;           //�����쳣ʱ��ʽ�������ӵ�������
//extern uint32_t  time ;                    //USART3���ն�ʱ����ms��  
extern struct    liquid_set  DeviceConfig;   //Һλ��������Ϣ�ṹ��
extern char      SetRev_OK;                  //�ɹ����շ���������
extern char      Alive;                      //�����߱�־����,Ϊ1ʱ˵�����������ߴ����д�����
extern char      DatRev_OK ;                 //�ɹ���ȷ����Һλ����
extern uint8_t   DataCollectCount;           //���ݲɼ�������
extern uint8_t   LiquidDataSend_Flag;
extern uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern uint8_t   DMA_UART3_RECEV_FLAG ;             //USART3 DMA���ձ�־����
extern char SLNoise_char[300];
extern int   Length_Frame; 

extern char  UploadCount;    //�����ϴ�����Դ���
extern char DeviceId[6];     //������¼���豸���

//extern char NoiseSensorFlag;
//uint8_t  Usart3_send_buff[500]={'\0'};     //USART3���ͻ���
//uint8_t   Usart3_send_count=0;             //USART3���ͼ�����

//extern char  Receive_Monitor_GPRS(void);
//extern void  Receive_Deal_GPRS(void);
//extern void  Receive_Analysis_GPRS(void);	
//extern void  RecvBuffInit_USART3(void);
extern  unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag);     //USART3�������ݼ�������ݽ���
extern  void      LevelDataMessageSend(void);      // ͨ�����ŷ���Һλ����
extern  char     PowerOffReset ;
extern  uint16_t  Noise_Count;

extern int char_hextochar(char* dealbuf,unsigned char* databuf, u8 length);

void  SIM5216_PowerOn(void)            //��SIM5216ģ��
{
   GPIO_SetBits(GPIOC,GPIO_Pin_3);	   //POWER_ON��������
   Delay_ms(120);                      //100ms��ʱ     64ms<Ton<180ms
   GPIO_ResetBits(GPIOC,GPIO_Pin_3);   //POWER_ON��������
   Delay_ms(5500);                     //5s��ʱ        Tuart>4.7s           
	
}

void  SIM5216_PowerOff(void)            //�ر�SIM5216ģ��
{
   GPIO_SetBits(GPIOC,GPIO_Pin_3);	    //POWER_ON��������
   Delay_ms(2000);                      //1s��ʱ       500ms<Ton<5s
   GPIO_ResetBits(GPIOC,GPIO_Pin_3);    //POWER_ON��������
   Delay_ms(6000);                             
}

void USART_DataBlock_Send(USART_TypeDef* USART_PORT,char* SendUartBuf,u16 SendLength)    //�����򴮿ڷ�������
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
	printf("length:%d\r\n",length);             //����ʹ��
//	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);   //��USART3��������ǰ���ȴ�USART3���տ����жϣ����ڼ�����ݽ������
	
	PowerOFF_433_SET();                              //433ģ��SET�����ͣ��������
	Delay_ms(100);
	PowerOFF_433_EN();                               //433ģ��EN�����ͣ��������
	Delay_ms(100);
	
	USART_GetFlagStatus(UART4, USART_FLAG_TC);          //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR����ֹ���ݷ���ʱ���ֽڱ�����
	Delay_ms(500);
	USART_DataBlock_Send(UART4,str,length);
	USART_DataBlock_Send(USART1,str,length);
	USART_DataBlock_Send(USART1,"\r\n",2);
	
	Delay_ms(2000);
	PowerON_433_SET();                    //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(2000);
}

void mput(char* str)
{
//	printf("length:%d\r\n",strlen(str));     //����ʹ��
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //��USART3��������ǰ���ȴ�USART3���տ����жϣ����ڼ�����ݽ������
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
   USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //���ô���DMA1����
   mput("AT^UARTRPT=1");  //����ͨ�Žӿ�ΪUART
	 Delay_ms(200);      
	 mput("AT+CMGF=1");     //���Ÿ�ʽ����Ϊ�ı�ģʽ
	 Delay_ms(300); 
	 mput("AT+CPMS=\"ME\",\"ME\",\"ME\"");     //���÷��ͺͽ��ն��Ŵ洢��ΪME��Ϣ�洢��������ʹ��AT+CMGL="REC UNREAD"����鲻������
	 Delay_ms(300); 
   mput("AT+CNMI=1,1");   //����Ϣָʾ����Ϊ�洢����֪ͨ
	 Delay_ms(200);
//	 if(DMA_UART3_RECEV_FLAG==1)
//	 {
//	   DMA_UART3_RecevDetect();	                    //�������ݽ�������
//	 }
//	 mput("AT+CMGD=1,3");   //����Ϣָʾ����Ϊ�洢����֪ͨ
//	 Delay_ms(500);
//	 mput("AT+CPMS?");   //����Ϣָʾ����Ϊ�洢����֪ͨ
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
  u8   NetSearchCount=6;           //��Ѱ��������Դ���
	u8   CSQ_DetectCount=3;           //�����ź�ǿ�ȼ������������ȶ���������
//	char SendArry1[80] ={'\0'};
//  u8   i=0;
	
  memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //���ô���DMA1����
	Delay_ms(3000);
	while(NetSearchCount!=0)
	{
	   NetSearchCount--;
		 printf("\r\nGPRS Net Searching...\r\n-------------NetSearchCount:%d\r\n",NetSearchCount);    //����ʹ��
		 mput("AT+CGREG=1");   //����ź��Զ���������3G�źź���ʱ�Զ��л���3G�ź�
		 Delay_ms(800);
		 mput("AT+CGREG?");   //����ź��Զ���������3G�źź���ʱ�Զ��л���3G�ź�
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
     gotoSleep(0);      	//�Ѳ����������
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
    unsigned char  TcpConnectFlag =0;  //TCP���ӽ�����־����
    unsigned char  NetErrorCount =6;   //TCP�������ӳ��Դ�������һ����ſ�����


	  printf(" Multi GPRS Start!\r\n");
//  GPRS_Init();

     while(NetErrorCount>0)
		{
				 
			 mput("at+mipcall=1");
			 Delay_ms(800);		
			 snprintf(SendArry,sizeof(SendArry),"at+miptrans=1,\"%s\",%s",DeviceConfig.ServerIP,DeviceConfig.ServerPort);  // mput("at+miptrans=1,\"58.210.41.202\",2015");   //������Ҫ�ӼĴ����ж�ȡ���д���һ������
       mput(SendArry);
			 Delay_ms(5000); 
						 TcpConnectFlag =DMA_UART3_RecevDetect( TCPCONNECT );    //��GPRSģ����յ������ݽ��н���
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
				 gotoSleep(0);     //��γ���δ���ӵ����磬ֱ�ӽ�������ģʽ�����ڼ��ģ�黽���Ժ��һ������״̬
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
//  TCP_Connect_Flag =0;    //�ر�ѭ������
}
/*******************************************************************************
* Function Name  : char* Find_String(char* Source, char* Object)
* Description    : ��Ŀ���ַ����з���һ��ָ�����ַ���
* Input          : 
* Output         : 
* Return         : ����ҵ����򷵻�Ŀ���ַ�����Դ�ַ����е��׵�ַ
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
/***********�������ܣ����ض������з���һ��ָ��������****************/
/***********����ҵ����򷵻�Ŀ��������Դ�����е��׵�ַ**************/
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
/********************�������ܣ����ŷ���ǰԤ����*********************/
/*************������Ҫ���͵������������һ��������0x1a**************/
void SendPretreatment(char* pSend)
{  
   uint8_t  i=0;
	 char*    pTemp =NULL;
	
	 i= strlen(pSend);
	 pTemp =pSend+i;
   *(pTemp) =0x1a;
}

/***********************�������ܣ����Ͷ���**************************/
/*******************************************************************/
//���Ͷ������̣�
//��һ�������ý��ն������Ѹ�ʽ��AT+CNMI=1,2,0,0,0    
//�ڶ������趨���Ž��շ����룺AT+CMGS="15116924685"
//�����������Ͷ������ģ�����16����0x1a��Ϊ��β
void Sms_Send(char*  pSend)
{ 
//	uint8_t  PhoneNum[13]={0x00};    //��������Ϊ�ӼĴ����ж�ȡ
	char  SendBuf[200]={0x00};                //���ŷ��ͻ���,���������ܵķ������ݳ���ȷ�����С

//	struct liquid_set* Para= &DeviceConfig;
	
//	memcpy(PhoneNum, DeviceConfig.AlarmPhoneNum,strlen((char*)DeviceConfig.AlarmPhoneNum));          
//	mput("AT+CNMI=1,2,0,0,0");
	
  memset(SendBuf,0,sizeof(SendBuf));
	snprintf(SendBuf,sizeof(SendBuf),"AT+CMGS=\"%s\"",ConfigData.CurrentPhoneNum); //���Ͷ��ź���Ϊ��ǰͨ���ֻ�����
	mput(SendBuf);
//	mput("AT+CMGS=\"861064617006426\"");  //����ʹ�ã��д�����

	Delay_ms(500);
  memset(SendBuf,0,sizeof(SendBuf));
	snprintf(SendBuf, (sizeof(SendBuf)-1), pSend, strlen(pSend));
  SendPretreatment(SendBuf);
	mput(SendBuf);
  Delay_ms(1000);  
//	DMA_UART3_RecevDetect();  //�ȴ�������Ϣ

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
//  uint16_t  IntegerPart =0x0000;        //��������
//  uint16_t  DecimalPart =0x0000;        //С������
//  uint8_t   DataLength =0;  //����ָʾ��Ч����ת������ݳ���
//  uint8_t*  pData =NULL;
//  uint8_t*  pDot[3] ={NULL};
//  uint8_t*  pDot1 =NULL;
//  uint8_t*  pTemp =NULL;
//  float     AlarmData_F=0;
	
	switch(InstructCode)
	{
//		case 1:            //�洢��������
//		{
//			 
//			 DataWrite_To_Flash(0,4,0,(uint8_t*)pSetPara,strlen(pSetPara));   //����������д��Flash 
//       DeviceConfig.MessageSetFlag =1;                                  //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
//       printf("\r\n Alarm Phone Number:%s\r\n",pSetPara);			          //����ʹ��
//			 return 1;
//		}
// 		case 2:            //�洢������ֵ
// 		{
// 			 for(j=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=5
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
//           IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //���㱨����ֵ��������
//        }
// 			 
// 			 Counter =strlen(CharTemp)+1; //����С����
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
//            DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //���㱨����ֵС������
//          }
//        }
// 			 //DeviceConfig.AlarmThreshold =DecimalPart*0.01 + IntegerPart;
//        //ConfigData.threshold.Threshold_float = DecimalPart*0.01 + IntegerPart;
// //		   DataWrite_To_Flash(0,7,0, ConfigData.threshold.SMS_Set_Threshold,4);   //��������ֵ��Hex��ʽ��д��Flash 
// 			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
// 			 return 1;
// 		}
		case 3:              //�洢������IP
		{

			 for(j=0,Counter=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=15
			 {
          if((pSetPara[j]>='0')&&(pSetPara[j]<='9'))
					{
//             CharTemp[j] =pSetPara[j];
						 ;
          }
				  else if(pSetPara[j]=='.')                                    //�ָ���ͳ��
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
          DataWrite_To_Flash(0,5,0, (uint8_t*)pSetPara,strlen(pSetPara));   //��������IPд��Flash 
       }
			 else
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
       }
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			 return 1;
			 
		}
		case 4:           //�洢�������˿ں�
		{
			 DataWrite_To_Flash(0,6,0,(uint8_t*)pSetPara,strlen(pSetPara));      //���������˿ں�д��Flash
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			 return 1;
		}
//		case 5:          //�洢Һλ�����̽ͷ��װ�߶�
//		{
//			for(j=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=5
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
//          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //����̽ͷ��װ�߶���������
//       }
//			 
//			 Counter =strlen(CharTemp)+1; //����С����
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
//           DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //����̽ͷ��װ�߶�С������
//         }
//       }
////			 DeviceConfig.MountingHeight =DecimalPart*0.01 + IntegerPart;
//     //  ConfigData.MountHeight.MountHeight_float = DecimalPart*0.01 + IntegerPart;
//		  // DataWrite_To_Flash(0,8,0, ConfigData.MountHeight.MountingHeigh,4);   //��̽ͷ��װ�߶ȣ�Hex��ʽ��д��Flash 
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
  uint16_t  IntegerPart =0x0000;        //��������
  uint16_t  DecimalPart =0x0000;        //С������
//  uint8_t   DataLength =0;  //����ָʾ��Ч����ת������ݳ���
//  uint8_t*  pData =NULL;
//  uint8_t*  pDot[3] ={NULL};
//  uint8_t*  pDot1 =NULL;
//  uint8_t*  pTemp =NULL;
//  float     AlarmData_F=0;
	
	switch(InstructCode)
	{
		case 1:       //�ֻ�����Ϸ����ж�
		{
			 
			 if(strlen(pSetPara)<11)   //Ĭ����Ч�绰������11λ
			 {
          printf("\r\nPhone Number ERROR_1!!\r\n");
					return 0;
       }
			 CharTemp[0] ='0';
			 memcpy(&(CharTemp[1]), pSetPara, strlen(pSetPara));   //���ú����Ѿ����˷��������
			 Char2Hex(HexData, CharTemp, 12);           
			 for(j=0;j<12;j++)
			 {
         printf("--%x-",HexData[j]);//����ʹ��
       }
			 printf("\r\n");//����ʹ��			 

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
       DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
       printf("\r\n ConfigData.PhoneNum:--%4x--%4x--%4x--\r\n",ConfigData.PhoneNum[0],ConfigData.PhoneNum[1],ConfigData.PhoneNum[2]);			  //����ʹ��
			 return 1;
		}
		case 2:
		{
			 for(j=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=5
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
          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //���㱨����ֵ��������
       }
			 
			 Counter =strlen(CharTemp)+1; //����С����
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
           DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));  //���㱨����ֵС������
         }
       }
			 DeviceConfig.AlarmThreshold =DecimalPart*0.01 + IntegerPart;     //���㱨����ֵ�������� 
			 ConfigData.Threshold = IntegerPart*256+DecimalPart;
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			 return 1;
		}
		case 3:
		{

			 for(j=0;j<strlen(pSetPara);j++)                       //���ú����Ѿ����˷��������,strlen(pSetPara)<=15
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
          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //���������IP��ַ��һ���ֶ�
       }
			 if((IntegerPart==0)||(IntegerPart>255))
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       }				 
			 Counter =strlen(CharTemp)+1;            //�����ָ���
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
          printf("\r\nInput Server IP ERROR !!\r\n");     //����ʹ��
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));   //���������IP��ַ�ڶ����ֶ�
       }
			  if(DecimalPart>255)
			 {
          printf("\r\nInput Input Server IP ERROR!!\r\n");
          return 0;
       }	
			 ConfigData.ServerIP[0] = IntegerPart*256+DecimalPart;
			
			 IntegerPart =0x0000;     //��λ
			 DecimalPart =0x0000;     //��λ
			 j =strlen(CharTemp);           
			 Counter = Counter+j+1;     //�����ָ���
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
            printf("\r\nInput Server IP ERROR !!\r\n");     //����ʹ��
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          IntegerPart =IntegerPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));     //���������IP��ַ�������ֶ�
       }
			  if(DecimalPart>255)
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       }	
			  
			 j =strlen(CharTemp);           
			 Counter = Counter+j+1;     //�����ָ���
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
            printf("\r\nInput Server IP ERROR !!\r\n");     //����ʹ��
          return 0;
       }
       for(i=strlen(CharTemp),j=0; i>=1; i--,j++)
			 {
          DecimalPart =DecimalPart + (CharTemp[i-1]-'0')*(int)(pow(10,j));     //���������IP��ַ���ĸ��ֶ�
       }
			  if(DecimalPart>255)
			 {
          printf("\r\nInput Server IP ERROR!!\r\n");
          return 0;
       }	
			 ConfigData.ServerIP[1] = IntegerPart*256+DecimalPart;
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
			 return 1;
			 
		}
		case 4:
		{
			 if(strlen(pSetPara)!=4)   //Ĭ����Ч�˿ں���4λ
			 {
          printf("\r\nInput Server port ERROR !!\r\n");
					return 0;
       }
	
			 Char2Hex(HexData, pSetPara, strlen(pSetPara));    //���ú����Ѿ����˷��������        
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
			 DeviceConfig.MessageSetFlag =1;     //��ʾ��ǰҺλ�ǲ���ͨ�������޸�		 
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
* Description    : ����δ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Sms_Consult(void)
{
  	
    mput("AT+CMGL=\"REC UNREAD\"");     //��ȡδ����Ϣ
		Delay_ms(1500); 
	  DMA_UART3_RecevDetect(DATARECEV);		
//		mput("AT+CMGD=,3");      //ɾ���Ѷ���Ϣ
		Delay_ms(2500);          //�ȴ�ģ���л������ݴ���״̬
		
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/***********************�������ܣ����Ž���**************************/
/*******************************************************************/
//AT+CNMI=1,2,0,0,0                                //���ն�������
//+CMT: "+8613121342836","","15/08/18,17:05:29+32" //���յ��Ķ��ű��ĸ�ʽ���س����У�0D 0A��֮��Ϊ��Ϣ����
//ok                                               //��Ϣ����
//AT+CNMI=2,1                                      //���ն�������
//+CMTI: "ME",1                                    //���յ��¶�������
//AT+CMGL="REC UNREAD"                             //�跢�Ͳ�ѯδ����������
//+CMGL: 1,"REC UNREAD","+8613121342836","","15/08/18,17:12:19+32"   //���յ��Ķ��ű��ĸ�ʽ���س����У�0D 0A��֮��Ϊ��Ϣ����
//ok                                               //��Ϣ����

void Sms_Analysis(char* pBuff)
{ 

//	char      MessageRead[11] ="REC UNREAD";                            //��ѯ��δ�����Ŵ���
	char      MessageRecevIndicate[5] ="+32\"";                         //���յ�����ָʾ���ַ���
	//char      AlarmPhoneSet[23] ="casic_set_alarm_phone_";              //����������������
	//char      AlarmThresholdSet[27] ="casic_set_alarm_threshold_";      //������ֵ��������
	char      ServerIpSet[21]   ="casic_set_server_ip_";                //������IP��������
	char      ServerPortSet[23] ="casic_set_server_port_";              //�������˿ں���������
	//char      DeviceMountHeight[27] ="casic_set_mounting_height_";      //̽ͷ��װ�߶�����
//	char      LevelDataInquire[33]  ="casic_inquire_current_level_data";//��ǰҺλ���ݲ�ѯ����
	char      CurrentPhoneNum[16] ={0x00};                              //�洢��ǰͨ���ֻ�����
//	char      AlarmThresholdTemp[6] ={0x00};                            //��ʱ���ASCII��ʽ������ֵ
	char*     pSmsRecevBuff =NULL;          //������������λ��ָ��
  char*     pSmsRecevData =NULL;          //����Ŀ������λ��ָ��
  uint8_t   UploadFlag =0;                //����ָʾ�������ò����޸Ľ��
	uint8_t   i=0;                          //ѭ������

//	int       LocationFlag =0;      
//	uint8_t   AlarmPhoneNum[6]={0x00};     //������ú�����Ϣ
//  uint8_t   AlarmThreshold[2]={0x00};    //������ñ�����ֵ
//  uint8_t   ServerIP[4]={0x00};          //��ŷ�����IP��ַ
//  uint8_t   ServerPort[2]={0x00};        //��ŷ������˿ڵ�ַ
//	char      Recev_OK[2]="+CMT:";       //433ģ�����ָʾ
//  uint8_t   SmsRecev_Flag =0;     //����������ȷ�Ա�־����
//****sample:***+CMGL: 0,"REC UNREAD","+861064617006426",,"15/11/12,09:30:02+32"
//****sample****1245
//  pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, MessageRead, sizeof(Usart3_recev_buff),strlen(MessageRead));    //����Ƿ��յ�����
	pSmsRecevBuff = Find_String(pBuff,"\"REC UNREAD\""); //����Ƿ��յ�����
	if(pSmsRecevBuff !=NULL)
	{
		if(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-29)) //��ָֹ��Խ��
		{
      
			 pSmsRecevBuff = pSmsRecevBuff+15;   //ָ��ָ���ֻ�����ǰ׺86
			
			 printf("\r\n111:%s\r\n",pSmsRecevBuff);         //����
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
        memcpy(ConfigData.CurrentPhoneNum,CurrentPhoneNum,i); //��ȡ��ǰͨ���ֻ����룬��ǰ׺86
			  printf("\r\nCurrentPhoneNum:%s\r\n",CurrentPhoneNum); //����ʹ��
    }
		
		 pSmsRecevBuff =NULL;
//		 pSmsRecevBuff = Find_String(pSmsRecevBuff,"+32\"");         //����Ƿ��յ�����
	   pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, MessageRecevIndicate,sizeof(Usart3_recev_buff),strlen(MessageRecevIndicate));    //����Ƿ��յ����ţ����˶��Ų�ѯ����
		 if((pSmsRecevBuff !=NULL) &&(pSmsRecevBuff <(Usart3_recev_buff+sizeof(Usart3_recev_buff)-1-strlen(MessageRecevIndicate))))   //��ָֹ��Խ��
		 {
       pSmsRecevData =pSmsRecevBuff +strlen(MessageRecevIndicate);                          //����ʹ��
			 pSmsRecevBuff =NULL;   
			 printf("\r\nReceive Short Message is: %s\r\n",pSmsRecevData);    //����ʹ��
			  //pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, ServerIpSet,sizeof(Usart3_recev_buff),strlen(ServerIpSet));  //���÷�����IP
			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_server_ip_");    //���÷�����IP
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
				  if((i>15)||(i<7))  //�������ݲ��Ϸ���ֱ�Ӷ�����������������Ϣ��������������ʾ
					{
						 printf("\r\nMessage set input ERROR!!\r\n");     //����ʹ�� 	
//						 i=15;  //i��ʾ��ЧIP��ַ���ȣ����Ƴ��ȣ���ֹ���
          }	
          else
					{
             memset(DeviceConfig.ServerIP,0x00,sizeof(DeviceConfig.ServerIP));
						 memcpy(DeviceConfig.ServerIP, pSmsRecevData, i);		
						 UploadFlag = UploadFlash(DeviceConfig.ServerIP, 3);
          }						
				  if(UploadFlag ==1)
					{
              printf("\r\nServer IP upload success!!\r\n");    //����ʹ��
					  	Delay_ms(2000);  //���ŷ��ͻ���
						  Sms_Send("\rServer IP upload success!!\r");
          }
				  else           //�ԷǷ�����ֵ������ʾ����ֵ�����Ƿ������Զ�ȡ����ֵ��
				  {
             printf("\r\nInput server IP not correct!\r\nPlease check and retry.\r\n");    //����ʹ��
						 Delay_ms(2000);  //���ŷ��ͻ���
						 Sms_Send("\rInput server IP not correct!\rPlease check and retry.\r");
          }
					UploadFlag =0;           //��λ����
				  pSmsRecevBuff = NULL;    //��λ����
       }
			// pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, ServerPortSet,sizeof(Usart3_recev_buff),strlen(ServerPortSet));   //���÷������˿ں�
			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_server_port_");    //���÷������˿ں�
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
				  if((i<2)||(i>5))    //�������ݲ��Ϸ���ֱ�Ӷ�����������������Ϣ��������������ʾ���˿ںŷ�Χ�д�ȷ��
					{
						 printf("\r\nMessage set input ERROR!!\r\n");     //����ʹ�� 	
//						 i=5;  //i��ʾ��Ч�˿ںų��ȣ����Ƴ��ȣ���ֹ���
          }	
   				else
					{
             memset(DeviceConfig.ServerPort,0x00,sizeof(DeviceConfig.ServerPort));
             memcpy(DeviceConfig.ServerPort, pSmsRecevData, i);				 
				     UploadFlag = UploadFlash(DeviceConfig.ServerPort, 4);
          }						
				  if(UploadFlag ==1)
					{
              printf("\r\nServer port upload success!!:%s\r\n",pSmsRecevData);    //����ʹ��
					  	Delay_ms(2000);  //���ŷ��ͻ���
						  Sms_Send("\rServer port upload success!!\r");
          }
				  else           //�ԷǷ�����ֵ������ʾ����ֵ�����Ƿ������Զ�ȡ����ֵ��
				  {
             printf("\r\nInput server port not correct!\r\nPlease check and retry.\r\n");    //����ʹ��
					   Delay_ms(2000);  //���ŷ��ͻ���
						 Sms_Send("\rInput server port not correct!\rPlease check and retry.\r");
          }
					UploadFlag =0;           //��λ����
				  pSmsRecevBuff = NULL;    //��λ����
       }
	    }
  }
}		 
			 
			 
/*****************************************�������ù���*************************************************************/		 			 
//			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_alarm_phone_");  //���ñ�������
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
// 				  if((i<11)||(i>16))    //�������ݲ��Ϸ���ֱ�Ӷ�����������������Ϣ��������������ʾ
// 					{
// 						 printf("\r\nMessage set input ERROR!!\r\n");     //����ʹ��
// //						 i=16;  //i��ʾ��Ч������ֵ���ȣ����Ƴ��ȣ���ֹ���
//           }		
// 					else
// 					{
//              memset(DeviceConfig.AlarmPhoneNum,0x00,sizeof(DeviceConfig.AlarmPhoneNum));
// 				     memcpy(DeviceConfig.AlarmPhoneNum, pSmsRecevData, i);
// 				     UploadFlag = UploadFlash(DeviceConfig.AlarmPhoneNum, 1);
//           }
// 				  if(UploadFlag ==1)
// 					{
//               printf("\r\nAlarm phone number upload success!!\r\n");    //����ʹ��
// 							Delay_ms(2000);  //���ŷ��ͻ���
// 						  Sms_Send("\rAlarm phone number upload success!!\r");
//           }
// 					else
// 					{
//               printf("\r\nAlarm phone number not correct!\r\nPlease check and retry.\r\n");    //����ʹ��
// 							Delay_ms(2000);  //���ŷ��ͻ���
// 						  Sms_Send("\rAlarm phone number not correct!\rPlease check and retry.\r");
//           }
// 					UploadFlag =0;           //��λ����
// 				  pSmsRecevBuff = NULL;    //��λ����
//        }
// 			 
// //			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_set_alarm_threshold_");    //���ñ�����ֵ
// 			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, AlarmThresholdSet,sizeof(Usart3_recev_buff),strlen(AlarmThresholdSet));   //���ñ�����ֵ
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
//              printf("\r\nMessage set input ERROR!!\r\n");     //����ʹ��
//           }
// 				  else 
// 					{
// 						 if(i>5)
// 						 {
//                i=5;  //i��ʾ��Ч������ֵ���ȣ����Ƴ��ȣ���ֹ���
//              }
// 						 memcpy(AlarmThresholdTemp, pSmsRecevData, i);		
// 				     UploadFlag = UploadFlash(AlarmThresholdTemp, 2);
//           }		
// 				  if(UploadFlag ==1)
// 					{
//               printf("\r\nAlarm threshold upload success!!\r\n");    //����ʹ��
// 						  Delay_ms(2000);  //���ŷ��ͻ���
// 						  Sms_Send("\rAlarm threshold upload success!!\r");
//           }
// 				  else           //�ԷǷ�����ֵ������ʾ����ֵ�����Ƿ������Զ�ȡ����ֵ��
// 				  {
//              printf("\r\nInput alarm threshold not correct!\r\nPlease check and retry.\r\n");    //����ʹ��
// 						 Delay_ms(2000);  //���ŷ��ͻ���
// 						 Sms_Send("\rInput alarm threshold not correct!\rPlease check and retry.\r");
//           }
// 					UploadFlag =0;           //��λ����
// 				  pSmsRecevBuff = NULL;    //��λ����
//        }
			
			 
// 			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, DeviceMountHeight,sizeof(Usart3_recev_buff),strlen(DeviceMountHeight));   //����̽ͷ��װ�߶�
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
//              printf("\r\nMessage set input ERROR!!\r\n");     //����ʹ��
//           }
// 				  else 
// 					{
// 						 if(i>5)
// 						 {
//                i=5;  //i��ʾ��Ч̽ͷ��װ�߶����ݳ��ȣ����Ƴ��ȣ���ֹ���
//              }
// 						 memset(AlarmThresholdTemp,0x00,sizeof(AlarmThresholdTemp));
// 						 memcpy(AlarmThresholdTemp, pSmsRecevData, i);		
// 				     UploadFlag = UploadFlash(AlarmThresholdTemp, 5);
//           }			
// 				  if(UploadFlag ==1)
// 					{
//               printf("\r\nMounting height upload success!!:%s\r\n",pSmsRecevData);    //����ʹ��
// 					  	Delay_ms(2000);  //���ŷ��ͻ���
// 						  Sms_Send("\rMounting height upload success!!\r");
//           }
// 				  else           //�ԷǷ�����ֵ������ʾ����ֵ�����Ƿ������Զ�ȡ����ֵ��
// 				  {
//              printf("\r\nInput mounting height not correct!\r\nPlease check and retry.\r\n");    //����ʹ��
// 					   Delay_ms(2000);  //���ŷ��ͻ���
// 						 Sms_Send("\rInput mounting height not correct!\rPlease check and retry.\r");
//           }
// 					UploadFlag =0;           //��λ����
// 				  pSmsRecevBuff = NULL;    //��λ����
//        }
// 			 
/*****************************************��ѯ����****************************************************************/		 
////			 pSmsRecevBuff = Find_String(pSmsRecevData,"casic_inquire_current_level_data");    //��ѯҺλ��Ϣ
//			 pSmsRecevBuff = Find_SpecialString(Usart3_recev_buff, LevelDataInquire,sizeof(Usart3_recev_buff),strlen(LevelDataInquire));   //��ѯҺλ��Ϣ
//			 if(pSmsRecevBuff != NULL)           //�в�ѯҺλ���ݲ�ѯ����
//			 {
//				  if(DataCollectCount==0)          //���ݲɼ������ֱ��ͨ�����ŷ��͵�ǰҺλ��Ϣ
//					{
//             LevelDataMessageSend();
//          }
//					else
//					{
//             ConfigData.LiquidDataInquireFlag =1;  //Һλ����δ���вɼ�ʱ���յ���ѯ�������־������1����Һλ���ݲɼ����ʱͨ�����ŷ��͵�ǰҺλ��Ϣ
//          }
//				  pSmsRecevBuff = NULL;            //��λ����
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

//   unsigned char    NetLoginFlag =0;     //ģ��ע���վ״̬��־����
	 char*        pRecevBuff =NULL;
//   char   SendArry[50] ={'\0'};
//	 char   Recev_OK[2]={0xAA,0x1D};          //433ģ�����ָʾ
//   char   NewMessageIndicate[7] ="+CMGL:";  //�յ�δ������ָʾ
//   char   ModuleError[6] ="ERROR";          //GPRSģ�����ָʾ


	 #if DEBUG_TEST
   printf("\r\nGPRS Net Login Receive Analysis ...\r\n");          //����ʹ��
	 #endif

	  ////////////////////////////////////////////////////////////////////////////////////////////////////////
    pRecevBuff = Find_SpecialString(Usart3_recev_buff, "+CGREG: 1,1", sizeof(Usart3_recev_buff), 11);  //���ģ������״̬
		if(pRecevBuff!=NULL)                                                
		{   
			 printf("\r\nNet Register OK!!\r\n");   
			 CSQ_OK =1;              //��������Ӧʱ����־������1			 
		   pRecevBuff =NULL;
			 return  1;
		}	 
		//////////////////////////////////////////////////////////////////////////////////////////////////////
    pRecevBuff = Find_SpecialString(Usart3_recev_buff, "+CGREG: 1,5", sizeof(Usart3_recev_buff), 11);  //���ģ������״̬
		if(pRecevBuff!=NULL)                                                
		{   
			 printf("\r\nNet Register OK!!Roaming on!!\r\n");   
			 CSQ_OK =1;              //��������Ӧʱ����־������1			 
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
   printf("\r\nTCP Connect is in process...\r\n");          //����ʹ��
	 #endif

		pRecevBuff = Find_SpecialString(Usart3_recev_buff, "CONNECT", sizeof(Usart3_recev_buff), 7);  //���ģ������״̬
		if(pRecevBuff != NULL)                                       //�������ӳ��ֹ���ʱ����3Gģ�鸴λ
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
	 unsigned char     Recev_Flag2 =0;     //����������ȷ�Ա�־����
	 char*  pRecevBuff =NULL;
	 char  i=0;
//   char   SendArry[50] ={'\0'};
//	 unsigned char   ServerNoiseSet[2] = {0xFF,0x02};
   unsigned char   ServerNoiseSet[2] = {0xA3,0x20};
   char   NewMessageIndicate[7] ="+CMGL:";  //�յ�δ������ָʾ
//   char   ModuleError[6] ="ERROR";          //GPRSģ�����ָʾ

	 #if DEBUG_TEST
   printf("\r\n�������ݽ���!!\r\n");          //����ʹ��
	 #endif
//////////////////////////////���Ź��ܣ���ʱ����////////////////////////////////////////////////////////////
	 pRecevBuff = Find_SpecialString(Usart4_recev_buff, NewMessageIndicate,sizeof(Usart4_recev_buff),strlen(NewMessageIndicate));    //����Ƿ��յ�����
   {
			if(pRecevBuff !=NULL)
			{
         Sms_Analysis(pRecevBuff);          //���ն��Ž���
				 pRecevBuff =NULL;                  //��λ��ѯָ��
				 Delay_ms(3000);                    //�ȴ�ʱ�䲻��̫�̣������޷��ɹ���ն��ż�¼   
				 mput("AT+CMGD=1,3");               //ɾ��ȫ���ѷ���δ�����Ѷ�����
				 Delay_ms(500);           
//				 GPRS_Init();             //���Ž�������Ժ���Ҫ���½�����������
				 return 1;
		  }  
   }
	 
///////////////////////////////////////////////////////////////////////////////////////////////////
	 
//	 printf("\r\nUart3���ڽ�����Ϣ���:%4x\r\n", Usart3_recev_buff);         //����ʹ��
	 printf("\r\nUart3���ڽ�����Ϣ���:");
						 for(i=0;i<Usart4_recev_count;i++)
						 {
								printf(" %x",Usart4_recev_buff[i]);
						 }
	 
	 pRecevBuff = Find_SpecialString(Usart4_recev_buff,(char*)ServerNoiseSet,sizeof(Usart4_recev_buff),sizeof(ServerNoiseSet));  //��������յ���վ�ظ�
	 if((pRecevBuff != NULL)&&(pRecevBuff< Usart4_recev_buff+sizeof(Usart4_recev_buff)-1-24))  //��ָֹ��Խ�� 
//	 if(pRecevBuff != NULL)  //��ָֹ��Խ��          		 
	 {	 
			
		  printf("\r\nReceive Server Config Data!\r\n");    //����ʹ��
		  for(i=0;i<60;i++)
		  {
         printf("\r\n--%x--\r\n",pRecevBuff[i]);         //����ʹ��
      }
		  Recev_Flag2 = ReceiveMessageVerify( pRecevBuff );
			if(Recev_Flag2==0)                                 //��ǰ���ҵ��Ľ�����������
			{
				pRecevBuff = NULL;
				#if DEBUG_TEST	 
				printf("\r\nReceive  data not correct!!\r\n");   //����ʹ��
				#endif
			}
			else                                               //����������ȷ
			{
				
				SetRev_OK = 1;
//				Receive_Analysis_GPRS();                         //����ʹ�� ��ʽ��ɾ��
				Noise_Count = 0;
				Delay_ms(500);	
				pRecevBuff =NULL;
			  return 1;
			}
	 }
	
	 #if DEBUG_TEST	 
	 printf("\r\n�������ݽ������!!\r\n");                 //����ʹ��
   #endif 
	 return 0;
	
}

/**********************************************END******************************************/













