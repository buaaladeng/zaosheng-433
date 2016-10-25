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


// ADC1ת���ĵ�ѹֵͨ��MDA��ʽ����SRAM
extern __IO uint16_t ADC_ConvertedValue;

// �ֲ����������ڱ���ת�������ĵ�ѹֵ 	 
//float ADC_ConvertedValueLocal;        

struct rtc_time    systmtime;        //RTCʱ�����ýṹ��
struct liquid_set  DeviceConfig;     //Һλ��������Ϣ�ṹ��
struct SMS_Config_RegPara   ConfigData;     //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
uint16_t  WWDOG_Feed =0x1FFF;        //���ڿ��Ź���λ����Ϊ��XX*1.8s = 7.6min
char      SetRev_OK =0;              //�ɹ����շ���������
char      DatRev_OK =0;              //�ɹ���ȷ����Һλ����
char      Alive =0;                  //�����߱�־����,Ϊ1ʱ˵�����������ߴ����д�����
uint8_t   PowerOffReset =0;          //����������־λ
uint8_t   LiquidDataSend_Flag =0;
uint8_t   SLNoiseConfigFlag = 0;
//uint8_t   Batch_Num =0x01;                     //�����������
//uint8_t   Batch_Sum =0x01;                     //������������
//uint8_t   DataCollectBkCount =0;               //�������ݲɼ�������
//uint8_t   DataCollectCache[13][4]={'\0'};      //Һλ���ݲɼ����棬���13�����ݣ�������HEX��ʽ�洢�����ֽ���ǰ�����ֽ��ں�
float     LevelData_Float[FILTER_ORDER]={0.0};//�ɼ�������ʱҺλ���ݣ�������
extern    uint8_t  LevelDataCount ;              //Һλ���ݼ���������ʾ��ǰ�ɼ�����������
uint8_t   DataCollectCount =1;                 //���ݲɼ�������
char      Usart1_recev_buff[300] ={'\0'};      //USART1���ջ���
uint16_t  Usart1_recev_count =0;               //USART1���ͼ�����
char      Usart2_recev_buff[4]={'\0'};        //USART2���ջ���
unsigned char 			Dense_Data[60] = {'\0'};
unsigned char 			Noise_Data[60] = {'\0'};
char      SLNoise_char[300]={'\0'};           //���ת�����char��ʽ����
int       Length_Frame = 0;
uint8_t   Usart2_recev_count =0;               //USART2���ռ�����
uint8_t   DMA_UART3_RECEV_FLAG =0;             //USART3 DMA���ձ�־����
uint16_t  Noise_Count=0x0000;   
//uint16_t  BATT_Capacity =19000;                //һ���ص�ǰʣ�������������19Ah��

uint16_t  SampleTime = 60;                       //�ɼ�ʱ��
uint16_t  SampleSpan = 60;                       //�ɼ����
char			Dense_Num = 20;                        //�ɼ�����
char			UploadCount = 1;                       //�ϴ����Դ���
uint16_t  WakeTime = 60;                       //�ɼ�ʱ��
uint16_t  WakeSecond = 0;                       //�ɼ�ʱ��

char DeviceId[6] = {0x21,0x20,0x16,0x09,0x99,0x01};   //������¼���豸���
u16 NodeAddr =0x0000;
char 			BkpCount;
char      NoiseSensorFlag=5;

extern  char      Usart3_recev_buff[1000];
extern  uint16_t  Usart3_recev_count;

extern  char      Usart4_recev_buff[1000];
extern  uint16_t  Usart4_recev_count;

extern  uint8_t   CSQ_OK ;                  //�ź�������־����
extern  uint8_t   DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE];
extern  struct    DMA_USART3_RecevConfig   DMA_USART3_RecevIndicator; 
extern  int NetErrorCount;
SLNoiseSet Msg;
SLNoiseSet* pMsg = &Msg;
uint32_t  time=0 ;                   // ms ��ʱ����  
char      Usart1_send_buff[300]={'\0'};       //USART1���ͻ���
uint8_t   Usart1_send_count=0;                 //USART1���ͼ�����
uint32_t  Tic_IWDG=0;                //�������Ź�ι��ʱ������
extern  char  Usart3_send_buff[];
extern  uint8_t  Usart3_send_count;                     
//extern  struct SMS_Config_RegPara   ConfigData;  //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
//extern  float  Data_Liquid_Level;
//extern void     Receive_Analysis_GPRS(void);	
//extern char     Receive_Monitor_GPRS(void);
//extern void     Receive_Deal_GPRS(void);
//extern uint8_t  NetStatus_Detection( void );
extern void     Delay(uint32_t nCount);
extern void     LSLIQUID_DataCollect(void);          
extern void     RecvBuffInit_USART3(void);

unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag);     //USART3�������ݼ�������ݽ���
void  PeripheralInit( void );          //��ʼ����Χ�豸
void  LevelDataMessageSend(void);      // ͨ�����ŷ���Һλ����
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
//		 Usart3_recev_count=0;                           //���USART3���ռ�����
//		 time=0;	                                       //��ʱ����λ
//   
//}
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : ��DMA���մ洢������ȡ��Ч���ݣ�����Usart3_recev_buff[],���ں������ݽ���
* Input          : None
* Output         : None
* Return         : �������ݳ���
*******************************************************************************/
int  DMA_UART3_RecevDataGet(void)
{
   int i=0,j=0;
	 uint8_t VoidCount=5;
	 uint8_t FilterDepth=10;   //�������Ϊ10
	
  
   printf("\r\nDMA_UART3_RecevDataGet start!!%d---%d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //����ʹ��
   memset(Usart3_recev_buff,'\0',sizeof(Usart3_recev_buff));
	 DMA_USART3_RecevIndicator.CurrentDataStartNum =DMA_USART3_RecevIndicator.NextDataStartNum ;
	 i = DMA_USART3_RecevIndicator.CurrentDataStartNum;
	 if(i>=RECEIVEBUFF_SIZE)
	 {
     printf("\r\nCurrent Data Start Number Error!! \r\n");    //����ʹ��
		 DMA_USART3_RecevIndicator.NextDataStartNum =0 ;   //��������ʱ��λ
		 return 0;
   }
	 if(DMA_USART3_RecevBuff[i]==0x00)   
	 {
     while(FilterDepth!=0)   //�˳���ʼ�Ŀ��ֽڣ��������Ϊ10
		 {
       FilterDepth--;
			 if(i< RECEIVEBUFF_SIZE-1)
			 {
					i++;
			 }
			 else   
			 {
					i=0;      //������DMA���ݴ洢������׶���Ȼδ������Ч���ݣ���ջ��ص��洢������
			 }
			 if(DMA_USART3_RecevBuff[i]!=0x00) 
			 {
          break;
       }
     }
   }		
   if(DMA_USART3_RecevBuff[i]!=0x00)    //��ʼ�ֽڷǿգ���ʾ����������Ч
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
					 i=0;      // ������DMA���ݴ洢������׶���Ȼδ������Ч���ݣ���ջ��ص��洢������
				 }
			 }
			 if(i==0)
			 {
          DMA_USART3_RecevIndicator.CurrentDataEndNum = RECEIVEBUFF_SIZE-1;
       }
			 else
			 {
          DMA_USART3_RecevIndicator.CurrentDataEndNum =i-1;  // DMA_USART3_RecevBuff[i]== 0x00, ��ʾ����λ��
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
					 i=0;      //������DMA���ݴ洢������׶���Ȼδ������Ч���ݣ���ջ��ص��洢������
				 }
				 if(DMA_USART3_RecevBuff[i]!=0x00)
				 {
					 VoidCount =5;      //���ǽ������н�β��������λ
					 break;
				 }
			 }
			 if(VoidCount==0)     //��⵽�������еĽ�β
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
///////////////////////////////////////�ҵ�������ʼ�����λ���Ժ󣬽����ݸ��Ƶ�ָ�����飬���ں�������/////////////////////////////////////////
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
		 printf("\r\nRecev OKK:%d\r\n",j);    //����ʹ��			
     return j;
   }
	 else
	 {
      printf("\r\nFirst Void\r\n");    //����ʹ��	 
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
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //���ô���DMA1����
	
  if(DMA_UART3_RECEV_FLAG ==1)
  {
		 DataLength = DMA_UART3_RecevDataGet();
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);           //����ʹ��
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);         //����ʹ��
			  for(i=0;i<DataLength;i++)
			  {
             printf("%x ", Usart3_recev_buff[i]);               //����ʹ��
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
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //��λDMA���ݽ���BUFF
		 
		 USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);  //���ô���DMA1����

     DMA_UART3_RECEV_FLAG =0;
		 
  }
	
	return StateFlag;
}


/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : ��ʼ���˿ڼ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void PeripheralInit( void )
{  
		
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//���ȿ������ڿ��Ź���������ֵΪ7f,���ڼĴ���Ϊ5f,��Ƶ��Ϊ8	
	USART1_Config();      /* USART1 ����ģʽΪ 9600 8-N-1��  �жϽ��� */
	USART2_Config();      /* USART2 ����ģʽΪ 115200 8-N-1���жϽ��� */
	USART3_Config();      /* USART3 ����ģʽΪ 115200 8-N-1���жϽ��� */
 	USART4_Config();      /* UART4  ����ģʽΪ 9600 8-N-1��  �жϽ��� */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();	
	
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //���ô���DMA1����
  TIM2_Configuration();       /* ��ʱ��TIM2�������� */	
	TIM2_NVIC_Configuration();  /* ���ö�ʱ��TIM2���ж����ȼ� */
//	TIM3_Configuration();       /* ��ʱ��TIM3�������� */	
//	TIM3_NVIC_Configuration();  /* ���ö�ʱ��TIM3���ж����ȼ� */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //��ʱ�رն�ʱ��TIM3
	
	RTC_NVIC_Config();                 /* ����RTC���ж����ȼ� */
//	RTC_Configuration();       //
	RTC_CheckAndConfig(&systmtime);
 	Time_Show(&systmtime);             /* Display time in infinite loop */
  SysTick_Init();
//	SPI_FLASH_Init();
//	PowerON_GPRS();                    //��GPRSģ���Դ
	PowerON_Flash(); 
//	Delay_ms(1000);  

	///////////////////���뽫ģ�����óɽ���״̬�����򴮿ڽ��ղ�������
	PowerON_433_SET();                      //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(100);
  PowerOFF_433_EN();                      //433ģ��EN�ܽ����ͣ��л������ٷ���ģʽ
	Delay_ms(100);

	ConfigData_Init(&DeviceConfig);
	//BatteryMonitorInit();              //���ؼƳ�ʼ��	
		
//  Delay_ms(1000);
//	StartupRequest(2);                  //�豸���ѻظ�
//	Delay_ms(5000);

	if (PowerOffReset ==1)     
  {		
		printf("\r\n�������������³�ʼ�����ؼ�\r\n");                 //����ʹ��
	 	Set_register_ds2780();    //�����Կ��ؼ����³�ʼ��
	  set_ACR(1000);           //�����Կ��ؼ����³�ʼ��
	  DS2780_CapacityInit();    //���������д�������
		DS2780_Test();
		SetNodeID( NodeAddr );    //���ýڵ�ID�ţ��Ӳ�ƷDevice�������ȡ
    
//		StartupRequest(PowerOffReset); //�豸��������
		Delay_ms(1500);
  }	
}

/*******************************************************************************
* Function Name  : int main(void)
* Description    : ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main( void )
{
		
	#ifdef DEBUG
  debug();
  #endif

	NodeAddr =DeviceId[4]*256 +DeviceId[5];     //��ȡ�豸ID������������ֽ���Ϊ�ڵ��ַ
		
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
		printf("���ø��¡���");
//		printf("\r\n The current BKP_ReadBackupRegister(BKP_DR4) value = %d \r\n",BKP_ReadBackupRegister(BKP_DR4));
	}	
  else
	{
    printf("����δ���¡���");
		SampleTime = 60;
		SampleSpan = 1;	 
		Dense_Num = 20;
    UploadCount = 3;		
	}
	while(1)
  {
		  WWDOG_Feed =0x1FFF;                          //���ڿ��Ź�ι��,��ʱ4��20�룬�ڶ��ֽ�Լ1��仯һ�Σ���0x09AF�䵽0x099FԼ��ʱ1��
		  Usart2_recev_count1 = 0;	
		
		  for(i=0;i<10;i++)
		  {
			   Delay_ms(500);
				 printf("\r\n The current i value =%d \r\n",i);
         receive_flag = Receive_Monitor_433();				
			   if(receive_flag == 1)                      //��ѯ���ݽ������
			   {
				   printf("\r\nUart4���ڽ�����Ϣ���:");
		       for(j;j<Usart4_recev_count;j++)
		       {
			       printf(" %x",Usart4_recev_buff[j]);
		       }

					 GPRS_Receive_DataAnalysis();                                 //!!!�豸�����Ժ��һ�η�������֮ǰ������н��ս���������ɾ�������뱣�����������׳���!!!
					 memset(Usart4_recev_buff,'\0',300);	
					 Usart4_recev_count =0;                                       //���USART4���ռ�����
					 time=0;	                                                     //��ʱ����λ    
			   }
      }
			
			Delay_ms(1000);				
			SLNoise_DataUpload(&DeviceConfig, 1); //�����ϴ�
			Delay_ms(1500);

			printf("\r\n??????????????????????");
			Delay_ms(5000);         //�ȴ����ŵ���
			gotoSleep(0);		 //������
		
//			printf("\r\nPowerON!\r\n");
//			Delay_ms(5000);
//																		
//			ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*3.3; // ��ȡת����ADֵ
//			printf("\r\n The current AD value = 0x%04X \r\n", ADC_ConvertedValue); 
//      printf("\r\n The current AD value = %f V \r\n", ADC_ConvertedValueLocal);
//							
//			Usart2_recev_buff[0] = ADC_ConvertedValue/100;// ������������ֵ��������
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
//				SLNoise_DataUpload(&DeviceConfig); //�����ϴ�
//				Delay_ms(1500);

//				printf("\r\n??????????????????????");
//				Delay_ms(5000);         //�ȴ����ŵ���
//				gotoSleep(0);		 //������
//			}
//			gotoSleep(1);	//������
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
    char* pRecevBuff =NULL;  //����ʹ��
	  u8 i=0;                               //����ʹ��
		if(Usart4_recev_count!=0)
		{
				USART_ClearITPendingBit(UART4, USART_FLAG_RXNE);
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);    /* �ر�TIM2  */
			
				#if DEBUG_TEST
				printf("\r\nUART4:");               //����ʹ��
				for(i=0;i<50;i++)
				{
					printf(" %x ",Usart4_recev_buff[i]);      //����ʹ��
				}
				pRecevBuff =&Usart4_recev_buff[i];    //����ʹ��
				printf("%s\r\n",pRecevBuff);          //����ʹ�� 
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
		 unsigned char     Recev_Flag2 =0;     //����������ȷ�Ա�־����
		 char*  pRecevBuff =NULL;
     char*  ptemp =NULL;
	   int   count =0;
		 char  Recev_OK[2]={0xAA,0x1E};       //433ģ�����ָʾ

		 pRecevBuff = Find_SpecialString(Usart4_recev_buff,Recev_OK,300,2);  //��������յ���վ�ظ�
		 if(pRecevBuff != NULL)
		 {	
			 
start:	   
 			  ptemp =pRecevBuff;
			  count =pRecevBuff -Usart4_recev_buff+2;            //��һ�β��Ұ�ͷ�ķ�Χ
			  Recev_Flag2 = ReceiveMessageVerify( pRecevBuff );
				if(Recev_Flag2==0)                                 //��ǰ���ҵ��Ľ�����������
				{
						pRecevBuff =NULL;
						pRecevBuff = Find_SpecialString(ptemp-2,Recev_OK,(300-count),2);  //��������յ���վ�ظ�
						if(pRecevBuff !=NULL)
						{
							#if DEBUG_TEST	 
							printf("\r\n�ڶ��β��ң���ǰ����λ��: %d\r\n",count);   //����ʹ��
						  #endif
							goto start;
						}
						else           //���Ҳ������հ�ͷ��ֱ�Ӷ�������
						{
							#if DEBUG_TEST	 
							printf("\r\nReceive  data not correct!!\r\n");   //����ʹ��
							#endif
							memset(Usart4_recev_buff,'\0',300);	
							Usart4_recev_count=0;                            //���USART3���ռ�����
							time=0;	                                         //��ʱ����λ
						}
			   }
				 else           //����������ȷ
			 	 {
//				  	Receive_Analysis_433();            //������������
//					  GPRS_Receive_DataAnalysis();
//					  DMA_UART3_RecevDetect(DATARECEV);	                    //�������ݽ�������
				    memset(Usart4_recev_buff,'\0',300);	
			      Usart4_recev_count=0;                          //���USART3���ռ�����
		        time=0;	                                       //��ʱ����λ
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
//	char  Section_Req[53]="SLRESET:FF,02,0E,51,212015090053,212015090053,030D0A";		//�豸��������
	char  section_GPRS[52] ="SLGPRS:FF,02,0E,61,212015090053,212015090053,030D0A";
	unsigned char     Recev_Flag1=0;     //USART3�������ݱ�־����
	u8    SetReq_Count =3;               //�����·����ó��Դ�������ʱ��Ϊ��ೢ������
	u8    NetStatusFlag =1;                //����״̬��־����

	SetRev_OK = 0;
	while(SetReq_Count !=0)
	{
		SetReq_Count--;
//		SendMessage(Section_Req,strlen(section_GPRS));
		NetStatusFlag = SendMessage(section_GPRS,strlen(section_GPRS));
		if(SetRev_OK ==1)         //�ɹ��������ã��˳�ѭ��
		{
			SetRev_OK = 0;
			break;
    }
		Delay_ms(10000);	
		Recev_Flag1 = Receive_Monitor_433();
		if(Recev_Flag1==1)                                    //USART3���յ�����		
		{
			Receive_Deal_433();
			Recev_Flag1 =0;
			#if DEBUG_TEST	 
			printf("\r\n�ڶ��ν��ղ鵽����!!\r\n");   //����ʹ��
			#endif
		}  		
	}
	
	if(NetStatusFlag==0)                         //��������쳣ʱ���豸��������
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
			   if(receive_flag == 1)                      //��ѯ���ݽ������
			   {
				   printf("\r\nUart4���ڽ�����Ϣ���:");
		       for(j=0;j<Usart4_recev_count;j++)
		       {
			       printf(" %x",Usart4_recev_buff[j]);
		       }

					 GPRS_Receive_DataAnalysis();                                 //!!!�豸�����Ժ��һ�η�������֮ǰ������н��ս���������ɾ�������뱣�����������׳���!!!
					 memset(Usart4_recev_buff,'\0',300);	
					 Usart4_recev_count =0;                                       //���USART4���ռ�����
					 time=0;	                                                     //��ʱ����λ    
			   }
      }
}