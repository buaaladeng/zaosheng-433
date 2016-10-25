/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   �����жϽ��ղ���
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO-MINI STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
#include "bsp_SysTick.h"
#include "string.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "common.h"
#include "gprs.h"
#include "SPI_Flash.h"

//#define snap 6
extern struct liquid_set  DeviceConfig;     //Һλ��������Ϣ�ṹ��
extern uint16_t  Noise_Count;
extern char      Usart2_recev_buff[50];        //USART2���ջ���
extern u8 		Dense_Data[60];
extern char			Dense_Num;
extern uint16_t	WakeSecond;
extern uint16_t	WakeTime;
extern uint16_t SampleSpan;
extern char SLNoise_char[300];
extern int   Length_Frame; 

uint8_t Tx_Buffer1[5] = {4,3,2,1};
uint8_t Rx_Buffer1[5];
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void gotoSleep(char noisesensorflag)
{

	int i;
	struct rtc_time* ptm;
	struct rtc_time tm;	
	
	ptm = &tm;
	//RTC_Configuration();
	//Sms_Consult();  SLNoiseDataBkp(SLNoise_char,Length_Frame);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	
	
	if((Noise_Count >= Dense_Num)&&(!noisesensorflag))
	{
		SLNoiseDataBkp(SLNoise_char,Length_Frame);
  }
	if(!noisesensorflag)
	{
		Noise_Count = 0;
		BKP_WriteBackupRegister(BKP_DR11, Noise_Count); 
	
		printf("Clear....!!!! %d \r\n",Noise_Count);
		printf("WakeTime....!!!! %d \r\n",WakeTime);
		Time_Show(ptm);
		printf("\r\n------Sleep 24 hours-----!!!!!\r\n");
    if((WakeTime*60 + WakeSecond) > ((ptm->tm_hour)*60+(ptm->tm_min))*60)
		{
			RTC_SetAlarm(RTC_GetCounter()+(WakeTime-(ptm->tm_hour)*60-(ptm->tm_min))*60 + WakeSecond);     //����ʱ��������豸ID���ɵ��������������豸������ͻ
			printf("\r\n------WakeTime > ((ptm->tm_hour)*60+(ptm->tm_min))-----!!!!!\r\n");
    }else
    {
			RTC_SetAlarm(RTC_GetCounter()+(WakeTime+1440-(ptm->tm_hour)*60-(ptm->tm_min))*60 + WakeSecond);
			printf("\r\n------WakeTime < ((ptm->tm_hour)*60+(ptm->tm_min))-----!!!!!\r\n");
    }			
//	  RTC_SetAlarm(RTC_GetCounter()+10);
//    RTC_SetAlarm(RTC_GetCounter()+((25-(ptm->tm_hour))*60-(ptm->tm_min))*60);                       //�ɼ�ʱ�䵽��ʼ����
		RTC_WaitForLastTask();
	}
	else
	{
		BKP_WriteBackupRegister(BKP_DR11, Noise_Count);
		Dense_Data[Noise_Count-1] = Usart2_recev_buff[0];
		TempNoiseDataBkp();//SPI_FLASH_BufferWrite(Dense_Data,Addr,Noise_Count);
		//SPI_FLASH_BufferRead(Dense_Data,Addr,Noise_Count);
		for(i=0;i<Noise_Count;i++)
	  {	
				printf(" %x  ",Dense_Data[i]);       
	  }
		Delay_ms(100);

		PowerOFF_NoiseSensor();
	  printf("\r\nPowerOFF!\r\n");		
	
		printf("\r\n------Sleep 3min-----!!!!!\r\n"); 
		RTC_SetAlarm(RTC_GetCounter()+3);   //�ɼ�ʱ�䵽��ʼ����3s
//		RTC_SetAlarm(RTC_GetCounter()+SampleSpan*60);   //�ɼ�������ɼ�ʱ�䵽��ʼ����SampleSpan*60

//		RTC_SetAlarm(RTC_GetCounter()+BKP_ReadBackupRegister(BKP_DR2)*60);   //�ɼ�������ɼ�ʱ�䵽��ʼ����BKP_ReadBackupRegister(BKP_DR2)*60
		RTC_WaitForLastTask();
	}
  
  if(DeviceConfig.MessageSetFlag ==1)	     //�ж����޸Ĳ���ʱ��������д��洢��
	{
//    ParaUpdateCheck();                     //д��������
		 printf("\r\nDevice Set Upload Success According to Short Messages!!\r\n");                    //����ʹ��
  }

  #if DEBUG_TEST	 
	printf("\r\n-----BKP_DR2:%d----BKP_DR3:%d-----BKP_DR4:%d----BKP_DR5:%d----\r\n",BKP_ReadBackupRegister(BKP_DR2),BKP_ReadBackupRegister(BKP_DR3),BKP_ReadBackupRegister(BKP_DR4),BKP_ReadBackupRegister(BKP_DR5));    //����ʹ��
	#endif
	PowerOFF_NoiseSensor();
	Delay_ms(100);
//	PowerOFF_GPRS(); 
	Delay_ms(100);
	PowerOFF_Flash(); 
	Delay_ms(500);
	PowerON_433_EN();         //Power_433_EN�������ߣ���������ģʽ
	Delay_ms(100);
	PowerON_433_SET();        //Power_433_SET�������ߣ���������ģʽ
	Delay_ms(100);
	
	PWR_WakeUpPinCmd(ENABLE);  //ʹ��WAKE-UP�ܽ�	
	PWR_EnterSTANDBYMode();	
}

