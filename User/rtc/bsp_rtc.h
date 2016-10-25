

#ifndef __RTC_H
#define	__RTC_H


#include "stm32f10x.h"
//#include "bsp_calendar.h"
#include "bsp_date.h"
#include "bsp_usart.h"
#include "AiderProtocol.h"
#include "433_Wiminet.h"

/* 如果定义了下面这个宏的话,PC13就会输出频率为RTC Clock/64的时钟 */   
//#define RTCClockOutput_Enable  /* RTC Clock/64 is output on tamper pin(PC.13) */

void RTC_NVIC_Config(void);
void RTC_Configuration(void);
//void Time_Regulate(struct rtc_time *tm);                                     //通过超级终端修改RTC时间使用
void Time_Auto_Regulate(SLNoiseSet* time_check);                        //通过服务器修改RTC时间使用
//void Time_Adjust(struct rtc_time *tm);                                         //通过超级终端修改RTC时间使用
void Time_Display(uint32_t TimeVar,struct rtc_time *tm);
void Time_Show(struct rtc_time *tm);
//void LowPower_Configuration(void);
//static uint8_t USART_Scanf(uint32_t value);
void RTC_CheckAndConfig(struct rtc_time *tm);

#endif /* __XXX_H */
