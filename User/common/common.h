#ifndef _COMMON_H_
#define _COMMON_H_

#include "stm32f10x.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define DEBUG_TEST 1   //���δ�ӡ����

void gotoSleep(char SendCount);

#define  PowerON_GPRS()               GPIO_SetBits(GPIOA,GPIO_Pin_7)   //Power_GPRS(433)_EN��������
#define  PowerOFF_GPRS()              GPIO_ResetBits(GPIOA,GPIO_Pin_7) //Power_GPRS(433)_EN��������
#define  PowerON_UltrasonicSensor()   GPIO_SetBits(GPIOA,GPIO_Pin_8)   //Power_12V_EN��������
#define  PowerOFF_UltrasonicSensor()  GPIO_ResetBits(GPIOA,GPIO_Pin_8) //Power_12V_EN��������
#define  PowerON_485()                GPIO_SetBits(GPIOA,GPIO_Pin_11)  //Power_485_EN��������
#define  PowerOFF_485()               GPIO_ResetBits(GPIOA,GPIO_Pin_11)//Power_485_EN��������
#define  PowerON_Flash()              GPIO_SetBits(GPIOA,GPIO_Pin_12)  //Power_Flash_EN��������
#define  PowerOFF_Flash()             GPIO_ResetBits(GPIOA,GPIO_Pin_12)//Power_Flash_EN��������
#define  PowerON_NoiseSensor()        GPIO_SetBits(GPIOA,GPIO_Pin_3)   //Power_12V_EN��������
#define  PowerOFF_NoiseSensor()       GPIO_ResetBits(GPIOA,GPIO_Pin_3) //Power_12V_EN��������
#define  PowerON_433_EN()            GPIO_SetBits(GPIOC,GPIO_Pin_7)  //Power_433_EN��������
#define  PowerOFF_433_EN()           GPIO_ResetBits(GPIOC,GPIO_Pin_7)//Power_433_EN��������
#define  PowerON_433_SET()            GPIO_SetBits(GPIOC,GPIO_Pin_8)  //Power_433_SET��������
#define  PowerOFF_433_SET()           GPIO_ResetBits(GPIOC,GPIO_Pin_8)//Power_433_SET��������

#endif

