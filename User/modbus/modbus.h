#ifndef _MODBUS_H_
#define _MODBUS_H_ 

///#include "stm32f10x_lib.h"

#include "stm32f10x.h"
#define  FILTER_ORDER    8        //定义数据滤波深度，用于液位数据平滑处理,不应小于4

extern vu8 Uart4_rev_comflag ;    //RS485接收完标志
extern u8 Senser_Address ;        //液位计的通讯地址
//extern u8 tmr0_uart35 ;         //串口3.5个字节延时
extern u8 Uart4_send_buff[100]; 
extern u8 Uart4_rev_buff[100];
extern vs8 Uart4_send_counter;
extern vu8 Uart4_rev_comflag;        //RS485接收完成状态标志
extern vu8 Uart4_send_ready_flag;    //RS485发送就绪状态标志
extern vu8 Crc_counter ;             //RS485校验计数器
extern vu8 *Uart4_send_pointer;      //RS485串口发送指针
extern u8 RS485_Receive_Complete_Flag;

//extern vu16 Mkgz_bz ;//模块故障标志 1:输入异常；2:过压 
//extern vu16 Out_current ;//输出电流
//extern vu16 Out_voltage ;//输出电压
//extern vu16 Mkzt_bz ;//模块状态标志 0:0浮充,1均充；2:0开机,1关机
//extern vu16 OutX_current ;//输出限流
//extern vu16 Jc_voltage ;//均充电压
//extern vu16 Fc_voltage ;//浮充电压
//extern vu16 user_day ;//使用天数

extern void RS485_Communication(void);
extern void mov_data(uint8_t *a, uint8_t *b, uint8_t c);
unsigned short getCRC16(unsigned char *ptr,unsigned char len); 
#endif
