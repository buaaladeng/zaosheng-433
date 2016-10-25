/*****************************************************
          modbus-rtu 通讯规约

通讯方式：rs-232 半双功
校验方式：crc16
停止位：2位
编写：徐勤利
编写日期：2011年3月26日
版本：v1.0(励磁的通讯)
编程思路:
    1.串口中断允许自动接收总线上的信息,当接收的
    字节后超过3.5个字节时间没有新的字节认为本次
    接收完成,接收完成标志置1;如果接收完成标志已
    经置1又有数据进来则丢弃新来的数据。
    2.串口接收数据的处理, 当接收完成标志置1进入
    接收数据处理, (1)首先判断接收的第一位数据与
    本机地址是否相同,如果不相同清空接收缓存不发
    送任何信息; (2)接收的第一位数据与本机地址相
    同,则对接收缓存中的数据进行crc16校验,如果接
    收的校验位与本校验结果不相同清空接收缓存不发
    送任何信息；
    (3)如果crc16校验正确则根据数据串中的命令码进
    行相应的处理。
******************************************************/
#include "modbus.h"
#include "bsp_usart.h"
#include "math.h"
#include "bsp_SysTick.h"

unsigned char Senser_Address = 0x01;             //超声液位计设备通信地址     
unsigned char Uart4_rev_buff[100]={'\0'};        //RS485串口接收缓存
unsigned char Uart4_send_buff[100]={'\0'};       //RS485串口发送缓存
volatile unsigned char Uart4_rev_count=0;        //RS485串口接收计数器
vs8 Uart4_send_counter = 0;                      //RS485串口发送计数器
volatile unsigned char Uart4_rev_comflag=0;      //接收完成状态标志
volatile unsigned char Uart4_send_ready_flag=0;  //发送就绪状态标志
volatile unsigned char Crc_counter = 0;          //RS485校验计数器
volatile unsigned char *Uart4_send_pointer = Uart4_send_buff;//RS485串口发送指针

extern  char  Usart1_recev_buff[300];
//extern  char  Usart1_send_buff[];
extern  uint16_t  Usart1_recev_count;
//extern  uint8_t  Usart1_send_count;
//extern u8  DataCollectCache[13][4];   //液位数据采集缓存，最多13组数据；浮点数HEX格式存储，低字节在前，高字节在后
extern u8  DataCollectCount;          //数据采集计数器
extern  float LevelData_Float[FILTER_ORDER];       //采集到的临时液位数据，浮点型
uint8_t  LevelDataCount = FILTER_ORDER;       //液位数据计数器，标示当前采集到数据数量
//float  Data_Liquid_Level=0.0;

extern void Delay(vu32 nCount);
//unsigned short getCRC16(volatile unsigned char *ptr,unsigned char len) ; 
void mov_data(uint8_t *a, uint8_t *b, uint8_t c);
float Bin754_to_float(uint32_t   data_bin754);
void Modbus_Function_3(void); 
//void Modbus_Function_10(void);       
/***************************************
函数名称：crc16校验
函数功能：crc16校验
函数输入：字节指针*ptr，数据长度len
函数返回：双字节crc
函数编写：孙可
编写日期：2008年6月9日
函数版本：v0.2
****************************************/
unsigned short getCRC16(unsigned char *ptr,unsigned char len) 
{ 
    unsigned char i; 
    unsigned short crc = 0xFFFF; 
	
    if(len==0)          //为什么这么处理？
    {
        len = 1;
    } 
    while(len--)  
    {   
        crc ^= *ptr; 
				for(i=0; i<8; i++)  
				{ 
							if(crc&1) 
							{ 
										crc >>= 1;  
										crc ^= 0xA001; 
							}  
							else 
							{
										crc >>= 1;
							} 
				}         
				ptr++; 
    } 
    return(crc); 
} 

/*******************************************************************************
    块数据复制数据函数
功能：把数组a的c个数据复制到数组b中
输入：指针a,指针b,数据个数c
返回：无
编写：孙可
编写日期：2008年3月28日
版本：v0.1
********************************************************************************/
void mov_data(uint8_t *a,uint8_t *b,uint8_t c)
{
    uint8_t i;
	  if((c>0)&&(c<=100))
		{	
			for(i=0; i<c; i++)
			{
					a[i] = b[i];
			}
		}
}
/*******************************************************************************
    32位ieee754格式二进制数转变为十进制浮点数
功能：32位ieee754格式二进制数转变为十进制浮点数
输入：32位ieee754格式二进制数
返回：转换后的浮点数
编写：
编写日期：XX年XX月XX日
版本：v0.1
********************************************************************************/
float Bin754_to_float(uint32_t   data_bin754)
{
		int   i;
		uint8_t   stand[32];
		uint32_t  temp;
	  float     temp_f=0;
		int       jiema=0;       //定义阶码变量，初始化防止溢出
	  float     jishu=0;       //定义基数变量，初始化防止溢出
	   
	  temp=data_bin754;
		for(i=31;i>=0;i--)
		{
			stand[i]=temp & 0x01;
			temp=temp>>1;
		}
		for(i=1;i<=8;i++)
		{
       jiema += stand[i]*pow(2,8-i);
    }
		if(jiema>=127)
		{  jiema=jiema-127;        //计算阶码
		}		
    else	                     //防止溢出
		{
			 jiema=-1*(127-jiema);
    }			               
		for(i=9;i<=31;i++)
		{
       jishu += stand[i]*pow(2,8-i);
    }
		jishu=jishu+1;                        //计算基数
		
//		printf("%d\r\n",jiema);        //测试使用
//		printf("%f\r\n",jishu);        //测试使用
		temp_f=jishu*pow(2,jiema);            //计算测量结果
		if(stand[0]==1)   temp_f = -temp_f;   //增加符号
		
		return  temp_f;
}
/*******************************************************************************
功能：按照埃德尔协议的数据格式，将浮点数转化为16进制表示
输入：浮点格式数据
返回：无
编写：
编写日期：XX年XX月XX日
版本：v0.1
********************************************************************************/
void  Float2Hex_Aider( float DataSmooth )
{
   uint8_t   i=0;
	 uint8_t   DataArry[4]={0x00};
	 float*  pData = &DataSmooth;
	 uint8_t* pData_Hex = NULL;
	
	 pData_Hex = (uint8_t*)pData;
   for(i=4;i>0;i--)
   {
     DataArry[i-1] = *pData_Hex;
		 pData_Hex++;
   }
	
   for(i=0;i<4;i++)
	 {
			if((i%2)==0)
			{
        //DataCollectCache[DataCollectCount-1][i+1] = DataArry[i];
      }
			else
			{
				//DataCollectCache[DataCollectCount-1][i-1] = DataArry[i]; //将液位计返回的数据hex格式液位数据进行“机器序”变换，并存储于数据缓存中
			}
   }	
 	 DataCollectCount--;
}
/**************************************************读传感器数据**************************************************/
/****************************************************************************************************************/
/*-----参数名称--------寄存器地址------数据类型--------占用空间-------读写权限-------------备注-----------------*/                                                
/*-----LED显示值---------0x0000----------浮点型------16位寄存器*2------只读-------液位计探头前沿与液面垂直距离--*/
/*-----传感器温度--------0x0002----------浮点型-------4Byte(同上)------只读------------对一体式液位计无效-------*/
/*-----电流输出值--------0x0004----------浮点型-------4Byte(同上)------只读---------------对本型号无效----------*/
/*-----测量值------------0x0006----------浮点型-------4Byte(同上)------只读-----------------暂无解析------------*/
/*-----原始值------------0x0008----------浮点型-------4Byte(同上)------只读-----------------暂无解析------------*/
/*-----百分比------------0x00010---------浮点型-------4Byte(同上)------只读------当前测量值占满量(10m)的百分比--*/
/****************************************************************************************************************/ 
 void Modbus_Function_3(void) 
{ 

	  uint32_t  data_temp=0;
	  uint32_t  data32_array[4]={0};
	  float     data_f=0;
	  uint8_t   i=0;
//    uint8_t*  pData =NULL;     //调试使用
//    float* pFloat = &data_f;     //调试使用

	  for(i=0;i<4;i++)
		{
			data32_array[i] = Uart4_rev_buff[3+i];
//			printf("\r\ndata[%d] = %x\r\n",i,data32_array[i]);  //调试使用
		} 
	  data_temp = (data32_array[0] <<24)+(data32_array[1] << 16)+(data32_array[2] << 8)+data32_array[3];
	  data_f = Bin754_to_float(data_temp);
		
		if(data_f > 0.1) 
		{
			LevelData_Float[LevelDataCount-1] = data_f;
			LevelDataCount--;        //当读到的液位数据不为‘0’时，数据计数器减1，丢弃无效数据
		}
		
//		#if DEBUG_TEST	 
	  printf("当前液位高: %0.3f 米\r\n",data_f);  //调试使用
//		#endif
//		pData = (uint8_t*)pFloat;    //调试使用
//		for(i=0;i<4;i++)
//		{
//			printf("\r\nFdata[%d] = %x\r\n",i,*pData);  //调试使用
//			pData++;          //调试使用
//		} 
		
} 
/****************************配置写入函数*************************************************/
void Modbus_Function_10(void) 
{ 
//根据需要再完善
} 
/******************************************************************************/ 
void RS485_Communication(void) 
{ 
     unsigned short crcresult; 
     unsigned char temp[2]; 
	   int  i=0;  
	   
     if(Uart4_rev_comflag == 1)//接收完成标志=1处理，否则退出
		 {  

			 if(Uart4_rev_buff[0] == Senser_Address)//地址正确
    	  {  		

            crcresult = getCRC16(Uart4_rev_buff,Crc_counter-2); 
            temp[1] = crcresult & 0xff; //低8位
            temp[0] = (crcresult >> 8) & 0xff; //高8位
            if((Uart4_rev_buff[Crc_counter-1] == temp[0])&&(Uart4_rev_buff[Crc_counter-2] == temp[1]))  //crc校验正确则运行下面 		
        	  { 											
                					
							  switch(Uart4_rev_buff[1])            //功能码
                { 
                    case 0x03:                       // 读运行数据和菜单参数
                    {
                        Modbus_Function_3();         //将读到的传感器数据进行相应的变换，然后通过USART1发到上位机。
                        
        			      }break;  
										case 0x04:                       //读运行数据
                    {	
                        ;                            //待完善
        			      } break;
                    case 0x10:                       //写菜单参数
                    {	
                        Modbus_Function_10();        //配置参数写入函数
        			      } break; 
										case 0x12:                       //参数保存到EEPROM
                    {	
                        ;                            //待完善
        			      } break;
					          default:                         //命令码无效不应答
        			      {
                        ;
					          }break; 
                } 				 
    		    }
						else                                     //CRC校验码不正确，不做任何处理
						{
						   ;
						}
    	  }
        Uart4_rev_comflag = 0;	                     //处理完成后，进行初始化
        for(i=99;i>=0;i--) 
    	  { 
             Uart4_rev_buff[i] = 0;
    	  } 
		}
		
		if(Uart4_send_ready_flag==1)                     //读液位命令准备就绪
    {  
        if(Uart4_send_buff[0] == Senser_Address)     //地址正确
    	  {  
 
					  DIR485_Send();                           //485发送使能
						Delay_ms(50);                            //原始3ms

            crcresult = getCRC16(Uart4_send_buff,Crc_counter); 
            temp[1] = crcresult & 0xff;              //低8位
            temp[0] = (crcresult >> 8) & 0xff;       //高8位
					  Uart4_send_buff[Crc_counter] =temp[1];   
					  Uart4_send_buff[Crc_counter+1] =temp[0]; //增加两字节CRC校验码
					  Uart4_send_counter=Crc_counter+2;
					  Crc_counter=0;
						 
				  	Uart4_send_pointer=Uart4_send_buff;
						for(i=0;i<Uart4_send_counter;i++)
						{  
							 USART_SendData(UART4, (uint8_t)*Uart4_send_pointer++);
				       while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //等待发送完毕 
						}
								
						Uart4_send_counter=0;          //发送计数器复位       
						for(i=0;i<100;i++)
						{
							 Uart4_send_buff[i]='\0';    //清空发送缓存    
            }
				 }
				 Uart4_send_ready_flag=0;     
//				 Delay_us(145);                  //在数据接收完成后，延迟一段时间再切换收发控制开关，默认延时200us
				 DIR485_Receive();
				 Delay_ms(100);                    //原始3ms
		}
					 
} 




