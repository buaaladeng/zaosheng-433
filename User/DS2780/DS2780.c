/*******
#include <iom8v.h>
#include <macros.h>
#define DATA      PIND
#define IO_PORT   PORTD
#define DRITION   DDRD
#define IO_BIT    3

#define SET_DRITION_IN()    DRITION &=~( 1<<IO_BIT )
//#define SET_DRITION_IN()    DRITION &=~( 1<<IO_BIT ) //DAT_OUT_1()
//#define SET_DRITION_OUT()   DRITION |= ( 1<<IO_BIT ) 
//#define READ_DAT()          (PIND&0x08)
#define READ_DAT()          (DATA&(1<<IO_BIT))
#define DAT_OUT_1()         IO_PORT |= ( 1<<IO_BIT ) 
#define DAT_OUT_0()         IO_PORT &=~( 1<<IO_BIT ) 
***************/
#include "stm32f10x.h"
#include "DS2780.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"
#include "SPI_Flash.h"

//P1DIR |= 0x01;                            // Set P1.0 to output direction
//#define SET_DRITION_OUT()   P1DIR |= BIT5   //设置P1.5为输出
//#define SET_DRITION_IN()    P1DIR &=~BIT5  //设置P1.5为输入
//#define READ_DAT()          (P1IN&0x20) 
//#define DAT_OUT_1()         P1OUT |=0x20// P1.5 输出高电平
//#define DAT_OUT_0()         P1OUT &=0xDF//p1.5 输出低电平


static  uint8_t   RegB=0;
extern  uint8_t   PowerOffReset;
//extern  uint16_t  BATT_Capacity ;                //一组电池当前剩余电量（满容量19Ah）

void DS2780_CapacityInit(void)
{
   RegB = 113;   //库仑计初始化参数，需要保存在Flash中，上电只能赋值一次，后续有待完善
	 printf("\n\r RegB:%d \r\n",RegB);
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
unsigned char  DS2780_Test(void)
{
    uint16_t  dats1=0x0000,dats2=0x0000,dats3=0x0000,dats6=0x0000,dats7=0x0000;
//  	uint16_t  RAAC_OUT=0x0000;
	  float     RAAC_OUT=0;
    uint8_t   FULL_status=0x00;
	  uint16_t  temp=0;
//	  uint8_t   count=3;
    //RegB = 5;   //库仑计初始化参数，需要保存在Flash中，上电只能赋值一次，后续有待完善
//	  printf("\n\r OKKKKLL\r\n");
		if(PowerOffReset==1)
		{
				BatteryCapacityWrite_to_Flash(&RegB);//BKP_WriteBackupRegister(BKP_DR8,RegB);
				PowerOffReset = 0;
		}
		else
		{
				RegB = BatteryCapacityRead_From_Flash();
			 printf("\n\r BatteryCapacityRead_From_Flash\r\n");
				if(RegB>113 || RegB ==0)    //防止发生Flash异常复位
				{
					 RegB=113;
					 printf("\n\r RegB  ERROR!!\r\n");
				}
		}
		
		dats1 = get_voltage();
		dats2 = get_current();
		dats3 = get_ACR();
		//temp = -10;
		//dats2 = temp;
		//temp = dats2;
		temp = dats2&0x7FFF;
		printf("\n\r 电池电流current:%d (mA)\r\n",temp);
		temp = RegB;            //暂时将电池电量系数保存，便于后续做比较
		//get_accumulate();
		//get_RAAC();电量累计，放电得负，充电得正，单位是mah
		//dats4 = get_AE();
		//dats5 = get_RARC(); 
//		get_AE();
//		get_RARC(); 
		dats6 = get_RAAC();   //得到剩余的电量，电量是mah
//	  while(count!=0)
//		{
//      dats6 = get_RAAC();   //得到剩余的电量，电量是mah
	  printf("\n\r 修正前电池剩余绝对电量RAAC:%d mAh\r\n",dats6);
//      count--;
//			if(dats6) break;
//    }
//		 dats7 = get_FULL();  //读现有电量状态，百分比
			
		 if(dats6 == 0)
		 {
				set_ACR(1000);
				Delay_ms(1200);       // 延时200ms，具体延时时间有待确认
				if(RegB !=0)           //防止出现溢出
				{
					RegB--;	
					dats6 = get_RAAC();  //电池系数修正以后，需要重新读取电池容量，否则后续电池相对剩余容量会出现波动
				}		
		 }
			RAAC_OUT = RegB + dats6/1000;//得到剩余电量；
			FULL_status=(RAAC_OUT*100)/114;//剩余电量百分比
		 printf("\n\r RegB:%d (mV)\r\n",RegB);
			printf("\n\r 电池电压voltage:%d (mV)\r\n",dats1);
			//printf("\n\r 电池电流current:%f (uA)\r\n",dats2);
			printf("\n\r 电量累计accumulate:%d mAh\r\n",dats3);
			printf("\n\r 剩余绝对电量RAAC:%d mAh\r\n",dats6);
			//printf("\n\r FULL",dats7);
			printf("\n\r 剩余相对电量FULL_status:%d%%\r\n",FULL_status);
			FULL_status = FULL_status*2.55;
			if(temp != RegB)      //当电池电量系数发生变化时，将最新的数值写入Flash；没有变化时，不更新Flash
			{
					BatteryCapacityWrite_to_Flash(&RegB);     //BKP_WriteBackupRegister(BKP_DR8,RegB);
			}
			return FULL_status;
}

void DS2780_DQ_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);
  
  /* DQ -PC7 configration */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void DS2780_DQ_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);
  /* DQ -PC7 configration */	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
unsigned char reset(void)
{ 
  unsigned char presence = 0 ;

  DS2780_DQ_OUT(); // 改PC_7 为输出口	
	DQ_OUT_0; // 拉低DQ 线
	Delay_us(t_RSTL); //延时至少 t_RSTL μs
	DQ_OUT_1;// 将DQ 线设置为逻辑高
	DS2780_DQ_IN(); // 改PC_7 为输入口
	Delay_us(t_PDH); // 延时等待presence 响应
	//Delay_us(t_PDL/2);
	//Delay_us(530);
	presence = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
	//presence=READ_DAT(); // 采样presence信号
	//presence= PIND&0x08;
	Delay_us(t_RSTH-t_PDH-t_PDL/2);//等待时序结束
	return(presence); //有presence 信号时返回0 ，否则返回1
}

//读一位函数
unsigned char read_bit(void)
{
  unsigned char result;
	DS2780_DQ_OUT(); // 改PC_7为输出口
	DQ_OUT_0;
	Delay_us(1);// 总线为低电平的时间至少持续1 μs
	DS2780_DQ_IN(); // 改PC_7 为输入口
	Delay_us(t_RDV);// 总线为低电平的时间至少持续1 μs
	result = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2); // 采样result 信号
	Delay_us(t_SLOT-t_RDV-1);// 等待时序结束
	return(result); // 返回采样值
}
/*//写一位函数
void write_bit(unsigned char bit_value)
{
    // 如果写逻辑"1"bit，使得DQ 为高
	//if(bit_value == 1)
	//DS2780_DQ_OUT();
	if(bit_value)
	{
	    DQ_OUT_0
		Delay_us(t_LOW1); //调用Delay_us()函数大约需要5 μs
		DQ_OUT_1;
		Delay_us(t_SLOT-t_LOW1);
	}
    else
	{
	    DQ_OUT_0
		Delay_us(t_LOW0);
		DQ_OUT_1;
		//Delay_us(t_REC); // 保持延时至时序结束
		Delay_us(t_SLOT-t_LOW0);
    }
}*/////
//写一个字节函数
void write_byte(unsigned char value)
{
  unsigned char i;
//	unsigned char temp;
	DS2780_DQ_OUT();
  for(i=0;i<8;i++)
	{
		if(value&0x01)
		{
	      DQ_OUT_0; 
		    Delay_us(t_LOW1); //调用Delay_us()函数大约需要5 μs
		    DQ_OUT_1;
		    Delay_us(t_SLOT-t_LOW1);		
		}
		else
		{
			DQ_OUT_0;
		    Delay_us(t_LOW0);
		    DQ_OUT_1;
		    Delay_us(t_SLOT-t_LOW0);
		}
		value=value>>1; // 将value 位右移i 位赋值给temp
  }
}
//读一个字节函数
unsigned char read_byte(void)
{
  unsigned char i,value;
//	  unsigned char m;
//	  m = 1;
	value = 0;
	//DS2780_DQ_IN(); // 改PC_7 为输入口
	for(i=0;i<8;i++)
	{
	    value>>=1; // 每读一位数据，左移一位
	    if(read_bit()) 
		  {
		    value|=0x80;
      }
  }
  return(value);
}	

/*
Read_data    0x69
#define Write_data   0x6c
#define Copy_data    0x48
#define recall_data  0xb8
#define lock_data    0x6a*/
//给ds2780指定地址读出一个字节
unsigned char read_byte_data(unsigned char start_address)
{
//    unsigned char i,k;
	unsigned char one_byte_data=0;
	write_ds2780_cmd(Read_data,start_address);
	one_byte_data = read_byte();
	return(one_byte_data);
}
//往ds2780指定地址写入一个字节
void write_data(unsigned char write_address,unsigned char dat)
{
//  unsigned char i;
//	unsigned char one_byte_data;
//	unsigned char pres;
	write_ds2780_cmd(Write_data,write_address);
	write_byte(dat);//写入数据
	Delay_us(10000);
}

void write_ds2780_cmd(unsigned char cmd,unsigned char address)
{
    unsigned char pres;
restart:
	pres = reset();
	if(pres) //return fause; // 如果没有器件，返回0
	{
	   Delay_us(100);
	   goto restart;
	}
	write_byte(0xCC); //主机发出Skip Net Address命令
  write_byte(cmd); // 主机发出xx命令
	write_byte(address); // 主机发出地址命令
	//return ture;
}
void recall_data_ds2780(unsigned char address)
{
  write_ds2780_cmd(recall_data,address);
	Delay_us(10000);
}
void copy_data_ds2780(unsigned char address)
{
  write_ds2780_cmd(Copy_data,address);
	Delay_us(10000);
}

//得到电压
unsigned int get_voltage(void)
{
  unsigned char v_h=0,v_l=0;
	unsigned int voltage=0;
  v_h = read_byte_data(0x0c);
	voltage = v_h;
	voltage <<=3;
	Delay_us(10000);
	v_l = read_byte_data(0x0d);
	voltage += v_l>>5;
	voltage = voltage&0x3ff;
	voltage = voltage*4.88;
	return voltage;
}
//剩余电量的百分比
/*unsigned int get_RSAC(void)
{
	unsigned int voltage;
    voltage = read_byte_data(0x06);
	return voltage;
}*/

//剩余电量的百分比
unsigned int get_RARC(void)
{
	unsigned int voltage=0;
  voltage = read_byte_data(0x06);
	//voltage = read_byte_data(0x07);
	return voltage;
}

//电流测量，单位是ma
unsigned int get_current(void)
{
    unsigned char a_h,a_l,pos_neg;
    unsigned int current;
    a_h = read_byte_data(0x0e);
	//a_h = read_byte_data(0x08);
	current = a_h;
	if(a_h&0x80)//放电此时是负数，则为补码
	{
	    pos_neg = 0;
	}
	else
	{
	    pos_neg = 1;
	}
	current <<=8;
	Delay_us(10000);
    //a_l = read_byte_data(0x09);
	a_l = read_byte_data(0x0f);
	current += a_l;
	current = current&0x7fff;
	if( pos_neg == 0 )//算出原码
	{
	    current =(~(current-1))&0x7fff;
	    current = current*0.07813;//current = current*0.3125;//0.03125对应5mO
 
	    return current|0x8000;
	}
	
	current = current*0.07813;//current = current*0.3125;
   
	return current;
}

//温度，放大了10倍，单位是度
unsigned int get_temperature(void)
{
    unsigned char t_h,t_l,pos_neg;
	unsigned int temper;
    t_h = read_byte_data(0x0a);
	temper = t_h;
	if(t_h&0x80)//放电此时是负数，则为补码
	{
	    pos_neg = 0;
	}
	else
	{
	    pos_neg = 1;
	}
	temper <<=3;
	Delay_us(10000);
	t_l = read_byte_data(0x0b);
	temper += t_l>>5;
	temper = temper&0x03ff;
	if( pos_neg == 0 )//算出原码
	{
	    temper =(~(temper-1))&0x03ff;
	    //temper = temper*1.25;
       temper = temper*0.3125;//对应20mo
	    return temper|0x8000;
	}
	
	//temper = temper*1.25;
	temper = temper*0.3125;//对应20mo
   
	return temper;
}


//电量累计，放电得负，充电得正，单位是mah
unsigned int get_ACR(void)
{
    unsigned char t_h,t_l,pos_neg;
  	unsigned int accumulate;
	//t_h = read_byte_data(0x04);
    t_h = read_byte_data(0x10);
	accumulate = t_h;
	if(t_h&0x80)//放电此时是负数，则为补码
	{
	    pos_neg = 0;
	}
	else
	{
	    pos_neg = 1;
	}
	accumulate <<=8;
	Delay_us(10000);
	//t_l = read_byte_data(0x05);
	t_l = read_byte_data(0x11);
	accumulate += t_l;
	if( pos_neg == 0 )//算出原码
	{
	    accumulate =(~(accumulate-1))&0x7fff;
	   // accumulate = accumulate*1.25;//1.25对应5mo；
     accumulate = accumulate*0.3125;//对应20mo
	    return accumulate|0x8000;
	}
	//accumulate = accumulate*1.25;
	accumulate = accumulate*0.3125;
   
	return accumulate;
}
//得到剩余的电量，电量是mah
unsigned int get_RAAC(void)
{
  unsigned char t_h=0,t_l=0;
	unsigned int  accumulate=0;
	
  t_h = read_byte_data(0x02);
	accumulate = t_h;
	accumulate <<=8;
	Delay_us(10000);
	t_l = read_byte_data(0x03);
	accumulate += t_l;
	
	accumulate = accumulate*1.6;
   
	return accumulate;
}


//读现有电量状态，百分比
unsigned int get_FULL(void)
{
    unsigned char t_h,t_l;
	unsigned int accumulate;
  t_h = read_byte_data(0x16);
	accumulate = t_h&0x3f;
	accumulate <<=8;
	Delay_us(10000);
	t_l = read_byte_data(0x17);
	accumulate += t_l;
	accumulate = accumulate*0.0061;
	return accumulate;
	
}

//读现有电量状态，百分比
unsigned int get_AE(void)
{
  unsigned char t_h,t_l;
	unsigned int accumulate;
  t_h = read_byte_data(0x18);
	accumulate = t_h&0x1f;
	accumulate <<=8;
	Delay_us(10000);
	t_l = read_byte_data(0x19);
	accumulate += t_l;
	accumulate = accumulate*0.0061;
	return accumulate;
}

//读控制寄存器
unsigned int get_STATUS(void)
{
    unsigned char cmds;
	//recall_data_ds2780(0x7b);
	//Delay_us(10000);
    cmds = read_byte_data(0x01);
	//cmds = read_byte_data(0x7b);
   /*recall_data_ds2780(0x68);
	Delay_us(10000);
    cmds = read_byte_data(0x68);*/
	return cmds;
}
//写控制寄存器
void set_STATUS(unsigned char cmds)
{
	write_ds2780_cmd(cmds,0x01);
	Delay_us(10000);
    copy_data_ds2780(0x01);
}

//读控制寄存器
unsigned int get_sssd(void)
{
    unsigned char cmds;
	recall_data_ds2780(0x66);
	Delay_us(10000);
    cmds = read_byte_data(0x66);
   
	return cmds;
}

//读控制寄存器
void set_ACR(unsigned int raac)
{
   unsigned int dats;
	 unsigned char b;
	 //dats = raac/1.25;//对应5mO
   dats = raac/0.3125;//对应20mO
	 b = dats>>8;
   write_data(0X10,b);
	 Delay_ms(40);
	 copy_data_ds2780(0X10);
	 Delay_ms(40);
	 b = dats&0x00ff;
	 write_data(0X11,b);
	 Delay_ms(40);
	 copy_data_ds2780(0X11);

}



/********************************************
针对四节铁锂电池设置的参数
充满电电压为14.4v       VCHG      0xb4
充满电电流设置为50ma    IMIN      0x05
放完电电压为11.8v       VAE       0x93
放完电电流为300ma       IAE       0x07

电阻采用5m欧            RSNSP     0x01
电流补偿放大为1.270倍   RSGAIN    0x0514
FULL存储40度下的容量100%    FULLS 0X3FFF
电池容量6000mah         AC        0x12c0
失调电流0.5ma           AB        0x02
电阻的温度补偿设为0     RSGAIN    0X0000

FULL3040   0x0f
FULL2030   0x1c
FULL1020   0x26
FULL0010   0x27

AE3040   0x07
AE2030   0x10
AE1020   0x1E
AE0010   0x12 

SE3040   0x02
SE2030   0x05
SE1020   0x05
SE0010   0x0A

控制寄存器 CONTROL  0x00
*******************************************/
/********************************************
高能聚合物电池设置的参数(一次性非充电电池)
充电压为7.2v       VCHG      0x5A
充满电电流设置为50ma    IMIN      0x05
放完电电压为6v       VAE       0x4B
放完电电流为10ma (估计值)      IAE       0x01

电阻采用20m欧            RSNSP     0x32
电流补偿放大为1.270倍   RSGAIN    0x0514
FULL存储40度下的容量100%    FULLS 0X3FFF
电池容量19000mah         AC        0xED80 ；
失调电流0.5ma           AB        0x02
电阻的温度补偿设为0     RSGAIN    0X0000

FULL3040   0x0f
FULL2030   0x1c
FULL1020   0x26
FULL0010   0x27

AE3040   0x07
AE2030   0x10
AE1020   0x1E
AE0010   0x12 

SE3040   0x02
SE2030   0x05
SE1020   0x05
SE0010   0x0A

控制寄存器 CONTROL  0x00

  


*******************************************/
void Set_register_ds2780(void)
{
//    unsigned char pres;
   write_data(CONTROL,0x00);
	 copy_data_ds2780(CONTROL);
	//Delay_ms(1);
 	
    write_data(AB,0x02);
	//write_data(AB,0x00);
	copy_data_ds2780(AB);	
	
    write_data(15,50);
	copy_data_ds2780(15);	
	
	write_data(AS,28);
	copy_data_ds2780(AS);	
	
	//write_data(ACMSB,0x12);
	//copy_data_ds2780(ACMSB);
   //write_data(ACLSB,0xc0);
	//copy_data_ds2780(ACLSB);	
		
//	write_data(ACMSB,0x1f);
//	copy_data_ds2780(ACMSB);
//  write_data(ACLSB,0x40);
//	copy_data_ds2780(ACLSB);	
	write_data(ACMSB,0xED);//19Ah
	copy_data_ds2780(ACMSB);
  write_data(ACLSB,0x80);
	copy_data_ds2780(ACLSB);
	
	
	//write_data(VCHG,0xa0);//电压时13.6v
	//write_data(VCHG,0xaf);//电压时14.0v
//	write_data(VCHG,0xb4);//电压时14.4v
//	copy_data_ds2780(VCHG);
	//write_data(VCHG,0x5A);//电压时7.2v
//	write_data(VCHG,0xB8);//电压时3.6v
//	write_data(VCHG,0xA9);//电压时3.3v
  write_data(VCHG,0x9B);  //电压为3.038v

	copy_data_ds2780(VCHG);
	
  write_data(IMIN,0x14);//电流50mA
	copy_data_ds2780(IMIN);		
	
	//write_data(VAE,0x4B);//6V时4B,11.3V时0x8d,11.8v时0x93
//	write_data(VAE,0x9C);//3.05V时9C
//	write_data(VAE,0x8F);  //2.8V时8F
  write_data(VAE,0x83);  //电压为2.566V

	copy_data_ds2780(VAE);
	
	write_data(IAE,0x08);//80mA
	copy_data_ds2780(IAE);
	
	//write_data(RSNSP,0x00);
	//copy_data_ds2780(RSNSP);
	write_data(RSNSP,0x32);
	copy_data_ds2780(RSNSP);
	
	
	/*write_data(FULLSMSB,0x3f);
	copy_data_ds2780(FULLSMSB);	
	write_data(FULLSLSB,0xff);
	copy_data_ds2780(FULLSLSB);	*/

	write_data(FULLSMSB,0x1f);
	copy_data_ds2780(FULLSMSB);	
	write_data(FULLSLSB,0x40);
	copy_data_ds2780(FULLSLSB);		
	
	write_data(0x68,0x06);
	copy_data_ds2780(0x68);	
/*
	write_data(FULL3040,0x00);
	copy_data_ds2780(FULL3040);	
	
	write_data(FULL2030,0x00);
	copy_data_ds2780(FULL2030);	
	
	write_data(FULL1020,0x00);
	copy_data_ds2780(FULL1020);	
	
	write_data(FULL0010,0x00);
	copy_data_ds2780(FULL0010);	
	
	write_data(AE3040,0x00);
	copy_data_ds2780(AE3040);	
	
	write_data(AE2030,0x00);
	copy_data_ds2780(AE2030);
	
	write_data(AE1020,0x00);
	copy_data_ds2780(AE1020);	
	
	write_data(AE0010,0x00);
	copy_data_ds2780(AE0010);	
	
	write_data(SE3040,0x00);
	copy_data_ds2780(SE3040);	
	
	write_data(SE2030,0x00);
	copy_data_ds2780(SE2030);
	
	write_data(SE1020,0x00);
	copy_data_ds2780(SE1020);	
	
	write_data(SE0010,0x00);
	copy_data_ds2780(SE0010);	
	*/
	
	write_data(FULL3040,0x0F);
	copy_data_ds2780(FULL3040);	
	
	write_data(FULL2030,0x1C);
	copy_data_ds2780(FULL2030);	
	
	write_data(FULL1020,0x26);
	copy_data_ds2780(FULL1020);	
	
	write_data(FULL0010,0x27);
	copy_data_ds2780(FULL0010);	
	
	write_data(AE3040,0x07);
	copy_data_ds2780(AE3040);	
	
	write_data(AE2030,0x10);
	copy_data_ds2780(AE2030);
	
	write_data(AE1020,0x1E);
	copy_data_ds2780(AE1020);	
	
	write_data(AE0010,0x12);
	copy_data_ds2780(AE0010);	
	
	write_data(SE3040,0x02);
	copy_data_ds2780(SE3040);	
	
	write_data(SE2030,0x05);
	copy_data_ds2780(SE2030);
	
	write_data(SE1020,0x05);
	copy_data_ds2780(SE1020);	
	
	write_data(SE0010,0x0A);
	copy_data_ds2780(SE0010);	
	
   /* write_data(RSGAINMSB,0x04);
	copy_data_ds2780(RSGAINMSB);	
	//write_data(RSGAINLSB,0x0f);
	write_data(RSGAINLSB,0x50);//大电流值
	copy_data_ds2780(RSGAINLSB);	*/
	
	write_data(RSGAINMSB,0x04);
	copy_data_ds2780(RSGAINMSB);	
	//write_data(RSGAINLSB,0x0f);
	write_data(RSGAINLSB,0x08);//大电流值
	copy_data_ds2780(RSGAINLSB);	
	/*
	write_data(RSGAINMSB,0x05);
	copy_data_ds2780(RSGAINMSB);	
	write_data(RSGAINLSB,0x19);
	copy_data_ds2780(RSGAINLSB);	*/
	
	write_data(RSTC,0x00);
	copy_data_ds2780(RSTC);
		
}
