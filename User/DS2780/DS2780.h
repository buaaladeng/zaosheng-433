#ifndef _DS2780_H_
#define _DS2780_H_
//#include "PIN_DEF.H"
//#include "msp430x54x.h"
#define DQ_OUT      PCout(7)
#define DQ_OUT_0    GPIO_ResetBits(GPIOA,GPIO_Pin_2) 
#define DQ_OUT_1    GPIO_SetBits(GPIOA,GPIO_Pin_2)

#define t_SLOT 110
#define t_LOW0 90
#define t_LOW1 7
//#define t_LOW1 10
#define t_REC  1
#define t_RDV  6//#define t_RDV  15
#define t_RSTH 480
#define t_RSTL 800
#define t_PDH  50
#define t_PDL  120

#define Read_data    0x69
#define Write_data   0x6c
#define Copy_data    0x48
#define recall_data  0xb8
#define lock_data    0x6a

#define fause  0
#define ture   1

#define STATUS  0x01
#define RAACMSB 0x02
#define RAACLSB 0x03
#define RSACMSB 0x04
#define RSACLSB 0x05
#define RARC    0x06
#define RSRC    0x07
#define IAVGMSB 0x08
#define IAVGLSB 0x09
#define TEMPMSB 0x0A
#define TEMPLSB 0x0B
#define VOLTMSB 0x0C
#define VOLTLSB 0x0D
#define CURRENTMSB 0x0E
#define CURRENTLSB 0x0F
#define ACRMSB     0x10
#define ACRLSB     0x11
#define ACRLMSB    0x12
#define ACRLLSB    0x13
#define AS         0x14
#define SFR        0x15
#define FULLMSB    0x16
#define FULLLSB    0x17
#define AEMSB      0x18
#define AELSB      0x19
#define SEMSB      0x1A
#define SELSB      0x1B

#define CONTROL 0x60
#define AB      0x61
#define ACMSB   0x62
#define ACLSB   0x63
#define VCHG    0x64
#define IMIN    0x65
#define VAE     0x66
#define IAE     0x67
#define RSNSP   0x69
#define FULLSMSB 0x6A
#define FULLSLSB 0x6B
#define FULL3040   0x6C
#define FULL2030   0x6D
#define FULL1020   0x6E
#define FULL0010   0x6F
#define AE3040     0x70
#define AE2030     0x71
#define AE1020     0x72
#define AE0010     0x73 
#define SE3040     0x74
#define SE2030     0x75
#define SE1020     0x76
#define SE0010     0x77
#define RSGAINMSB  0X78
#define RSGAINLSB  0X79
#define RSTC       0X7A






void DS2780_Init(void);
unsigned char reset(void);
unsigned char read_bit(void);
void write_byte(unsigned char value);
unsigned char read_byte(void);
unsigned char read_byte_data(unsigned char start_address);
void write_data(unsigned char write_address,unsigned char dat);
void write_ds2780_cmd(unsigned char cmd,unsigned char address);
void recall_data_ds2780(unsigned char address);
void copy_data_ds2780(unsigned char address);
unsigned int get_voltage(void);
unsigned int get_RARC(void);
unsigned int get_current(void);
unsigned int get_temperature(void);
unsigned int get_ACR(void);
unsigned int get_RAAC(void);
unsigned int get_FULL(void);
unsigned int get_AE(void);
unsigned int get_STATUS(void);
void set_STATUS(unsigned char cmds);
unsigned int get_sssd(void);
void set_ACR(unsigned int raac);
void Set_register_ds2780(void);
unsigned char DS2780_Test(void);
void DS2780_CapacityInit(void);

#endif
