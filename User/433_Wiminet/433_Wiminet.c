
// File Name: 433_Wiminet.c

#include "stdlib.h"
#include "string.h"
#include "API-Platform.h"
#include "433_Wiminet.h"
#include "bsp_SysTick.h"
#include "gprs.h"
#include "bsp_usart.h"
#include "common.h"
#include "bsp_rtc.h"

extern  char      Usart3_recev_buff[1000];
extern  char      Usart4_recev_buff[1000];

extern  uint16_t  Usart3_recev_count;
extern  uint16_t  Usart4_recev_count;

extern  uint8_t   DMA_UART3_RECEV_FLAG ;       //USART3 DMA接收标志变量
extern uint8_t SLNoiseConfigFlag;
extern  uint32_t  time;                     // ms 计时变量 

extern unsigned char  DMA_UART3_RecevDetect(unsigned char RecevFlag);     //USART3接收数据监测与数据解析
//extern void  RecvBuffInit_USART3(void);
extern  void  ReceiveAnalysis_433(void);
extern void Receive_Deal_433(void);
extern char  Receive_Monitor_433(void);
extern void ConfigData_Upload(void);
extern char DeviceId[6];     //噪声记录仪设备编号
char Config_Data[50];     //噪声记录仪设备编号
extern struct liquid_set  DeviceConfig;     //液位计配置信息结构体

//extern  char  Receive_Monitor_GPRS(void);
//extern  void Receive_Deal_GPRS(void);
//extern u8  NetStatus_Detection( void );
// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ReverseBitOrder08( unsigned char iSrc )
{
   unsigned char index;
   unsigned char iDst;
   
   iDst = iSrc & 0X01;
   for( index = 0X00; index < 0X07; index++ )
   {
      iDst <<= 0X01;
      iSrc >>= 0X01;
      iDst |= ( iSrc & 0X01 );
   }
   return iDst;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short ReverseBitOrder16( unsigned short iSrc )
{
   unsigned char index;
   unsigned short iDst;
   
   iDst = iSrc & 0X01;
   for( index = 0X00; index < 0X0F; index++ )
   {
      iDst <<= 0X01;
      iSrc >>= 0X01;
      iDst |= ( iSrc & 0X01 );
   }
   return iDst;
}

// *****************************************************************************
// Design Notes: CRC-16
// f(X)=X^16 + X^15 + X^2 + X^0
// POLYNOMIALS = 0X8005
// -----------------------------------------------------------------------------
unsigned short CRC16( unsigned char * pMsg, unsigned short iSize )
{
   unsigned char  index;
   unsigned short iCRC;
   
   // The default value
   iCRC = 0XFFFF;
   while ( iSize-- )
   {
      iCRC ^= ( ( ( unsigned short ) ReverseBitOrder08( *pMsg ) ) << 0X08 );
      for ( index = 0X00; index < 0X08; index++ )
      {
         if ( iCRC & 0X8000 )
         {
            iCRC = ( iCRC << 1 ) ^ 0X8005;
         }
         else
         {
            iCRC <<= 1;
         }
      }
      pMsg++;
   }
   return ReverseBitOrder16( iCRC );
}

//// -----------------------------------------------------------------------------
//// DESCRIPTION: CRC-16校验的高位字节表
//// -----------------------------------------------------------------------------
//static const unsigned char HiCRCTable[] = { 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
//0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40 };

//// -----------------------------------------------------------------------------
//// DESCRIPTION: CRC-16校验的低位字节表
//// -----------------------------------------------------------------------------
//static const unsigned char LoCRCTable[] = { 
//0X00, 0XC0, 0XC1, 0X01, 0XC3, 0X03, 0X02, 0XC2, 0XC6, 0X06, 0X07, 0XC7, 0X05, 0XC5, 0XC4, 0X04, 
//0XCC, 0X0C, 0X0D, 0XCD, 0X0F, 0XCF, 0XCE, 0X0E, 0X0A, 0XCA, 0XCB, 0X0B, 0XC9, 0X09, 0X08, 0XC8, 
//0XD8, 0X18, 0X19, 0XD9, 0X1B, 0XDB, 0XDA, 0X1A, 0X1E, 0XDE, 0XDF, 0X1F, 0XDD, 0X1D, 0X1C, 0XDC, 
//0X14, 0XD4, 0XD5, 0X15, 0XD7, 0X17, 0X16, 0XD6, 0XD2, 0X12, 0X13, 0XD3, 0X11, 0XD1, 0XD0, 0X10, 
//0XF0, 0X30, 0X31, 0XF1, 0X33, 0XF3, 0XF2, 0X32, 0X36, 0XF6, 0XF7, 0X37, 0XF5, 0X35, 0X34, 0XF4, 
//0X3C, 0XFC, 0XFD, 0X3D, 0XFF, 0X3F, 0X3E, 0XFE, 0XFA, 0X3A, 0X3B, 0XFB, 0X39, 0XF9, 0XF8, 0X38, 
//0X28, 0XE8, 0XE9, 0X29, 0XEB, 0X2B, 0X2A, 0XEA, 0XEE, 0X2E, 0X2F, 0XEF, 0X2D, 0XED, 0XEC, 0X2C, 
//0XE4, 0X24, 0X25, 0XE5, 0X27, 0XE7, 0XE6, 0X26, 0X22, 0XE2, 0XE3, 0X23, 0XE1, 0X21, 0X20, 0XE0, 
//0XA0, 0X60, 0X61, 0XA1, 0X63, 0XA3, 0XA2, 0X62, 0X66, 0XA6, 0XA7, 0X67, 0XA5, 0X65, 0X64, 0XA4, 
//0X6C, 0XAC, 0XAD, 0X6D, 0XAF, 0X6F, 0X6E, 0XAE, 0XAA, 0X6A, 0X6B, 0XAB, 0X69, 0XA9, 0XA8, 0X68, 
//0X78, 0XB8, 0XB9, 0X79, 0XBB, 0X7B, 0X7A, 0XBA, 0XBE, 0X7E, 0X7F, 0XBF, 0X7D, 0XBD, 0XBC, 0X7C, 
//0XB4, 0X74, 0X75, 0XB5, 0X77, 0XB7, 0XB6, 0X76, 0X72, 0XB2, 0XB3, 0X73, 0XB1, 0X71, 0X70, 0XB0, 
//0X50, 0X90, 0X91, 0X51, 0X93, 0X53, 0X52, 0X92, 0X96, 0X56, 0X57, 0X97, 0X55, 0X95, 0X94, 0X54, 
//0X9C, 0X5C, 0X5D, 0X9D, 0X5F, 0X9F, 0X9E, 0X5E, 0X5A, 0X9A, 0X9B, 0X5B, 0X99, 0X59, 0X58, 0X98, 
//0X88, 0X48, 0X49, 0X89, 0X4B, 0X8B, 0X8A, 0X4A, 0X4E, 0X8E, 0X8F, 0X4F, 0X8D, 0X4D, 0X4C, 0X8C, 
//0X44, 0X84, 0X85, 0X45, 0X87, 0X47, 0X46, 0X86, 0X82, 0X42, 0X43, 0X83, 0X41, 0X81, 0X80, 0X40 };


//// *****************************************************************************
//// Design Notes:  
//// -----------------------------------------------------------------------------
//unsigned short QuickCRC16( unsigned char * pMsg, unsigned short iSize )
//{
//   unsigned char iHiVal;                // high byte of CRC initialized
//   unsigned char iLoVal;                // low byte of CRC initialized
//   unsigned char index;                 // will index into CRC lookup table
//   
//   // Initial value for the CRC
//   iHiVal = 0XFF;
//   iLoVal = 0XFF;
//   
//   while ( iSize-- )
//   {
//      // Calculate the CRC
//      index = iLoVal ^ ( unsigned char )( *pMsg++ );
//      
//      iLoVal = iHiVal ^ HiCRCTable[index];
//      iHiVal = LoCRCTable[index];
//   }
//   return ( iHiVal << 8 | iLoVal );
//}
// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void UpdateNodeMsgCRC( NodeMsg * pMsg )
{
   unsigned char  iSize;
   unsigned short iCRC;
   
   // The message header
//   pMsg->m_iHeader = 0xAA;

   // The defualt CRC value
   pMsg->m_iCRCode = 0x00;

   // The message size
   iSize = 0x09;
   iSize += pMsg->m_iAmount;

   // Update the CRC value
   iCRC = CRC16( ( unsigned char * )pMsg, iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if ( CPU_ENDIAN_MODE == LITTLE_ENDIAN_MODE )

   // Change the byte order
   iCRC = ntohs( iCRC );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif

   // Restore the CRC of this message
   pMsg->m_iCRCode = iCRC;
}

//unsigned short ReverseByteOrder (unsigned short OriginalData)
//{
//   char* pData = NULL;
//	 pData = (char*)&OriginalData;
//	 return  (*pData<<8) + *(pData+1);
//}

// *****************************************************************************
// Design Notes: 初始化结构体
// -----------------------------------------------------------------------------
void InitCommandMessage( NodeMsg * pMsg )
{
//   // Initialize the message
//   InitWiMinetMessage( pMsg, 0X01 );   
	 // Reset the message body
   memset( pMsg, 0x00, sizeof( NodeMsg ) );
}

// *****************************************************************************
// Design Notes:  对查询主站ID函数封包

void IMP_GetCoordinatorID( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iHeader = 0xAA;
	 // The command for this packet
//   pMsg->m_iOpCode = CMDMSG_GET_COORDINATOR_ID;
   pMsg->m_iOpCode = 0x76;  
}
// *****************************************************************************
// Design Notes: 查询主站ID
// -----------------------------------------------------------------------------
 unsigned short GetCoordinatorID (void)
{
   NodeMsg  Msg;      //正式使用时不用
	 NodeMsg* pMsg = &Msg;
   unsigned short iSize;
//	 char  i=0;                   //调试使用
	 char  receive_flag =0;
	 //char  SendBuffer[12]={0xAA,0x1D,0x33,0x86,0x03,0x00,0x03,0x00,0x00,0x01,0x02,0x03};
   char  ID_GetFlag[2]={0xAA,0xF6};
   char*  pRecevBuff=NULL;
   unsigned short  ID_Temp=0x0000;

   // Initialize the message body
   InitCommandMessage( pMsg );

   // Construct the message
   IMP_GetCoordinatorID( pMsg );
   UpdateNodeMsgCRC( pMsg );
	 // The total message size
   iSize = pMsg->m_iAmount + 0x09;

	 USART_DataBlock_Send(USART1,(char* )(pMsg),iSize);    //调试使用
   USART_DataBlock_Send(USART1,"\r\n",2);                //调试使用
	 USART_DataBlock_Send(USART3,(char* )(pMsg),iSize);
	 
	 Delay_ms(1000);
   receive_flag = Receive_Monitor_433();
	 if(receive_flag == 1)
	 {
		 pRecevBuff = Find_SpecialString(Usart3_recev_buff,ID_GetFlag,300,2);  //检查有无收到主站回复
		 if(pRecevBuff!=NULL)                                     //有回复，提取地址信息   //暂时缺少CRC校验
		 {   
			
//			 printf("\r\nResponse from 433 is:");                   //调试使用
//			 for(i=0;i<Usart3_recev_count;i++)                      //调试使用
//			 {
//					printf(" %x",pRecevBuff[i]);                        //调试使用
//			 }
			 ID_Temp = (pRecevBuff[9]*256) +pRecevBuff[10];
			 pRecevBuff =NULL;
			 receive_flag = 0;
			 memset(Usart3_recev_buff,'\0',300);	
			 Usart3_recev_count =0;                                 //清空USART3接收计数器
			 time=0;	                                               //定时器复位
			 return  ID_Temp;
		 }
		 else
		 {	 
				memset(Usart3_recev_buff,'\0',300);	
				Usart3_recev_count =0;                                 //清空USART3接收计数器
				time=0;	                                               //定时器复位
		 }
	 }
	 return  0x0000;
}

// *****************************************************************************
// Design Notes:  发送数据封包
// -----------------------------------------------------------------------------
void IMP_SendMessage( NodeMsg * pMsg )
{ 
	 // The command for this packet
   pMsg->m_iHeader = 0xAA;
	 // The command for this packet
   pMsg->m_iOpCode = 0x1E;  
	 pMsg->m_iValueC = 0x03;
}
// *****************************************************************************
// Design Notes:  通过433模块发送数据主函数
// -----------------------------------------------------------------------------
uint8_t  SendMessage(char* Psend, unsigned short iSize)
{
		NodeMsg  Msg;
		NodeMsg* pMsg = &Msg;

		char  Send_OK[2]={0xAA,0x9D};
		char  i=0;
		char  j=0;
		char receive_flag = 0;
		char*  pRecevBuff = NULL;
		unsigned short Address_AP = DeviceId[4]*256 +DeviceId[5];

		// Initialize the message body
		InitCommandMessage( pMsg );
		IMP_SendMessage( pMsg );

		#if DEBUG_TEST	 
		printf("\r\nAP ID get success!!");                     //调试使用
		printf("\r\nMain Station is: %4x\r\n",Address_AP);     //调试使用
		#endif

		memset(Usart4_recev_buff,'\0',300);	                                                 //memset(Usart3_recev_buff,'\0',300);
		Usart4_recev_count=0;                          //清空USART3接收计数器
		time=0;	                                       //定时器复位	

		Delay_ms(1000);	
		pMsg->m_iValueB =( unsigned char )Address_AP;
		pMsg->m_iValueA =( unsigned char )(Address_AP>>8);
		memcpy(pMsg->m_pBuffer,Psend,iSize);
		pMsg->m_iAmount = iSize;
		UpdateNodeMsgCRC( pMsg );
		iSize = pMsg->m_iAmount + 0x09;				    

//		PowerOFF_433_SET();                              //433模块SET脚拉低，方便接收
//		Delay_ms(100);
//		PowerOFF_433_EN();                               //433模块EN脚拉低，方便接收
//		Delay_ms(100);
//		USART_DataBlock_Send(USART1,(char* )(pMsg),iSize);
//		USART_DataBlock_Send(USART1,"\r\n",2);
//		
//		USART_GetFlagStatus(UART4, USART_FLAG_TC);          //串口硬件复位之后，发送首字节之前，先读一下USART_SR，防止数据发送时首字节被覆盖
//		Delay_ms(500);
//		USART_DataBlock_Send(UART4,(char* )(pMsg),iSize);
//		USART_DataBlock_Send(UART4,"\r\n",2);	
//		Delay_ms(5000);
//		
//		PowerON_433_SET();                    //433模块SET管脚拉高，切换到接收模式
//	  Delay_ms(5000);
		
		mput_mix((char*)pMsg,iSize);
		
		Delay_ms(8000);
		Delay_ms((DeviceId[5]%16)*1000);
		printf("DeviceId[5]....!!!! %d \r\n",DeviceId[5]);                //
		printf("DeviceId[5]%16....!!!! %d \r\n",DeviceId[5]%16);
					
		receive_flag = Receive_Monitor_433();
		if(receive_flag == 1)
		{
					 printf("\r\nUart4串口接收信息输出:");
					 for(j=0;j<Usart4_recev_count;j++)
					 {
							printf(" %x",Usart4_recev_buff[j]);
					 }
				pRecevBuff = Find_String(Usart4_recev_buff,Send_OK);     //
				if(pRecevBuff!=NULL)                                     // 
				{    
					 #if DEBUG_TEST	
					 printf("\r\nData send success!!");
					 printf("\r\nResponse from 433 is:");
					 for(i=0;i<Usart4_recev_count;i++)
					 {
							printf(" %x",pRecevBuff[i]);
					 }
					 #endif
				} 
				GPRS_Receive_DataAnalysis();
				memset(Usart4_recev_buff,'\0',300);	
				Usart4_recev_count =0;                                       //清空USART3接收计数器
				time=0;	                                                     //定时器复位
		}			
		return 1;	 	 

}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char IsValidNodeMsg( NodeMsg * pMsg )
{
   unsigned short iCRC1;
   unsigned short iCRC2;
   unsigned short iSize;

   // Check the header
   if ( pMsg->m_iHeader != 0XAA )
   {
      return 0X00;
   }

   // The original CRC
   iCRC1 = pMsg->m_iCRCode;

   // Clear the CRC
   pMsg->m_iCRCode = 0X00;

   // The total message size
   iSize = 0X09;
   iSize += pMsg->m_iAmount;

   // Validate the CRC of this message
   iCRC2 = CRC16( ( unsigned char * )pMsg, iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if ( CPU_ENDIAN_MODE == LITTLE_ENDIAN_MODE )

   // Change the byte order
   iCRC2 = ntohs( iCRC2 );
//	 printf("CRC is :%4x\r\n", iCRC2);    测试使用

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif

   // Restore the CRC of this message
   pMsg->m_iCRCode = iCRC1;

   // Check the CRC value
   return ( iCRC1 == iCRC2 );
}

// *****************************************************************************
// Design Notes: 接收数据解析函数
// -----------------------------------------------------------------------------
unsigned char  ReceiveMessageVerify(char*  pRecevBuff)
{
	 SLNoiseSet  Msg;
	 SLNoiseSet* pMsg = &Msg;
   uint16_t i=0,temp=0, n;
	
	 u8  SetCommand[13];  //3个0xFF字段分别为节点ID高字节、节点ID低字节和校验和，需要输入或者计算得到
	 u16 Verify =0x0000;
	 u8  m =0;

	 pMsg->m_Preamble[0]= pRecevBuff[0];                                               //前导码
	 pMsg->m_Version[0]	= pRecevBuff[1];                                               //版本号
	 pMsg->m_Leng[0]	= pRecevBuff[2];                                                 //数据长度
	 pMsg->m_Leng[1]	= pRecevBuff[3];                                                 //数据长度
	 for(i=4; i<10; i++)
	 {
		  pMsg->m_DeviceId[i-4]	= pRecevBuff[i];                                         //设备编号
   }
	 pMsg->m_RouteMark[0]	= pRecevBuff[10];                                            //路由标志
	 pMsg->m_RouteSite[0]	= pRecevBuff[11];                                            //路由地址
	 pMsg->m_RouteSite[1]	= pRecevBuff[12];                                            //路由地址
   pMsg->m_PDUType[0]	= pRecevBuff[13];                                              //PDUType
	 pMsg->m_PDUType[1]	= pRecevBuff[14];                                              //PDUType
	 printf("\r\npRecevBuff[13] is :%x",pRecevBuff[13]);
	 
	 if(pRecevBuff[13] == 0x03)                                                    // 操作类型为0x03(下发配置信息) 
	 {

		 pMsg->m_Sep[0]	= pRecevBuff[15];                                                //报文序列号
		 
		 for(i=16; i<20; i++)
		 {
				pMsg->m_SystemTimeOid[i-16]	= pRecevBuff[i];                                 //系统日期时间Oid
		 }
		 
		 pMsg->m_SystemTimeLen[0]	= pRecevBuff[20];                                      //系统日期时间长度
		 pMsg->m_SystemTimeLen[1]	= pRecevBuff[21];                                      //系统日期时间长度
		 
		 for(i=22; i<28; i++)
		 {
				pMsg->m_SystemTimeValue[i-22]	= pRecevBuff[i];                               //系统日期时间值
		 }
		 
		 for(i=28; i<32; i++)
		 {
				pMsg->m_StartTimeOid[i-28]	= pRecevBuff[i];                                  //采样开始时间Oid
		 }
		 pMsg->m_StartTimeLen[0]	= pRecevBuff[32];                                       //采样开始时间长度
		 pMsg->m_StartTimeLen[1]	= pRecevBuff[33];                                       //采样开始时间长度
		 pMsg->m_StartTimeValue[0]	= pRecevBuff[34];                                     //采样开始时间值
		 pMsg->m_StartTimeValue[1]	= pRecevBuff[35];                                     //采样开始时间值

		 for(i=36; i<40; i++)
		 {
				pMsg->m_SpanOid[i-36]	= pRecevBuff[i];                                        //采样间隔Oid
		 }
		 pMsg->m_SpanLen[0]	= pRecevBuff[40];                                             //采样间隔数值长度
		 pMsg->m_SpanLen[1]	= pRecevBuff[41];                                             //采样间隔数值长度
		 pMsg->m_SpanValue[0]	= pRecevBuff[42];                                           //采样间隔值
		 pMsg->m_SpanValue[1]	= pRecevBuff[43];                                           //采样间隔值

		 for(i=44; i<48; i++)
		 {
				pMsg->m_NumberOid[i-44]	= pRecevBuff[i];                                      //采样次数Oid
		 }
		 pMsg->m_NumberLen[0]	= pRecevBuff[48];                                           //采样次数数值长度
		 pMsg->m_NumberLen[1]	= pRecevBuff[49];                                           //采样次数数值长度
		 pMsg->m_NumberValue[0]	= pRecevBuff[50];                                         //采样次数值
		 pMsg->m_NumberValue[1]	= pRecevBuff[51];                                         //采样次数值
		 
		 for(i=52; i<56; i++)
		 {
				pMsg->m_UploadCountOid[i-52]	= pRecevBuff[i];                                //上传次数Oid
		 }
		 pMsg->m_UploadCountLen[0]	= pRecevBuff[56];                                     //上传次数数值长度
		 pMsg->m_UploadCountLen[1]	= pRecevBuff[57];                                     //上传次数数值长度
		 pMsg->m_UploadCountValue[0]	= pRecevBuff[58];                                   //上传次数值
		 pMsg->m_UploadCountValue[1]	= pRecevBuff[59];                                   //上传次数值
		 
		 pMsg->m_CRC[0]	= pRecevBuff[60];                                                 //CRC校验值
		 pMsg->m_CRC[1]	= pRecevBuff[61];                                                 //CRC校验值	 
		 
				
		Time_Auto_Regulate(pMsg);                                                         //校准日期时间
					
		temp = (pMsg->m_StartTimeValue[0]<<8)+(pMsg->m_StartTimeValue[1]); 
		BKP_WriteBackupRegister(BKP_DR2, temp);                                           //备份采集开始时间
		temp = (pMsg->m_SpanValue[0]<<8)+(pMsg->m_SpanValue[1]);
		BKP_WriteBackupRegister(BKP_DR3, temp);                                           //备份采集间隔
		temp = (pMsg->m_NumberValue[0]<<8)+(pMsg->m_NumberValue[1]);
		BKP_WriteBackupRegister(BKP_DR4, temp);                                           //备份采集次数
		temp = (pMsg->m_UploadCountValue[0]<<8)+(pMsg->m_UploadCountValue[1]); 
		BKP_WriteBackupRegister(BKP_DR5, temp);                                           //备份上传次数
		SLNoiseConfigFlag = 1;
		BKP_WriteBackupRegister(BKP_DR7, SLNoiseConfigFlag);
				
		 if(pRecevBuff< (Usart4_recev_buff+sizeof(Usart4_recev_buff)-25))                 //防止指针越界 
		 {
			 for(i=0;i<62;i++)
			 {
	//			 #if DEBUG_TEST	
				 printf("\r\nData receive from Server is :%x",pRecevBuff[i]);
			 }

		   ConfigData_Upload();
			 
	//			 #endif
			 return 1;
		 }
		 else
		 {
				printf("\r\nWarnning!Memory OverFlow Occur!!\r\n");
		 }
		 return 0;

   }else if(pRecevBuff[13] == 0x01)                                                    // 操作类型为0x01(服务器查询配置信息) 
	 { 
		 ConfigData_Upload();
   }else if(pRecevBuff[13] == 0x05)                                                    // 操作类型为0x05(上传噪声数据回复) 
	 { 
		 printf("\r\n SLNoise_Data Upload successfully!!\r\n");
   }else if(pRecevBuff[13] == 0x09)                                                    // 操作类型为0x09(开机注册回复) 
	 { 
		 printf("\r\n Device Start Login successfully!!\r\n");
   }else if(pRecevBuff[13] == 0x0A)                                                    // 操作类型为0x0A(唤醒回复) 
	 { 
		 SLNoise_DataUpload(&DeviceConfig, 2);                                             // 唤醒回复带数据上传
   }else if(pRecevBuff[13] == 0x12)                                                    // 操作类型为0x12(配置433模块参数) 
	 { 
		 printf("\r\n set 433!!\r\n");

		 for(m=0;m<13;m++)
		 {
			 SetCommand[m] = pRecevBuff[15+m];
		 }
		 SetCommand[0] = 0xAF;                                                             //为了使改433配置信息的13个字节不被433模块识别出来，人为地把第一个字节AF改为BF，接收后再改回去
		 for(m=0;m<10;m++)
		 {
			 Verify =Verify + SetCommand[m];
		 }
		 SetCommand[10] = Verify %256;
		 
		 memset(Usart4_recev_buff,'\0',300);	
		 Usart4_recev_count =0;                                       //清空USART3接收计数器
		 time=0;
		 
		 mput_mix((char*)SetCommand,13);
		 
		if(Receive_Monitor_433() == 1)
		{
			  printf("\r\n433模块信息输出:");
				 for(n=0;n<Usart4_recev_count;n++)
				 {
						printf(" %x",Usart4_recev_buff[n]);
				 }
//				DeviceId[1] = Usart4_recev_buff[8];
//				DeviceId[2] = Usart4_recev_buff[9];
//				DeviceId[3] = Usart4_recev_buff[10];
				
				for(n=0;n<6;n++)
				{
          Config_Data[n] = DeviceId[n];
        }
				
				Config_Data[6] = 0xBF;

				for(n=7;n<Usart4_recev_count+7;n++)
				{
          Config_Data[n] = Usart4_recev_buff[n-6];
        }
				  
//				Usart4_recev_buff[0] = DeviceId[4];
//				Usart4_recev_buff[1] = DeviceId[5];
				SendMessage((char*)Config_Data,sizeof(Config_Data));
        				 
				memset(Usart4_recev_buff,'\0',300);	
				Usart4_recev_count =0;                                       //清空USART3接收计数器
				time=0;	                                                     //定时器复位
		}	
   }else
	 {
		 printf("\r\npRecevBuff[13] is :%x",pRecevBuff[13]);
   }
}

//// *****************************************************************************
//// Design Notes: 设置433模块ID
//// -----------------------------------------------------------------------------
void SetNodeID (unsigned short sNodeID)
{
	u8  SetCommand[13] ={0xAF,0xAF,0x00,0x00,0xAF,0x80,0x13,0x02,0xFF,0xFF,0xFF,0x0D,0x0A};  //3个0xFF字段分别为节点ID高字节、节点ID低字节和校验和，需要输入或者计算得到
  u16 Verify =0x0000;
  u8  i =0;

  SetCommand[8] =sNodeID >>8;
  SetCommand[9] =sNodeID &0xFF;
  for(i=0;i<10;i++)
  {
    Verify =Verify + SetCommand[i];
  }
	SetCommand[10] =Verify %256;
  mput_mix((char*)SetCommand,13);
	
	Delay_ms(2000);

}

