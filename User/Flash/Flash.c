 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   spi flash 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 ISO-MINI STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

 /**
  ******************************************************************************
    block sector page
  0x FF     F     F    FF
  256个block 16
  ******************************************************************************
  */  






#include "SPI_Flash.h"
#include "bsp_SysTick.h"
#include "bsp_usart.h"

/* Private typedef -----------------------------------------------------------*/
//#define SPI_FLASH_PageSize         4096
#define SPI_FLASH_PageSize           256
#define SPI_FLASH_PerWritePageSize   256

/* Private define ------------------------------------------------------------*/
#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		    0x05 
#define W25X_WriteStatusReg		    0x01 
#define W25X_ReadData			        0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7 
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	    0xAB 
#define W25X_DeviceID			        0xAB             //原始数据：
#define W25X_ManufactDeviceID   	0x17             //原始数据：0x90  
#define W25X_JedecDeviceID		    0x9F 

#define WIP_Flag                  0x01  /* Write In Progress (WIP) flag */
#define Dummy_Byte                0xFF


//***********************************************************************
typedef enum { FAILED = 0, PASSED = !FAILED} TestStatus;
/* 获取缓冲区的长度 */
//#define TxBufferSize1   (countof(TxBuffer1) - 1)
//#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define BufferSize      (countof(Tx_Buffer))

#define  FLASH_WriteAddress     0x00000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress
//#define  sFLASH_ID                0xEF4017	 //W25Q64
#define  sFLASH_ID                  0xEF4018	 //W25Q128BV
     
/* 发送缓冲区初始化 */
uint8_t Tx_Buffer[5] = {1,2,3,4};
uint8_t Rx_Buffer[BufferSize];
extern char  Dense_Data[60]; 
extern uint16_t  Noise_Count;
__IO uint32_t DeviceID = 0;
__IO uint32_t FlashID = 0;
__IO TestStatus TransferStatus1 = FAILED;

// 函数原型声明
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
//***********************************************************************



u32 AddrMapping(u32 block,u32 sector,u32 page)
{
  u32 result;
	if(block>0xFF)
		{
      printf("block max :256!!");
			block = 0xFF;
    }
 if(sector>0x0F)
		{
      printf("sector max :16!!");
			sector = 0x0F;
    }
 if(page>0x0F)
		{
      printf("page max :16!!");
			page = 0x0F;
    }
 result = (block<<16) + (sector<<12) + (page<<8);
		return result;
}


/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable SPI1 and GPIO clocks */
  /*!< SPI_FLASH_SPI_CS_GPIO, SPI_FLASH_SPI_MOSI_GPIO, 
       SPI_FLASH_SPI_MISO_GPIO, SPI_FLASH_SPI_DETECT_GPIO 
       and SPI_FLASH_SPI_SCK_GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

  /*!< SPI_FLASH_SPI Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
 
  
  /*!< Configure SPI_FLASH_SPI pins: SCK */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure SPI_FLASH_SPI pins: MISO */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure SPI_FLASH_SPI pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /*!< Configure SPI_FLASH_SPI_CS_PIN pin（PB12）: SPI_FLASH Card CS pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* SPI1 configuration */
  // W25X16: data input on the DIO pin is sampled on the rising edge of the CLK. 
  // Data on the DO and DIO pins are clocked out on the falling edge of CLK.
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);

  /* Enable SPI2  */
  SPI_Cmd(SPI2, ENABLE);
}
/*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase
* Description    : Erases the specified FLASH sector.
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_SectorErase(u32 SectorAddr)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();
  SPI_FLASH_WaitForWriteEnd();
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  SPI_FLASH_SendByte(W25X_SectorErase);
  /* Send SectorAddr high nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BulkErase
* Description    : Erases the entire FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BulkErase(void)
{
  /* Send write enable instruction */
  SPI_FLASH_WriteEnable();

  /* Bulk Erase */
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */
  SPI_FLASH_SendByte(W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_PageWrite
* Description    : Writes more than one byte to the FLASH with a single WRITE
*                  cycle(Page WRITE sequence). The number of byte can't exceed
*                  the FLASH page size.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH,
*                    must be equal or less than "SPI_FLASH_PageSize" value.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */
  SPI_FLASH_WriteEnable();

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  SPI_FLASH_SendByte(W25X_PageProgram);
  /* Send WriteAddr high nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
     //printf("\n\r Err: SPI_FLASH_PageWrite too large!");
  }

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    SPI_FLASH_SendByte(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  SPI_FLASH_WaitForWriteEnd();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferWrite
* Description    : Writes block of data to the FLASH. In this function, the
*                  number of WRITE cycles are reduced, using Page WRITE sequence.
* Input          : - pBuffer : pointer to the buffer  containing the data to be
*                    written to the FLASH.
*                  - WriteAddr : FLASH's internal address to write to.
*                  - NumByteToWrite : number of bytes to write to the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize; //页对齐
  count = SPI_FLASH_PageSize - Addr; //首页可存数据数
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;//总共页数
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;//末尾数据数

  if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
  {
    if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
    {
      if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
      {
        temp = NumOfSingle - count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
        WriteAddr +=  count;
        pBuffer += count;

        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
      }
      else
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
      }
    }
    else /* NumByteToWrite > SPI_FLASH_PageSize */
    {
      NumByteToWrite -= count;
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/*******************************************************************************
* Function Name  : SPI_FLASH_BufferRead
* Description    : Reads a block of data from the FLASH.
* Input          : - pBuffer : pointer to the buffer that receives the data read
*                    from the FLASH.
*                  - ReadAddr : FLASH's internal address to read from.
*                  - NumByteToRead : number of bytes to read from the FLASH.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send ReadAddr high nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
    /* Read a byte from the FLASH */
    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
    /* Point to the next location where the byte read will be saved */
		if(NumByteToRead!=0)    //自己增加
		{
      pBuffer++; 
    }
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadID(void)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(W25X_JedecDeviceID);

  /* Read a byte from the FLASH */
  Temp0 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp1 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp2 = SPI_FLASH_SendByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_ReadID
* Description    : Reads FLASH identification.
* Input          : None
* Output         : None
* Return         : FLASH identification
*******************************************************************************/
u32 SPI_FLASH_ReadDeviceID(void)
{
  u32 Temp = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  SPI_FLASH_SendByte(W25X_DeviceID);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  
  /* Read a byte from the FLASH */
  Temp = SPI_FLASH_SendByte(Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();

  return Temp;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(u32 ReadAddr)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 SPI_FLASH_ReadByte(void)
{
  return (SPI_FLASH_SendByte(Dummy_Byte));
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_FLASH_SendByte(u8 byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
u16 SPI_FLASH_SendHalfWord(u16 HalfWord)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  /* Send Half Word through the SPI2 peripheral */
  SPI_I2S_SendData(SPI2, HalfWord);

  /* Wait to receive a Half Word */
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the Half Word read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI2);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WriteEnable
* Description    : Enables the write access to the FLASH.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WriteEnable(void)
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */
  SPI_FLASH_SendByte(W25X_WriteEnable);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}

/*******************************************************************************
* Function Name  : SPI_FLASH_WaitForWriteEnd
* Description    : Polls the status of the Write In Progress (WIP) flag in the
*                  FLASH's status  register  and  loop  until write  opertaion
*                  has completed.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_WaitForWriteEnd(void)
{
  u8 FLASH_Status = 0;

  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Read Status Register" instruction */
  SPI_FLASH_SendByte(W25X_ReadStatusReg);

  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);	 
  }
  while ((FLASH_Status & WIP_Flag) == SET); /* Write in progress */

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}


//进入掉电模式
void SPI_Flash_PowerDown(void)   
{ 
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  SPI_FLASH_SendByte(W25X_PowerDown);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
}   

//唤醒
void SPI_Flash_WAKEUP(void)   
{
  /* Select the FLASH: Chip Select low */
  SPI_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  SPI_FLASH_SendByte(W25X_ReleasePowerDown);

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();                   //等待TRES1
}   
   
/*
 * 函数名：Buffercmp
 * 描述  ：比较两个缓冲区中的数据是否相等
 * 输入  ：-pBuffer1     src缓冲区指针
 *         -pBuffer2     dst缓冲区指针
 *         -BufferLength 缓冲区长度
 * 输出  ：无
 * 返回  ：-PASSED pBuffer1 等于   pBuffer2
 *         -FAILED pBuffer1 不同于 pBuffer2
 */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return PASSED;
}
/*
 * 函数名：FlashTest
 * 描述  ：Flash测试函数
 * 输入  ：无
 * 输出  ：无
 */
//void FlashTest(void)
//{ 	
//   
//	u8 i=0;
//	printf("\r\n ===== 0x%x=======,  ==== 0x%x=====\r\n", sizeof(Tx_Buffer), sizeof(*(Tx_Buffer)));
//	/* 2M串行flash W25Q16初始化 */
//	SPI_FLASH_Init();
//	/* Get SPI Flash Device ID */
//	DeviceID = SPI_FLASH_ReadDeviceID();
//  Delay_us(800);                        //有待确认
//	/* Get SPI Flash ID */
//	FlashID = SPI_FLASH_ReadID();
//	printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
//	
//	/* Check the SPI Flash ID */
//	if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4018 */
//	{	
//		printf("\r\n 检测到华邦串行flash W25Q128BV !\r\n");
//		/* Erase SPI FLASH Sector to write on */
//		SPI_FLASH_SectorErase(FLASH_SectorToErase);	 
/////* 将发送缓冲区的数据写到flash中 */
//		SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress, BufferSize);		
//		SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, 1);//测试使用
//		if(Rx_Buffer[0]!=255)
//		{
//			/* 将发送缓冲区的数据写到flash中 */
//			SPI_FLASH_BufferWrite(Tx_Buffer, FLASH_WriteAddress+BufferSize*2, BufferSize);
//    }
//		
//		
////		printf("\r\n 写入的数据为：");             //测试使用
////		for(i=0;i<BufferSize;i++)               
////		{
////		  printf("%d  ", Tx_Buffer[i]);           //测试使用
////		}                                     
//		
//		/* 将刚刚写入的数据读出来放到接收缓冲区中 */
//		SPI_FLASH_BufferRead(Rx_Buffer, FLASH_ReadAddress, BufferSize*3);
//		printf("\r\n 读出的数据为:");
//		for(i=0;i<BufferSize*3;i++)
//		{
//			printf("%d  ", Rx_Buffer[i]);
//		}
//		/* 检查写入的数据与读出的数据是否相等 */
//		TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);
//		
//		if( PASSED == TransferStatus1 )
//		{    
//			printf("\r\n 16M串行flash(W25Q128BV)测试成功!\n\r");
//		}
//		else
//		{        
//			printf("\r\n 16M串行flash(W25Q128BV)测试失败!\n\r");
//		}
//	}// if (FlashID == sFLASH_ID)
//	else
//	{    
//		printf("\r\n 获取不到 W25Q128BV ID!\n\r");
//	}
//	SPI_Flash_PowerDown();  

//}

/*
 * 函数名：SLNoiseDataBkp
 * 描述  ：数据备份函数
 * 输入  ：噪声数据、长度
 * 输出  ：void
 */

void SLNoiseDataBkp(char *SLNoise_Char,char Length_Frame) //噪声数据备份
{
  u32 Addr;
	//char Data_Num[2] = {'/0'};
	u8 IndexBkp[10] = {0};
//	u8 temp;
	
  Addr = AddrMapping(0,1,0); //所备份噪声数据的字典
		DeviceID = SPI_FLASH_ReadDeviceID();
		Delay_us(800);                          //有待确认
		/* Get SPI Flash ID */
		FlashID = SPI_FLASH_ReadID();
		printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
		/* Check the SPI Flash ID */
		if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
		{
			SPI_FLASH_BufferRead(IndexBkp,Addr,1);//噪声数据个数
		}
	printf("Data_Num: %d",IndexBkp[0]);
	if(IndexBkp[0]<15||IndexBkp[0] == 0xFF)
	{
			if(IndexBkp[0] == 0xFF)
		{
			IndexBkp[0] = 1;
		}
		else
		{
			SPI_FLASH_BufferRead(IndexBkp,Addr,IndexBkp[0]+1);
			IndexBkp[0] = IndexBkp[0] + 1;
		}
		IndexBkp[IndexBkp[0]] = Length_Frame;
		
		DeviceID = SPI_FLASH_ReadDeviceID();
		Delay_us(800);                          //有待确认
		/* Get SPI Flash ID */
		FlashID = SPI_FLASH_ReadID();
		printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
		/* Check the SPI Flash ID */
		if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
		{	
			printf("\r\n Write !\r\n");   //测试使用
			SPI_FLASH_SectorErase(Addr);    
			/* 将发送缓冲区的数据写到flash中 */
			SPI_FLASH_BufferWrite(IndexBkp,Addr,IndexBkp[0] + 1);	
			Delay_us(200); 
//		SPI_FLASH_BufferRead(Rx_Buffer, (FLASH_ReadAddress + ExcursionAddress), 2); //测试使用	
		}
		SPI_Flash_PowerDown(); 
 
		Addr = AddrMapping(1,IndexBkp[0],0);
		DeviceID = SPI_FLASH_ReadDeviceID();
		Delay_us(800);                          //有待确认
		/* Get SPI Flash ID */
		FlashID = SPI_FLASH_ReadID();
		printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
		/* Check the SPI Flash ID */
		if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
		{	
			printf("\r\n Write !\r\n");   //测试使用
			SPI_FLASH_SectorErase(Addr);	
			SPI_FLASH_BufferWrite((u8*)SLNoise_Char,Addr,Length_Frame);
			Delay_us(200); 
//		SPI_FLASH_BufferRead(Rx_Buffer, (FLASH_ReadAddress + ExcursionAddress), 2); //测试使用	
		}
		SPI_Flash_PowerDown(); 
		
		
	printf("Data_Num: %d",IndexBkp[0]);		
	}

	else
	{
		
		printf("storage is full!!!!");
	}



}


/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          :  数据存储在Flash中的偏移地址, 电池容量系数
* Output         : None
* Return         : None
*******************************************************************************/
void BatteryCapacityWrite_to_Flash(uint8_t* BT_CapacityFactor)
{
   u32 Addr;
	Addr = AddrMapping(0,3,0);
	SPI_FLASH_Init();
	/* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
  Delay_us(800);                          //有待确认
	/* Get SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
	/* Check the SPI Flash ID */
	if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
	{	
		printf("\r\n Write !\r\n");   //测试使用
//		SPI_FLASH_SectorErase(FLASH_SectorToErase);	    
		SPI_FLASH_SectorErase(Addr);	 
    /* 将发送缓冲区的数据写到flash中 */
//		Tx_Buffer[0] = BT_CapacityFactor>>8;
//		Tx_Buffer[1] = BT_CapacityFactor &0xFF;
	  SPI_FLASH_BufferWrite(BT_CapacityFactor,Addr,1);		
		Delay_us(200); 
//		SPI_FLASH_BufferRead(Rx_Buffer, (FLASH_ReadAddress + ExcursionAddress), 2); //测试使用	
	}
	SPI_Flash_PowerDown(); 
 
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : 数据存储在Flash中的偏移地址
* Output         : None
* Return         : 电池容量系数（例如：单节电池为19Ah，刚开始总容量为19*N，返回值则为N-1，N随着电池电量的消耗，逐渐减小，直到0
*******************************************************************************/
uint8_t BatteryCapacityRead_From_Flash(void)
{
  uint8_t  BatteryCapacityFactor =0; //电池容量系数
	u32 Addr;
  Addr = AddrMapping(0,3,0);
	SPI_FLASH_Init();
	
	/* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
  Delay_us(800);                          //有待确认
	/* Get SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
	/* Check the SPI Flash ID */
	if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
	{	
		printf("\r\n Read !\r\n");   //测试使用
		
		SPI_FLASH_BufferRead(&BatteryCapacityFactor,Addr, 1);
	}
	SPI_Flash_PowerDown(); 
//  BatteryCapacityFactor	= Rx_Buffer[0]*256 +  Rx_Buffer[1];
	printf("\r\n BatteryCapacityFactor:%x!\r\n", BatteryCapacityFactor);     //测试使用
	
	return  BatteryCapacityFactor;
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : 数据存储在Flash中的偏移地址
* Output         : None
* Return         : 循环数据
*******************************************************************************/
void TempNoiseDataBkp(void)
{
  //uint16_t  BatteryCapacityFactor =0x0000; //电池容量系数
	u32 Addr;
  Addr = AddrMapping(0,2,0);
	SPI_FLASH_Init();
	
	/* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
  Delay_us(800);                          //有待确认
	/* Get SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
	/* Check the SPI Flash ID */
	if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
	{	
		printf("\r\n Write !\r\n");   //测试使用
		SPI_FLASH_SectorErase(Addr);	
		SPI_FLASH_BufferWrite((u8*)Dense_Data,Addr,Noise_Count);
		Delay_us(200); 
	}
	SPI_Flash_PowerDown(); 
  //BatteryCapacityFactor	= Rx_Buffer[0]*256 +  Rx_Buffer[1];
	//printf("\r\n BatteryCapacityFactor:%x!\r\n", BatteryCapacityFactor);     //测试使用
	
	//return  BatteryCapacityFactor;
}

/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : 数据存储在Flash中的偏移地址
* Output         : None
* Return         : 循环数据
*******************************************************************************/
void TempNoiseDataRead(void)
{
	u32 Addr;
  Addr = AddrMapping(0,2,0);
	SPI_FLASH_Init();
	
	/* Get SPI Flash Device ID */
	DeviceID = SPI_FLASH_ReadDeviceID();
  Delay_us(800);                          //有待确认
	/* Get SPI Flash ID */
	FlashID = SPI_FLASH_ReadID();
	printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);
	
	/* Check the SPI Flash ID */
	if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4017 */
	{	
		printf("\r\n Read !\r\n");   //测试使用
		//SPI_FLASH_SectorErase(Addr);	
		SPI_FLASH_BufferRead((u8*)Dense_Data,Addr,Noise_Count);
		Delay_us(200); 
	}
	SPI_Flash_PowerDown(); 
}

/******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DataWrite_To_Flash(u32 BlockIndex, u32 SectorIndex, u32 PageIndex, u8* pBuffer, u16 NumByteToWrite)
{
		u32 Addr;   //Flash中存储指定数据的地址
		 
	  Addr = AddrMapping(BlockIndex, SectorIndex, PageIndex);  //根据Flash分区索引，获取Flash数据存储地址
		SPI_FLASH_Init();
		/* Get SPI Flash Device ID */
		DeviceID = SPI_FLASH_ReadDeviceID();
		Delay_us(800);                          //有待确认
		/* Get SPI Flash ID */
		FlashID = SPI_FLASH_ReadID();
		printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);  //测试使用
		/* Check the SPI Flash ID */
		if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4018 */
		{	
			printf("\r\nSPI Flash W25Q128BV is ready to write!\r\n");   //测试使用
			SPI_FLASH_SectorErase( Addr );    
			/* 将发送缓冲区的数据写到flash中 */
			SPI_FLASH_BufferWrite(pBuffer, Addr, NumByteToWrite);	
			Delay_us(200);
		}
		SPI_Flash_PowerDown();  
}
	


/******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DataRead_From_Flash(u32 BlockIndex, u32 SectorIndex, u32 PageIndex, u8* pBuffer, u16 NumByteToRead)
{
		u32 Addr;   //Flash中存储指定数据的地址
		 
	  Addr = AddrMapping(BlockIndex, SectorIndex, PageIndex);  //根据Flash分区索引，获取Flash数据存储地址
		SPI_FLASH_Init();
		/* Get SPI Flash Device ID */
		DeviceID = SPI_FLASH_ReadDeviceID();
		Delay_us(800);                          //有待确认
		/* Get SPI Flash ID */
		FlashID = SPI_FLASH_ReadID();
		printf("\r\n FlashID is 0x%x,  Manufacturer Device ID is 0x%x\r\n", FlashID, DeviceID);  //测试使用
		
		/* Check the SPI Flash ID */
		if (FlashID == sFLASH_ID)  /* #define  sFLASH_ID  0xEF4018 */
		{	
			printf("\r\nSPI Flash W25Q128BV is ready to read!\r\n");   //测试使用
			SPI_FLASH_BufferRead(pBuffer, Addr, NumByteToRead);
		}
		SPI_Flash_PowerDown();  
}


/*********************************************END OF FILE**********************/
