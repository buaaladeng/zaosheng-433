// #############################################################################
// *****************************************************************************
//          Copyright (c) 2007-2008, WiMi-net (Beijing) Tech. Co., Ltd.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//         INFORMATION WHICH IS THE PROPERTY OF WIMI-NET TECH. CO., LTD.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               WIMI-NET TECH. CO., LTD, IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    crc.cpp
// Author:  Mickle.ding
// Created: 1/22/2007
//
// Description:  Define the class crc
// -----------------------------------------------------------------------------




// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-CRCheck.h"

// -----------------------------------------------------------------------------
// DESCRIPTION: CRC-16校验的高位字节表
// -----------------------------------------------------------------------------
static const unsigned char HiCRCTable[] = { 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40, 0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 
0X00, 0XC1, 0X81, 0X40, 0X01, 0XC0, 0X80, 0X41, 0X01, 0XC0, 0X80, 0X41, 0X00, 0XC1, 0X81, 0X40 };

// -----------------------------------------------------------------------------
// DESCRIPTION: CRC-16校验的低位字节表
// -----------------------------------------------------------------------------
static const unsigned char LoCRCTable[] = { 
0X00, 0XC0, 0XC1, 0X01, 0XC3, 0X03, 0X02, 0XC2, 0XC6, 0X06, 0X07, 0XC7, 0X05, 0XC5, 0XC4, 0X04, 
0XCC, 0X0C, 0X0D, 0XCD, 0X0F, 0XCF, 0XCE, 0X0E, 0X0A, 0XCA, 0XCB, 0X0B, 0XC9, 0X09, 0X08, 0XC8, 
0XD8, 0X18, 0X19, 0XD9, 0X1B, 0XDB, 0XDA, 0X1A, 0X1E, 0XDE, 0XDF, 0X1F, 0XDD, 0X1D, 0X1C, 0XDC, 
0X14, 0XD4, 0XD5, 0X15, 0XD7, 0X17, 0X16, 0XD6, 0XD2, 0X12, 0X13, 0XD3, 0X11, 0XD1, 0XD0, 0X10, 
0XF0, 0X30, 0X31, 0XF1, 0X33, 0XF3, 0XF2, 0X32, 0X36, 0XF6, 0XF7, 0X37, 0XF5, 0X35, 0X34, 0XF4, 
0X3C, 0XFC, 0XFD, 0X3D, 0XFF, 0X3F, 0X3E, 0XFE, 0XFA, 0X3A, 0X3B, 0XFB, 0X39, 0XF9, 0XF8, 0X38, 
0X28, 0XE8, 0XE9, 0X29, 0XEB, 0X2B, 0X2A, 0XEA, 0XEE, 0X2E, 0X2F, 0XEF, 0X2D, 0XED, 0XEC, 0X2C, 
0XE4, 0X24, 0X25, 0XE5, 0X27, 0XE7, 0XE6, 0X26, 0X22, 0XE2, 0XE3, 0X23, 0XE1, 0X21, 0X20, 0XE0, 
0XA0, 0X60, 0X61, 0XA1, 0X63, 0XA3, 0XA2, 0X62, 0X66, 0XA6, 0XA7, 0X67, 0XA5, 0X65, 0X64, 0XA4, 
0X6C, 0XAC, 0XAD, 0X6D, 0XAF, 0X6F, 0X6E, 0XAE, 0XAA, 0X6A, 0X6B, 0XAB, 0X69, 0XA9, 0XA8, 0X68, 
0X78, 0XB8, 0XB9, 0X79, 0XBB, 0X7B, 0X7A, 0XBA, 0XBE, 0X7E, 0X7F, 0XBF, 0X7D, 0XBD, 0XBC, 0X7C, 
0XB4, 0X74, 0X75, 0XB5, 0X77, 0XB7, 0XB6, 0X76, 0X72, 0XB2, 0XB3, 0X73, 0XB1, 0X71, 0X70, 0XB0, 
0X50, 0X90, 0X91, 0X51, 0X93, 0X53, 0X52, 0X92, 0X96, 0X56, 0X57, 0X97, 0X55, 0X95, 0X94, 0X54, 
0X9C, 0X5C, 0X5D, 0X9D, 0X5F, 0X9F, 0X9E, 0X5E, 0X5A, 0X9A, 0X9B, 0X5B, 0X99, 0X59, 0X58, 0X98, 
0X88, 0X48, 0X49, 0X89, 0X4B, 0X8B, 0X8A, 0X4A, 0X4E, 0X8E, 0X8F, 0X4F, 0X8D, 0X4D, 0X4C, 0X8C, 
0X44, 0X84, 0X85, 0X45, 0X87, 0X47, 0X46, 0X86, 0X82, 0X42, 0X43, 0X83, 0X41, 0X81, 0X80, 0X40 };


// -----------------------------------------------------------------------------
// DESCRIPTION: CRC-32校验的字节表
// -----------------------------------------------------------------------------
static const unsigned long CRC32Table[256] = {
0X00000000, 0X77073096, 0XEE0E612C, 0X990951BA, 0X076DC419, 0X706AF48F, 0XE963A535, 0X9E6495A3, 
0X0EDB8832, 0X79DCB8A4, 0XE0D5E91E, 0X97D2D988, 0X09B64C2B, 0X7EB17CBD, 0XE7B82D07, 0X90BF1D91, 
0X1DB71064, 0X6AB020F2, 0XF3B97148, 0X84BE41DE, 0X1ADAD47D, 0X6DDDE4EB, 0XF4D4B551, 0X83D385C7, 
0X136C9856, 0X646BA8C0, 0XFD62F97A, 0X8A65C9EC, 0X14015C4F, 0X63066CD9, 0XFA0F3D63, 0X8D080DF5, 
0X3B6E20C8, 0X4C69105E, 0XD56041E4, 0XA2677172, 0X3C03E4D1, 0X4B04D447, 0XD20D85FD, 0XA50AB56B, 
0X35B5A8FA, 0X42B2986C, 0XDBBBC9D6, 0XACBCF940, 0X32D86CE3, 0X45DF5C75, 0XDCD60DCF, 0XABD13D59, 
0X26D930AC, 0X51DE003A, 0XC8D75180, 0XBFD06116, 0X21B4F4B5, 0X56B3C423, 0XCFBA9599, 0XB8BDA50F, 
0X2802B89E, 0X5F058808, 0XC60CD9B2, 0XB10BE924, 0X2F6F7C87, 0X58684C11, 0XC1611DAB, 0XB6662D3D, 
0X76DC4190, 0X01DB7106, 0X98D220BC, 0XEFD5102A, 0X71B18589, 0X06B6B51F, 0X9FBFE4A5, 0XE8B8D433, 
0X7807C9A2, 0X0F00F934, 0X9609A88E, 0XE10E9818, 0X7F6A0DBB, 0X086D3D2D, 0X91646C97, 0XE6635C01, 
0X6B6B51F4, 0X1C6C6162, 0X856530D8, 0XF262004E, 0X6C0695ED, 0X1B01A57B, 0X8208F4C1, 0XF50FC457, 
0X65B0D9C6, 0X12B7E950, 0X8BBEB8EA, 0XFCB9887C, 0X62DD1DDF, 0X15DA2D49, 0X8CD37CF3, 0XFBD44C65, 
0X4DB26158, 0X3AB551CE, 0XA3BC0074, 0XD4BB30E2, 0X4ADFA541, 0X3DD895D7, 0XA4D1C46D, 0XD3D6F4FB, 
0X4369E96A, 0X346ED9FC, 0XAD678846, 0XDA60B8D0, 0X44042D73, 0X33031DE5, 0XAA0A4C5F, 0XDD0D7CC9, 
0X5005713C, 0X270241AA, 0XBE0B1010, 0XC90C2086, 0X5768B525, 0X206F85B3, 0XB966D409, 0XCE61E49F, 
0X5EDEF90E, 0X29D9C998, 0XB0D09822, 0XC7D7A8B4, 0X59B33D17, 0X2EB40D81, 0XB7BD5C3B, 0XC0BA6CAD, 
0XEDB88320, 0X9ABFB3B6, 0X03B6E20C, 0X74B1D29A, 0XEAD54739, 0X9DD277AF, 0X04DB2615, 0X73DC1683, 
0XE3630B12, 0X94643B84, 0X0D6D6A3E, 0X7A6A5AA8, 0XE40ECF0B, 0X9309FF9D, 0X0A00AE27, 0X7D079EB1, 
0XF00F9344, 0X8708A3D2, 0X1E01F268, 0X6906C2FE, 0XF762575D, 0X806567CB, 0X196C3671, 0X6E6B06E7, 
0XFED41B76, 0X89D32BE0, 0X10DA7A5A, 0X67DD4ACC, 0XF9B9DF6F, 0X8EBEEFF9, 0X17B7BE43, 0X60B08ED5, 
0XD6D6A3E8, 0XA1D1937E, 0X38D8C2C4, 0X4FDFF252, 0XD1BB67F1, 0XA6BC5767, 0X3FB506DD, 0X48B2364B, 
0XD80D2BDA, 0XAF0A1B4C, 0X36034AF6, 0X41047A60, 0XDF60EFC3, 0XA867DF55, 0X316E8EEF, 0X4669BE79, 
0XCB61B38C, 0XBC66831A, 0X256FD2A0, 0X5268E236, 0XCC0C7795, 0XBB0B4703, 0X220216B9, 0X5505262F, 
0XC5BA3BBE, 0XB2BD0B28, 0X2BB45A92, 0X5CB36A04, 0XC2D7FFA7, 0XB5D0CF31, 0X2CD99E8B, 0X5BDEAE1D, 
0X9B64C2B0, 0XEC63F226, 0X756AA39C, 0X026D930A, 0X9C0906A9, 0XEB0E363F, 0X72076785, 0X05005713, 
0X95BF4A82, 0XE2B87A14, 0X7BB12BAE, 0X0CB61B38, 0X92D28E9B, 0XE5D5BE0D, 0X7CDCEFB7, 0X0BDBDF21, 
0X86D3D2D4, 0XF1D4E242, 0X68DDB3F8, 0X1FDA836E, 0X81BE16CD, 0XF6B9265B, 0X6FB077E1, 0X18B74777, 
0X88085AE6, 0XFF0F6A70, 0X66063BCA, 0X11010B5C, 0X8F659EFF, 0XF862AE69, 0X616BFFD3, 0X166CCF45, 
0XA00AE278, 0XD70DD2EE, 0X4E048354, 0X3903B3C2, 0XA7672661, 0XD06016F7, 0X4969474D, 0X3E6E77DB, 
0XAED16A4A, 0XD9D65ADC, 0X40DF0B66, 0X37D83BF0, 0XA9BCAE53, 0XDEBB9EC5, 0X47B2CF7F, 0X30B5FFE9, 
0XBDBDF21C, 0XCABAC28A, 0X53B39330, 0X24B4A3A6, 0XBAD03605, 0XCDD70693, 0X54DE5729, 0X23D967BF, 
0XB3667A2E, 0XC4614AB8, 0X5D681B02, 0X2A6F2B94, 0XB40BBE37, 0XC30C8EA1, 0X5A05DF1B, 0X2D02EF8D };


// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short QuickCRC16( unsigned char * pMsg, unsigned short iSize )
{
   unsigned char iHiVal;                // high byte of CRC initialized
   unsigned char iLoVal;                // low byte of CRC initialized
   unsigned char index;                 // will index into CRC lookup table
   
   // Initial value for the CRC
   iHiVal = 0XFF;
   iLoVal = 0XFF;
   
   while ( iSize-- )
   {
      // Calculate the CRC
      index = iLoVal ^ ( unsigned char )( *pMsg++ );
      
      iLoVal = iHiVal ^ HiCRCTable[index];
      iHiVal = LoCRCTable[index];
   }
   return ( iHiVal << 8 | iLoVal );
}

// *****************************************************************************
// Design Notes:
// f(x) = X^32 + X^26 + X^23 + X^22 + X^16 + X^12 + X^11 + X^10 + X^8 + X^7 + 
// X^5 + X^4 + X^2 + X + 1  
// -----------------------------------------------------------------------------
unsigned long QuickCRC32( unsigned char * pMsg, unsigned short iSize )
{
   register unsigned long iCRC32;
   register unsigned long iValue;
   register unsigned long iIndex;
   
   // Initial CRC value
   iCRC32 = 0XFFFFFFFF;
   while( iSize-- ) 
   {
      iValue = *pMsg;
      iIndex = ( iCRC32 ^ iValue ) & 0XFF;
      iCRC32 = ( ( iCRC32 >> 0X08 ) & 0X00FFFFFF ) ^ CRC32Table[iIndex];
      pMsg++;
   }
   return ( iCRC32 ^ 0XFFFFFFFF );
}

// *****************************************************************************
// Design Notes: 
// f(x) = X^32 + X^26 + X^23 + X^22 + X^16 + X^12 + X^11 + X^10 + X^8 + X^7 + 
// X^5 + X^4 + X^2 + X + 1
// -----------------------------------------------------------------------------
unsigned long QuickFileCRC32( char * pFileName )
{
   FILE * pFile;
   char buffer[1024];
   register unsigned long iCRC32;
   register unsigned long iValue;
   register unsigned long iIndex;
   register unsigned long iCount;

   // Open this file for CRC caclulation
   iCRC32 = 0XFFFFFFFF;   
   pFile = fopen( pFileName, "rb" );
   if ( !pFile )
   {
      return iCRC32;
   }

   while ( !feof( pFile ) )
   {
      iCount = fread( buffer, 0X01, sizeof( buffer ), pFile );
      for ( iIndex = 0X00; iIndex < iCount; iIndex++ )
      {
         iValue = ( iCRC32 ^ buffer[iIndex] ) & 0XFF;
         iCRC32 = ( ( iCRC32 >> 0X08 ) & 0X00FFFFFF ) ^ CRC32Table[iValue];         
      }
   }

   // Close this file
   fclose( pFile );
   return ( iCRC32 ^ 0XFFFFFFFF );   
}

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

// *****************************************************************************
// Design Notes: 
// f(X) = X^16 + X^12 + X^5 + X^0                 
// POLYNOMIALS = 0X1021
// -----------------------------------------------------------------------------
unsigned short CRC_CCITT( unsigned char * pMsg, unsigned short iSize )
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
            iCRC = ( iCRC << 1 ) ^ 0X1021;
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

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ASCII_CRC ( unsigned char * pMsg, unsigned short  iSize ) 
{
   // LRC char initialized
   unsigned char iCRC;
   iCRC = 0X00;
   
   // pass through message buffer
   while ( iSize-- )
   {
      // add buffer byte without carry
      iCRC += *pMsg++;
   }
   
   // return twos complement
   return ( ( unsigned char )( -( ( char )iCRC ) ) ) ;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char NEMA_CRC( unsigned char * pMsg, unsigned short iSize )
{
   unsigned char  iCRC;
   unsigned short index;
   
   // Reset the CRC of this string
   iCRC = 0X00;
   
   // Make sure this string is valid
   if ( iSize <= 1 )
   {
      return iCRC;
   }
   
   // Calculate the CRC of this string
   for ( index = 0; index < iSize; index++ )
   {
      iCRC = iCRC ^ pMsg[index];
   }
   return iCRC;
}



