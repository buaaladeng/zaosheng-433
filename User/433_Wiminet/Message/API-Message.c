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
// File:    communication.c
// Author:  Mickle.ding
// Created: 1/22/2007
//
// Description:  Define the class message
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Platform.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-CRC.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-File.h"

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
   iCRC2 = QuickCRC16( ( unsigned char * )pMsg, iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#if ( CPU_ENDIAN_MODE == LITTLE_ENDIAN_MODE )

   // Change the byte order
   iCRC2 = ntohs( iCRC2 );

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
// Design Notes:  
// -----------------------------------------------------------------------------
void UpdateNodeMsgCRC( NodeMsg * pMsg )
{
   unsigned char  iSize;
   unsigned short iCRC;
   
   // The message header
   pMsg->m_iHeader = 0XAA;

   // The defualt CRC value
   pMsg->m_iCRCode = 0X00;

   // The message size
   iSize = 0X09;
   iSize += pMsg->m_iAmount;

   // Update the CRC value
   iCRC = QuickCRC16( ( unsigned char * )pMsg, iSize );

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

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void WriteNodeMsg( unsigned char hFile, NodeMsg * pMsg )
{
   unsigned char iSize;

   // Update the message CRC value
   UpdateNodeMsgCRC( pMsg );

   // The packet whole size
   iSize = 0X09 + pMsg->m_iAmount;

   // Write out message header
   WriteFile( hFile, ( char * )pMsg, iSize );
}
