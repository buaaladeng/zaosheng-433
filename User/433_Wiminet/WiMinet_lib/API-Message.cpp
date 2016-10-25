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
// File:    message.cpp
// Author:  Mickle.ding
// Created: 1/22/2007
//
// Description:  Define the class message
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Windows.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#pragma comment ( lib, "ws2_32.lib" )

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <stdio.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-CRCheck.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

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
   iSize = pMsg->m_iAmount + 0X09;
   
   // Calculate the CRC
   iCRC2 = QuickCRC16( ( unsigned char * )pMsg, iSize );
   
   // Reverse the order
   iCRC2 = ntohs( iCRC2 );
   pMsg->m_iCRCode = iCRC1;
   
   return ( iCRC1 == iCRC2 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void UpdateNodeMsgCRC( NodeMsg * pMsg )
{
   unsigned short iCRC;
   unsigned short iSize;
   
   // The header
   pMsg->m_iHeader = 0XAA;

   // Reset value to calculate the CRC
   pMsg->m_iCRCode = 0X00;
   
   // The message size
   iSize = pMsg->m_iAmount + 0X09;
   
   // The message CRC
   iCRC = QuickCRC16( ( unsigned char * )pMsg, iSize );

   // Reverse the data order
   pMsg->m_iCRCode = ntohs( iCRC );
}
