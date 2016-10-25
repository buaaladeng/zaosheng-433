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
// File:    API-Platform.C
// Author:  Mickle.ding
// Created: 5/18/2007
//
// Description:  
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Platform.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short ntohs( unsigned short iValue )
{
   return ( iValue >> 0X08 ) + ( ( iValue & 0XFF ) << 0X08 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned long ntohl( unsigned long dwValue )
{
   unsigned short iHVal;
   unsigned short iLVal;

   // The MSB part
   iHVal = ntohs( dwValue >> 0X10 );

   // The LSB part
   iLVal = ntohs( dwValue & 0XFFFF );

   // Reverse the high and low part
   return ( ( unsigned long )iLVal << 0X10 ) + iHVal;
}
