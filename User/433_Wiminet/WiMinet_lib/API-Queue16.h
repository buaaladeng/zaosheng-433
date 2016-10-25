// #############################################################################
// *****************************************************************************
//          Copyright (c) 2007-2009, WiMi-net (Beijing) Tech. Co., Ltd.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//         INFORMATION WHICH IS THE PROPERTY OF WIMI-NET TECH. CO., LTD.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               WIMI-NET TECH. CO., LTD, IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    simplequeue16.h
// Author:  Mickle.ding
// Created: 7/22/2009
//
// Description:  Define the class simplequeue16
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#ifndef _SIMPLE_QUEUE16_INC_
#define _SIMPLE_QUEUE16_INC_

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void InitAtomicQueue16( 
   AtomicQueue16 * pQueue, 
   char     * pBuffer, 
   unsigned short  iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned char ReadAtomicQueue16( 
   AtomicQueue16 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned char WriteAtomicQueue16( 
   AtomicQueue16 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void SnapAtomicQueue16( AtomicQueue16 * pQueue, char * pBuffer, unsigned char iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void PeekQueue16Status( AtomicQueue16 * pQueue, QueueStatus16 * pStatus );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif