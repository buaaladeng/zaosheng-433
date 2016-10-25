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
void InitAtomicQueue32( 
   AtomicQueue32 * pQueue, 
   char     * pBuffer, 
   unsigned short  iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned char ReadAtomicQueue32( 
   AtomicQueue32 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned char WriteAtomicQueue32( 
   AtomicQueue32 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void SnapAtomicQueue32( AtomicQueue32 * pQueue, char * pBuffer, unsigned char iSize );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void PeekQueue32Status( AtomicQueue32 * pQueue, QueueStatus32 * pStatus );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif