// #############################################################################
// *****************************************************************************
//                  Copyright (c) 2007-2009, WiMi-net Corp.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//               INFORMATION WHICH IS THE PROPERTY OF WIMI-NET CORP.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//                   WIMI-NET CORP., IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    api-service.h
// Author:  Mickle.ding
// Created: 11/4/2011
//
// Description:  Define the class api-service
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#ifndef _API_SERVICE_INC_

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#define _API_SERVICE_INC_


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Queue16.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-TxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-TxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-RxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-RxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
typedef struct _API_SERVICE_
{
   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   HANDLE                                                      m_hISRThread;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   HANDLE                                                      m_hExitEvent;

   // --------------------------------------------------------------------------
   // DESCRIPTION:
   // --------------------------------------------------------------------------
   ShellQueue                                                  m_ShellQueue;

} API_Service;

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
unsigned int _stdcall API_ServiceThread( LPVOID lpParameter );

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_MessageDispatcher();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_StartService();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
void API_CloseService();

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#endif