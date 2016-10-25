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
// File:    api-service.cpp
// Author:  Mickle.ding
// Created: 11/4/2011
//
// Description:  Define the class api-service
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Process.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <stdio.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <stdlib.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Service.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-RxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-TxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-RxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-TxQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WiMinet.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Monitor.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-Uniform.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
API_Service  APIService;

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char OpenWiMinetShell( char * pDevice, long dwParam )
{
   char iRetVal;
   
   // Open the hardware interface
   iRetVal = OpenHardwareDeviceInterface( pDevice, dwParam );
   if ( !iRetVal )
   {
      return 0X00;
   }
   
   // Initialize the API-TxStateMachine
   API_SetupTxQueueStateMachine();
   
   // Initialize the API-RxStateMachine
   API_SetupRxQueueStateMachine();

   // Setup thread for message dispatch
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char StopWiMinetShell( char iShell )
{
   // Shutdown thread for message dispatch
   
   // Close the API-RxStateMachine
   API_CloseRxQueueStateMachine();
   
   // Close the API-TxStateMachine
   API_CloseTxQueueStateMachine();
   
   // Close the hardware interface
   CloseHardwareDeviceInterface();
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_StartService()
{
   unsigned long dwTreadID;

   // Initialize the WinSock
   WSADATA wsaData;
   WSAStartup( MAKEWORD( 2, 2 ), &wsaData );
   
   // Initialize the shell interface
   API_InitShellInterface();

   // Create a event for exit notification
   APIService.m_hExitEvent = CreateEvent( NULL, FALSE, FALSE, NULL );

   // Create the thread
   dwTreadID = _beginthreadex( NULL, 0X00, API_ServiceThread, 0X00, 0X00, NULL );

   // Convert to thread handle
   APIService.m_hISRThread = ( HANDLE )dwTreadID;

   // Switch to this new thread
   Sleep( 0X00 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseService()
{
   DWORD dwRetVal;
   ShellQueue * pQueue;
   ShellQueue::iterator iter;

   // Set the event exit status
   SetEvent( APIService.m_hExitEvent );
   
   // Wait for the thread exit
   dwRetVal = WaitForSingleObject( APIService.m_hISRThread, INFINITE );

   // Close the event handle
   CloseHandle( APIService.m_hExitEvent );

   // Get the queue pointer
   pQueue = &APIService.m_ShellQueue;

   // Make sure the queue is not empty
   if ( !APIService.m_ShellQueue.empty() )
   {
      // Close all the shell 
      for ( iter = ( *pQueue ).begin(); iter != ( *pQueue ).end(); iter++ )
      {
         DeleteUniformShell( &( *iter ) );
      }
   }

   // Clear the TCP/IP stack
   WSACleanup();
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned int _stdcall API_ServiceThread( LPVOID lpParameter )
{
   DWORD dwRetVal; 
   
   while ( 0X01 )
   {
      // Wait for the exit notify event
      dwRetVal = WaitForSingleObject( APIService.m_hExitEvent, 0X01 );

      // Exit event notify?
      if ( dwRetVal == WAIT_OBJECT_0 )
      {
         return 0X00;
      }

      // Check the shell active status
      if ( !API_IsShellActive() )
      {
         continue;
      }
      
      // The message dispatcher
      API_MessageDispatcher();
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_MessageDispatcher()
{
   // Read the input message and dispatch
   API_IOShellHandler();

   // Update the node status
   API_MonitorHandler();

   // The Tx-Routine
   API_TxQueueHandler();

   // The Rx-Routine
   API_RxQueueHandler();
}
