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
// File:    hal-uniform.cpp
// Author:  Mickle.ding
// Created: 1/5/2012
//
// Description:  Define the class hal-uniform
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-Uniform.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WiMinet.h"

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
#include "API-Monitor.h"


// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellIODevice( ShellIODevice * pIODevice )
{
   // Reset the control block
   memset( pIODevice, 0X00, sizeof( ShellIODevice ) );
   
   // The phsical ID
   pIODevice->m_iPhysicalID = NUL_SHELL_INTERFACE;
   
   // The file handle
   pIODevice->m_hFileHandle = INVALID_HANDLE_VALUE; 
   
   // The message poointer
   pIODevice->m_pMsg = ( NodeMsg * )pIODevice->m_pBuffer;

   // Initialize the packet queue
   MallocQueueBuffer( &pIODevice->m_PacketQueue, 1024 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellIODevice( ShellIODevice * pIODevice )
{
   // Delete the queue buffer
   DeleteQueueBuffer( &pIODevice->m_PacketQueue );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellMsgQueue( ShellMsgQueue * pMsgQueue )
{
   // Initialize the string queue
   MallocQueueBuffer( &pMsgQueue->m_StringQueue, 1024 );
   
   // Initialize the Txd-Ack queue
   MallocQueueBuffer( &pMsgQueue->m_TxdAckQueue, 1024 );
   
   // Initialize the Rxd-Pkt queue
   MallocQueueBuffer( &pMsgQueue->m_RxdPktQueue, 1024 );
   
   // Initialize the Sig-Pkt queue
   MallocQueueBuffer( &pMsgQueue->m_SigPktQueue, 1024 );
   
   // Initialize the Cmd-Ack queue
   MallocQueueBuffer( &pMsgQueue->m_CmdAckQueue, 1024 );
   
   // Initialize the Net-Mon queue
   MallocQueueBuffer( &pMsgQueue->m_NetMonQueue, 1024 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellMsgQueue( ShellMsgQueue * pMsgQueue )
{
   // Delete the string queue buffer
   DeleteQueueBuffer( &pMsgQueue->m_StringQueue );
   
   // Delete the Txd-Ack queue buffer
   DeleteQueueBuffer( &pMsgQueue->m_TxdAckQueue );
   
   // Delete the Rxd-Pkt queue buffer
   DeleteQueueBuffer( &pMsgQueue->m_RxdPktQueue );
   
   // Delete the Sig-Pkt queue buffer
   DeleteQueueBuffer( &pMsgQueue->m_SigPktQueue );
   
   // Delete the Cmd-Ack queue buffer
   DeleteQueueBuffer( &pMsgQueue->m_CmdAckQueue );
   
   // Delete the Net-Mon queue buffer
   DeleteQueueBuffer( &pMsgQueue->m_NetMonQueue );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellSendTask( ShellSendTask * pSendTask )
{
   memset( pSendTask, 0X00, sizeof( ShellSendTask ) );
   
   // The task waiting for 
   pSendTask->m_iStatus = TXD_TASK_STATUS_REPORTTABLE;
   
   // The buffer size
   pSendTask->m_dwBufferSize = sizeof( pSendTask->m_pBuffer );
   
   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&pSendTask->m_qwInitTimer );   
   
   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&pSendTask->m_qwOpenTimer );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellSendTask( ShellSendTask * pSendTask )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellRecvTask( ShellRecvTask * pRecvTask )
{
   memset( pRecvTask, 0X00, sizeof( ShellRecvTask ) );
   
   // The buffer size
   pRecvTask->m_dwBufferSize = sizeof( pRecvTask->m_pBuffer );
   
   // At the header of this buffer
   pRecvTask->m_dwOffsetSize = 0X00;
   
   // Waiting for new job
   pRecvTask->m_iStatus = RXD_TASK_STATUS_JOB_WAITING;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellRecvTask( ShellRecvTask * pRecvTask )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void QueryModuleStatus( ShellTxDriver * pTxDriver )
{
   // Commit new task
   pTxDriver->m_iMainStatus = HAL_QUERY_FIFO_SET;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellTxDriver( ShellTxDriver * pTxDriver )
{
   // Reset the control block
   memset( pTxDriver, 0X00, sizeof( ShellTxDriver ) );
   
   // Initialize the message pointer
   pTxDriver->m_pMsg = ( NodeMsg * )pTxDriver->m_pBuffer;
   
   // Query the module status
   QueryModuleStatus( pTxDriver );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellTxDriver( ShellTxDriver * pTxDriver )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellRxDriver( ShellRxDriver * pRxDriver )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellRxDriver( ShellRxDriver * pRxDriver )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellTxEngine( ShellTxEngine * pTxEngine )
{   
   // Reset the control block
   memset( pTxEngine, 0X00, sizeof( ShellTxEngine ) );
   
   // Set the item value
   pTxEngine->m_iMainStatus = API_TX_STATUS_INIT;
   
   // Initialize the message pointer
   pTxEngine->m_pMsg = ( NodeMsg * )pTxEngine->m_pBuffer;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellTxEngine( ShellTxEngine * pTxEngine )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellRxEngine( ShellRxEngine * pRxEngine )
{   
   // Clear the state machine
   memset( pRxEngine, 0X00, sizeof( ShellRxEngine ) );
   
   // Set the item value
   pRxEngine->m_iMainStatus = API_RX_STATUS_IDLE;
   
   // Initialize the message pointer
   pRxEngine->m_pMsg = ( NodeMsg * )pRxEngine->m_pBuffer; 
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellRxEngine( ShellRxEngine * pRxEngine )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitStatusMonitor( StatusMonitor * pSMonitor )
{
   // Initialize the status monitor
   memset( pSMonitor, 0X00, sizeof( StatusMonitor ) );

   // Set the monitor update mask code
   pSMonitor->m_iUpdate |= ( MONITOR_DEVICE_MODE | MONITOR_STATUS_NODE ); 
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvStatusMonitor( StatusMonitor * pSMonitor )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitShellInstance( ShellInstance * pDevShell )
{
   // Initialize the IODevice
   InitShellIODevice( &pDevShell->m_IODevice );

   // Initialize the MsgQueue
   InitShellMsgQueue( &pDevShell->m_MsgQueue );

   // Initialize the SendTask
   InitShellSendTask( &pDevShell->m_SendTask );

   // Initialize the RecvTask
   InitShellRecvTask( &pDevShell->m_RecvTask );

   // Initialize the TxDriver
   InitShellTxDriver( &pDevShell->m_TxDriver );

   // Initialize the RxDriver
   InitShellRxDriver( &pDevShell->m_RxDriver );

   // Initialize the TxEngine
   InitShellTxEngine( &pDevShell->m_TxEngine );

   // Initialize the RxEngine
   InitShellRxEngine( &pDevShell->m_RxEngine );

   // Initialize the SMonitor
   InitStatusMonitor( &pDevShell->m_SMonitor );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void RemvShellInstance( ShellInstance * pDevShell )
{
   // Remove the IODevice
   RemvShellIODevice( &pDevShell->m_IODevice );
   
   // Remove the MsgQueue
   RemvShellMsgQueue( &pDevShell->m_MsgQueue );
   
   // Remove the SendTask
   RemvShellSendTask( &pDevShell->m_SendTask );
   
   // Remove the RecvTask
   RemvShellRecvTask( &pDevShell->m_RecvTask );
   
   // Remove the TxDriver
   RemvShellTxDriver( &pDevShell->m_TxDriver );
   
   // Remove the RxDriver
   RemvShellRxDriver( &pDevShell->m_RxDriver );
   
   // Remove the TxEngine
   RemvShellTxEngine( &pDevShell->m_TxEngine );
   
   // Remove the RxEngine
   RemvShellRxEngine( &pDevShell->m_RxEngine );

   // Remove the SMonitor
   RemvStatusMonitor( &pDevShell->m_SMonitor );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void MallocQueueBuffer( AtomicQueue16 * pQueue, unsigned short iSize )
{
   char * pXMemBuffer;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
      
   // Initialize the queue
   InitAtomicQueue16( pQueue, pXMemBuffer, iSize );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void DeleteQueueBuffer( AtomicQueue16 * pQueue )
{
   // Validate the pointer
   if ( !pQueue->m_pBuffer )
   {
      return;
   }

   // Free the memory
   free( pQueue->m_pBuffer );

   // Invalidate the pointer
   pQueue->m_pBuffer = NULL;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
ShellInstance * CreateUniformShell()
{
   ShellInstance * pDevShell;
   
   // Alocate the uniform shell
   pDevShell = ( ShellInstance * )malloc( sizeof( ShellInstance ) );

   // Clear the instance
   memset( pDevShell, 0X00, sizeof( pDevShell ) );
   
   // Validate the shell pointer
   if ( !pDevShell )
   {
      return NULL;
   }
   
   // Initialize the uniform shell
   InitShellInstance( pDevShell );
   return pDevShell;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void DeleteUniformShell( ShellInstance * pDevShell )
{
   // Validate the pointer
   if ( !pDevShell )
   {
      return;
   }

   // Remove the shell instance
   RemvShellInstance( pDevShell );

   // Delete the memory buffer
   free( pDevShell );
}
