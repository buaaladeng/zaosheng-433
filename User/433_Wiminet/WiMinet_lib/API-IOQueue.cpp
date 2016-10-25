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
// File:    api-ioqueue.cpp
// Author:  Mickle.ding
// Created: 11/22/2011
//
// Description:  Define the class api-ioqueue
// -----------------------------------------------------------------------------

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
#include "API-Message.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Queue16.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WiMinet.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
API_IOQueue_Center API_IOQueue;

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupStringQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
   
   // The buffer pointer
   API_IOQueue.m_pStringBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOQueue.m_StringQueue, pXMemBuffer, iSize );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupTxdAckQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );

   // The buffer pointer
   API_IOQueue.m_pTxdAckBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOQueue.m_TxdAckQueue, pXMemBuffer, iSize );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupRxdPktQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
   
   // The buffer pointer
   API_IOQueue.m_pRxdPktBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOQueue.m_RxdPktQueue, pXMemBuffer, iSize );  
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupSigPktQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
   
   // The buffer pointer
   API_IOQueue.m_pSigPktBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOQueue.m_SigPktQueue, pXMemBuffer, iSize );  
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupCmdAckQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
   
   // The buffer pointer
   API_IOQueue.m_pCmdAckBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOQueue.m_CmdAckQueue, pXMemBuffer, iSize );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupNetMonQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
   
   // The buffer pointer
   API_IOQueue.m_pNetMonBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOQueue.m_NetMonQueue, pXMemBuffer, iSize );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseStringQueue()
{
   // Release the queue memory
   if ( !API_IOQueue.m_pStringBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOQueue.m_pStringBuff );
   
   // Invalidate the memory pointer
   API_IOQueue.m_pStringBuff = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseTxdAckQueue()
{
   // Release the queue memory
   if ( !API_IOQueue.m_pTxdAckBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOQueue.m_pTxdAckBuff );

   // Invalidate the memory pointer
   API_IOQueue.m_pTxdAckBuff = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseRxdPktQueue()
{
   // Release the queue memory
   if ( !API_IOQueue.m_pRxdPktBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOQueue.m_pRxdPktBuff );
   
   // Invalidate the memory pointer
   API_IOQueue.m_pRxdPktBuff = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseSigPktQueue()
{
   // Release the queue memory
   if ( !API_IOQueue.m_pSigPktBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOQueue.m_pSigPktBuff );
   
   // Invalidate the memory pointer
   API_IOQueue.m_pSigPktBuff = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseCmdAckQueue()
{
   // Release the queue memory
   if ( !API_IOQueue.m_pCmdAckBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOQueue.m_pCmdAckBuff );
   
   // Invalidate the memory pointer
   API_IOQueue.m_pCmdAckBuff = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseNetMonQueue()
{
   // Release the queue memory
   if ( !API_IOQueue.m_pNetMonBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOQueue.m_pNetMonBuff );
   
   // Invalidate the memory pointer
   API_IOQueue.m_pNetMonBuff = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutAtomicQueue( AtomicQueue16 * pQueue, NodeMsg * pMsg )
{
   unsigned char iSize;
   
   // The message header size
   iSize = pMsg->m_iAmount + 0X09;
      
   // Write data into this queue
   WriteAtomicQueue16( pQueue, ( char * )pMsg, iSize );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_SeeAtomicQueue( AtomicQueue16 * pQueue, NodeMsg * pMsg )
{
   unsigned char iCounter;
   QueueStatus16 QStatus;
   
   // Get the queue status
   PeekQueue16Status( pQueue, &QStatus );
   
   // No data inside this queue
   if ( !QStatus.m_iUsed )
   {
      return 0X00;
   }
   
   // The packet header invalid
   if ( QStatus.m_iHead != 0XAA )
   {
      ReadAtomicQueue16( pQueue, ( char * )pMsg, 0X01 );
      return 0X01;
   }
   
   // The packet is not complete
   if ( QStatus.m_iUsed < 0X09 )
   {
      return 0X00;
   }
   
   // Snap this UART Rx queue
   SnapAtomicQueue16( pQueue, ( char * )pMsg, 0X09 );
   
   // Validate the message size
   if ( pMsg->m_iAmount >= 0XF6 )
   {
      ReadAtomicQueue16( pQueue, ( char * )pMsg, 0X01 );
      return 0X01;      
   }
   
   // The Rx queue is not ready
   iCounter = pMsg->m_iAmount + 0X09;
   
   // Validate the Rx queue counter
   if ( iCounter > QStatus.m_iUsed )
   {
      return 0X00;
   }
   
   // Snap this packet
   SnapAtomicQueue16( pQueue, ( char * )pMsg, iCounter );
   if ( !IsValidNodeMsg( pMsg ) )
   {
      ReadAtomicQueue16( pQueue, ( char * )pMsg, 0X01 );
      return 0X01;      
   }
   return iCounter;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetAtomicQueue( AtomicQueue16 * pQueue, NodeMsg * pMsg )
{
   unsigned char iCounter;

   // Check the atomic queue
   iCounter = API_SeeAtomicQueue( pQueue, pMsg );
   if ( iCounter >= 0X09 )
   {
      ReadAtomicQueue16( pQueue, ( char * )pMsg, iCounter );
   }

   // Read the header into this queue
   return iCounter;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutStringQueue( char * pBuffer, char iSize )
{
   // Insert the char into the string queue
   WriteAtomicQueue16( &API_IOQueue.m_StringQueue, pBuffer, iSize );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetStringQueue( char * pBuffer, char iSize )
{
   return ReadAtomicQueue16( &API_IOQueue.m_StringQueue, pBuffer, iSize );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutTxdAckQueue( NodeMsg * pMsg )
{
   API_PutAtomicQueue( &API_IOQueue.m_TxdAckQueue, pMsg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_SeeTxdAckQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_SeeAtomicQueue( &API_IOQueue.m_TxdAckQueue, pMsg );
   
   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetTxdAckQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_GetAtomicQueue( &API_IOQueue.m_TxdAckQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutRxdPktQueue( NodeMsg * pMsg )
{
   API_PutAtomicQueue( &API_IOQueue.m_RxdPktQueue, pMsg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_SeeRxdPktQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;
   
   // Get the packet size
   iRetVal = API_SeeAtomicQueue( &API_IOQueue.m_RxdPktQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetRxdPktQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_GetAtomicQueue( &API_IOQueue.m_RxdPktQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutSigPktQueue( NodeMsg * pMsg )
{
   API_PutAtomicQueue( &API_IOQueue.m_SigPktQueue, pMsg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_SeeSigPktQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_SeeAtomicQueue( &API_IOQueue.m_SigPktQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetSigPktQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_GetAtomicQueue( &API_IOQueue.m_SigPktQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutCmdAckQueue( NodeMsg * pMsg )
{
   API_PutAtomicQueue( &API_IOQueue.m_CmdAckQueue, pMsg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_SeeCmdAckQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_SeeAtomicQueue( &API_IOQueue.m_CmdAckQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetCmdAckQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;
   
   // Get the packet size
   iRetVal = API_GetAtomicQueue( &API_IOQueue.m_CmdAckQueue, pMsg );
   
   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_PutNetMonQueue( NodeMsg * pMsg )
{
   API_PutAtomicQueue( &API_IOQueue.m_NetMonQueue, pMsg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_SeeNetMonQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;

   // Get the packet size
   iRetVal = API_SeeAtomicQueue( &API_IOQueue.m_NetMonQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_GetNetMonQueue( NodeMsg * pMsg )
{
   unsigned char iRetVal;
   
   // Get the packet size
   iRetVal = API_GetAtomicQueue( &API_IOQueue.m_NetMonQueue, pMsg );

   // Validate the packet size
   if ( iRetVal < 0X09 )
   {
      iRetVal = 0X00;
   }
   return iRetVal;      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckVoid_TxdAckQueue()
{
   QueueStatus16 status;

   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_TxdAckQueue, &status );
   return ( status.m_iUsed ? 0X00 : 0X01 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckFull_TxdAckQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_TxdAckQueue, &status );
   return ( status.m_iFree ? 0X00 : 0X01 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckVoid_RxdPktQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_RxdPktQueue, &status );
   return ( status.m_iUsed ? 0X00 : 0X01 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckFull_RxdPktQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_RxdPktQueue, &status );
   return ( status.m_iFree ? 0X00 : 0X01 );      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckVoid_SigPktQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_SigPktQueue, &status );
   return ( status.m_iUsed ? 0X00 : 0X01 );      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckFull_SigPktQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_SigPktQueue, &status );
   return ( status.m_iFree ? 0X00 : 0X01 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckVoid_CmdAckQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_CmdAckQueue, &status );
   return ( status.m_iUsed ? 0X00 : 0X01 );         
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckFull_CmdAckQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_CmdAckQueue, &status );
   return ( status.m_iFree ? 0X00 : 0X01 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckVoid_NetMonQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_NetMonQueue, &status );
   return ( status.m_iUsed ? 0X00 : 0X01 );         
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_CheckFull_NetMonQueue()
{
   QueueStatus16 status;
   
   // Peek the queue status
   PeekQueue16Status( &API_IOQueue.m_NetMonQueue, &status );
   return ( status.m_iFree ? 0X00 : 0X01 );   
}