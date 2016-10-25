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
// File:    api-rxqueue.c
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class api-rxqueue
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
#include <Windows.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-CRCheck.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOShell.h"

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
#include "API-WiMinet.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
RxTaskInfo  RxTask;

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
API_RxQueue_StateMachine API_RxQueue;

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupRxTask()
{
   memset( &RxTask, 0X00, sizeof( RxTask ) );
   
   // The buffer size
   RxTask.m_dwBufferSize = sizeof( RxTask.m_pBuffer );

   // At the header of this buffer
   RxTask.m_dwOffsetSize = 0X00;

   // Waiting for new job
   RxTask.m_iStatus = RXD_TASK_STATUS_JOB_WAITING;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseRxTask()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupRxQueueStateMachine()
{
   // Initialize the HAL-RxStateMachine
   HAL_SetupRxQueueStateMachine(); 
   
   // Clear the state machine
   memset( &API_RxQueue, 0X00, sizeof( API_RxQueue ) );
   
   // Set the item value
   API_RxQueue.m_iMainStatus = API_RX_STATUS_IDLE;
   
   // Initialize the message pointer
   API_RxQueue.m_pMsg = ( NodeMsg * )API_RxQueue.m_pBuffer; 
   
   // Setup the Rx-Task
   API_SetupRxTask();
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseRxQueueStateMachine()
{
   // Close the HAL-RxStateMachine
   HAL_CloseRxQueueStateMachine(); 
   
   // Close the Rx-Task
   API_CloseRxTask();   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxStateHandler_Init()
{
   // Clear the task status
   API_SetupRxTask();  
   
   // Switch to next status
   API_RxQueue.m_iMainStatus = API_RX_STATUS_SINF; 
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxStateHandler_SInf()
{
   SessionHeader * pHeader;

   // Make sure the queue is ready
   if ( !API_SeeRxdPktQueue( API_RxQueue.m_pMsg ) )
   {
      return;
   }
            
   // Check the packet attribute
   if ( API_RxQueue.m_pMsg->m_iValueC == IOSHELL_SINF_PACKET_ATTR )
   {   
      // Get the packet from this queue
      API_GetRxdPktQueue( API_RxQueue.m_pMsg );
      
      // The session descriptor
      pHeader = ( SessionHeader * )API_RxQueue.m_pMsg->m_pBuffer;

      // Debug information
      //printf( "[SIN:Offset=%d,AllSize=%d]\r\n", RxTask.m_dwOffsetSize, pHeader->m_dwAllSize );
      
      // The total stream size
      RxTask.m_dwPacketSize = pHeader->m_dwTotalSize;

      // The task receive percent
      RxTask.m_iTaskPercent = 0X00;

      // Clear the offset value
      RxTask.m_dwOffsetSize = 0X00;

      // The stream CRC32 value
      RxTask.m_dwCRC32 = pHeader->m_dwCRC32Code;

      // The MSB of the TxAddr
      RxTask.m_iNode = API_RxQueue.m_pMsg->m_iValueA;

      // X8 to X16 conversion
      RxTask.m_iNode <<= 0X08;

      // The LSB of the TxAddr
      RxTask.m_iNode += API_RxQueue.m_pMsg->m_iValueB;

      // The SINF status
      RxTask.m_iSInf = 0X01;

      // Record last check timer
      GetSystemTimeAsFileTime( ( LPFILETIME )&RxTask.m_qwOpenTimer );       
   }
	else
	{
		// The packet attribute
		RxTask.m_iAttribute = API_RxQueue.m_pMsg->m_iValueC;

		// Common data packet
		if ( ( RxTask.m_iAttribute >= IOSHELL_DATA_PACKET_BODY ) && ( RxTask.m_iAttribute <= IOSHELL_DATA_PACKET_UNIT ) )
		{
			RxTask.m_iAttribute = IOSHELL_DATA_PACKET_ATTR;				
		}
		
		// Common data packet
		if ( ( RxTask.m_iAttribute >= IOSHELL_VOIP_PACKET_BODY ) && ( RxTask.m_iAttribute <= IOSHELL_VOIP_PACKET_UNIT ) )
		{
			RxTask.m_iAttribute = IOSHELL_VOIP_PACKET_ATTR;				
		}
	}

   // Record last check timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&RxTask.m_qwRecvTimer ); 
   
   // Update the task status
   RxTask.m_iStatus = RXD_TASK_STATUS_JOB_ONMARCH;

   // Switch to next status
   API_RxQueue.m_iMainStatus = API_RX_STATUS_RECV;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_RxStateHandler_InputPkt()
{
   char * pBuffer;
   unsigned char iSize;
   
   // This packet size
   iSize = API_RxQueue.m_pMsg->m_iAmount;

   // Buffer overrun protection
   if ( RxTask.m_iSInf && ( RxTask.m_dwOffsetSize + iSize > RxTask.m_dwPacketSize ) )
   {
      return 0X01;
   }

   // Buffer overrun protection
   if ( RxTask.m_dwOffsetSize + iSize >= sizeof( RxTask.m_pBuffer ) )
   {
      return 0X01;
   }

   // The packet payload address
   pBuffer = RxTask.m_pBuffer + RxTask.m_dwOffsetSize;

   // Move packet to the packet load buffer
   memcpy( pBuffer, API_RxQueue.m_pMsg->m_pBuffer, iSize );

   // Increment the packet size
   RxTask.m_dwOffsetSize += iSize;
   return 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_RxStateHandler_CheckCRC()
{
   unsigned char * pBuffer;
   unsigned long dwCRC32;
   
   // Check the CRC32 value
   pBuffer = ( unsigned char * )RxTask.m_pBuffer;
   dwCRC32 = QuickCRC32( pBuffer, ( unsigned short )RxTask.m_dwOffsetSize );
   return ( dwCRC32 == RxTask.m_dwCRC32 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxStateHandler_Recv()
{
   unsigned char iMode;
   unsigned char iEndRx;
   
   // Make sure the queue is ready
   if ( !API_SeeRxdPktQueue( API_RxQueue.m_pMsg ) )
   {
      return;
   }

   // Record last check timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&RxTask.m_qwRecvTimer );    
   
   // Check the packet attribute
   if ( API_RxQueue.m_pMsg->m_iValueC == IOSHELL_SINF_PACKET_ATTR )
   {
      // Switch to the next status
      API_RxQueue.m_iMainStatus = API_RX_STATUS_INIT;      
      return;
   }
      
   // Check if packet arrives
   if ( !API_GetRxdPktQueue( API_RxQueue.m_pMsg ) )
   {
      return;
   }

   //printf( "[Recv=%d]\r\n", API_RxQueue.m_pMsg->m_iAmount );
   

   // Update the stream information
   if ( !RxTask.m_iSInf )
   {
      // The MSB of the TxAddr
      RxTask.m_iNode = API_RxQueue.m_pMsg->m_iValueA;
      
      // X8 to X16 conversion
      RxTask.m_iNode <<= 0X08;
      
      // The LSB of the TxAddr
      RxTask.m_iNode += API_RxQueue.m_pMsg->m_iValueB;      
   }

   // Get the packet mode
   iMode = API_RxQueue.m_pMsg->m_iValueC;

   // Header indentifier
   if ( iMode & COMM_ATTRIBUTE_HEAD )
   {
      // Start of this packet
      RxTask.m_iTaskPercent = 0X00;

      // The head information
      RxTask.m_iHead = 0X01;
      
      // Reset the receive offset
      RxTask.m_dwOffsetSize = 0X00;
   }

   // Move the packet into the payload buffer
   iEndRx = API_RxStateHandler_InputPkt();

   // Check the end status of this stream
   if ( ( iMode & COMM_ATTRIBUTE_TAIL ) || iEndRx )
   {
      // End of this packet      
      RxTask.m_iTaskPercent = 100;

      // The head information
      RxTask.m_iTail = 0X01;
      
      // Switch to next status
      API_RxQueue.m_iMainStatus = API_RX_STATUS_REPT; 
   }

   // The broadcast command
   if ( ( iMode == IOSHELL_WSMS_PACKET_ATTR ) || ( iMode == IOSHELL_SGPS_PACKET_ATTR ) )
   {
      // The head information
      RxTask.m_iHead = 0X01;
      
      // The head information
      RxTask.m_iTail = 0X01;
      
      // Reset the receive offset
      RxTask.m_dwOffsetSize = API_RxQueue.m_pMsg->m_iAmount;
      
      // The total packet size
      RxTask.m_dwPacketSize = API_RxQueue.m_pMsg->m_iAmount;
            
      // Switch to next status
      API_RxQueue.m_iMainStatus = API_RX_STATUS_REPT; 
   }
   
   // Validate the stream size
   if ( !RxTask.m_dwPacketSize )
   {
      RxTask.m_iStatus = 0X00;
   }
   
   // Get the receiving progress
   if ( RxTask.m_dwPacketSize )
   {
      RxTask.m_iTaskPercent = 100 * RxTask.m_dwOffsetSize / RxTask.m_dwPacketSize;   
   }

   // Update the task status
   RxTask.m_iStatus = ( unsigned char )RxTask.m_iTaskPercent;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxStateHandler_Rept()
{
   // Switch to next status
   API_RxQueue.m_iMainStatus = API_RX_STATUS_OVER;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxStateHandler_Over()
{
   char iRetVal;

   // Check the SINF status
   if ( !RxTask.m_iSInf )
   {
      RxTask.m_iStatus = RXD_TASK_STATUS_END_NOCRC32;
      return;
   }
   
   // End of reception
   iRetVal = API_RxStateHandler_CheckCRC();
   if ( iRetVal )
   {
      RxTask.m_iStatus = RXD_TASK_STATUS_END_SUCCESS;
   }
   else
   {
      RxTask.m_iStatus = RXD_TASK_STATUS_END_FAILURE;
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxStateHandler_Idle()
{
   // Validate the Rx-Task status
   if ( RxTask.m_iStatus != RXD_TASK_STATUS_JOB_WAITING )
   {
      return;
   }

/*
   // Data ready for user read out
   if ( RxTask.m_iStatus == RXD_TASK_STATUS_END_SUCCESS )
   {
      return;
   }

   // Data ready for user read out
   if ( RxTask.m_iStatus == RXD_TASK_STATUS_END_FAILURE )
   {
      return;
   }

   // Data ready for user read out
   if ( RxTask.m_iStatus == RXD_TASK_STATUS_END_NOCRC32 )
   {
      return;
   }
   
   // Last packet for user read
   if ( RxTask.m_dwOffsetSize )
   {
      return;
   }
*/
   

   // Check if the RxPktQueue is void
   if ( !API_SeeRxdPktQueue( API_RxQueue.m_pMsg ) )
   {
      return;
   }

   // Switch to the next status
   API_RxQueue.m_iMainStatus = API_RX_STATUS_INIT;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_RxQueueHandler()
{
   switch ( API_RxQueue.m_iMainStatus )
   {
   case API_RX_STATUS_INIT:
      {
         API_RxStateHandler_Init();
      }
      break;

   case API_RX_STATUS_SINF:
      {
         API_RxStateHandler_SInf();
      }
      break;

   case API_RX_STATUS_RECV:
      {
         API_RxStateHandler_Recv();
      }
      break;

   case API_RX_STATUS_REPT:
      {
         API_RxStateHandler_Rept();
      }
      break;

   case API_RX_STATUS_OVER:
      {
         API_RxStateHandler_Over();
      }
      break;
      
   case API_RX_STATUS_IDLE:
      {
         API_RxStateHandler_Idle();         
      }
      break;

   default:
      {
      }
      break;
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetRxMessageSize( char iShell, unsigned long * pSize )
{
   char iRetVal;
   char iStatus;

   // Check the Rx-Task status
   iRetVal = QueryRxMsgStatus( iShell, &iStatus );

   // Check if reportable
   iStatus &= RXD_TASK_STATUS_REPORTTABLE;

   // Check the status
   if ( !iStatus )
   {
      return 0X00;
   }

   // Validate the pointer
   if ( pSize )
   {
      *pSize = RxTask.m_dwOffsetSize;
   }
   
   // This task status
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetRxMessageAttr( char iShell, char * pAttr )
{
	// The packet attribute
	if ( pAttr )
	{
		*pAttr = RxTask.m_iAttribute;
	}
	return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetRxMessageNode( char iShell, short * pNode )
{
   // Validate the pointer
   if ( pNode )
   {
      *pNode = RxTask.m_iNode;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char QueryRxMsgStatus( char iShell, char * pStatus )
{
   // Validate the pointer
   if ( pStatus )
   {
      *pStatus = RxTask.m_iStatus;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char ReadInputMessage( char iShell, char * pBuffer, long dwSize )
{
	// Adjust the packet size
	if ( dwSize > RxTask.m_dwOffsetSize )
	{
		dwSize = RxTask.m_dwOffsetSize;
	}

   // Copy the buffer contents
	if ( pBuffer )
	{
		memcpy( pBuffer, RxTask.m_pBuffer, dwSize );
	}
   	
   // Switch for next status
   API_RxQueue.m_iMainStatus = API_RX_STATUS_IDLE;
	
   // Clear the task status
   API_SetupRxTask();
   return 0X01;	
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetRxMessageTime( char iShell, unsigned long * pTime )
{
   return 0X01;
}
