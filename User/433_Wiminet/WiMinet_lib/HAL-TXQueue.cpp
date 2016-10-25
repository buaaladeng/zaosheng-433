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
// File:    hal-txqueue.c
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class hal-txqueue
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

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
#include "API-IOQueue.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WiMinet.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
HAL_TxQueue_StateMachine  HAL_TxQueue;


// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char QueryShellErrors( char iShell, unsigned long * pErrors )
{
	// Validate the error counter
	if ( pErrors )
	{
		*pErrors = HAL_TxQueue.m_dwShellError;
	}
	return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char ResetShellErrors( char iShell )
{
	HAL_TxQueue.m_dwShellError = 0X00;
	return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_SetupTxQueueStateMachine( char iShell )
{
   // Reset the control block
   memset( &HAL_TxQueue, 0X00, sizeof( HAL_TxQueue ) );

   // Enable the descriptor
   EnableDescriptor( iShell, 0X01 );

   // Initialize the message pointer
   HAL_TxQueue.m_pMsg = ( NodeMsg * )HAL_TxQueue.m_pBuffer;

   // Set the item value
   HAL_QueryPacket();   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_CloseTxQueueStateMachine( char iShell )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_IsTxAckMsgQueueWritable()
{
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_WriteRxDataMsgQueue( NodeMsg * pMsg )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_WriteSignalMsgQueue( NodeMsg * pMsg )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_WriteCmdAckMsgQueue( NodeMsg * pMsg )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_CheckTxPrepareStatus()
{
   // Finished successfully
   if ( HAL_TxQueue.m_iMainStatus == HAL_READY_TASK_NEW )
   {
      return 0X01;
   }

   // Finished with error
   if ( HAL_TxQueue.m_iMainStatus == HAL_READY_TASK_ERR )
   {
      return 0X01;
   }

   // Not prepared for not data packets
   return 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_CheckTxSuccessStatus()
{
   return ( HAL_TxQueue.m_iMainStatus == HAL_READY_TASK_NEW );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_CheckRequestTimer( unsigned long dwTimerX )
{
   // Read current timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&HAL_TxQueue.m_qwThisTimer );

   // Change to timer span
   HAL_TxQueue.m_qwTimerSpan.QuadPart = HAL_TxQueue.m_qwThisTimer.QuadPart;
   
   // Get the timer offset
   HAL_TxQueue.m_qwTimerSpan.QuadPart -= HAL_TxQueue.m_qwOpenTimer.QuadPart;

   // 100 nano-second to mili-second
   HAL_TxQueue.m_qwTimerSpan.QuadPart /= 10000;
   
   // Check if the timer is over
   if ( HAL_TxQueue.m_qwTimerSpan.QuadPart >= dwTimerX )
   {
      return 0X01;
   }   
   return 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_QueryHostRxTxStatus()
{
   NodeMsg msg;
   
   // The request command
   memset( &msg, 0X00, sizeof( NodeMsg ) );
   msg.m_iOpCode = CMDMSG_GET_FIFO_STATISTICS;
   
   // In HAL-TxQueue mode
   msg.m_iValueD = 0X00;

   // The extended parameter size
   msg.m_iAmount = 0X00;

   // Write shell message
   WriteShell( &msg );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_GetTxPacketSize()
{
   unsigned char iSize;

   // The range protection
   if ( HAL_TxQueue.m_iTxFreeSize >= 0XF6 )
   {
      iSize = 0XF6;
   }
   else
   {
      // The default packet size
      iSize = ( unsigned char )HAL_TxQueue.m_iTxFreeSize;
   }

   // Regulate to 16 bytes
   //iSize = ( iSize / 16 ) * 16;
   iSize = ( iSize / 24 ) * 24;   
   return iSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short HAL_GetTxQueueFreeSize()
{
   return HAL_TxQueue.m_iTxFreeSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short HAL_GetTxQueueUsedSize()
{
   return HAL_TxQueue.m_iTxUsedSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short HAL_GetTxQueueFIFOSize()
{
   return HAL_TxQueue.m_iTxFIFOSize;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_ValidateTxQueueStatus()
{
   QueueStatus16 * pStatus;
   
   // The communication port FIFO status
   pStatus = ( QueueStatus16 * )( HAL_TxQueue.m_pMsg->m_pBuffer + 0X02 );
   
   // The Tx-Queue Size
   HAL_TxQueue.m_iTxFreeSize = ntohs( pStatus->m_iFree );

   // The Tx-Queue Size
   HAL_TxQueue.m_iTxUsedSize = ntohs( pStatus->m_iUsed );

   // The Tx-Queue Size
   HAL_TxQueue.m_iTxFIFOSize = ntohs( pStatus->m_iSize );
   
   //printf( "\t-->Tick=%d:Code=0X%02X,Used=%d\r\n", GetTickCount(), HAL_TxQueue.m_pMsg->m_iOpCode, HAL_TxQueue.m_iTxUsedSize );

   // The Tx memory is empty
   if ( HAL_TxQueue.m_iTxUsedSize )
   {
      HAL_TxQueue.m_iTxMemEmpty = 0X00;
   }
   else
   {
      HAL_TxQueue.m_iTxMemEmpty = 0X01;
   }
   
   // The queue free space
   if ( !HAL_TxQueue.m_iTxFreeSize )
   {
      return 0X00;
   }

/*
   // Update the Rx-Node
   HAL_TxQueue.m_iThisNodeID = HAL_TxQueue.m_pMsg->m_pBuffer[0X01]; 
   
   // Check the node status
   HAL_TxQueue.m_iNodeStatus = HAL_TxQueue.m_pMsg->m_pBuffer[0X02];
   
   // Node suspend error?
   if ( HAL_TxQueue.m_iNodeStatus & 0X04 )
   {
      HAL_TxQueue.m_iMainStatus = HAL_READY_TASK_ERR;
   }
   else
   {
      HAL_TxQueue.m_iMainStatus = HAL_READY_TASK_NEW;
   }
*/

   HAL_TxQueue.m_iMainStatus = HAL_READY_TASK_NEW;
   
   // Switch to new state machine
   return 0X01;
}


// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_GetRxNodeStatus()
{
   return HAL_TxQueue.m_iNodeStatus;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char HAL_ValidateTxMemEmpty()
{
   return HAL_TxQueue.m_iTxMemEmpty;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_InitMsgHead( short iNode, char iAttr, char iQoS )
{
   // Clear the message
   memset( HAL_TxQueue.m_pMsg, 0X00, sizeof( HAL_TxQueue.m_pMsg ) );

   // The MSB of the Message Node
   HAL_TxQueue.m_pMsg->m_iValueA = ( iNode >> 0X08 );   
   
   // The LSB of the Message Node
   HAL_TxQueue.m_pMsg->m_iValueB = ( iNode & 0XFF );
      
   // The message Attribute
   HAL_TxQueue.m_pMsg->m_iValueC = iAttr;

   // The QoS information
   if ( iQoS )
   {
      HAL_TxQueue.m_pMsg->m_iValueD |= 0X80;
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_WritePacket( char * pBuffer, char iSize )
{
   // The upper limit of this buffer size
   if ( ( unsigned char )iSize > sizeof( HAL_TxQueue.m_pMsg->m_pBuffer ) )
   {
      iSize = sizeof( HAL_TxQueue.m_pMsg->m_pBuffer );
   }
      
   // The OpCode
   HAL_TxQueue.m_pMsg->m_iOpCode = CMDMSG_TX_PACKET;   
   
   // The message size
   HAL_TxQueue.m_pMsg->m_iAmount = iSize;
   
   // The memssage buffer
   memcpy( HAL_TxQueue.m_pMsg->m_pBuffer, pBuffer, ( unsigned char )iSize );
   
   // Write out this packet
   WriteShell( HAL_TxQueue.m_pMsg );
   
   // Just for debugging
   //printf( "\t++>Tick=%d:Code=0X%02X,Size=%d\r\n", GetTickCount(), CMDMSG_TX_PACKET, HAL_TxQueue.m_pMsg->m_iAmount ); 
         
   // Commit new task
   HAL_TxQueue.m_iMainStatus = HAL_WRITE_TASK_REQ;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_QueryPacket()
{
   // Commit new task
   HAL_TxQueue.m_iMainStatus = HAL_QUERY_FIFO_SET;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_MainTxStatusHandler()
{
   unsigned char iRetVal;
   
   //printf( "K=%d,N=0X%02X,S=0X%02X ", HAL_TxQueue.m_iMainStatus, HAL_TxQueue.m_iThisNodeID, HAL_TxQueue.m_iNodeStatus );

   // The Tx-Ack state machine  
   switch ( HAL_TxQueue.m_iMainStatus )
   {
   case HAL_WRITE_TASK_REQ:
      {
         // Update the Tx-Timer
         GetSystemTimeAsFileTime( ( LPFILETIME )&HAL_TxQueue.m_qwOpenTimer );

         // Clear the Tx-Queue Size
         HAL_TxQueue.m_iTxFreeSize = 0X00;

         // Switch to new state machine
         HAL_TxQueue.m_iMainStatus = HAL_WRITE_WAIT_ACK;        
      }
      break;

   case HAL_WRITE_WAIT_ACK:
      {
         // Check if the timer is over
         if ( HAL_CheckRequestTimer( 2500 ) )
         {
            HAL_TxQueue.m_iMainStatus = HAL_READY_TASK_ERR;
				
				// Update the error counter
				HAL_TxQueue.m_dwShellError++;							
            break;            
         }

         // Read valide packet from the queue
         iRetVal = API_GetTxdAckQueue( HAL_TxQueue.m_pMsg );
         if ( !iRetVal )
         {
            break;
         }
         
         // Only handle the 0X9D command
         if ( HAL_TxQueue.m_pMsg->m_iOpCode != 0X9D )
         {
            break;
         }

         // Validate the Tx-Queue status
         iRetVal = HAL_ValidateTxQueueStatus();
         if ( iRetVal )
         {
            break;
         }

         // Switch to new state machine
         HAL_TxQueue.m_iMainStatus = HAL_QUERY_FIFO_SET;            
      }
      break;

   case HAL_QUERY_FIFO_SET:
      {
         // Clear the Tx-Queue Size
         HAL_TxQueue.m_iTxFreeSize = 0X00;
         
         // Update the Tx-Timer
         GetSystemTimeAsFileTime( ( LPFILETIME )&HAL_TxQueue.m_qwOpenTimer ); 
         
         // Switch to new state machine
         HAL_TxQueue.m_iMainStatus = HAL_QUERY_FIFO_REQ;
      }
      break;
            
   case HAL_QUERY_FIFO_REQ:
      {
         // Check the query timer span
         if ( !HAL_CheckRequestTimer( 20 ) )
         {
            return;
         }

         // Query the host RTx status
         HAL_QueryHostRxTxStatus();
         
         // Update the Tx-Timer
         GetSystemTimeAsFileTime( ( LPFILETIME )&HAL_TxQueue.m_qwOpenTimer );
         
         // Switch to new state machine
         HAL_TxQueue.m_iMainStatus = HAL_QUERY_WAIT_ACK;         
      }
      break;

   case HAL_QUERY_WAIT_ACK:
      {
         // Check if the timer is over
         if ( HAL_CheckRequestTimer( 2500 ) )
         {
            HAL_TxQueue.m_iMainStatus = HAL_READY_TASK_ERR;

				// Update the error counter
				HAL_TxQueue.m_dwShellError++;								
            break;            
         }
         
         // Read valide packet from the queue
         iRetVal = API_GetTxdAckQueue( HAL_TxQueue.m_pMsg );
         if ( !iRetVal )
         {
            break;
         }
         
         // Only handle the 0X9C command
         if ( HAL_TxQueue.m_pMsg->m_iOpCode != 0X9C )
         {
            break;
         }
         
         // Validate the Tx-Queue status
         iRetVal = HAL_ValidateTxQueueStatus();
         if ( iRetVal )
         {
            break;
         } 
         
         // Switch to new state machine
         HAL_TxQueue.m_iMainStatus = HAL_QUERY_FIFO_SET; 
      }
      break;

   case HAL_READY_TASK_NEW:
      {
         // Ready for new write task
      }
      break;

   case HAL_READY_TASK_ERR:
      {
         // There are communication errors
      }
      break;

   default:
      {
      }
      break;
   } 
}
