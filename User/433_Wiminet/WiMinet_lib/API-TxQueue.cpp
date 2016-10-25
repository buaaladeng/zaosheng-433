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
// File:    api-txqueue.c
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class api-txqueue
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Windows.h>

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
#include "HAL-TxQueue.h"

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
#include "HAL-Command.h"


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
API_TxQueue_StateMachine API_TxQueue;

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
TxTaskInfo TxTask;

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char EnableDescriptor( char iShell, char iEnable )
{
   // Enable or disable the descriptor
   API_TxQueue.m_iDescriptor = ( iEnable ? 0X01 : 0X00 );
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupTxTask()
{
   memset( &TxTask, 0X00, sizeof( TxTask ) );

   // The task waiting for 
   TxTask.m_iStatus = TXD_TASK_STATUS_JOB_WAITING;

   // The buffer size
   TxTask.m_dwBufferSize = sizeof( TxTask.m_pBuffer );

   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwInitTimer );   

   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwOpenTimer );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseTxTask()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupTxQueueStateMachine()
{   
   // Initialize the HAL-TxStateMachine
   HAL_SetupTxQueueStateMachine( 0X00 );
   
   // Reset the control block
   memset( &API_TxQueue, 0X00, sizeof( API_TxQueue ) );
     
   // Set the item value
   API_TxQueue.m_iMainStatus = API_TX_STATUS_INIT;
      
   // Initialize the message pointer
   API_TxQueue.m_pMsg = ( NodeMsg * )API_TxQueue.m_pBuffer;

   // Initialize the network monitor
   InitNetworkMornitor();

   // Initialize the Tx-Task
   API_SetupTxTask();
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseTxQueueStateMachine()
{   
   // Close the HAL-TxStateMachine
   HAL_CloseTxQueueStateMachine( 0X00 );
   
   // Close the Tx-Task
   API_CloseTxTask();
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DWORD CommitTransmitTask( short iNode, char * pBuffer, DWORD dwSize, char iQoS )
{
   DWORD dwTaskID;

	// Update the task status
	TxTask.m_iStatus = TXD_TASK_STATUS_JOB_ONMARCH;

   // The CRC32 value
   TxTask.m_dwCRC32 = QuickCRC32( ( unsigned char * )pBuffer, ( unsigned short )dwSize );

   // The node id
   TxTask.m_iNode = iNode;

   // The QoS status
   TxTask.m_iQoS = iQoS;

   // Backup the data
   memcpy( TxTask.m_pBuffer, pBuffer, dwSize );
   
   // The Tx out pointer offset
   TxTask.m_dwOffsetSize = 0X00;
   
   // The data size
   TxTask.m_dwPacketSize = dwSize; 

   // New task routine begins
   if ( API_TxQueue.m_iMainStatus == API_TX_STATUS_OVER )
   {
      // Switch to idle status
      API_TxQueue.m_iMainStatus = API_TX_STATUS_IDLE;         		
   }
      
   // The TaskID
   dwTaskID = 0X01;
   return dwTaskID;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Init()
{  
   //unsigned char iStatus;
   
   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwThisTimer );
   
   // Get the timer offset
   TxTask.m_qwThisTimer.QuadPart -= TxTask.m_qwInitTimer.QuadPart;
   
   // 100 nano-second to mili-second
   TxTask.m_qwThisTimer.QuadPart /= 10000; 
   
   // Check the timer span
   if ( TxTask.m_qwThisTimer.QuadPart <= 500 )
   {
      return;
   }
   
/*
   // Get the monitor update status
   iStatus = GetMonitorUpdateMask();

   // Query the device mode
   if ( iStatus & MONITOR_DEVICE_MODE )
   {
      NBlock_GetNetworkWiMode( 0X00 );

      // Read transfer start timer
      GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwInitTimer );      
      return;
   }
   
   // Query all the node status
   if ( iStatus & MONITOR_STATUS_NODE )
   {
      NBlock_GetAllNodeStatus( 0X00 );

      // Read transfer start timer
      GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwInitTimer );      
      return;
   }   
*/
	
   // Switch to the next state machine
   if ( TxTask.m_dwPacketSize )
   {
      API_TxQueue.m_iMainStatus = API_TX_STATUS_OPEN; 
		
		// The task status
		TxTask.m_iStatus = TXD_TASK_STATUS_JOB_ONMARCH;
   }
   else
   {
      API_TxQueue.m_iMainStatus = API_TX_STATUS_IDLE;

		// The task status
		TxTask.m_iStatus = TXD_TASK_STATUS_JOB_WAITING;
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Open()
{
   char iMode;
   char buffer[0XFF];
   unsigned char iSize;

   // Get the monitor mode
   iMode = GetMonitorWiMode();

   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwInitTimer ); 
   
   // Get the suspend node table
   iSize = GetSuspendNodeTable( buffer, sizeof( buffer ) );

   // The node status initialization
   switch ( iMode )
   {
   case 0X00:
      {
      }
      break;

   case 0X01:
      {
         // Check if the node status valid
         if ( !iSize )
         {
            break;
         }

/*
         // Activate the suspend nodes
         NBlock_InsertActiveNode( 0X00, buffer, iSize );
*/
         
         // Set the monitor mask
         SetMonitorUpdateMask( MONITOR_STATUS_NODE );      
         
         // Switch to next status
         API_TxQueue.m_iMainStatus = API_TX_STATUS_INIT;            
         
      }
      break;

   default:
      {
      }
      break;
   }

   // Message type switch
   if ( TxTask.m_iQoS == QOS_SHORT_MESSAGE )
   {
      API_TxQueue.m_iMainStatus = API_TX_STATUS_SEND;
   }
   else
   {
      API_TxQueue.m_iMainStatus = API_TX_STATUS_SINF;
   }   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_SInf()
{
   unsigned char iAttr;
   SessionHeader * pHeader;

   // The session descriptor
   pHeader = ( SessionHeader * )API_TxQueue.m_pBuffer;

   // The total size value
   pHeader->m_dwTotalSize = TxTask.m_dwPacketSize;

   // The CRC32 value
   pHeader->m_dwCRC32Code = TxTask.m_dwCRC32;

   // The packet attribute
   iAttr = IOSHELL_SINF_PACKET_ATTR;

   // Initialize the message head
   HAL_InitMsgHead( TxTask.m_iNode, iAttr, TxTask.m_iQoS );

   // Write out packet
   HAL_WritePacket( ( char * )API_TxQueue.m_pBuffer, 0X08 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Send()
{
   char * pBuffer;
   unsigned char iSize;
   unsigned char iAttr;
   unsigned long dwOffset;
   
   // The default attribute
   if ( TxTask.m_dwOffsetSize )
   {
      iAttr = IOSHELL_DATA_PACKET_BODY;
   }  
   else
   {
      iAttr = IOSHELL_DATA_PACKET_HEAD;
   }

   // Get the packet size
   iSize = HAL_GetTxPacketSize();
  
   // The un-send packet size
   dwOffset = TxTask.m_dwPacketSize - TxTask.m_dwOffsetSize;
   
   // The buffer size limit
   if ( iSize > dwOffset )
   {
      iSize = ( unsigned char )dwOffset;
   }
   
   // The buffer pointer address
   pBuffer = TxTask.m_pBuffer + TxTask.m_dwOffsetSize;
   
   // The buffer contents
   memcpy( API_TxQueue.m_pBuffer, pBuffer, iSize );
   
   // Update the offset
   TxTask.m_dwOffsetSize += iSize;
   
   // The upper limit
   if ( TxTask.m_dwOffsetSize >= TxTask.m_dwPacketSize )
   {
      TxTask.m_dwOffsetSize = TxTask.m_dwPacketSize;
      iAttr |= COMM_ATTRIBUTE_TAIL; 
   } 

   // The short message service
   if ( TxTask.m_iQoS == QOS_SHORT_MESSAGE )
   {
      iAttr = IOSHELL_WSMS_PACKET_ATTR;
   }

   // Initialize the message head
   HAL_InitMsgHead( TxTask.m_iNode, iAttr, TxTask.m_iQoS );
   
   // Write out packet
   HAL_WritePacket( ( char * )API_TxQueue.m_pBuffer, iSize );
   
   // Update the Tx percent
   dwOffset = 100 * TxTask.m_dwOffsetSize / TxTask.m_dwPacketSize;
   
   // The task status
   TxTask.m_iStatus = ( unsigned char )dwOffset;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char API_TxStateHandler_CheckEnd()
{
   return ( TxTask.m_dwOffsetSize >= TxTask.m_dwPacketSize );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Wait()
{
   unsigned char iNode;

   // Get the Rx-Node
   iNode = HAL_GetRxNodeStatus();

   // Update the monitor status
   SetMonitorRxNode( iNode );

   // Query all the node status
   NBlock_GetAllNodeStatus( 0X00 );

   // Set the update status
   SetMonitorUpdateMask( MONITOR_STATUS_NODE );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Rept()
{  
   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwThisTimer );
   
   // Change to timer span
   TxTask.m_qwTimerSpan.QuadPart = TxTask.m_qwThisTimer.QuadPart;
   
   // Get the timer offset
   TxTask.m_qwTimerSpan.QuadPart -= TxTask.m_qwInitTimer.QuadPart;
   
   // 100 nano-second to mili-second
   TxTask.m_qwTimerSpan.QuadPart /= 10000;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Over()
{
   unsigned char iStatus;

   // The short message is UDP communication
   if ( TxTask.m_iQoS == QOS_SHORT_MESSAGE )
   {
      TxTask.m_iStatus = TXD_TASK_STATUS_END_SUCCESS;
   }
   else
   {
      // Check the success status
      if ( HAL_CheckTxSuccessStatus() ) 
      {
         TxTask.m_iStatus = TXD_TASK_STATUS_END_SUCCESS;
      }
      else
      {
         TxTask.m_iStatus = TXD_TASK_STATUS_END_FAILURE;
      }
      
      // Get the node status
      iStatus = HAL_GetRxNodeStatus();
      if ( iStatus & 0X04 )
      {
         TxTask.m_iStatus = TXD_TASK_STATUS_END_FAILURE;  
      }      
   }

   // Clear the task information
   TxTask.m_dwPacketSize = 0X00;

   // Clear the offset size
   TxTask.m_dwOffsetSize = 0X00;
   
   // Switch to next status
   API_TxQueue.m_iMainStatus = API_TX_STATUS_IDLE;	
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxStateHandler_Idle()
{   
   // Check the Tx Prepare Status
   if ( !HAL_CheckTxPrepareStatus() )
   {
      return;
   }
	
	// The communicatio failed
	if ( !HAL_CheckTxSuccessStatus() && TxTask.m_dwPacketSize )
	{
		// Clear the task information
		TxTask.m_dwPacketSize = 0X00;
		
		// Clear the offset size
		TxTask.m_dwOffsetSize = 0X00;		
		
		// The task status
		TxTask.m_iStatus = TXD_TASK_STATUS_END_FAILURE;	
	}
   
   // Read Rx timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&RxTask.m_qwThisTimer );
   
   // Get the timer offset
   RxTask.m_qwThisTimer.QuadPart -= RxTask.m_qwRecvTimer.QuadPart;
   
   // 100 nano-second to mili-second
   RxTask.m_qwThisTimer.QuadPart /= 10000; 
   
   // Read transfer start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwThisTimer );
      
   // Get the timer offset
   TxTask.m_qwThisTimer.QuadPart -= TxTask.m_qwOpenTimer.QuadPart;
   
   // 100 nano-second to mili-second
   TxTask.m_qwThisTimer.QuadPart /= 10000; 
   
   // Check the timer span
   if ( ( TxTask.m_qwThisTimer.QuadPart > 500 ) && ( RxTask.m_qwThisTimer.QuadPart > 500 ) )
   {      
      // Query the node status
      HAL_QueryPacket();      
      
      // Record last check timer
      GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwOpenTimer );             
   }

   // Validate the offset value
   if ( !TxTask.m_dwPacketSize )
   {
      return;
   }
   
   // Validate the write FIFO size
   if ( !HAL_GetTxPacketSize() )
   {
      return;
   }   

   // Set the monitor mask
   SetMonitorUpdateMask( MONITOR_STATUS_NODE );   
   
   // Switch to new status
   API_TxQueue.m_iMainStatus = API_TX_STATUS_INIT; 
   
   // The task status
   TxTask.m_iStatus = TXD_TASK_STATUS_JOB_ONMARCH;

   // Record last check timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwOpenTimer );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char API_GetTxPrepareStatus()
{
   // Check the Tx Prepare Status
   if ( !HAL_CheckTxPrepareStatus() )
   {
      return API_TX_STATUS_ONMARCH;
   }

   // Check if success on last write 
   if ( HAL_CheckTxSuccessStatus() )
   {
      return API_TX_STATUS_SUCCESS;
   }

   // Operation failed
   return API_TX_STATUS_FAILURE;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SwitchTxStatus( unsigned char iStatus )
{
   // Status change time
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwOpenTimer );

   // Status change time
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwSendTimer );
   
   // Switch the status
   API_TxQueue.m_iMainStatus = iStatus;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned long GetSendTimerSpan()
{
   // Status change time
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwThisTimer );
      
   // Get the timer offset
   TxTask.m_qwThisTimer.QuadPart -= TxTask.m_qwOpenTimer.QuadPart;
   
   // 100 nano-second to mili-second
   TxTask.m_qwThisTimer.QuadPart /= 10000; 
   return TxTask.m_qwThisTimer.u.LowPart;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned long GetOpenTimerSpan()
{
   // Status change time
   GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwThisTimer );
      
   // Get the timer offset
   TxTask.m_qwThisTimer.QuadPart -= TxTask.m_qwSendTimer.QuadPart;
   
   // 100 nano-second to mili-second
   TxTask.m_qwThisTimer.QuadPart /= 10000;
   return TxTask.m_qwThisTimer.u.LowPart;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_TxQueueHandler()
{
   unsigned char iRetVal;
   unsigned long dwTimer;

   // The HAL-TxQueue state machine
   HAL_MainTxStatusHandler();

   // The Tx-Routine
   switch ( API_TxQueue.m_iMainStatus )
   {
   case API_TX_STATUS_INIT:
      {
         // Record the open timer
         API_TxStateHandler_Init();
      }
      break;

   case API_TX_STATUS_OPEN:
      {
         // Open this session
         API_TxStateHandler_Open();
      }
      break;

   case API_TX_STATUS_SINF:
      {       
         // Write out the session descriptor
         if ( API_TxQueue.m_iDescriptor )
         {
            API_TxStateHandler_SInf();         
         }         
                  
         // Switch to the next state machine
         API_TxQueue.m_iMainStatus = API_TX_STATUS_SEND;
      }
      break;

   case API_TX_STATUS_SEND:
      {
         // Get the prepare status
         iRetVal = API_GetTxPrepareStatus();
         
         // Not get the prepare status
         if ( iRetVal == API_TX_STATUS_ONMARCH )
         {
            break;
         }
         
         // Operation failure
         if ( iRetVal == API_TX_STATUS_FAILURE )
         {
            API_SwitchTxStatus( API_TX_STATUS_WAIT );            
            break;            
         }
         
         // Operation success
         if ( iRetVal != API_TX_STATUS_SUCCESS )
         {
            break;
         }

         // Write out the session descriptor
         API_TxStateHandler_Send(); 
         
         // The short message service
         if ( TxTask.m_iQoS == QOS_SHORT_MESSAGE )
         {
            // Switch to the next state machine
            API_TxQueue.m_iMainStatus = API_TX_STATUS_OVER;
            break;
         }
               
         // Come to end of the body state
         if ( !API_TxStateHandler_CheckEnd() )
         {
            break;
         }
         
         // Switch to the next state machine
         API_SwitchTxStatus( API_TX_STATUS_WAIT );         
      }
      break;

   case API_TX_STATUS_WAIT:
      {       
         // Get the prepare status
         iRetVal = API_GetTxPrepareStatus();
         
         // Make sure the Tx-operation is complete
         if ( iRetVal == API_TX_STATUS_ONMARCH  )
         {
            break;
         }

         // Operation failure
         if ( iRetVal == API_TX_STATUS_FAILURE )
         {
            // Wait status handler
            API_TxStateHandler_Wait();

            // Switch to the next state machine
            API_SwitchTxStatus( API_TX_STATUS_REPT );            
            break;            
         }         
                  
         // Validate the Tx-Memory empty
         if ( HAL_ValidateTxMemEmpty() )
         {
            // Wait status handler
            API_TxStateHandler_Wait();
            
            // Switch to the next state machine
            API_SwitchTxStatus( API_TX_STATUS_REPT );            
            break;
         }
                  
         // Check the timer span
         dwTimer = GetSendTimerSpan();
         if ( dwTimer > 50 )
         {            
            // Query the status
            HAL_QueryPacket();
            
            // Status change time
            GetSystemTimeAsFileTime( ( LPFILETIME )&TxTask.m_qwSendTimer );
         }   
      }
      break;

   case API_TX_STATUS_REPT:
      {     
         // Rept status handler
         API_TxStateHandler_Rept();
         
         // Switch to over status
         API_SwitchTxStatus( API_TX_STATUS_OVER );         
      }
      break;

   case API_TX_STATUS_OVER:
      {         
         // Get the prepare status
         iRetVal = API_GetTxPrepareStatus();
         
         // Not get the prepare status
         if ( iRetVal == API_TX_STATUS_ONMARCH )
         {
            break;
         }                          
         
         // Over status handler
         API_TxStateHandler_Over();
      }
      break;

   case API_TX_STATUS_IDLE:
      {             
         API_TxStateHandler_Idle();
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
DLLEXPORT char QueryTxMsgStatus( char iShell, char * pStatus )
{
   // Validate pointer
   if ( pStatus )
   {
      *pStatus = TxTask.m_iStatus;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SendHiQoSMessage( char iShell, short iNode, char * pBuffer, long dwSize )
{
   // Task protection: Job on march now
	if ( !( TxTask.m_iStatus & 0X80 ) )
   {
      return 0X00;
   }
   
   // Commit the task
   CommitTransmitTask( iNode, pBuffer, dwSize, QOS_HIQOS_MESSAGE );
   
   // Switch to other thread     
   return 0X01;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SendNoQoSMessage( char iShell, short iNode, char * pBuffer, long dwSize )
{
   // Task protection
   if ( API_TxQueue.m_iMainStatus != API_TX_STATUS_IDLE )
   {	
      return 0X00;
   }
	
   // Commit the task
   CommitTransmitTask( iNode, pBuffer, dwSize, QOS_NOQOS_MESSAGE );
   
   // Switch to other thread     
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SendShortMessage( char iShell, short iNode, char * pBuffer, long dwSize )
{
   // Task protection
   if ( API_TxQueue.m_iMainStatus != API_TX_STATUS_IDLE )
   {
      return 0X00;
   }

   // The max size of the packet
   if ( !dwSize || dwSize > 0X06 )
   {
      return 0X00;
   }
   
   // Commit the task
   CommitTransmitTask( iNode, pBuffer, dwSize, QOS_SHORT_MESSAGE );
   
   // Switch to other thread     
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetTxMessageTime( char iShell, unsigned long * pTime )
{   
   char iStatus;

   // The task status
   iStatus = TxTask.m_iStatus & TXD_TASK_STATUS_REPORTTABLE;
   if ( !iStatus )
   {
      return 0X00;
   }

   // Get the task timer
   if ( pTime )
   {
      *pTime = TxTask.m_qwTimerSpan.u.LowPart;
   }
   return 0X01;
}
