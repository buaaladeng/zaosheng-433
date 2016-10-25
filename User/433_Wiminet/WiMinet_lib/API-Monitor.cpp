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
// File:    api-monitor.cpp
// Author:  Mickle.ding
// Created: 11/23/2011
//
// Description:  Define the class api-monitor
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Windows.h>

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
#include "API-Monitor.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
API_Network_Monitor API_Monitor;

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitNetworkMornitor()
{
   // Clear the item status
   memset( &API_Monitor, 0X00, sizeof( API_Monitor ) );
               
   // Set the monitor mask
   SetMonitorUpdateMask( MONITOR_DEVICE_MODE | MONITOR_STATUS_NODE );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void SetMonitorNodeStatus( unsigned short iNode, unsigned char iStatus )
{
   unsigned char index;
   
   // Validate the node
   if ( !iNode )
   {
      return;
   }

   // The base index
   index = iNode - 0X01;
   
   // Get the node status
   API_Monitor.m_iStatus[index] = iStatus;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char GetMonitorNodeStatus( unsigned short iNode )
{
   unsigned char index;

   // Validate the node
   if ( !iNode )
   {
      return 0X00;
   }

   // The base index
   index = iNode - 0X01;

   // Get the node status
   return API_Monitor.m_iStatus[index];
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void SetMonitorRxNode( unsigned short iNode )
{
   API_Monitor.m_iRxNode = iNode;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned short GetMonitorRxNode()
{
   return API_Monitor.m_iRxNode;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void SetMonitorWiMode( unsigned char iWiMode )
{
   API_Monitor.m_iWiMode = iWiMode;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char GetMonitorWiMode()
{
   return API_Monitor.m_iWiMode;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char GetMonitorUpdateMask()
{
   return API_Monitor.m_iUpdate;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void SetMonitorUpdateMask( unsigned char iMask )
{
   API_Monitor.m_iUpdate |= iMask;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void ClearMonitorUpdateMask( unsigned char iMask )
{
   API_Monitor.m_iUpdate &= ~iMask;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void UpdateNodeStatusTable( unsigned char * pBuffer, unsigned char iSize )
{
   unsigned char index;
   unsigned char iStatus;
   unsigned char iOffset;
   unsigned char bActive;
   unsigned char bOnline;
   unsigned char bPktAck;
   unsigned char bExcept;
   
   for ( index = 0X01; index; index++ )
   {
      // The Multicast
      iOffset = index >> 0X03;
      iStatus = pBuffer[0X20 * 0X00 + iOffset];
      
      bActive = iStatus >> ( index & 0X07 );
      bActive &= 0X01;
      
      // The Sync-ModeA
      iOffset = index >> 0X03;
      iStatus = pBuffer[0X20 * 0X01 + iOffset];
      
      bOnline = iStatus >> ( index & 0X07 );
      bOnline &= 0X01;
      
      // The Tx-Acked
      iOffset = index >> 0X03;
      iStatus = pBuffer[0X20 * 0X02 + iOffset];
      
      bPktAck = iStatus >> ( index & 0X07 );
      bPktAck &= 0X01;
      
      // The Syspend
      iOffset = index >> 0X03;
      iStatus = pBuffer[0X20 * 0X03 + iOffset];
      
      bExcept = iStatus >> ( index & 0X07 );
      bExcept &= 0X01;  
      
      // The node status
      iStatus = 0X00;
      iStatus += ( bActive << 0X07 );
      iStatus += ( bOnline << 0X06 );
      iStatus += ( bPktAck << 0X04 );
      iStatus += ( bExcept << 0X02 ); 
      
      // Set the node monitor status
      SetMonitorNodeStatus( index, iStatus );
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char GetSuspendNodeTable( char * pBuffer, unsigned char iSize )
{
   unsigned char index;
   unsigned char iCount;
   unsigned char iStatus;

   // The suspend node counter
   iCount = 0X00;

   // Get the suspend node table
   for ( index = 0X01; index; index++ )
   {
      // Get the node status
      iStatus = GetMonitorNodeStatus( index );

      // Make sure this node is active
      if ( !( iStatus & 0X80 ) )
      {
         continue;
      }

      // Make sure this node is suspend
      if ( !( iStatus & 0X04 ) )
      {
         continue;
      }

      // Copy the node id
      *pBuffer++ = index;

      // The suspend node counter
      if ( iCount++ >= iSize )
      {
         break;
      }
   }      
   return iCount;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_MonitorHandler()
{
   NodeMsg msg;
   char iRetVal;
   unsigned char iStatus;
   
   // Read message from the queue
   iRetVal = API_GetNetMonQueue( &msg );
   if ( !iRetVal )
   {
      return;
   }
   
   // Get the update status
   iStatus = GetMonitorUpdateMask();

   // Process the commands
   switch ( msg.m_iOpCode )
   {
   case ACKMSG_SET_WIMODE:
      {
         // Update the WiMode
         SetMonitorWiMode( msg.m_pBuffer[0X00] );

         // Clear the update status bit
         ClearMonitorUpdateMask( MONITOR_DEVICE_MODE );
      }
      break;

   case ACKMSG_GET_WIMODE:
      {
         // Update the WiMode
         SetMonitorWiMode( msg.m_pBuffer[0X00] );

         // Clear the update status bit
         ClearMonitorUpdateMask( MONITOR_DEVICE_MODE );
      }
      break;

   case ACKMSG_GET_ALL_NODE_STATUS:
      {
         // Update the node status table
         UpdateNodeStatusTable( msg.m_pBuffer, msg.m_iAmount );

         // Clear the update status bit
         ClearMonitorUpdateMask( MONITOR_STATUS_NODE );
      }
      break;

   default:
      {
      }
      break;
   }
}


