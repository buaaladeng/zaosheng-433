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
// File:    hal-command.cpp
// Author:  Mickle.ding
// Created: 11/10/2011
//
// Description:  Define the class hal-command
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
#include "API-WiMinet.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Message.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Command.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOShell.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-IOQueue.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitWiMinetMessage( NodeMsg * pMsg, char iMode )
{
   ULARGE_INTEGER  qwTimerA;
   ULARGE_INTEGER  qwTimerB;
   
   // Check the shell active status
   if ( !API_IsShellActive() )
   {
      return;
   }
   
   // Get the start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&qwTimerA ); 
   
   while ( 0X01 )
   {
      // Switch to other thread
      Sleep( 0X00 );
      
      // Read off the CmdAck commands
      if ( iMode )
      {
         API_GetCmdAckQueue( pMsg );
      }
      else
      {
         API_GetNetMonQueue( pMsg );
      }
      
      // Get current timer
      GetSystemTimeAsFileTime( ( LPFILETIME )&qwTimerB ); 
      
      // Get the offset value
      qwTimerB.QuadPart -= qwTimerA.QuadPart;
      
      // 100 nano-second to mili-second
      qwTimerB.QuadPart /= 10000;  
      
      // Check the timer offset
      if ( qwTimerB.QuadPart >= 0X01 )
      {
         break;
      }
   }
   
   // Reset the message body
   memset( pMsg, 0X00, sizeof( NodeMsg ) );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char SendWiMinetMessage( NodeMsg * pMsg, long iTimerSpan, char iMode )
{
   char iRetVal;
   NodeMsg * pSource;
   unsigned char   iOpCode;
   ULARGE_INTEGER  qwTimerA;
   ULARGE_INTEGER  qwTimerB;
   
   // Check the shell active status
   if ( !API_IsShellActive() )
   {
      return 0X00;
   }

   // WiMinet message
   pMsg->m_iValueD = ( iMode ? 0X01 : 0X00 );
   
   // Write out the command
   WriteShell( pMsg );
   
   // The ACK command
   if ( pMsg->m_iOpCode == CMDMSG_TX_PACKET )
   {
      // Get the source command
      pSource = ( NodeMsg * )pMsg->m_pBuffer;

      // The ack command
      iOpCode = ( pSource->m_iOpCode | ACKMSG_BASE );
   }
   else
   {
      iOpCode = ( pMsg->m_iOpCode | ACKMSG_BASE );      
   }
   
   // Get the start timer
   GetSystemTimeAsFileTime( ( LPFILETIME )&qwTimerA ); 
   
   while ( 0X01 )
   {
      // Switch to other thread
      Sleep( 0X01 );
      
      // Get current timer
      GetSystemTimeAsFileTime( ( LPFILETIME )&qwTimerB ); 
      
      // Get the offset value
      qwTimerB.QuadPart -= qwTimerA.QuadPart;
      
      // 100 nano-second to mili-second
      qwTimerB.QuadPart /= 10000;  
      
      // Check the timer offset
      if ( qwTimerB.QuadPart >= iTimerSpan )
      {
         return 0X00;            
      }
      
      // Read the ACK message
      if ( iMode )
      {
         iRetVal = API_GetCmdAckQueue( pMsg );      
      }
      else
      {
         iRetVal = API_GetNetMonQueue( pMsg );
      }
               
      // Check the CmdAck queue      
      if ( !iRetVal )
      {
         continue;
      }
      return ( pMsg->m_iOpCode == iOpCode );
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitCommandMessage( NodeMsg * pMsg )
{
   // Initialize the message
   InitWiMinetMessage( pMsg, 0X01 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void InitMonitorMessage( NodeMsg * pMsg )
{
   // Initialize the message
   InitWiMinetMessage( pMsg, 0X00 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char SendCommandMessage( NodeMsg * pMsg, long iTimerSpan )
{
   return SendWiMinetMessage( pMsg, iTimerSpan, 0X01 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char SendMonitorMessage( NodeMsg * pMsg, long iTimerSpan )
{
   return SendWiMinetMessage( pMsg, iTimerSpan, 0X00 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_IOShellConfigure( NodeMsg * pPacket, NodeMsg * pCmdMsg, short iTxAddr )
{
   // Reset the message body
   pPacket->m_iOpCode = CMDMSG_TX_PACKET;
   
   // The MSB value
   pPacket->m_iValueA = ( iTxAddr >> 0X08 );
   
   // The LSB value
   pPacket->m_iValueB = ( iTxAddr & 0XFF );
   
   // The packet attribute
   pPacket->m_iValueC = IOSHELL_CMD_REQUEST_UNIT;
   
   // The extent packet size
   pPacket->m_iAmount = 0X09 + pCmdMsg->m_iAmount;
   
   // Update the message CRC value
   UpdateNodeMsgCRC( pCmdMsg );
   
   // The message buffer contents
   memcpy( pPacket->m_pBuffer, pCmdMsg, pPacket->m_iAmount );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_WiMinetConfigure( NodeMsg * pPacket, NodeMsg * pCmdMsg, short iTxAddr )
{
   // Reset the message body
   pPacket->m_iOpCode = CMDMSG_TX_PACKET;

   // The MSB value
   pPacket->m_iValueA = ( iTxAddr >> 0X08 );

   // The LSB value
   pPacket->m_iValueB = ( iTxAddr & 0XFF );

   // The packet attribute
   pPacket->m_iValueC = WIMINET_CFG_REQUEST_UNIT;

   // The extent packet size
   pPacket->m_iAmount = 0X09 + pCmdMsg->m_iAmount;

   // Update the message CRC value
   UpdateNodeMsgCRC( pCmdMsg );
   
   // The message buffer contents
   memcpy( pPacket->m_pBuffer, pCmdMsg, pPacket->m_iAmount );     
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetUARTPortSpeed( NodeMsg * pMsg, long dwBaudRate )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_BAUDRATE;
   
   // The 1st extended parameter
   switch ( dwBaudRate )
   {
   case 110:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_110;
      }
      break;

   case 300:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_300;
      }
      break;

   case 600:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_600;
      }
      break;

   case 1200:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_1200;
      }
      break;

   case 2400:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_2400;
      }
      break;

   case 4800:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_4800;
      }
      break;

   case 9600:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_9600;
      }
      break;

   case 14400:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_14400;
      }
      break;

   case 19200:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_19200;
      }
      break;

   case 38400:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_38400;
      }
      break;

   case 56000:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_56000;
      }
      break;

   case 57600:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_57600;
      }
      break;

   case 115200:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_115200;
      }
      break;

   case 128000:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_128000;
      }
      break;

   case 256000:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_256000;
      }
      break;

   default:
      {
         pMsg->m_pBuffer[0X00] = UART_BAUD_19200;
      }
      break;
   }
      
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = 0X01;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetConfigureMode( NodeMsg * pMsg, char iMode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_HALT_MACHINE;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iMode;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_EnableRSSIOutput( NodeMsg * pMsg, char iEnable )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_DEBUG_MESSAGE;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iEnable;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetRxAttenuation( NodeMsg * pMsg, char index, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_RX_ATTENUATION;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = index;
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetRxAttenuation( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_RX_ATTENUATION;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetTxOutputPower( NodeMsg * pMsg, char index, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_PA_INDEX;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = index;
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetTxOutputPower( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_PA_INDEX;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetNodeTxAddress( NodeMsg * pMsg, short iAddr, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_NODE_TX_ADDRESS;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( iAddr >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = ( iAddr & 0XFF );   
   
   // The 3rd extended parameter
   pMsg->m_pBuffer[0X02] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X03;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetNodeTxAddress( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_NODE_TX_ADDRESS;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetWirelessSpeed( NodeMsg * pMsg, char iSpeed, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_CHSPEED;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iSpeed;
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetWirelessSpeed( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_CHSPEED;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetModuleVersion( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_FIRMWARE_VERSION;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_InsertActiveNode( NodeMsg * pMsg, char * pBuffer, char iCount )
{
   unsigned char index;
   unsigned char iSuitID;
   unsigned char iOffset;
   
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_NODE_ACTIVE;
      
   // The extended parameters
   for ( index = 0X00; index < ( unsigned char )iCount; index++ )
   {
      iSuitID = ( unsigned char )pBuffer[index] >> 0X03;
      iOffset = ( unsigned char )pBuffer[index] &  0X07;
      pMsg->m_pBuffer[iSuitID] |= 0X01 << iOffset;      
   }
   
   // The extented parameter count
   pMsg->m_iAmount = 0X20;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_DeleteActiveNode( NodeMsg * pMsg, char * pBuffer, char iCount )
{
   unsigned char index;
   unsigned char iSuitID;
   unsigned char iOffset;
   
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_NODE_INACTIVE;
      
   // The extended parameters
   for ( index = 0X00; index < ( unsigned char )iCount; index++ )
   {
      iSuitID = ( unsigned char )pBuffer[index] >> 0X03;
      iOffset = ( unsigned char )pBuffer[index] &  0X07;
      pMsg->m_pBuffer[iSuitID] |= 0X01 << iOffset;      
   }
   
   // The extented parameter count
   pMsg->m_iAmount = 0X20;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_CleanupTxMessage( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SHUTDOWN_NETWORK;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetNodeRunStatus( NodeMsg * pMsg, short iNode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_QUERY_NODESTATUS;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iNode;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SearchActiveNode( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SEARCH_WINET;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = 0X01;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetSearchProfile( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_FIFO_STATISTICS;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_ScanAccessPoints( NodeMsg * pMsg, char iCount )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SEARCH_WINET;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iCount;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetAPScanProfile( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SELECT_OPTIMAL_WIAP;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetAPScanChannel( NodeMsg * pMsg, char * pBuffer, char iSize )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_NETSCAN_CHANNEL;
      
   // The size protection
   if ( ( unsigned char )iSize > 0X08 )
   {
      iSize = 0X08;
   }
   
   // The 1st extended parameter
   if ( pBuffer )
   {
      memcpy( pMsg->m_pBuffer, pBuffer, iSize );
   }
   
   // The extented parameter count
   pMsg->m_iAmount = iSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetAPScanChannel( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_NETSCAN_CHANNEL;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetAPChannelMask( NodeMsg * pMsg, char iMask )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_NETSCAN_MASKID;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iMask;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetAPChannelMask( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_NETSCAN_MASKID;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetWirelessChanA( NodeMsg * pMsg, char iChanA, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_FREQUENCY_POINTX1;
   
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iChanA;

   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetWirelessChanA( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_FREQUENCY_POINTX1;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetWirelessChanB( NodeMsg * pMsg, char iChanB, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_FREQUENCY_POINTX2;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iChanB;
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetWirelessChanB( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_FREQUENCY_POINTX2;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetNetworkWiMode( NodeMsg * pMsg, char iWiMode, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_WIMODE;
      
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iWiMode;
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetNetworkWiMode( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_WIMODE;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetAllNodeStatus( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_ALL_NODE_STATUS;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetAllActiveNode( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_ACTIVE_NODES;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}


// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetAdminPassword( NodeMsg * pMsg, char * pBuffer, unsigned char iSize, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_ADMIN_PASSWORD;

   // Memory protection
   if ( iSize >= sizeof( pMsg->m_pBuffer ) )
   {
      iSize = sizeof( pMsg->m_pBuffer ) - 0X01;
   }
   
   // The security password
   memcpy( pMsg->m_pBuffer, pBuffer, iSize );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[iSize] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = iSize + 0X01;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetWirelessPANID( NodeMsg * pMsg, unsigned short iPANID, char iSave )
{   
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_WIRELESS_PAN_ID;

   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( iPANID >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = ( iPANID & 0XFF );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X02] = iSave;

   // The extented parameter count
   pMsg->m_iAmount = 0X03;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetWirelessPANID( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_WIRELESS_PAN_ID;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetSecurityLevel( NodeMsg * pMsg, unsigned char iSecurity, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_SECURITY_LEVEL;

   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = iSecurity;
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iSave;

   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetSecurityLevel( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_SECURITY_LEVEL;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetWModuleStatus( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_WMODULE_STATUS;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetNetworkStatus( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_NETWORK_STATUS;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetCoordinatorID( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_COORDINATOR_ID;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetParentAddress( NodeMsg * pMsg, short iNode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_PARENT_ADDRESS;

   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( ( unsigned short )iNode >> 0X08 );

   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iNode & 0XFF;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetMemberAddress( NodeMsg * pMsg, short iNode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_MEMBER_ADDRESS;
   
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( ( unsigned short )iNode >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iNode & 0XFF;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetMeshCenterHop( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_MESH_CENTER_HOP;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_NodeAddressToMAC( NodeMsg * pMsg, unsigned short iNode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_NODE_ADDRESS_TO_MAC;
   
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( ( unsigned short )iNode >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iNode & 0XFF;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_MACToNodeAddress( NodeMsg * pMsg, char * pMAC )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_MAC_TO_NODE_ADDRESS;

   // The 1st-8th extended parameter
   memcpy( pMsg->m_pBuffer, pMAC, 0X08 );
   
   // The extented parameter count
   pMsg->m_iAmount = 0X08;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_QueryNetworkRSSI( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_QUERY_NETWORK_RSSI;
      
   // The extented parameter count
   pMsg->m_iAmount = 0X00;      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_QueryNodeRegInfo( NodeMsg * pMsg, short iNode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_QUERY_NODE_REG_INFO;
   
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( ( unsigned short )iNode >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iNode & 0XFF;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_QueryWiMinetPath( NodeMsg * pMsg, short iNode )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_QUERY_WIMINET_PATH;
   
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( ( unsigned short )iNode >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = iNode & 0XFF;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X02;      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetActiveAPQueue( NodeMsg * pMsg, char iHead, char iSize )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_ACTIVE_WIAP;
   
   // The start item index to read
   pMsg->m_iValueA = iHead;
   
   // The total items to read
   pMsg->m_iValueB = iSize;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void NBlock_GetNetworkWiMode( char iShell )
{
   NodeMsg msg;

   // Initialize the node message
   memset( &msg, 0X00, sizeof( msg ) );
      
   // Construct the message
   IMP_GetNetworkWiMode( &msg );

   // Write out this command
   WriteShell( &msg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void NBlock_GetAllNodeStatus( char iShell )
{
   NodeMsg msg;
   
   // Initialize the node message
   memset( &msg, 0X00, sizeof( msg ) );
   
   // Construct the message
   IMP_GetAllNodeStatus( &msg );
   
   // Write out this command
   WriteShell( &msg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void NBlock_InsertActiveNode( char iShell, char * pBuffer, char iCount )
{
   NodeMsg msg;
   
   // Initialize the node message
   memset( &msg, 0X00, sizeof( msg ) );
   
   // Construct the message
   IMP_InsertActiveNode( &msg, pBuffer, iCount );
   
   // Write out this command
   WriteShell( &msg );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetNodeRxAddress( NodeMsg * pMsg, short iAddr, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_PEER_ADDRESS;
	
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( iAddr >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = ( iAddr & 0XFF );   
   
   // The 3rd extended parameter
   pMsg->m_pBuffer[0X02] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X03;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetNodeRxAddress( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_PEER_ADDRESS;
	
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_SetHostNetworkAP( NodeMsg * pMsg, short iAddr, char iSave )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_SET_HOST_NETWORK_AP;
	
   // The 1st extended parameter
   pMsg->m_pBuffer[0X00] = ( iAddr >> 0X08 );
   
   // The 2nd extended parameter
   pMsg->m_pBuffer[0X01] = ( iAddr & 0XFF );   
   
   // The 3rd extended parameter
   pMsg->m_pBuffer[0X02] = iSave;
   
   // The extented parameter count
   pMsg->m_iAmount = 0X03;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void IMP_GetHostNetworkAP( NodeMsg * pMsg )
{
   // The command for this packet
   pMsg->m_iOpCode = CMDMSG_GET_HOST_NETWORK_AP;
	
   // The extented parameter count
   pMsg->m_iAmount = 0X00;
}
