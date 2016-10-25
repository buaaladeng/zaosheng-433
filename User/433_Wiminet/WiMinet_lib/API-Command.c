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
// File:    api-command.cpp
// Author:  Mickle.ding
// Created: 11/9/2011
//
// Description:  Define the class api-command
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
#include "HAL-Command.h"

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
#include "API-WiMinet.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetUARTPortSpeed( char iShell, long dwBaudRate )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetUARTPortSpeed( &msg, dwBaudRate );  
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetConfigureMode( char iShell, char iMode )
{
   NodeMsg msg;

   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetConfigureMode( &msg, iMode );         

   // Wait for the ACK command
   return SendCommandMessage( &msg, 1000 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char EnableRSSIOutput( char iShell, char iEnable )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_EnableRSSIOutput( &msg, iEnable );
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetRxAttenuation( char iShell, char index, char iSave )
{
   NodeMsg msg;
   unsigned long dwTimer;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetRxAttenuation( &msg, index, iSave );   

   // The operation timer
   dwTimer = iSave ? 500 : 100;
      
   // Wait for the ACK command
   return SendCommandMessage( &msg, dwTimer );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetRxAttenuation( char iShell, char * pIndex )
{
   NodeMsg msg;
     
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetRxAttenuation( &msg );
      
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // Get the new node ID
   if ( pIndex )
   {
      *pIndex = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetTxOutputPower( char iShell, char index, char iSave )
{
   NodeMsg msg;
   unsigned long dwTimer;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetTxOutputPower( &msg, index, iSave );   
   
   // The operation timer
   dwTimer = iSave ? 500 : 100;
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, dwTimer );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetTxOutputPower( char iShell, char * pIndex )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetTxOutputPower( &msg );   
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // The PA index
   if ( pIndex )
   {
      *pIndex = msg.m_pBuffer[0X00] & 0X07;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetNodeTxAddress( char iShell, short iObject, short iAddr, char iSave )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long dwTime;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetNodeTxAddress( &msg, iAddr, iSave );   
   
   // The operation timer
   dwTime = iSave ? 500 : 100;
   
   // The native request packet
   pMsg = &msg;
      
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwTime += 1000;
      
      // The remote request packet
      pMsg = &packet;      
   }      
   
   // Wait for the ACK command
   return SendCommandMessage( pMsg, dwTime );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetNodeTxAddress( char iShell, short iObject, short * pAddr )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long  dwIime;
   unsigned short iValue;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetNodeTxAddress( &msg );

   // Native time out
   dwIime = 100;
   
   // The native request packet
   pMsg = &msg;

   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );

      // The remote time out
      dwIime += 1000;

      // The remote request packet
      pMsg = &packet;
   }
   
   // Wait for the ACK command
   if ( !SendCommandMessage( pMsg, dwIime ) )
   {
      return 0X00;
   }

   // Get the new node ID
   iValue = pMsg->m_pBuffer[0X00];
   iValue <<= 0X08;
   iValue |= pMsg->m_pBuffer[0X01];

   // Return value
   if ( pAddr )
   {
      *pAddr = iValue;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetWirelessPANID( char iShell, short iObject, short iPANID, char iSave )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;   
   unsigned long dwTime;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_SetWirelessPANID( &msg, iPANID, iSave );
   
   // The operation timer
   dwTime = iSave ? 500 : 100;
   
   // The native request packet
   pMsg = &msg;
   
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwTime += 1000;
      
      // The remote request packet
      pMsg = &packet;      
   }      
   
   // Wait for the ACK command
   return SendCommandMessage( pMsg, dwTime );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetWirelessPANID( char iShell, short iObject, short * pPANID )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long  dwIime;   
   unsigned short iValue;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetWirelessPANID( &msg );
   
   // Native time out
   dwIime = 100;
   
   // The native request packet
   pMsg = &msg;
   
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwIime += 1000;
      
      // The remote request packet
      pMsg = &packet;
   }
   
   // Wait for the ACK command
   if ( !SendCommandMessage( pMsg, dwIime ) )
   {
      return 0X00;
   } 
   
   // The name service id
   iValue  = pMsg->m_pBuffer[0X00];
   iValue <<= 0X08;
   iValue |= pMsg->m_pBuffer[0X01];
   
   // The return value
   if ( pPANID )
   {
      *pPANID  = iValue;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetSecurityLevel( char iShell, short iObject, char iSecurity, char iSave )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long dwTime;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_SetSecurityLevel( &msg, iSecurity, iSave );
   
   // The operation timer
   dwTime = iSave ? 500 : 100;
   
   // The native request packet
   pMsg = &msg;
   
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwTime += 1000;
      
      // The remote request packet
      pMsg = &packet;      
   }      
   
   // Wait for the ACK command
   return SendCommandMessage( pMsg, dwTime );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetSecurityLevel( char iShell, short iObject, char * pSecurity )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long  dwIime;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetSecurityLevel( &msg );
   
   // Native time out
   dwIime = 100;
   
   // The native request packet
   pMsg = &msg;
   
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwIime += 1000;
      
      // The remote request packet
      pMsg = &packet;
   }
   
   // Wait for the ACK command
   if ( !SendCommandMessage( pMsg, dwIime ) )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pSecurity )
   {
      *pSecurity = pMsg->m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetAdminPassword( char iShell, short iObject, char * pBuffer, char iSize, char iSave )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long dwTime;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_SetAdminPassword( &msg, pBuffer, iSize, iSave );
   
   // The operation timer
   dwTime = iSave ? 500 : 100;
   
   // The native request packet
   pMsg = &msg;
   
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwTime += 1000;
      
      // The remote request packet
      pMsg = &packet;      
   }   

   // Wait for the ACK command
   return SendCommandMessage( pMsg, dwTime );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetWirelessSpeed( char iShell, char iSpeed, char iSave )
{
   NodeMsg msg;
   unsigned long dwTimer;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetWirelessSpeed( &msg, iSpeed, iSave );   
   
   // The operation timer
   dwTimer = iSave ? 500 : 100;
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, dwTimer );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetWirelessSpeed( char iShell, char * pSpeed )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetWirelessSpeed( &msg );   
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // Get the new node ID
   if ( pSpeed )
   {
      *pSpeed = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetModuleVersion( char iShell, VersionInfo * pVersion )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetModuleVersion( &msg );   
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 1000 ) )
   {
      return 0X00;
   }   
   
   // Validate the pointer
   if ( !pVersion )
   {
      return 0X00;
   }  
   
   // Reset the version information
   memset( pVersion, 0X00, sizeof( VersionInfo ) );

   // Get the buffer contents
   memcpy( pVersion, msg.m_pBuffer, msg.m_iAmount );

   // The info-size
   pVersion->m_dwInfoSize = ntohl( pVersion->m_dwInfoSize );

   // The major version
   pVersion->m_dwMajorVersion = ntohl( pVersion->m_dwMajorVersion );

   // The minor version
   pVersion->m_dwMinorVersion = ntohl( pVersion->m_dwMinorVersion );

   // The build number
   pVersion->m_dwBuildNumber = ntohl( pVersion->m_dwBuildNumber );

   // The platform-id
   pVersion->m_dwPlatformId = ntohl( pVersion->m_dwPlatformId );   
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char InsertActiveNode( char iShell, char * pBuffer, char iCount )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_InsertActiveNode( &msg, pBuffer, iCount );   
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char DeleteActiveNode( char iShell, char * pBuffer, char iCount )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_DeleteActiveNode( &msg, pBuffer, iCount );   
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char CleanupTxMessage( char iShell )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_CleanupTxMessage( &msg );
      
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetNodeRunStatus( char iShell, short iNode, char * pStatus )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetNodeRunStatus( &msg, iNode );
      
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // Not enough memory space
   if ( pStatus )
   {
      *pStatus = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SearchActiveNode( char iShell )
{
   NodeMsg msg;
     
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SearchActiveNode( &msg );
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetSearchProfile( char iShell, char * pStatus )
{
   NodeMsg msg;
   unsigned char iStatus;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetSearchProfile( &msg );   
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }  

   // The host status
   iStatus = msg.m_pBuffer[0X00] & 0X0F;

   // Validate the pointer
   if ( pStatus )
   {
      *pStatus = ( iStatus != HOST_SCAN_NETWORK ) ? 0X01 : 0X00;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char ScanAccessPoints( char iShell, char iCount )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_ScanAccessPoints( &msg, iCount );   
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetAPScanProfile( char iShell, char * pStatus )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetAPScanProfile( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // The status of this operation
   if ( pStatus )
   {
      *pStatus = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetAPScanChannel( char iShell, char * pBuffer, char iSize )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_SetAPScanChannel( &msg, pBuffer, iSize );
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetAPScanChannel( char iShell, char * pBuffer, char iSize )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetAPScanChannel( &msg );
   
   // Wait for the ACK command
   if ( ! SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // The ACK commands
   if ( msg.m_iAmount > iSize )
   {
      return 0X00;
   }
   
   // The buffer contents
   if ( pBuffer )
   {
      memcpy( pBuffer, msg.m_pBuffer, msg.m_iAmount );
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetAPChannelMask( char iShell, char iMask )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetAPChannelMask( &msg, iMask );   
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, 100 );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetAPChannelMask( char iShell, char * pMask )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetAPChannelMask( &msg );   
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // The channel mask code
   if ( pMask )
   {
      *pMask = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetWirelessChanA( char iShell, char iChanA, char iSave )
{
   NodeMsg msg;   
   unsigned long dwTimer;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetWirelessChanA( &msg, iChanA, iSave );   
      
   // The operation timer
   dwTimer = iSave ? 500 : 100;

   // Wait for the ACK command
   return SendCommandMessage( &msg, dwTimer );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetWirelessChanA( char iShell, char * pChannel )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetWirelessChanA( &msg );   
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // Get the new node ID
   if ( pChannel )
   {
      *pChannel = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetWirelessChanB( char iShell, char iChanB, char iSave )
{
   NodeMsg msg;
   unsigned long dwTimer;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetWirelessChanB( &msg, iChanB, iSave );
   
   // The operation timer
   dwTimer = iSave ? 500 : 100;
   
   // Wait for the ACK command
   return SendCommandMessage( &msg, dwTimer );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetWirelessChanB( char iShell, char * pChannel )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetWirelessChanB( &msg );  
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }
   
   // Get the new node ID
   if ( pChannel )
   {
      *pChannel = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetNetworkWiMode( char iShell, char iWiMode, char iSave )
{
   NodeMsg msg;
   unsigned long dwTimer;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_SetNetworkWiMode( &msg, iWiMode, iSave );
   
   // The operation timer
   dwTimer = iSave ? 500 : 100;

   // Wait for the ACK command
   return SendCommandMessage( &msg, dwTimer );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetNetworkWiMode( char iShell, char * pMode )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );

   // Construct the message
   IMP_GetNetworkWiMode( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // The PA index
   if ( pMode )
   {
      *pMode = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char ReadSearchStatus( char * pStatus )
{
   return 0X01;
}


// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetWModuleStatus( char iShell, char * pMainStatus, char * pSubStatus )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetWModuleStatus( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // The main status
   if ( pMainStatus )
   {
      *pMainStatus = msg.m_pBuffer[0X00];
   }

   // The sub status
   if ( pSubStatus )
   {
      *pSubStatus = msg.m_pBuffer[0X01];
   }

   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetNetworkStatus( char iShell, unsigned char * pStatus )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetNetworkStatus( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pStatus )
   {
      *pStatus = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetCoordinatorID( char iShell, unsigned short * pTxAddr )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetCoordinatorID( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pTxAddr )
   {
      *pTxAddr = ( ( unsigned short )msg.m_pBuffer[0X00] << 0X08 );
      *pTxAddr += msg.m_pBuffer[0X01];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetParentAddress( char iShell, short iNode, short * pParent )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetParentAddress( &msg, iNode );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   } 
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }
   
   // The name service id
   if ( pParent )
   {
      *pParent = ( ( unsigned short )msg.m_pBuffer[0X00] << 0X08 );
      *pParent += ( unsigned short )msg.m_pBuffer[0X01];      
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetMemberAddress( char iShell, short iNode, short * pMember, char * pSize )
{
   NodeMsg msg;
   unsigned char index;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetMemberAddress( &msg, iNode );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
      
   // The name service id
   if ( pSize )
   {
      *pSize = ( msg.m_iAmount >> 0X01 );
   }

   for ( index = 0X00; ( index < msg.m_iAmount ) && pMember; index += 0X02 )
   {
      *pMember = ( ( unsigned short )msg.m_pBuffer[index] << 0X08 );
      *pMember += msg.m_pBuffer[index + 0X01];
      pMember++;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetMeshCenterHop( char iShell, unsigned char * pCenterHop )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetMeshCenterHop( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pCenterHop )
   {
      *pCenterHop = msg.m_pBuffer[0X00];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char NodeAddressToMAC( char iShell, unsigned short iNode, char * pMAC )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_NodeAddressToMAC( &msg, iNode );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   }   
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }
   
   // The name service id
   if ( pMAC )
   {
      memcpy( pMAC, msg.m_pBuffer, 0X08 );
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char MACToNodeAddress( char iShell, char * pMAC, unsigned short * pNode )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_MACToNodeAddress( &msg, pMAC );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 100 ) )
   {
      return 0X00;
   } 
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pNode )
   {
      *pNode = msg.m_pBuffer[0X00];
      *pNode <<= 0X08;
      *pNode |= msg.m_pBuffer[0X01];
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char QueryNetworkRSSI( char iShell, char * pRSSI, char * pLNASet, char * pdBmVal, char * pLinkup )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_QueryNetworkRSSI( &msg );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 1000 ) )
   {
      return 0X00;
   } 
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }   
   
   // The RSSI value
   if ( pRSSI )
   {
      *pRSSI = msg.m_pBuffer[0X00];
   }

   // The LNASet
   if ( pLNASet )
   {
      *pLNASet = msg.m_pBuffer[0X01];
   }

   // The dBm value
   if ( pdBmVal )
   {
      *pdBmVal = msg.m_pBuffer[0X02];
   }

   // The network link status
   if ( pLinkup )
   {
      *pLinkup = msg.m_pBuffer[0X03];
   }

   return 0X01; 
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char QueryNodeRegInfo( char iShell, short iNode, WiMinet_MeshItem * pInfo )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_QueryNodeRegInfo( &msg, iNode );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 1000 ) )
   {
      return 0X00;
   } 
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pInfo )
   {
      memcpy( pInfo, msg.m_pBuffer, sizeof( WiMinet_MeshItem ) );
   }
   return 0X01;      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char QueryWiMinetPath( char iShell, short iNode, char * pBuffer, char * pSize )
{         
   NodeMsg msg;
   unsigned char index;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_QueryWiMinetPath( &msg, iNode );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 1000 ) )
   {
      return 0X00;
   } 
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }
   
   // The hop counter
   if ( pSize )
   {
      *pSize = msg.m_iAmount;
   }

   // The start index
   index = 0X00;

   // Reverse the byte order
   while ( pBuffer && ( index < msg.m_iAmount ) )
   {
      pBuffer[index + 0X00] = msg.m_pBuffer[index + 0X01];
      pBuffer[index + 0X01] = msg.m_pBuffer[index + 0X00];
      index += 0X02;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetActiveAPQueue( char iShell, char iHead, char iSize, char * pQueue, char * pSize )
{
   NodeMsg msg;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetActiveAPQueue( &msg, iHead, iSize );
   
   // Wait for the ACK command
   if ( !SendCommandMessage( &msg, 1000 ) )
   {
      return 0X00;
   } 
   
   // Validate the ack packet
   if ( !msg.m_iAmount )
   {
      return 0X00;
   }   
   
   // The name service id
   if ( pQueue )
   {
      memcpy( pQueue, msg.m_pBuffer, msg.m_iAmount );
   }

	// The item counter
	if ( pSize )
	{
		*pSize = msg.m_iValueB;
	}
   return 0X01; 
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetNodeRxAddress( char iShell, short iObject, short iAddr, char iSave )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long dwTime;
   
   // Initialize the message body
   InitCommandMessage( &msg );
	
   // Construct the message
   IMP_SetNodeRxAddress( &msg, iAddr, iSave );   
   
   // The operation timer
   dwTime = iSave ? 500 : 100;
   
   // The native request packet
   pMsg = &msg;
	
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwTime += 1000;
      
      // The remote request packet
      pMsg = &packet;      
   }      
   
   // Wait for the ACK command
   return SendCommandMessage( pMsg, dwTime );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetNodeRxAddress( char iShell, short iObject, short * pAddr )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long  dwIime;
   unsigned short iValue;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetNodeRxAddress( &msg );
	
   // Native time out
   dwIime = 100;
   
   // The native request packet
   pMsg = &msg;
	
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
		
      // The remote time out
      dwIime += 1000;
		
      // The remote request packet
      pMsg = &packet;
   }
   
   // Wait for the ACK command
   if ( !SendCommandMessage( pMsg, dwIime ) )
   {
      return 0X00;
   }
	
   // Get the new node ID
   iValue = pMsg->m_pBuffer[0X00];
   iValue <<= 0X08;
   iValue |= pMsg->m_pBuffer[0X01];
	
   // Return value
   if ( pAddr )
   {
      *pAddr = iValue;
   }
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char SetHostNetworkAP( char iShell, short iObject, short iAddr, char iSave )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long dwTime;
   
   // Initialize the message body
   InitCommandMessage( &msg );
	
   // Construct the message
   IMP_SetHostNetworkAP( &msg, iAddr, iSave );   
   
   // The operation timer
   dwTime = iSave ? 500 : 100;
   
   // The native request packet
   pMsg = &msg;
	
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
      
      // The remote time out
      dwTime += 1000;
      
      // The remote request packet
      pMsg = &packet;      
   }      
   
   // Wait for the ACK command
   return SendCommandMessage( pMsg, dwTime );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
DLLEXPORT char GetHostNetworkAP( char iShell, short iObject, short * pAddr )
{
   NodeMsg msg;
   NodeMsg packet;
   NodeMsg * pMsg;
   unsigned long  dwIime;
   unsigned short iValue;
   
   // Initialize the message body
   InitCommandMessage( &msg );
   
   // Construct the message
   IMP_GetHostNetworkAP( &msg );
	
   // Native time out
   dwIime = 100;
   
   // The native request packet
   pMsg = &msg;
	
   // WiMinet configure
   if ( iObject )
   {
      IMP_WiMinetConfigure( &packet, &msg, iObject );
		
      // The remote time out
      dwIime += 1000;
		
      // The remote request packet
      pMsg = &packet;
   }
   
   // Wait for the ACK command
   if ( !SendCommandMessage( pMsg, dwIime ) )
   {
      return 0X00;
   }
	
   // Get the new node ID
   iValue = pMsg->m_pBuffer[0X00];
   iValue <<= 0X08;
   iValue |= pMsg->m_pBuffer[0X01];
	
   // Return value
   if ( pAddr )
   {
      *pAddr = iValue;
   }
   return 0X01;
}
