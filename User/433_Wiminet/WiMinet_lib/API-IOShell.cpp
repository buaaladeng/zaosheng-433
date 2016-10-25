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
// File:    api-ioshell.c
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class api-ioshell
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Windows.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-IOShell.h"

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
API_IOShell_StateMachine  API_IOShell;

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char OpenHardwareDeviceInterface( char * pDeviceID, unsigned long dwParam )
{
   HANDLE hFileHandle;
   unsigned long dwSize;
   unsigned char iPhysicalID;
      
   // The default value
   iPhysicalID = NUL_SHELL_INTERFACE;
   hFileHandle = INVALID_HANDLE_VALUE;
   
   // The string size
   dwSize = strlen( pDeviceID );

   if ( !_memicmp( pDeviceID, "COM", 0X03 ) )
   {
      // Open the COM port as shell
      iPhysicalID = COM_SHELL_INTERFACE;
      hFileHandle = OpenCOMShell( pDeviceID, dwParam );

      // Validate the device handle
      if ( hFileHandle == INVALID_HANDLE_VALUE )
      {
         return 0X00;
      }
   }
   else if ( !_memicmp( pDeviceID, "USB-API-DEVICE", 0X0E ) )
   {
      // Open the USB port as shell
      iPhysicalID = USB_SHELL_INTERFACE;      
      hFileHandle = OpenUSBShell( pDeviceID, dwParam );

      // Validate the device handle
      if ( hFileHandle == INVALID_HANDLE_VALUE )
      {
         return 0X00;      
      }
   }   
   else if ( inet_addr( pDeviceID ) != INADDR_NONE )
   {
      // Validate the IP address
      iPhysicalID = NET_SHELL_INTERFACE;            
      hFileHandle = OpenNETShell( pDeviceID, ( unsigned short )dwParam );

      // Validate the device handle      
      if ( hFileHandle == INVALID_HANDLE_VALUE )
      {
         return 0X00;      
      }
   }
   else if ( !pDeviceID && !dwParam )
   {
      // The SPI device
      iPhysicalID = SPI_SHELL_INTERFACE;            
      hFileHandle = OpenSPIShell();

      // Validate the device handle
      if ( hFileHandle == INVALID_HANDLE_VALUE )
      {
         return 0X00; 
      }
   }

   // Check the phisical ID
   if ( iPhysicalID == NUL_SHELL_INTERFACE )
   {
      return 0X00;
   }

   // Initiate the IOShell
   API_SetupIOShellStateMachine();

   // Update the device information
   API_IOShell.m_hFileHandle = hFileHandle;
   API_IOShell.m_iPhysicalID = iPhysicalID;
   return 0X01;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupIOShellStateMachine()
{
   // Initialize the HAL_IOShell
   HAL_SetupIOShellStateMachine();   
   
   // Reset the control block
   API_InitShellInterface();
         
   // Initialize the message pointer
   API_IOShell.m_pMsg = ( NodeMsg * )API_IOShell.m_pBuffer;

   // Setup the packet queue
   API_SetupPacketQueue();  
   
   // Setup the string queue
   API_SetupStringQueue();

   // Setup the Txd-Ack queue
   API_SetupTxdAckQueue();

   // Setup the Rxd-Pkt queue
   API_SetupRxdPktQueue();

   // Setup the Sig-Pkt queue
   API_SetupSigPktQueue();

   // Setup the Cmd-Ack queue   
   API_SetupCmdAckQueue();

   // Setup the Net-Mon queue
   API_SetupNetMonQueue();
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_CloseIOShellStateMachine()
{
   // Close the packet queue
   API_ClosePacketQueue();

   // Close the string queue
   API_CloseStringQueue();

   // Close the Txd-Ack queue
   API_CloseTxdAckQueue();
   
   // Close the Rxd-Pkt queue
   API_CloseRxdPktQueue();
   
   // Close the Sig-Pkt queue
   API_CloseSigPktQueue();
   
   // Close the Cmd-Ack queue   
   API_CloseCmdAckQueue();  
   
   // Close the Net-Mon queue
   API_CloseNetMonQueue();
   
   // Close the HAL_IOShell
   HAL_CloseIOShellStateMachine();
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_SetupPacketQueue()
{
   char * pXMemBuffer;
   unsigned short iSize;
   
   // The queue memory buffer size
   iSize = 1024;
   
   // The queue memory buffer
   pXMemBuffer = ( char * )malloc( iSize );
   
   // The buffer pointer
   API_IOShell.m_pPacketBuff = pXMemBuffer;
   
   // Initialize the queue
   InitAtomicQueue16( &API_IOShell.m_PacketQueue, pXMemBuffer, iSize );      
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_ClosePacketQueue()
{
   // Release the queue memory
   if ( !API_IOShell.m_pPacketBuff )
   {
      return;
   }
   
   // Delete the memory for this queue
   free( API_IOShell.m_pPacketBuff );
   
   // Invalidate the memory pointer
   API_IOShell.m_pPacketBuff = 0X00;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char API_IsShellActive()
{
   return ( API_IOShell.m_iPhysicalID != NUL_SHELL_INTERFACE );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_InitShellInterface()
{
   // Reset the control block
   memset( &API_IOShell, 0X00, sizeof( API_IOShell ) );

   // The phsical ID
   API_IOShell.m_iPhysicalID = NUL_SHELL_INTERFACE;

   // The file handle
   API_IOShell.m_hFileHandle = INVALID_HANDLE_VALUE;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
char ValidateDataPacketAttribute( unsigned char iAttr )
{
   // Stream information descriptor
   if ( iAttr == IOSHELL_SINF_PACKET_ATTR )
   {
      return 0X01;
   }

   // The short message service
   if ( iAttr == IOSHELL_WSMS_PACKET_ATTR )
   {
      return 0X01;
   }

	// The GPS message
   if ( iAttr == IOSHELL_SGPS_PACKET_ATTR )
   {
      return 0X01;
   }

   // The valid range
   if ( ( iAttr >= IOSHELL_DATA_PACKET_BODY ) && ( iAttr <= IOSHELL_DATA_PACKET_UNIT ) )
   {
      return 0X01;
   }
   return 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_IOShellHandler()
{
   unsigned char iRetVal;

   // Read the packet
   while ( 0X01 )
   {
      // Read the shell packet
      iRetVal = ReadShell( ( char * )API_IOShell.m_pBuffer, sizeof( API_IOShell.m_pBuffer ) );
      if ( iRetVal )
      {
         WriteAtomicQueue16( &API_IOShell.m_PacketQueue, ( char * )API_IOShell.m_pBuffer, iRetVal );
      }
      
      // Dispatch the messag queue
      iRetVal = API_GetAtomicQueue( &API_IOShell.m_PacketQueue, API_IOShell.m_pMsg );
      if ( !iRetVal )
      {
         return;
      }
      
      // Check the packe size
      if ( iRetVal >= 0X09 )
      {
         break;
      }

      // The string message
      API_PutStringQueue( ( char * )API_IOShell.m_pMsg, 0X01 );
   }

   // Process the message command
   switch ( API_IOShell.m_pMsg->m_iOpCode )
   {
   case CMDMSG_RX_PACKET:
      {
         // Validate the data packet attribute
         if ( ValidateDataPacketAttribute( API_IOShell.m_pMsg->m_iValueC ) )
         {
            API_PutRxdPktQueue( API_IOShell.m_pMsg );
         }
         else
         {
            API_PutCmdAckQueue( ( NodeMsg * )API_IOShell.m_pMsg->m_pBuffer );
         }
      }
      break;

   case ACKMSG_TX_PACKET:
      {
         // IOShell packet
         API_PutTxdAckQueue( API_IOShell.m_pMsg );
      }
      break;

   case ACKMSG_SET_WIMODE:
      {
         // Only direct the user command
         if ( API_IOShell.m_pMsg->m_iValueD )
         {
            API_PutCmdAckQueue( API_IOShell.m_pMsg );
         }

         // Monitor this user command message
         API_PutNetMonQueue( API_IOShell.m_pMsg );         
      }
      break;

   case ACKMSG_GET_WIMODE:
      {
         // Only direct the user command
         if ( API_IOShell.m_pMsg->m_iValueD )
         {
            API_PutCmdAckQueue( API_IOShell.m_pMsg );
         }

         // Monitor this user command message         
         API_PutNetMonQueue( API_IOShell.m_pMsg );
      }
      break;

   case ACKMSG_WIRELESS_SINAL_PROFILE:
      {
         API_PutSigPktQueue( API_IOShell.m_pMsg );
      }
      break;

   case ACKMSG_GET_FIFO_STATISTICS:
      {
         // This command can be send out in two modes
         if ( API_IOShell.m_pMsg->m_iValueD )
         {
            API_PutCmdAckQueue( API_IOShell.m_pMsg );
         }
         else
         {
            API_PutTxdAckQueue( API_IOShell.m_pMsg );
         }
      }
      break;

   default:
      {
         // Only direct the user command
         if ( API_IOShell.m_pMsg->m_iValueD )
         {
            API_PutCmdAckQueue( API_IOShell.m_pMsg );
         }
         else
         {
            API_PutNetMonQueue( API_IOShell.m_pMsg );
         }
      }
      break;
   }
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void API_MonitorNetwork()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void CloseHardwareDeviceInterface()
{
   HANDLE hFileHandle;
   
   // Validate the device ID
   if ( API_IOShell.m_iPhysicalID == NUL_SHELL_INTERFACE )
   {
      return;
   }

   // Get the handle value
   hFileHandle = API_IOShell.m_hFileHandle;

   // Close the IOShell
   API_CloseIOShellStateMachine();
   
   switch ( API_IOShell.m_iPhysicalID )
   {
   case COM_SHELL_INTERFACE:
      {
         CloseCOMShell( hFileHandle );
      }
      break;

   case USB_SHELL_INTERFACE:
      {
         CloseUSBShell( hFileHandle );
      }
      break;

   case NET_SHELL_INTERFACE:
      {
         CloseNETShell( hFileHandle );
      }
      break;

   case SPI_SHELL_INTERFACE:
      {
         CloseSPIShell( hFileHandle );
      }
      break;

   default:
      {
      }
      break;
   }

   // The default file handle value
   API_IOShell.m_hFileHandle = INVALID_HANDLE_VALUE;
   
   // The file physical ID
   API_IOShell.m_iPhysicalID = NUL_SHELL_INTERFACE;   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void WriteShell( NodeMsg * pMsg )
{
   // Validate the device ID
   if ( API_IOShell.m_iPhysicalID == NUL_SHELL_INTERFACE )
   {
      return;
   }
   
   // Update the message CRC value
   UpdateNodeMsgCRC( pMsg );
   
   switch ( API_IOShell.m_iPhysicalID )
   {
   case COM_SHELL_INTERFACE:
      {
         WriteCOMShell( API_IOShell.m_hFileHandle, pMsg );
      }
      break;

   case USB_SHELL_INTERFACE:
      {
         WriteUSBShell( API_IOShell.m_hFileHandle, pMsg );
      }
      break;

   case NET_SHELL_INTERFACE:
      {
         WriteNETShell( API_IOShell.m_hFileHandle, pMsg );
      }
      break;
      
   case SPI_SHELL_INTERFACE:
      {
         WriteSPIShell( API_IOShell.m_hFileHandle, pMsg );
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
unsigned char ReadShell( char * pBuffer, unsigned char iSize )
{
   unsigned char iRetVal;

   // Validate the device ID
   if ( API_IOShell.m_iPhysicalID == NUL_SHELL_INTERFACE )
   {
      return 0X00;
   }
   
   switch ( API_IOShell.m_iPhysicalID )
   {
   case COM_SHELL_INTERFACE:
      {
         iRetVal = ReadCOMShell( API_IOShell.m_hFileHandle, pBuffer, iSize );
      }
      break;

   case USB_SHELL_INTERFACE:
      {
         iRetVal = ReadUSBShell( API_IOShell.m_hFileHandle, pBuffer, iSize );
      }
      break;

   case NET_SHELL_INTERFACE:
      {
         iRetVal = ReadNETShell( API_IOShell.m_hFileHandle, pBuffer, iSize );            
      }
      break;

   case SPI_SHELL_INTERFACE:
      {
         iRetVal = ReadSPIShell( API_IOShell.m_hFileHandle, pBuffer, iSize );            
      }
      break;

   default:
      {
         return 0X00;         
      }
      break;
   }    
   return iRetVal;
}
