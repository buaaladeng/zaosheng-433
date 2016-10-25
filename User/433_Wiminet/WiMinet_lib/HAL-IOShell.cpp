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
// File:    hal-ioshell.c
// Author:  Mickle.ding
// Created: 11/2/2011
//
// Description:  Define the class hal-ioshell
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <WinSock2.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WinUART.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "HAL-IOShell.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_SetupIOShellStateMachine()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void HAL_CloseIOShellStateMachine()
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenNET_TCPServer( unsigned short iPortNumber )
{
   SOCKET  hServer;
   SOCKET  hClient;
   sockaddr_in server;
   sockaddr_in client;
   
   // Allocate one socket
   hServer = socket( AF_INET, SOCK_STREAM, 0 );
   
   // The protocol is TCP/IP
   server.sin_family = AF_INET;

   // The listen IPV4 address
   server.sin_addr.s_addr = INADDR_ANY;

   // The port number
   server.sin_port = htons( iPortNumber ); 

   // Bind on this socket
   if ( bind( hServer, ( struct sockaddr * )( &server ), sizeof( sockaddr ) ) < 0 )
   {
      return INVALID_HANDLE_VALUE;
   }

   // Listen on this socket
   listen( hServer, 0X01 );

   // Accept incomming connections
   int iAddrSize = sizeof( sockaddr_in );

   // Accept for new connection
   hClient = accept( hServer, ( sockaddr * )&client, &iAddrSize );

   // Close the server socket
   closesocket( hServer );

   // The socket handle
   return ( HANDLE )hClient;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenNET_TCPClient( char * pIP4Address, unsigned short iPortNumber )
{
   int iRetVal;
   SOCKET  hSocket;
   sockaddr_in addr;
   unsigned long dwValue;
   const char * pOptVal;

   // The active communication device
   addr.sin_family = AF_INET;
   addr.sin_addr.S_un.S_addr = inet_addr( pIP4Address ); 
   addr.sin_port = htons( iPortNumber ); 
   
   // Allocate one socket
   hSocket = socket( AF_INET, SOCK_STREAM, 0 );
   
   // Connect with the peer node
   iRetVal = connect( hSocket, ( PSOCKADDR )&addr, sizeof( SOCKADDR_IN ) );
   
   // Check the connection status
   if ( iRetVal == SOCKET_ERROR )
   {
      return INVALID_HANDLE_VALUE;
   }
   
   // The send buffer=0X00 to disables the Nagle algorithm for send coalescing.
   dwValue = 1024L * 1024L;
   pOptVal = ( const char * )&dwValue;
   iRetVal = setsockopt( hSocket, SOL_SOCKET, SO_SNDBUF, pOptVal, sizeof( dwValue ) ); 

/*
   // Non-blocking mode
   dwValue = 0X01;
   iRetVal = ioctlsocket( hSocket, FIONBIO, &dwValue ); 
*/

   // The socket handle
   return ( HANDLE )hSocket;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenCOMShell( char * pDeviceName, unsigned long dwBaudRate )
{
   HANDLE hFile = OpenUART( pDeviceName, dwBaudRate );
   return hFile;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenUSBShell( char * pDeviceName, unsigned long dwUSBIndex )
{
   return INVALID_HANDLE_VALUE;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenNETShell( char * pIP4Address, unsigned short iPortNumber )
{
   HANDLE hSocket;
   
   // Validate the IP address
   if ( !inet_addr( pIP4Address ) )
   {
      return 0X00;
   } 
   
   // Open as TCP server
   if ( inet_addr( pIP4Address ) == inet_addr( "127.0.0.1" ) )
   {
      hSocket = OpenNET_TCPServer( iPortNumber );
   }
   else
   {
      hSocket = OpenNET_TCPClient( pIP4Address, iPortNumber );
   }
   
   // The socket handle
   return hSocket;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenSPIShell()
{
   return INVALID_HANDLE_VALUE;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void CloseCOMShell( HANDLE hFile )
{
   CloseHandle( hFile );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void CloseUSBShell( HANDLE hFile )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void CloseNETShell( HANDLE hFile )
{
   SOCKET  hSocket;
   
   // The socket
   hSocket = ( SOCKET )hFile;

   // Shutdown this connection
   shutdown( hSocket, SD_BOTH );

   // Close the socket
   closesocket( hSocket );
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void CloseSPIShell( HANDLE hFile )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void WriteCOMShell( HANDLE hFile, NodeMsg * pMsg )
{
   DWORD dwSize;
      
   // Write out the message header
   dwSize = pMsg->m_iAmount + 0X09;
   WriteFile( hFile, pMsg, dwSize, &dwSize, 0X00 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void WriteUSBShell( HANDLE hFile, NodeMsg * pMsg )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void WriteNETShell( HANDLE hFile, NodeMsg * pMsg )
{
   DWORD dwSize;
      
   // The total message size
   dwSize = pMsg->m_iAmount + 0X09;
   
   // Send out this packet to the peer node
   send( ( SOCKET )hFile, ( const char * )pMsg, dwSize, 0X00 );   
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void WriteSPIShell( HANDLE hFile, NodeMsg * pMsg )
{
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ReadCOMShell( HANDLE hFile, char * pBuffer, unsigned char iSize )
{
   DWORD dwSize;
   COMSTAT status;

   // Get the queue status
   ClearCommError( hFile, &dwSize, &status );
   
   // Make sure the queue is not empty
   if ( !status.cbInQue )
   {
      return 0X00;
   }
   
   // The input stream size
   dwSize = status.cbInQue;
   
   // Buffer overrun protection
   if ( dwSize > iSize )
   {
      dwSize = iSize;
   }
   
   // Read stream from the COM port
   ReadFile( hFile, pBuffer, dwSize, &dwSize, NULL );
   return ( unsigned char )dwSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ReadUSBShell( HANDLE hFile, char * pBuffer, unsigned char iSize )
{
   return 0X00;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ReadNETShell( HANDLE hFile, char * pBuffer, unsigned char iSize )
{
   int iRetVal;
   DWORD dwSize;
   SOCKET hSocket;

   // The socket handle
   hSocket = ( SOCKET )hFile;

   // Get the size of this read queue   
   dwSize = 0X00;
   iRetVal = ioctlsocket( hSocket, FIONREAD, &dwSize );
   if ( iRetVal || !dwSize )
   {      
      return 0X00;
   }
   
   // Buffer overrun protection
   if ( dwSize > iSize )
   {
      dwSize = iSize;
   }
   
   // Read this packet from the socket
   dwSize = recv( hSocket, pBuffer, dwSize, 0X00 ); 
   
   // Validate the connection status
   if ( dwSize == SOCKET_ERROR )
   {
      dwSize = 0X00;
   }

   // The data read back from the socket
   return ( unsigned char )dwSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
unsigned char ReadSPIShell( HANDLE hFile, char * pBuffer, unsigned char iSize )
{
   return 0X00;
}
