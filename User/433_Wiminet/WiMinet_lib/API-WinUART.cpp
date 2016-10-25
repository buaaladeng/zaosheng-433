// #############################################################################
// *****************************************************************************
//          Copyright (c) 2007-2008, WiMi-net (Beijing) Tech. Co., Ltd.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//         INFORMATION WHICH IS THE PROPERTY OF WIMI-NET TECH. CO., LTD.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               WIMI-NET TECH. CO., LTD, IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    uart.cpp
// Author:  Mickle.ding
// Created: 1/22/2007
//
// Description:  Define the class uart
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <windows.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <stdio.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WinUART.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
HANDLE OpenUART( char * pDeviceName, unsigned long dwBaudRate )
{
   HANDLE hUART;
   char buffer[16];
   
   // The file name
   memset( buffer, 0X00, sizeof( buffer ) );
   sprintf( buffer, "\\\\.\\%s", pDeviceName );

   // Open this file
   hUART = CreateFile( 
      buffer,
      GENERIC_READ | GENERIC_WRITE,
      0,
      NULL,
      OPEN_EXISTING,
      0,
      NULL );

   if ( hUART == INVALID_HANDLE_VALUE )
   {
      return hUART;
   }

   // Set the Tx/Rx buffer
   SetupComm( hUART, 32768, 32768 );

   // Retrieve the time-out parameters for all read and write operations
   // on the port. 
   COMMTIMEOUTS CommTimeouts;
   GetCommTimeouts ( hUART, &CommTimeouts );
   
   // Change the COMMTIMEOUTS structure settings.
   CommTimeouts.ReadIntervalTimeout         = MAXDWORD; 
   CommTimeouts.ReadTotalTimeoutMultiplier  = 0;  
   CommTimeouts.ReadTotalTimeoutConstant    = 0; 
   
   CommTimeouts.WriteTotalTimeoutMultiplier = 0;  
   CommTimeouts.WriteTotalTimeoutConstant   = 0; 
   SetCommTimeouts( hUART, &CommTimeouts );

   // Read current settings   
   DCB PortDCB;
   memset( &PortDCB, 0X00, sizeof( DCB ) );
   GetCommState ( hUART, &PortDCB );
 
   // The baud rate
   PortDCB.BaudRate = dwBaudRate;

   // Number of bits/byte, 4-8 
   PortDCB.ByteSize = 0X08;		
   
   // Parity
   PortDCB.Parity = NOPARITY;
   
   // Stop bits
   PortDCB.StopBits = ONESTOPBIT;

   // Enable parity checking 
   PortDCB.fParity = FALSE;
   
   // Binary mode; no EOF check 
   PortDCB.fBinary = TRUE;		
   
   // No CTS output flow control 
   PortDCB.fOutxCtsFlow = FALSE;	

	// No DSR output flow control				
   PortDCB.fOutxDsrFlow = FALSE;
   
   // DSR sensitivity 
   PortDCB.fDsrSensitivity = FALSE;	
   
   // DTR flow control type 
   PortDCB.fDtrControl = DTR_CONTROL_DISABLE;       

   // RTS flow control 
   PortDCB.fRtsControl = RTS_CONTROL_DISABLE;  
   
   // No XON/XOFF out flow control 
   PortDCB.fOutX = FALSE;	
   
   // No XON/XOFF in flow control 
   PortDCB.fInX = FALSE;		
   
   // XOFF continues Tx 
   PortDCB.fTXContinueOnXoff = FALSE;
   
   // Disable error replacement 
   PortDCB.fErrorChar = FALSE;
   
   // Disable null stripping 	
   PortDCB.fNull = FALSE;		
   
   // Do not abort reads/writes on error
   PortDCB.fAbortOnError = FALSE;
   
   // Change the parameters
   SetCommState ( hUART, &PortDCB );

   // Clear the UART Tx/Rx buffers
   PurgeComm( hUART, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );
   return hUART;
}
