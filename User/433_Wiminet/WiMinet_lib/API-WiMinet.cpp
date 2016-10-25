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
// File:    wimi-net.cpp
// Author:  Mickle.ding
// Created: 11/17/2011
//
// Description:  Define the class wimi-net
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <Windows.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Service.h"

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-WiMinet.h"

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
BOOL APIENTRY DllMain( HANDLE hModule, DWORD fdwReason, LPVOID lpReserved )
{      
   switch ( fdwReason )
   {
   case DLL_PROCESS_ATTACH:
      {
         // Start the service
         API_StartService();
      }
      break;
      
   case DLL_THREAD_ATTACH:
      {
      }
      break;
      
   case DLL_THREAD_DETACH:
      {
      }
      break;
      
   case DLL_PROCESS_DETACH:
      {   
         // Close the service
         API_CloseService();
      }
      break;
      
   default:
      {
      }
      break;
   }
   
   return TRUE;
}
