// #############################################################################
// *****************************************************************************
//          Copyright (c) 2007-2009, WiMi-net (Beijing) Tech. Co., Ltd.
//      THIS IS AN UNPUBLISHED WORK CONTAINING CONFIDENTIAL AND PROPRIETARY
//         INFORMATION WHICH IS THE PROPERTY OF WIMI-NET TECH. CO., LTD.
//
//    ANY DISCLOSURE, USE, OR REPRODUCTION, WITHOUT WRITTEN AUTHORIZATION FROM
//               WIMI-NET TECH. CO., LTD, IS STRICTLY PROHIBITED.
// *****************************************************************************
// #############################################################################
//
// File:    simplequeue16.cpp
// Author:  Mickle.ding
// Created: 7/22/2009
//
// Description:  Define the class simplequeue16
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include <string.h>

// -----------------------------------------------------------------------------
// DESCRIPTION:
// -----------------------------------------------------------------------------
#include "API-Queue16.h"

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
void InitAtomicQueue16( 
   AtomicQueue16 * pQueue, 
   char  * pBuffer, 
   unsigned short  iSize )
{
   // The read  and write pointer
   pQueue->m_iReadIndex = 0X00;
   pQueue->m_iWriteIndex = 0X00;
   pQueue->m_iTotalSize = iSize;

   // The buffer contents
   pQueue->m_pBuffer = pBuffer;
}

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
unsigned char ReadAtomicQueue16( 
   AtomicQueue16 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize )
{
   unsigned short index;      
   
   // Read out the write index
   index = pQueue->m_iWriteIndex;
   
   // Queue empty?
   if ( index == pQueue->m_iReadIndex )
   {
      return 0X00;
   }
   
   if ( index < pQueue->m_iReadIndex )
   {
      index = pQueue->m_iTotalSize - pQueue->m_iReadIndex + index;
   }
   else
   {
      index -= pQueue->m_iReadIndex;
   }
   
   // Adjust the read size
   if ( iSize > index )
   {
      iSize = ( unsigned char )index;
   }
   
   // Copy the memory to the buffer
   if ( pQueue->m_iReadIndex <= pQueue->m_iTotalSize - iSize )
   {
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_iReadIndex, iSize );      
   }
   else
   {
      index = pQueue->m_iTotalSize - pQueue->m_iReadIndex;
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_iReadIndex, index );
      memcpy( pBuffer + index, pQueue->m_pBuffer, iSize - index );
   }
   
   index = pQueue->m_iTotalSize - pQueue->m_iReadIndex;
   if ( iSize >= index )
   {
      index = iSize - index;
   }
   else
   {
      index = iSize + pQueue->m_iReadIndex;
   }
   
   // Update the read index
   pQueue->m_iReadIndex = index;
   return iSize;
}

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
unsigned char WriteAtomicQueue16( 
   AtomicQueue16 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize )
{
   unsigned short index;
   unsigned short iWIndex;
   
   // Validate the queue size
   if ( !pQueue->m_iTotalSize )
   {
      return 0X00;
   }
   
   // The read position
   index = pQueue->m_iReadIndex;
   
   // The write position
   iWIndex = pQueue->m_iWriteIndex + 0X01;
   if ( index == iWIndex )
   {
      return 0X00;
   }
   
   // Get the max spaces in this queue
   if ( index < iWIndex )
   {
      index = pQueue->m_iTotalSize - iWIndex + index;
   }
   else
   {
      index = index - iWIndex;
   }   
   
   // Adjust the read size
   if ( iSize > index )
   {
      return 0X00;
   }
   
   // Copy the memory to the buffer
   if ( pQueue->m_iWriteIndex <= pQueue->m_iTotalSize - iSize )
   {
      memcpy( pQueue->m_pBuffer + pQueue->m_iWriteIndex, pBuffer, iSize );      
   }
   else
   {
      index = pQueue->m_iTotalSize - pQueue->m_iWriteIndex;
      memcpy( pQueue->m_pBuffer + pQueue->m_iWriteIndex, pBuffer, index );
      memcpy( pQueue->m_pBuffer, pBuffer + index, iSize - index );
   }
   
   index = pQueue->m_iTotalSize - pQueue->m_iWriteIndex;
   if ( iSize >= index )
   {
      index = iSize - index;
   }
   else
   {
      index = iSize + pQueue->m_iWriteIndex;
   }  
   
   pQueue->m_iWriteIndex = index;
   return iSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void SnapAtomicQueue16( AtomicQueue16 * pQueue, char * pBuffer, unsigned char iSize )
{
   unsigned char index;
   
   // Validate the queue buffer
   if ( !pQueue->m_iTotalSize )
   {
      return;
   }
   
   // Validate the size
   if ( iSize > pQueue->m_iTotalSize )
   {
      iSize = ( unsigned char )pQueue->m_iTotalSize;
   }
   
   // Copy the memory to the buffer
   if ( pQueue->m_iReadIndex <= pQueue->m_iTotalSize - iSize )
   {
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_iReadIndex, iSize );
   }
   else
   {
      index = pQueue->m_iTotalSize - pQueue->m_iReadIndex;
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_iReadIndex, index );
      memcpy( pBuffer + index, pQueue->m_pBuffer, iSize - index );
   }
}

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
void PeekQueue16Status( AtomicQueue16 * pQueue, QueueStatus16 * pStatus )
{
   // One data is reservered for queue management
   pStatus->m_iSize = pQueue->m_iTotalSize - 0X01;
   if ( pQueue->m_iWriteIndex >= pQueue->m_iReadIndex )
   {
      pStatus->m_iUsed = pQueue->m_iWriteIndex - pQueue->m_iReadIndex;
   }
   else
   {
      pStatus->m_iUsed = pQueue->m_iTotalSize - pQueue->m_iReadIndex 
         + pQueue->m_iWriteIndex;
   }
   pStatus->m_iFree = pStatus->m_iSize - pStatus->m_iUsed;
   
   // The 1st key in the buffer
   pStatus->m_iHead = pQueue->m_pBuffer[pQueue->m_iReadIndex];
}