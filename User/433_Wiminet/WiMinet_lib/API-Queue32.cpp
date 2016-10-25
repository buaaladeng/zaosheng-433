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
#include "API-Queue32.h"

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
void InitAtomicQueue32( 
   AtomicQueue32 * pQueue, 
   char  * pBuffer, 
   unsigned short  iSize )
{
   // The read  and write pointer
   pQueue->m_dwReadIndex = 0X00;
   pQueue->m_dwWriteIndex = 0X00;
   pQueue->m_dwTotalSize = iSize;

   // The buffer contents
   pQueue->m_pBuffer = pBuffer;
}

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
unsigned char ReadAtomicQueue32( 
   AtomicQueue32 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize )
{
   unsigned long dwIndex;      
   
   // Read out the write index
   dwIndex = pQueue->m_dwWriteIndex;
   
   // Queue empty?
   if ( dwIndex == pQueue->m_dwReadIndex )
   {
      return 0X00;
   }
   
   if ( dwIndex < pQueue->m_dwReadIndex )
   {
      dwIndex = pQueue->m_dwTotalSize - pQueue->m_dwReadIndex + dwIndex;
   }
   else
   {
      dwIndex -= pQueue->m_dwReadIndex;
   }
   
   // Adjust the read size
   if ( iSize > dwIndex )
   {
      iSize = ( unsigned char )dwIndex;
   }
   
   // Copy the memory to the buffer
   if ( pQueue->m_dwReadIndex <= pQueue->m_dwTotalSize - iSize )
   {
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_dwReadIndex, iSize );      
   }
   else
   {
      dwIndex = pQueue->m_dwTotalSize - pQueue->m_dwReadIndex;
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_dwReadIndex, dwIndex );
      memcpy( pBuffer + dwIndex, pQueue->m_pBuffer, iSize - dwIndex );
   }
   
   dwIndex = pQueue->m_dwTotalSize - pQueue->m_dwReadIndex;
   if ( iSize >= dwIndex )
   {
      dwIndex = iSize - dwIndex;
   }
   else
   {
      dwIndex = iSize + pQueue->m_dwReadIndex;
   }
   
   // Update the read index
   pQueue->m_dwReadIndex = dwIndex;
   return iSize;
}

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
unsigned char WriteAtomicQueue32( 
   AtomicQueue32 * pQueue, 
   char          * pBuffer, 
   unsigned char   iSize )
{
   unsigned long dwNewIndex;
   unsigned long dwOldIndex;
   
   // Validate the queue size
   if ( !pQueue->m_dwTotalSize )
   {
      return 0X00;
   }
   
   // The read position
   dwNewIndex = pQueue->m_dwReadIndex;
   
   // The write position
   dwOldIndex = pQueue->m_dwWriteIndex + 0X01;
   if ( dwNewIndex == dwOldIndex )
   {
      return 0X00;
   }
   
   // Get the max spaces in this queue
   if ( dwNewIndex < dwOldIndex )
   {
      dwNewIndex = pQueue->m_dwTotalSize - dwOldIndex + dwNewIndex;
   }
   else
   {
      dwNewIndex = dwNewIndex - dwOldIndex;
   }   
   
   // Adjust the read size
   if ( iSize > dwNewIndex )
   {
      return 0X00;
   }
   
   // Copy the memory to the buffer
   if ( pQueue->m_dwWriteIndex <= pQueue->m_dwTotalSize - iSize )
   {
      memcpy( pQueue->m_pBuffer + pQueue->m_dwWriteIndex, pBuffer, iSize );      
   }
   else
   {
      dwNewIndex = pQueue->m_dwTotalSize - pQueue->m_dwWriteIndex;
      memcpy( pQueue->m_pBuffer + pQueue->m_dwWriteIndex, pBuffer, dwNewIndex );
      memcpy( pQueue->m_pBuffer, pBuffer + dwNewIndex, iSize - dwNewIndex );
   }
   
   dwNewIndex = pQueue->m_dwTotalSize - pQueue->m_dwWriteIndex;
   if ( iSize >= dwNewIndex )
   {
      dwNewIndex = iSize - dwNewIndex;
   }
   else
   {
      dwNewIndex = iSize + pQueue->m_dwWriteIndex;
   }  
   
   pQueue->m_dwWriteIndex = dwNewIndex;
   return iSize;
}

// *****************************************************************************
// Design Notes:  
// -----------------------------------------------------------------------------
void SnapAtomicQueue32( AtomicQueue32 * pQueue, char * pBuffer, unsigned char iSize )
{
   unsigned long dwIndex;
   
   // Validate the queue buffer
   if ( !pQueue->m_dwTotalSize )
   {
      return;
   }
   
   // Validate the size
   if ( iSize > pQueue->m_dwTotalSize )
   {
      iSize = ( unsigned char )pQueue->m_dwTotalSize;
   }
   
   // Copy the memory to the buffer
   if ( pQueue->m_dwReadIndex <= pQueue->m_dwTotalSize - iSize )
   {
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_dwReadIndex, iSize );
   }
   else
   {
      dwIndex = pQueue->m_dwTotalSize - pQueue->m_dwReadIndex;
      memcpy( pBuffer, pQueue->m_pBuffer + pQueue->m_dwReadIndex, dwIndex );
      memcpy( pBuffer + dwIndex, pQueue->m_pBuffer, iSize - dwIndex );
   }
}

// *****************************************************************************
// Design Notes:
// -----------------------------------------------------------------------------
void PeekQueue32Status( AtomicQueue32 * pQueue, QueueStatus32 * pStatus )
{
   // One data is reservered for queue management
   pStatus->m_dwSize = pQueue->m_dwTotalSize - 0X01;
   if ( pQueue->m_dwWriteIndex >= pQueue->m_dwReadIndex )
   {
      pStatus->m_dwUsed = pQueue->m_dwWriteIndex - pQueue->m_dwReadIndex;
   }
   else
   {
      pStatus->m_dwUsed = pQueue->m_dwTotalSize - pQueue->m_dwReadIndex 
         + pQueue->m_dwWriteIndex;
   }
   pStatus->m_dwFree = pStatus->m_dwSize - pStatus->m_dwUsed;
   
   // The 1st key in the buffer
   pStatus->m_iHead = pQueue->m_pBuffer[pQueue->m_dwReadIndex];
}