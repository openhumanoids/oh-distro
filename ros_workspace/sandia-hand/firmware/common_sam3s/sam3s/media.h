/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 *  \unit
 *  \section Purpose
 *
 *  Generic Media type, which provides transparent access to all types of
 *  memories.
 *
 *  \note The physical or HW related media operations (physical device
 *        connection & protection detecting, PIO configurations and interface
 *        driver initialization) are excluded.
 *
 *  \section Usage
 *  -# Do PIO initialization for peripheral interfaces.
 *  -# Initialize peripheral interface driver & device driver.
 *  -# Initialize specific media interface and link to this initialized driver.
 *
 */

#ifndef _MEDIA_
#define _MEDIA_

#include <stdint.h>

/*------------------------------------------------------------------------------
 *      Definitions
 *------------------------------------------------------------------------------*/

/**
 *  \brief  Operation result code returned by media methods
 */
#define MED_STATUS_SUCCESS      0x00
#define MED_STATUS_ERROR        0x01
#define MED_STATUS_BUSY         0x02
#define MED_STATUS_PROTECTED    0x04

/**
 *  \brief Media statuses
 */
#define MED_STATE_NOT_READY     0xFF     /* Media is not connected */
#define MED_STATE_READY         0x00     /* Media is ready for access */
#define MED_STATE_BUSY          0x01     /* Media is busy */

/*------------------------------------------------------------------------------
//      Types
 *------------------------------------------------------------------------------*/
typedef struct _Media Media ;

typedef void (*MediaCallback)( void *argument, uint8_t status, uint32_t transferred, uint32_t  remaining ) ;

typedef uint8_t (*Media_write)( Media* pMedia, uint32_t address, void *data, uint32_t length, MediaCallback callback, void *argument ) ;

typedef uint8_t (*Media_read)( Media* pMedia, uint32_t address, void *data, uint32_t length, MediaCallback callback, void *argument ) ;

typedef uint8_t (*Media_cancelIo)( Media* pMedia ) ;

typedef uint8_t (*Media_lock)( Media* pMedia, uint32_t start, uint32_t end, uint32_t *pActualStart, uint32_t *pActualEnd ) ;

typedef uint8_t (*Media_unlock)( Media* pMedia, uint32_t start, uint32_t end, uint32_t *pActualStart, uint32_t *pActualEnd ) ;

typedef uint8_t (*Media_ioctl)( Media* pMedia, uint8_t ctrl, void *buff ) ;

typedef uint8_t (*Media_flush)( Media* pMedia ) ;

typedef void (*Media_handler)( Media* pMedia ) ;

/**
 *  \brief  Media transfer
 *  \see    TransferCallback
 */
typedef struct
{
    void* data ;               /* < Pointer to the data buffer */
    uint32_t address ;         /* < Address where to read/write the data */
    uint32_t length ;          /* < Size of the data to read/write */
    MediaCallback callback;    /* < Callback to invoke when the transfer done */
    void* argument ;           /* < Callback argument */
} MEDTransfer ;

/**
 *  \brief  Media object
 *  \see    MEDTransfer
 */
struct _Media
{
  Media_write    write;        /* < Write method */
  Media_read     read;         /* < Read method */
  Media_cancelIo cancelIo;     /* < Cancel pending IO method */
  Media_lock     lock;         /* < lock method if possible */
  Media_unlock   unlock;       /* < unlock method if possible */
  Media_flush    flush;        /* < Flush method */
  Media_handler  handler;      /* < Interrupt handler */

  uint32_t   blockSize;    /* < Block size in bytes (1, 512, 1K, 2K ...) */
  uint32_t   baseAddress;  /* < Base address of media in number of blocks */
  uint32_t   size;         /* < Size of media in number of blocks */
  MEDTransfer    transfer;     /* < Current transfer operation */
  void           *interface;   /* < Pointer to the physical interface used */
  uint8_t  bReserved:4,
    mappedRD:1,   /* < Mapped to memory space to read */
    mappedWR:1,   /* < Mapped to memory space to write */
    protected:1,  /* < Protected media? */
    removable:1;  /* < Removable/Fixed media? */

  uint8_t  state;        /* < Status of media */
  uint16_t reserved ;
} ;

/*  Available medias. */
//extern Media medias[] ;

/*
 *  Number of medias which are effectively used.
 *  Defined by Media, shared usage by USB MSD & FS ...
 */
extern uint32_t numMedias ;

/*------------------------------------------------------------------------------
 *      Exported functions
 *------------------------------------------------------------------------------*/
extern uint32_t MED_Write( Media* pMedia, uint32_t address, void* data, uint32_t length, MediaCallback callback, void* argument ) ;
extern uint32_t MED_Read( Media* pMedia, uint32_t address, void* data, uint32_t length, MediaCallback callback, void* argument ) ;
extern uint32_t MED_Lock( Media* pMedia, uint32_t start, uint32_t end, uint32_t *pActualStart, uint32_t *pActualEnd ) ;
extern uint32_t MED_Unlock( Media* pMedia, uint32_t start, uint32_t end, uint32_t *pActualStart, uint32_t *pActualEnd ) ;
extern uint32_t MED_Flush( Media* pMedia ) ;
extern void MED_Handler( Media* pMedia ) ;
extern void MED_DeInit( Media* pMedia ) ;
extern uint32_t MED_IsInitialized( Media* pMedia ) ;

extern void MED_HandleAll( Media *medias, uint8_t numMedias ) ;

#endif /* _MEDIA_ */

