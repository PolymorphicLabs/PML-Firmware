//*****************************************************************************
//! @file       bsp_key.h
//! @brief      Key board support package header file.
//!
//! Revised     $Date: 2014-11-04 03:41:20 -0800 (Tue, 04 Nov 2014) $
//! Revision    $Revision: 14353 $
//
//  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__


/******************************************************************************
* If building with a C++ compiler, make all of the definitions in this header
* have a C binding.
******************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************
* INCLUDES
*/
#include "bsp.h"                // Contains BSP key definitions
#include "driverlib/gpio.h"     // Bitmask defines (e.g. GPIO_FALLING_EDGE)

/******************************************************************************
* DEFINES
*/
#define BSP_KEY_MODE_POLL       0
#define BSP_KEY_MODE_ISR        1


//! Key events. Not all key events are necessarily used/available for all hw.
enum {
	BSP_KEY_EVT_NONE = 0,
	BSP_KEY_EVT_UP,
    BSP_KEY_EVT_DOWN,
    BSP_KEY_EVT_LEFT,
    BSP_KEY_EVT_RIGHT,
    BSP_KEY_EVT_CENTER,
    BSP_KEY_EVT_PUSHED,
    BSP_KEY_EVT_SELECT
};


/******************************************************************************
* FUNCTION PROTOTYPES
*/
extern void bspKeyInit(uint32_t ui32Mode);
extern uint32_t bspKeyPushed(uint32_t ui32ReadMask);
extern uint32_t bspKeyGetDir(void);
extern void bspKeyIntRegister(uint32_t ui32Keys, void (*pfnHandler)(void));
extern void bspKeyIntUnregister(uint32_t ui32Keys);
extern void bspKeyIntEnable(uint32_t ui32Keys);
extern void bspKeyIntDisable(uint32_t ui32Keys);
extern void bspKeyIntClear(uint32_t ui32Keys);


/******************************************************************************
* Mark the end of the C bindings section for C++ compilers.
******************************************************************************/
#ifdef __cplusplus
}
#endif
#endif /* #ifndef __BSP_KEY_H__ */
