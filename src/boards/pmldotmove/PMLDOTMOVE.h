/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       CC2650STK.h
 *
 *  @brief      CC2650SENSORTAG Board Specific header file.
 *
 *  NB! This is the board file for PCB versions 1.2 and 1.3
 *
 *  ============================================================================
 */
#ifndef __CC2650STK_SENSORTAG_BOARD_H__
#define __CC2650STK_SENSORTAG_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Same RF Configuration as 7x7 EM */
#define CC2650EM_5XD

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

/* Discrete outputs */
#define Board_LED_R              IOID_3
#define Board_LED_G              IOID_1
#define Board_LED_B              IOID_2
#define Board_LED_ON                1
#define Board_LED_OFF               0

/* Discrete inputs */
#define Board_BUTTON              IOID_0

/* Sensor outputs */
#define Board_MPU_INT               IOID_7
#define Board_TMP_RDY               IOID_4

/* Power control */
#define Board_MPU_POWER             IOID_10
#define Board_MPU_POWER_ON          1
#define Board_MPU_POWER_OFF         0

/* I2C */
#define Board_I2C0_SDA1             IOID_8
#define Board_I2C0_SCL1             IOID_9

/* SPI */
#define Board_SPI_FLASH_CS          IOID_11
#define Board_FLASH_CS_ON           0
#define Board_FLASH_CS_OFF          1

#define Board_SPI0_MISO             IOID_12
#define Board_SPI0_MOSI             IOID_13
#define Board_SPI0_CLK              IOID_14


/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC2650STK_I2C0
/* Generic PDM instance identifiers */
#define Board_PDM                   CC2650STK_PDM0
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC2650STK_SPI0
#define Board_SPI1                  CC2650STK_SPI1
/* Generic UART instance identifiers */
#define Board_UART                  CC2650STK_UART0


/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650STK_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum CC2650STK_I2CName {
    CC2650STK_I2C0 = 0,

    CC2650STK_I2CCOUNT
} CC2650STK_I2CName;

/*!
 *  @def    CC2650STK_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650STK_CryptoName {
    CC2650STK_CRYPTO0 = 0,

    CC2650STK_CRYPTOCOUNT
} CC2650STK_CryptoName;

/*!
 *  @def    CC2650STK_PdmName
 *  @brief  Enum of PDM names on the CC2650 dev board
 */
typedef enum CC2650STK_PDMName {
    CC2650STK_PDM0 = 0,
    CC2650STK_PDMCOUNT
} CC2650STK_PDMName;

/*!
 *  @def    CC2650STK_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650STK_SPIName {
    CC2650STK_SPI0 = 0,
    CC2650STK_SPI1,

    CC2650STK_SPICOUNT
} CC2650STK_SPIName;

/*!
 *  @def    CC2650STK_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650STK_UARTName {
    CC2650STK_UART0 = 0,

    CC2650STK_UARTCOUNT
} CC2650STK_UARTName;

/*!
 *  @def    CC2650STK_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650STK_UdmaName {
    CC2650STK_UDMA0 = 0,

    CC2650STK_UDMACOUNT
} CC2650STK_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2650STK_SENSORTAG_BOARD_H__ */
