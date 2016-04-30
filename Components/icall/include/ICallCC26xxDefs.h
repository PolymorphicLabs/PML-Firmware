/**
  @file  ICallCC26xxDefs.h
  @brief Indirect function Call dispatcher constant definitions specific
         to CC26xx platform.

         Note that the constants in this file is unique to the CC26xx
         and are not generic.

  <!--
  Copyright 2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ``AS IS'' WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
  -->
*/
#ifndef ICALLCC26XXDEFS_H
#define ICALLCC26XXDEFS_H

/* Note that this header file must not have dependency on actual TI-RTOS
 * header file because the TI-RTOS header file must not be included
 * when building a stack image. */


/* Power state transition enumeration */

/**
 * Power state transition to active power state
 * from standby power state.
 */
#define ICALL_PWR_AWAKE_FROM_STANDBY       0

/**
 * Power state transition to standby power state
 * from active power state.
 */
#define ICALL_PWR_ENTER_STANDBY            2

/**
 * Power state transition to shut down power state
 * from active power state
 */
#define ICALL_PWR_ENTER_SHUTDOWN           4

/**
 * Power state transition to active power state
 * where IOs can be accessed, from standby power state.
 */
#define ICALL_PWR_AWAKE_FROM_STANDBY_LATE  5


/**
 * Return value of ICall_pwrGetTransitionState().
 * Power state transition cannot be retrieved.
 */
#define ICALL_PWR_TRANSITION_UNKNOWN        0

/**
 * Return value of ICall_pwrGetTransitionState().
 * Power state transition not happening.
 */
#define ICALL_PWR_TRANSITION_STAY_IN_ACTIVE 1

/**
 * Return value of ICall_pwrGetTransitionState().
 * Transitioning into sleep.
 */
#define ICALL_PWR_TRANSITION_ENTERING_SLEEP 2

/**
 * Return value of ICall_pwrGetTransitionState().
 * Transitioning out of sleep.
 */
#define ICALL_PWR_TRANSITION_EXITING_SLEEP  3


/* Constraints and dependencies */

/**
 * A power API constraint flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_C_SB_VIMS_CACHE_RETAIN       0x00000001l

/**
 * A power API constraint flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_C_SD_DISALLOW                0x00000002l

/**
 * A power API constraint flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_C_SB_DISALLOW                0x00000004l

/**
 * A power API constraint flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_C_IDLE_PD_DISALLOW           0x00000008l

/**
 * A power API constraint flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_C_NEED_FLASH_IN_IDLE         0x00000010l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_GPT0                0x00000020l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_GPT1                0x00000040l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_GPT2                0x00000080l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_GPT3                0x00000100l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_SSI0                0x00000200l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_SSI1                0x00000400l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_UART0               0x00000800l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_I2C0                0x00001000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_TRNG                0x00002000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_GPIO                0x00004000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_UDMA                0x00008000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_CRYPTO              0x00010000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_I2S                 0x00020000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_PERIPH_RFCORE              0x00040000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_XOSC_HF                    0x00080000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_DOMAIN_PERIPH              0x00100000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_DOMAIN_SERIAL              0x00200000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_DOMAIN_RFCORE              0x00400000l

/**
 * A power API dependency flag that can be added to a bitmap used
 * as the argument to ICall_pwrConfigACAction(), ICall_pwrRequire()
 * and ICall_pwrDispense().
 */
#define ICALL_PWR_D_DOMAIN_SYSBUS              0x00800000l

#endif /* ICALLCC26XXDEFS_H */
