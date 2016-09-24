/******************************************************************************

 @file  sensortag_keys.c

 @brief This file contains the Sensor Tag sample application,
        Keys part, for use with the TI Bluetooth Low
        Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2015-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_00_31
 Release Date: 2016-06-16 18:57:29
 *****************************************************************************/

#ifndef EXCLUDE_KEYS

/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "sensortag_keys.h"
#include "sensortag_io.h"
#include "ioservice.h"
#include "sensortag_factoryreset.h"
#include "board.h"
#include "peripheral.h"
#include "simplekeys.h"
#include "sensortag_audio.h"
#include "util.h"

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

/*********************************************************************
 * MACROS
 */
// Adaptation for LaunchPad
#ifndef Board_BUTTON
#define Board_BUTTON          Board_BUTTON
#endif

/*********************************************************************
 * CONSTANTS
 */
#define SK_PUSH_KEYS            (SK_KEY_RIGHT)

// Key press time-outs (seconds)
#define POWER_PRESS_PERIOD      3
#define RESET_PRESS_PERIOD      6

// Events
#define SK_EVT_FACTORY_RESET    0x01
#define SK_EVT_DISCONNECT       0x02

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t keys;
static uint16_t keyTimer;
static Clock_Struct periodicClock;
static uint8_t event;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processGapStateChange(void);
static void SensorTagKeys_clockHandler(UArg arg);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagKeys_init
 *
 * @brief   Initialization function for the SensorTag keys
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_init(void)
{
  // Add service
  SK_AddService(GATT_ALL_SERVICES);

  // Initialize the module state variables
  SensorTagKeys_reset();

  // Create one-shot clock for key press timing (tick per second)
  Util_constructClock(&periodicClock, SensorTagKeys_clockHandler,
                      100, 1000, false, 0);
}

/*********************************************************************
 * @fn      SensorTagKeys_processKeyRight
 *
 * @brief   Interrupt handler for BUTTON 1(right)
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_processButton(void)
{
  if (PIN_getInputValue(Board_BUTTON))
  {
    keys &= ~SK_KEY_RIGHT;
  }
  else
  {
    keys |= SK_KEY_RIGHT;
  }

  // Wake up the application thread
  Semaphore_post(sem);
}




/*********************************************************************
 * @fn      SensorTagKeys_processEvent
 *
 * @brief   SensorTag Keys event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_processEvent(void)
{
  static uint8_t current_keys = 0;

  // Factory reset by six second simultaneous key press
  if (event & SK_EVT_FACTORY_RESET)
  {
      event &= ~SK_EVT_FACTORY_RESET;

      // Indicate that we're entering factory reset
      SensorTagIO_blinkLed(IOID_RED_LED, 10);

      // Apply factory image and reboot
      SensorTagFactoryReset_applyFactoryImage();
  }

  // Disconnect on three seconds press on the power switch (right key)
  if (event & SK_EVT_DISCONNECT)
  {
      event &= ~SK_EVT_DISCONNECT;
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        processGapStateChange();
      }
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  if (current_keys != keys)
  {
    SK_SetParameter(SK_KEY_ATTR, sizeof(uint8_t), &keys);

    // Insert key state into advertising data
    if (gapProfileState == GAPROLE_ADVERTISING)
    {
      SensorTag_updateAdvertisingData(keys);
    }

    // Check if right key was pressed
    if ((current_keys & SK_KEY_RIGHT)!=0 && (keys & SK_KEY_RIGHT)==0)
    {
      if (gapProfileState != GAPROLE_CONNECTED)
      {
        // Not connected; change state immediately (power/right button)
        processGapStateChange();
      }
    }


    // Has a key been pressed ?
    if ((keys & SK_PUSH_KEYS) && (current_keys == 0))
    {
        if (!Util_isActive(&periodicClock))
        {
            Util_startClock(&periodicClock);
            keyTimer = 0;
        }
    }
  }

  current_keys = keys;
}

/*********************************************************************
 * @fn      SensorTagKeys_reset
 *
 * @brief   Reset key state to 'not pressed'
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagKeys_reset(void)
{
  keyTimer = 0;
  keys = 0;
  event = 0;
  SK_SetParameter(SK_KEY_ATTR, sizeof(uint8_t), &keys);
}

/*********************************************************************
 * @fn      SensorTagKeys_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void SensorTagKeys_clockHandler(UArg arg)
{
    // Are both keys pressed?
    if (keys & SK_KEY_RIGHT)
    {
        keyTimer++;
    }
    else
    {
        keyTimer = 0;
    }



    // Both keys have been pressed for 6 seconds -> restore factory image
    if (keyTimer >= RESET_PRESS_PERIOD )
    {
        // Stop the clock
        if (Util_isActive(&periodicClock))
        {
            Util_stopClock(&periodicClock);
            keyTimer = 0;

            // set event flag and wake up the application thread
            event |= SK_EVT_FACTORY_RESET;
            Semaphore_post(sem);
        }
    }
//    // Right key (POWER) pressed for three seconds, disconnect if connected
//    else if (keyRightTimer >= POWER_PRESS_PERIOD && keyLeftTimer == 0)
//    {
//        // Stop the clock
//        if (Util_isActive(&periodicClock))
//        {
//            Util_stopClock(&periodicClock);
//            keyRightTimer = 0;
//
//            // set event flag and wake up the application thread
//            event |= SK_EVT_DISCONNECT;
//            Semaphore_post(sem);
//        }
//    }
    else if (keyTimer == 0)
    {
        // Stop the clock
        if (Util_isActive(&periodicClock))
        {
            Util_stopClock(&periodicClock);
        }
    }
}

/*********************************************************************
 * @fn      processGapStateChange
 *
 * @brief   Change the GAP state.
 *          1. Connected -> disconnect and start advertising
 *          2. Advertising -> stop advertising
 *          3. Disconnected/not advertising -> start advertising
 *
 * @param   none
 *
 * @return  none
 */
static void processGapStateChange(void)
{
  if (gapProfileState != GAPROLE_CONNECTED)
  {
    uint8_t current_adv_enabled_status;
    uint8_t new_adv_enabled_status;

    // Find the current GAP advertising status
    GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);

    if (current_adv_enabled_status == FALSE)
    {
      new_adv_enabled_status = TRUE;
    }
    else
    {
      new_adv_enabled_status = FALSE;
    }

    // Change the GAP advertisement status to opposite of current status
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &new_adv_enabled_status);
  }

  if (gapProfileState == GAPROLE_CONNECTED)
  {
    uint8_t adv_enabled = TRUE;

    // Disconnect
    GAPRole_TerminateConnection();

    // Start advertising
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &adv_enabled);
  }
}
#endif // EXCLUDE_KEYS

/*********************************************************************
*********************************************************************/

