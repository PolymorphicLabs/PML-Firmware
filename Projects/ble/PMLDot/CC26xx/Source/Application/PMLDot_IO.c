/*******************************************************************************
  Filename:       PMLDot_IO.c
  Revised:        $Date: 2013-08-23 20:45:31 +0200 (fr, 23 aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    This file contains the Sensor Tag sample application,
                  Input/Output control.

  Copyright 2014  Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "PMLDot_IO.h"
#include "ioservice.h"

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Queue.h> // Needed for util.h

#include "Board.h"
#include "peripheral.h"
#include "sensor.h"
#include "util.h"
#include "sensor_mpu9250.h"
#include "ext_flash.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define IO_DATA_LED_R           0x01
#define IO_DATA_LED_G           0x02
#define IO_DATA_LED_B           0x04
#define IO_DATA_MPU_POWER       0x08
//#define IO_DATA_MIC_POWER       0x10
#define IO_DATA_FLASH_POWER     0x20

#define COLOR_PERIOD_10HZ      100
#define COLOR_PERIOD_20HZ      50
#define COLOR_PERIOD_50HZ      20
#define COLOR_PERIOD_100HZ     10
#define COLOR_PERIOD_200HZ     5
#define COLOR_PERIOD_500HZ     2
#define COLOR_PERIOD_1000HZ    1

#define COLOR_PERIOD           COLOR_PERIOD_500HZ
   
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
static uint8_t ioMode;
static uint8_t ioValue[3];

// Clock object used for buzzer (interim solution)
static Clock_Struct colorClockStruct;
static Clock_Handle colorClockHandle;

static uint8_t redLed = 0;
static uint8_t greenLed = 0;
static uint8_t blueLed = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void ioChangeCB(uint8_t newParamID);
static void initColorTimer(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorTag_ioCBs =
{
  ioChangeCB,               // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      PMLDotIO_init
 *
 * @brief   Initialization function for the PMLDot IO
 *
 */
void PMLDotIO_init(void)
{
  // Add service
  Io_addService();
  Io_registerAppCBs(&sensorTag_ioCBs);

  // Initialize the module state variables
  ioMode = IO_MODE_LOCAL;
  ioValue[0] = 0;
  ioValue[1] = 0;
  ioValue[2] = 0;

  // Initialize timer for buzzer (PWM)
  initColorTimer();

  // Set internal state
  PMLDotIO_reset();
}


/*********************************************************************
 * @fn      PMLDotIO_processCharChangeEvt
 *
 * @brief   Process a change in the IO characteristics
 *
 * @return  none
 */
void PMLDotIO_processCharChangeEvt(uint8_t paramID)
{ 
  if( paramID == SENSOR_CONF )
  {
    
    Io_getParameter(SENSOR_CONF, &ioMode);
    if (ioMode == IO_MODE_SELFTEST)
    {
      ioValue[0] = sensorTestResult();
      Io_setParameter(SENSOR_DATA, 1, &ioValue);
    }
    else 
    {
      // Mode change: make sure LEDs and buzzer are off
      Io_setParameter(SENSOR_DATA, 1, &ioValue);
      
      PIN_setOutputValue(hGpioPin, Board_LED_R, Board_LED_OFF);
      PIN_setOutputValue(hGpioPin, Board_LED_G, Board_LED_OFF);
      PIN_setOutputValue(hGpioPin, Board_LED_B, Board_LED_OFF);
      Clock_stop(colorClockHandle);
//      PIN_setOutputValue(hGpioPin, Board_BUZZER, Board_BUZZER_OFF);
    }
  } 
  else if (paramID == SENSOR_DATA)
  {
    Io_getParameter(SENSOR_DATA, ioValue);
  }
  
  if (ioMode == IO_MODE_REMOTE)
  {
	  redLed = ioValue[0];
	  greenLed = ioValue[1];
	  blueLed = ioValue[2];

	  Clock_start(colorClockHandle);

//     Control by remote client:
//     - possible to operate the LEDs and buzzer
//     - right key functionality overridden (will not terminate connection)
//    if (!!(ioValue & IO_DATA_LED_R))
//    {
//      PIN_setOutputValue(hGpioPin, Board_LED_R, Board_LED_ON);
//    }
//    else
//    {
//      PIN_setOutputValue(hGpioPin, Board_LED_R, Board_LED_OFF);
//    }
//
//    if (!!(ioValue & IO_DATA_LED_G))
//    {
//      PIN_setOutputValue(hGpioPin, Board_LED_G, Board_LED_ON);
//    }
//    else
//    {
//      PIN_setOutputValue(hGpioPin, Board_LED_G, Board_LED_OFF);
//    }
//
//    if (!!(ioValue & IO_DATA_LED_B))
//    {
//      PIN_setOutputValue(hGpioPin, Board_LED_B, Board_LED_ON);
//    }
//    else
//    {
//      PIN_setOutputValue(hGpioPin, Board_LED_B, Board_LED_OFF);
//    }


  }
}

/*********************************************************************
 * @fn      PMLDotIO_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void PMLDotIO_reset(void)
{
  ioValue[0] = sensorTestResult();
  Io_setParameter( SENSOR_DATA, 1, &ioValue);

  ioMode = IO_MODE_LOCAL;
  Io_setParameter( SENSOR_CONF, 1, &ioMode);
  
  // Normal mode; make sure LEDs and buzzer are off
  PIN_setOutputValue(hGpioPin, Board_LED_R, Board_LED_OFF);
  PIN_setOutputValue(hGpioPin, Board_LED_G, Board_LED_OFF);
  PIN_setOutputValue(hGpioPin, Board_LED_B, Board_LED_OFF);
}


/*********************************************************************
 * @fn      PMLDotIO_GetIoMode
 *
 * @brief   Get the current IO mode
 *
 */
uint8_t PMLDotIO_GetMode( void )
{
  return ioMode;
}


/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      ioChangeCB
 *
 * @brief   Callback from IO service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void ioChangeCB( uint8_t paramID )
{
  // Wake up the application thread
  PMLDot_charValueChangeCB(SERVICE_ID_IO, paramID);
}

/*********************************************************************
 * @fn      timerIsr
 *
 * @brief   Interrupt service routine for buzzer.
 *
 * @param   none
 *
 * @return  none
 */
static void timerIsr(UArg arg0)
{
//  uint32_t v;

//  v = PIN_getOutputValue(Board_BUZZER);
//  PIN_setOutputValue(hGpioPin, Board_BUZZER, !v);

	static uint8_t colorCount = 0;

	if(redLed < colorCount)
		PIN_setOutputValue(hGpioPin, Board_LED_R, Board_LED_ON);
	else

		PIN_setOutputValue(hGpioPin, Board_LED_R, Board_LED_OFF);



	if(greenLed < colorCount)
		PIN_setOutputValue(hGpioPin, Board_LED_G, Board_LED_ON);
	else

		PIN_setOutputValue(hGpioPin, Board_LED_G, Board_LED_OFF);



	if(blueLed < colorCount)
		PIN_setOutputValue(hGpioPin, Board_LED_B, Board_LED_ON);
	else

		PIN_setOutputValue(hGpioPin, Board_LED_B, Board_LED_OFF);

	colorCount++;


}

/*********************************************************************
 * @fn      initBuzzTimer
 *
 * @brief   Set up timer for buzz generation (use PWM when RTOS driver in place)
 *
 * @param   none
 *
 * @return  none
 */
static void initColorTimer(void)
{
  Clock_Params clockParams;

  // Setup parameters.
  Clock_Params_init(&clockParams);

  // Setup argument.
  clockParams.arg = 0;

  // Period
  clockParams.period = COLOR_PERIOD;

  // Do not start until called.
  clockParams.startFlag = false;

  // Initialize clock instance.
  Clock_construct(&colorClockStruct, timerIsr, COLOR_PERIOD, &clockParams);
  colorClockHandle = Clock_handle(&colorClockStruct);
}


/*********************************************************************
*********************************************************************/

