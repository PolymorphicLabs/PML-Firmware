/*******************************************************************************
  Filename:       PMLDot_AHRS.c
  Revised:        $Date: 2013-11-06 17:27:44 +0100 (on, 06 nov 2013) $
  Revision:       $Revision: 35922 $

  Description:    This file contains the PMLDot sample application,
                  AHRS part, for use with the TI Bluetooth Low
                  Energy Protocol Stack.

  Copyright 2015  Texas Instruments Incorporated. All rights reserved.

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
//#include <math.h>
#include "gatt.h"
#include "gattservapp.h"
#include "Board.h"
#include "string.h"

#include "AHRSservice.h"
#include "PMLDot_AHRS.h"
#include "MadgwickAHRS.h"
#include "PMLDot.h"
#include "sensor.h"

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

/*********************************************************************
 * MACROS
 */

#define PI 3.141592F

/*********************************************************************
 * CONSTANTS
 */

// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD   100

// Time start measurement and data ready
#define AHRS_DELAY_PERIOD          15

// Length of the data for this sensor
#define SENSOR_DATA_LEN         AHRS_DATA_LEN

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern float gyroX, gyroY, gyroZ;
extern float accelX, accelY, accelZ;
extern float magX, magY, magZ;

//Motion Data Mailbox
//extern Mailbox_Handle motionMailbox;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sensorSem;

// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

// Parameters
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTaskFxn(UArg a0, UArg a1);
static void sensorConfigChangeCB( uint8_t paramID);
static void initCharacteristicValue( uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */
/*********************************************************************
 * @fn      PMLDotAHRS_createTask
 *
 * @brief   Task creation function for the PMLDot
 *
 * @param   none
 *
 * @return  none
 */
void PMLDotAHRS_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

/*********************************************************************
 * @fn      PMLDotAHRS_processCharChangeEvt
 *
 * @brief   PMLDot AHRS event handling
 *
 */
void PMLDotAHRS_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;

  switch (paramID)
  {
  case  SENSOR_CONF:
	  //No need for self test since we aren't a sensor
//    if ((sensorTestResult() & ST_AHRS) == 0 )
//    {
//      sensorConfig = ST_CFG_ERROR;
//    }

    if (sensorConfig != ST_CFG_ERROR)
    {
      AHRS_getParameter( SENSOR_CONF, &newValue);

      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Reset characteristics
        initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);

        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);
      }
      else
      {
        // Activate task
        Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
      }

      sensorConfig = newValue;
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF, sensorConfig, sizeof ( uint8_t ));
    }
    break;

  case SENSOR_PERI:
    AHRS_getParameter( SENSOR_PERI, &newValue);
    sensorPeriod = newValue * SENSOR_PERIOD_RESOLUTION;
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      PMLDotAHRS_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void PMLDotAHRS_reset (void)
{
  sensorConfig = ST_CFG_SENSOR_DISABLE;
  initCharacteristicValue(SENSOR_DATA, 0, SENSOR_DATA_LEN);
  initCharacteristicValue(SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof(uint8_t));
}

/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      sensorTaskInit
 *
 * @brief   Initialization function for the PMLDot AHRS sensor
 *
 */
static void sensorTaskInit(void)
{
  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  // Add service
  AHRS_addService();

  // Register callbacks with profile
  AHRS_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  PMLDotAHRS_reset();
  initCharacteristicValue(SENSOR_PERI, SENSOR_DEFAULT_PERIOD
                          / SENSOR_PERIOD_RESOLUTION, sizeof ( uint8_t ));

  // Initialize the driver
//  sensorHdc1000Init();
}

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the AHRS readout task
 *
 * @return  none
 */
static void sensorTaskFxn(UArg a0, UArg a1)
{
//  typedef union {
//    struct {
//      float gyroX, gyroY, gyroZ;
//      float accelX, accelY, accelZ;
//      float magX, magY, magZ;
//    } v;
//    uint8_t a[SENSOR_DATA_LEN];
//  } Data_t;



  // Initialize the task
  sensorTaskInit();

  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);

  //Init Madgwick filter
  MadgwickAHRSinit();

  // Task loop
  while (true)
  {
    if (sensorConfig == ST_CFG_SENSOR_ENABLE)
    {
//      Data_t data;

      // 1. Start temperature measurement
//      sensorHdc1000Start();
//      delay_ms(AHRS_DELAY_PERIOD);
//
//      // 2. Read data
//      sensorHdc1000Read(&data.v.rawTemp, &data.v.rawAHRS);

      //TODO: Calculate AHRS Data
//      Mailbox_pend(motionMailbox, )
      //Convert to radians/s
//      gyroX = gyroX * (PI / 180);
//      gyroY = gyroY * (PI / 180);
//      gyroZ = gyroZ * (PI / 180);
//      MadgwickAHRSupdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);

      // 3. Send data
      AHRS_setParameter( SENSOR_DATA, SENSOR_DATA_LEN, (void *)&qData);

      // 4. Wait until next cycle
      delay_ms(sensorPeriod);
    }
    else
    {
      delay_ms(SENSOR_DEFAULT_PERIOD);
    }
  }
}

/*********************************************************************
 * @fn      sensorConfigChangeCB
 *
 * @brief   Callback from AHRS Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  PMLDot_charValueChangeCB(SERVICE_ID_AHRS, paramID);
}

/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  uint8_t data[SENSOR_DATA_LEN];

  memset(data,value,paramLen);
  AHRS_setParameter( paramID, paramLen, data);
}

/*********************************************************************
*********************************************************************/

