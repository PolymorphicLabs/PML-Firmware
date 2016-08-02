/******************************************************************************

 @file  sensortag_mov.c

 @brief This file contains the Movement Processor sub-application. It uses the
        MPU-9250 Wake-on-movement feature to allow the
        MPU to turn off the gyroscope and magnetometer when no activity is
        detected.

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

#ifndef EXCLUDE_MOV
/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "gatt.h"
#include "gattservapp.h"

#include "board.h"
#include "movementservice.h"
#include "sensortag_mov.h"
#include "SensorBno055.h"
#include "SensorTagTest.h"
#include "SensorUtil.h"
#include "util.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */
#define MOVEMENT_INACT_CYCLES   (MOVEMENT_INACT_TIMEOUT * (1000/sensorPeriod))

/*********************************************************************
 * CONSTANTS and MACROS
 */
// How often to perform sensor reads (milliseconds)
#define SENSOR_DEFAULT_PERIOD     1000

// Length of the data for this sensor
#define SENSOR_DATA1_LEN           MOVEMENT_DATA1_LEN
#define SENSOR_DATA2_LEN           MOVEMENT_DATA2_LEN
#define SENSOR_DATA3_LEN           MOVEMENT_DATA3_LEN

// Event flag for this sensor
#define SENSOR_EVT                ST_GYROSCOPE_SENSOR_EVT

// Movement task states
#define APP_STATE_ERROR           0xFF
#define APP_STATE_OFF             0
#define APP_STATE_IDLE            1
#define APP_STATE_ACTIVE          2

// Movement task configuration
#define MOVEMENT_INACT_TIMEOUT    10     // 10 seconds
#define GYR_SHAKE_THR             10.0
#define WOM_THR                   10

// Configuration bit-masks (bits 0-6 defined in sensor_mpu9250.h)
#define MOV_WOM_ENABLE            0x0080
#define MOV_MASK_WOM_THRESHOLD    0x3C00 // TBD
#define MOV_MASK_INACT_TIMEOUT    0xC000 // TBD

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/
struct bno055_t bno055;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct periodicClock;
static uint16_t sensorPeriod;
static volatile bool sensorReadScheduled;
static uint8_t sensorData1[SENSOR_DATA1_LEN];//3x int16_t accel x, y, z; 3x int16_t mag x, y, z; 3x int16_t gyro x, y, z
static uint8_t sensorData2[SENSOR_DATA2_LEN];
static uint8_t sensorData3[SENSOR_DATA3_LEN];

// Application state variables



//Bit 6 : Data3 Enable
//Bit 5 : Data2 Enable
//Bit 4 : Data1 Enable
//Bits 3:0 : Operating mode
//BNO Operating Mode
//BNO055_OPERATION_MODE_CONFIG
//BNO055_OPERATION_MODE_ACCONLY
//BNO055_OPERATION_MODE_MAGONLY
//BNO055_OPERATION_MODE_GYRONLY
//BNO055_OPERATION_MODE_ACCMAG
//BNO055_OPERATION_MODE_ACCGYRO
//BNO055_OPERATION_MODE_MAGGYRO
//BNO055_OPERATION_MODE_AMG
//BNO055_OPERATION_MODE_IMUPLUS
//BNO055_OPERATION_MODE_COMPASS
//BNO055_OPERATION_MODE_M4G
//BNO055_OPERATION_MODE_NDOF_FMC_OFF
//BNO055_OPERATION_MODE_NDOF
#define BNO055_OPERATION_MODE_M 0x0F
#define BNO055_DATA1_ENABLE     0x10
#define BNO055_DATA2_ENABLE     0x20
#define BNO055_DATA3_ENABLE     0x40
static uint8_t bnoConfig1;

static uint8_t bnoConfig2;

static uint16_t bnoAxisMap;

static uint8_t appState;
static volatile bool bnoDataRdy;
static uint32_t nActivity;
static uint8_t movThreshold;
static uint8_t mpuIntStatus;
static bool shakeDetected;
static uint8_t nMotions;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);
static void SensorTagMov_clockHandler(UArg arg);
static void appStateSet(uint8_t newState);
static void SensorTagMov_processInterrupt(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagMov_init
 *
 * @brief   Initialization function for the SensorTag movement sub-application
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_init(void)
{
  // Add service
  Movement_addService();

  // Register callbacks with profile
  Movement_registerAppCBs(&sensorCallbacks);

  // Initialize the module state variables
  bnoConfig1 = ST_CFG_SENSOR_DISABLE;
  bnoConfig2 = 0;
  bnoAxisMap = 0x24;
  sensorPeriod = SENSOR_DEFAULT_PERIOD;
  sensorReadScheduled = false;

  appState = APP_STATE_OFF;
  nMotions = 0;

  if(bno055_init(&bno055) == BNO055_SUCCESS)
  {

	  SensorTagMov_reset();
	  bno055_register_callback(SensorTagMov_processInterrupt);
  }

//  if (SensorMpu9250_init())
//  {
//    SensorTagMov_reset();
//    SensorMpu9250_registerCallback(SensorTagMov_processInterrupt);
//  }

  // Initialize characteristics
  initCharacteristicValue(SENSOR_PERI,
                          SENSOR_DEFAULT_PERIOD / SENSOR_PERIOD_RESOLUTION,
                          sizeof(uint8_t));

  // Create continuous clock for internal periodic events.
  Util_constructClock(&periodicClock, SensorTagMov_clockHandler,
                      1000, sensorPeriod, false, 0);
}

/*********************************************************************
 * @fn      SensorTagMov_processSensorEvent
 *
 * @brief   SensorTag Movement sensor event processor.
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_processSensorEvent(void)
{
  //Are we scheduled to read data?
  if (sensorReadScheduled)
  {

	//Read the data
	switch(bnoConfig1 & BNO055_OPERATION_MODE_M){

		case BNO055_OPERATION_MODE_ACCONLY:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
			}
			break;
		case BNO055_OPERATION_MODE_MAGONLY:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
			}
			break;
		case BNO055_OPERATION_MODE_GYRONLY:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_ACCMAG:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
			}
			break;
		case BNO055_OPERATION_MODE_ACCGYRO:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_MAGGYRO:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_AMG:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_IMUPLUS:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			if(bnoConfig1 & BNO055_DATA2_ENABLE){
				bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
			}
			if(bnoConfig1 & BNO055_DATA3_ENABLE){
				bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
				bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
			}
			break;
		case BNO055_OPERATION_MODE_COMPASS:
		case BNO055_OPERATION_MODE_M4G:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
			}
			if(bnoConfig1 & BNO055_DATA2_ENABLE){
				bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
			}
			if(bnoConfig1 & BNO055_DATA3_ENABLE){
				bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
				bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
			}
			break;
		case BNO055_OPERATION_MODE_NDOF_FMC_OFF:
		case BNO055_OPERATION_MODE_NDOF:
			if(bnoConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			if(bnoConfig1 & BNO055_DATA2_ENABLE){
				bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
			}
			if(bnoConfig1 & BNO055_DATA3_ENABLE){
				bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
				bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
			}
			break;

		default:
		case BNO055_OPERATION_MODE_CONFIG:
			break;
	}

	//Send the data
	if(bnoConfig1 & BNO055_DATA1_ENABLE){
		Movement_setParameter(SENSOR_DATA1, SENSOR_DATA1_LEN, sensorData1);
	}
	if(bnoConfig1 & BNO055_DATA2_ENABLE){
		Movement_setParameter(SENSOR_DATA2, SENSOR_DATA2_LEN, sensorData2);
	}
	if(bnoConfig1 & BNO055_DATA3_ENABLE){
		Movement_setParameter(SENSOR_DATA3, SENSOR_DATA3_LEN, sensorData3);
	}


    sensorReadScheduled = false;
  }
}

/*********************************************************************
 * @fn      SensorTagMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 * @param   paramID - identifies which characteristic has changed
 *
 * @return  none
 */
void SensorTagMov_processCharChangeEvt(uint8_t paramID)
{
  uint16_t newCfg;
  uint8_t newValue8;

  switch (paramID)
  {
  case SENSOR_CONF1:
    if ((SensorTag_testResult() & SENSOR_MOV_TEST_BM) == 0)
    {
      bnoConfig1 = ST_CFG_ERROR;
    }

    if (bnoConfig1 != ST_CFG_ERROR)
    {
      Movement_getParameter(SENSOR_CONF1, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        bnoConfig1 = newCfg;
        appStateSet(APP_STATE_OFF);
      }
      else
      {
        // Some axes on; power up and activate MPU
        bnoConfig1 = newCfg;
        appStateSet(APP_STATE_ACTIVE);
//        if (SensorMpu9250_powerIsOn())
//        {
//          DELAY_MS(5);
//          mpuConfig = newCfg | (SensorMpu9250_accReadRange() << 8);
//        }
      }

      Movement_setParameter(SENSOR_CONF1, sizeof(bnoConfig1), (uint8_t*)&bnoConfig1);
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF1, bnoConfig1, sizeof(bnoConfig1));
    }

    // Data initially zero
    initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
    initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
    initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
    break;

  case SENSOR_CONF2:
    if ((SensorTag_testResult() & SENSOR_MOV_TEST_BM) == 0)
    {
      bnoConfig2 = ST_CFG_ERROR;
    }

    if (bnoConfig2 != ST_CFG_ERROR)
    {
      Movement_getParameter(SENSOR_CONF2, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        bnoConfig2 = newCfg;
//        appStateSet(APP_STATE_OFF);
      }
      else
      {
        // Some axes on; power up and activate MPU
        bnoConfig2 = newCfg;
//        appStateSet(APP_STATE_ACTIVE);

      }

      Movement_setParameter(SENSOR_CONF2, sizeof(bnoConfig2), (uint8_t*)&bnoConfig2);
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF2, bnoConfig2, sizeof(bnoConfig2));
    }

    // Data initially zero
    initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
    initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
    initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
    break;

  case SENSOR_AXISMAP:
	  Movement_getParameter(SENSOR_AXISMAP, &newCfg);
	  if(newCfg != bnoAxisMap)
	  {
		  bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
		  bno055_set_axis_remap_value(newCfg & 0x3F);
		  bno055_set_remap_x_sign((newCfg & 0x0040) >> 6);
		  bno055_set_remap_y_sign((newCfg & 0x0080) >> 7);
		  bno055_set_remap_z_sign((newCfg & 0x0010) >> 8);
		  bno055_set_operation_mode(bnoConfig1 & BNO055_OPERATION_MODE_M);
		  bnoAxisMap = newCfg;
	  }
	  break;

  case SENSOR_PERI:
    Movement_getParameter(SENSOR_PERI, &newValue8);
    sensorPeriod = newValue8 * SENSOR_PERIOD_RESOLUTION;
    Util_rescheduleClock(&periodicClock,sensorPeriod);
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagMov_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_reset(void)
{
  initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
  initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
  initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
  bnoConfig1 = 0;
  bnoConfig2 = 0;
  Movement_setParameter(SENSOR_CONF1, sizeof(bnoConfig1), (uint8_t*)&bnoConfig1);

  // Remove power from the BNO
  appStateSet(APP_STATE_OFF);
}


/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      SensorTagMov_processInterrupt
 *
 * @brief   Interrupt handler for BNO
 *
 * @param   none
 *
 * @return  none
 */
static void SensorTagMov_processInterrupt(void)
{
  // Wake up the application thread
//  bnoDataRdy = true;
//  sensorReadScheduled = true;
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SensorTagMov_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - not used
 *
 * @return  none
 */
static void SensorTagMov_clockHandler(UArg arg)
{
  // Schedule readout periodically
  sensorReadScheduled = true;
  Semaphore_post(sem);
}


/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from Movement Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_MOV, paramID);
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
  memset(sensorData1,value,paramLen);
  memset(sensorData2,value,paramLen);
  memset(sensorData3,value,paramLen);
  Movement_setParameter(paramID, paramLen, sensorData1);
  Movement_setParameter(paramID, paramLen, sensorData2);
  Movement_setParameter(paramID, paramLen, sensorData3);
}

/*******************************************************************************
 * @fn      appStateSet
 *
 * @brief   Set the application state
 *
 */
static void appStateSet(uint8_t newState)
{
  if (newState == APP_STATE_OFF)
  {
    appState = APP_STATE_OFF;

    bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);

//    SensorMpu9250_enable(0);
//    SensorMpu9250_powerOff();

    // Stop scheduled data measurements
    Util_stopClock(&periodicClock);
  }

  if (newState == APP_STATE_ACTIVE || newState == APP_STATE_IDLE)
  {
    appState = APP_STATE_ACTIVE;
    nActivity = MOVEMENT_INACT_CYCLES;
    movThreshold = WOM_THR;
    mpuIntStatus = 0;
    shakeDetected = false;
    bnoDataRdy = false;

//    SensorMpu9250_powerOn();
//    SensorMpu9250_enable(mpuConfig & 0xFF);

    //Reset Sensor, after reset device comes up in normal mode
    bno055_set_sys_rst(1);
    DELAY_MS(650);
    //Set axis mapping
    bno055_set_operation_mode(bnoConfig1 & BNO055_OPERATION_MODE_M);




    if (newState == APP_STATE_ACTIVE)
    {
      // Start scheduled data measurements
      Util_startClock(&periodicClock);
    }
    else
    {
      // Stop scheduled data measurements
      Util_stopClock(&periodicClock);
    }
  }
}
#endif // EXCLUDE_MOV

/*********************************************************************
*********************************************************************/

