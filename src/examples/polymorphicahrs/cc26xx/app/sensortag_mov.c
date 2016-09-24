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
#include <ti/sysbios/knl/Task.h>

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

// Time start measurement and data ready
#define MOV_DELAY_PERIOD        15

// Length of the data for this sensor
#define SENSOR_DATA1_LEN           MOVEMENT_DATA1_LEN
#define SENSOR_DATA2_LEN           MOVEMENT_DATA2_LEN
#define SENSOR_DATA3_LEN           MOVEMENT_DATA3_LEN
#define SENSOR_CONF1_LEN		   MOVEMENT_CONF1_LEN
#define SENSOR_CONF2_LEN		   MOVEMENT_CONF2_LEN
#define SENSOR_AXISMAP_LEN		   MOVEMENT_AXISMAP_LEN

// Event flag for this sensor
#define SENSOR_EVT                ST_GYROSCOPE_SENSOR_EVT

// Movement task states
#define APP_STATE_ERROR           0xFF
#define APP_STATE_OFF             0
#define APP_STATE_IDLE            1
#define APP_STATE_ACTIVE          2

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600

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
 // Entity ID globally used to check for source and/or destination of messages
 static ICall_EntityID sensorSelfEntity;

 // Semaphore globally used to post events to the application thread
 static ICall_Semaphore sensorSem;

 // Task setup
 static Task_Struct sensorTask;
 static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];


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
static uint8_t sensorConfig1;

static uint8_t sensorConfig2;

static uint16_t sensorAxisMap;

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

static void sensorTaskFxn(UArg a0, UArg a1);
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
 * @fn      SensorTagHum_createTask
 *
 * @brief   Task creation function for the SensorTag
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagMov_createTask(void)
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
  sensorConfig1 = ST_CFG_SENSOR_DISABLE;
  sensorConfig2 = 0;
  sensorAxisMap = 0x24;
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
//  Util_constructClock(&periodicClock, SensorTagMov_clockHandler,
//                      1000, sensorPeriod, false, 0);
}

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the humidity readout task
 *
 * @param   a0 - not used
 *
 * @param   a1 - not used
 *
 * @return  none
 */
static void sensorTaskFxn(UArg a0, UArg a1)
{

  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  // Deactivate task (active only when measurement is enabled)
  Task_setPri(Task_handle(&sensorTask), -1);

  // Task loop
  while (true)
  {
    if (sensorConfig1 != ST_CFG_SENSOR_DISABLE)
    {

		//Read the data
		switch(sensorConfig1 & BNO055_OPERATION_MODE_M){

			case BNO055_OPERATION_MODE_ACCONLY:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				}
				break;
			case BNO055_OPERATION_MODE_MAGONLY:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				}
				break;
			case BNO055_OPERATION_MODE_GYRONLY:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
				}
				break;
			case BNO055_OPERATION_MODE_ACCMAG:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
					bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				}
				break;
			case BNO055_OPERATION_MODE_ACCGYRO:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
					bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
				}
				break;
			case BNO055_OPERATION_MODE_MAGGYRO:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
					bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
				}
				break;
			case BNO055_OPERATION_MODE_AMG:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
					bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
					bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
				}
				break;
			case BNO055_OPERATION_MODE_IMUPLUS:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
					bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
				}
				if(sensorConfig1 & BNO055_DATA2_ENABLE){
					bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
				}
				if(sensorConfig1 & BNO055_DATA3_ENABLE){
					bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
					bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
				}
				break;
			case BNO055_OPERATION_MODE_COMPASS:
			case BNO055_OPERATION_MODE_M4G:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
					bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				}
				if(sensorConfig1 & BNO055_DATA2_ENABLE){
					bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
				}
				if(sensorConfig1 & BNO055_DATA3_ENABLE){
					bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
					bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
				}
				break;
			case BNO055_OPERATION_MODE_NDOF_FMC_OFF:
			case BNO055_OPERATION_MODE_NDOF:
				if(sensorConfig1 & BNO055_DATA1_ENABLE){
					bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
					bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
					bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
				}
				if(sensorConfig1 & BNO055_DATA2_ENABLE){
					bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
				}
				if(sensorConfig1 & BNO055_DATA3_ENABLE){
					bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
					bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
				}
				break;

			default:
			case BNO055_OPERATION_MODE_CONFIG:
				break;
		}

		//Send the data
		if(sensorConfig1 & BNO055_DATA1_ENABLE){
			Movement_setParameter(SENSOR_DATA1, SENSOR_DATA1_LEN, sensorData1);
		}
		if(sensorConfig1 & BNO055_DATA2_ENABLE){
			Movement_setParameter(SENSOR_DATA2, SENSOR_DATA2_LEN, sensorData2);
		}
		if(sensorConfig1 & BNO055_DATA3_ENABLE){
			Movement_setParameter(SENSOR_DATA3, SENSOR_DATA3_LEN, sensorData3);
		}

	    // 4. Wait until next cycle
	    DELAY_MS(sensorPeriod - MOV_DELAY_PERIOD);
    }
    else
    {
      DELAY_MS(SENSOR_DEFAULT_PERIOD);
    }
  }
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
//  if (sensorReadScheduled)
//  {

	//Read the data
	switch(sensorConfig1 & BNO055_OPERATION_MODE_M){

		case BNO055_OPERATION_MODE_ACCONLY:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
			}
			break;
		case BNO055_OPERATION_MODE_MAGONLY:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
			}
			break;
		case BNO055_OPERATION_MODE_GYRONLY:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_ACCMAG:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
			}
			break;
		case BNO055_OPERATION_MODE_ACCGYRO:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_MAGGYRO:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_AMG:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			break;
		case BNO055_OPERATION_MODE_IMUPLUS:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			if(sensorConfig1 & BNO055_DATA2_ENABLE){
				bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
			}
			if(sensorConfig1 & BNO055_DATA3_ENABLE){
				bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
				bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
			}
			break;
		case BNO055_OPERATION_MODE_COMPASS:
		case BNO055_OPERATION_MODE_M4G:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
			}
			if(sensorConfig1 & BNO055_DATA2_ENABLE){
				bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
			}
			if(sensorConfig1 & BNO055_DATA3_ENABLE){
				bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
				bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
			}
			break;
		case BNO055_OPERATION_MODE_NDOF_FMC_OFF:
		case BNO055_OPERATION_MODE_NDOF:
			if(sensorConfig1 & BNO055_DATA1_ENABLE){
				bno055_read_accel_xyz((struct bno055_accel_t *)&sensorData1[0]);
				bno055_read_mag_xyz((struct bno055_mag_t *)&sensorData1[6]);
				bno055_read_gyro_xyz((struct bno055_gyro_t *)&sensorData1[12]);
			}
			if(sensorConfig1 & BNO055_DATA2_ENABLE){
				bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
			}
			if(sensorConfig1 & BNO055_DATA3_ENABLE){
				bno055_read_linear_accel_xyz((struct bno055_linear_accel_t *)&sensorData3[0]);
				bno055_read_gravity_xyz((struct bno055_gravity_t *)&sensorData3[6]);
			}
			break;

		default:
		case BNO055_OPERATION_MODE_CONFIG:
			break;
	}

	//Send the data
	if(sensorConfig1 & BNO055_DATA1_ENABLE){
		Movement_setParameter(SENSOR_DATA1, SENSOR_DATA1_LEN, sensorData1);
	}
	if(sensorConfig1 & BNO055_DATA2_ENABLE){
		Movement_setParameter(SENSOR_DATA2, SENSOR_DATA2_LEN, sensorData2);
	}
	if(sensorConfig1 & BNO055_DATA3_ENABLE){
		Movement_setParameter(SENSOR_DATA3, SENSOR_DATA3_LEN, sensorData3);
	}


//    sensorReadScheduled = false;
//  }
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
  volatile uint8_t test;

  switch (paramID)
  {
  case SENSOR_CONF1:
    if ((SensorTag_testResult() & SENSOR_MOV_TEST_BM) == 0)
    {
      sensorConfig1 = ST_CFG_ERROR;
    }

    if (sensorConfig1 != ST_CFG_ERROR)
    {
      Movement_getParameter(SENSOR_CONF1, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        sensorConfig1 = newCfg;
        Movement_setParameter(SENSOR_CONF1, sizeof(sensorConfig1), (uint8_t*)&sensorConfig1);
        appStateSet(APP_STATE_OFF);

        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);
      }
      else
      {
        // Some axes on; power up and activate BNO
        sensorConfig1 = newCfg;

        if(sensorConfig1 == 0x0C){
        	asm(" NOP");
        	test++;
        }

        if(sensorConfig1 == 0x20){
        	asm(" NOP");
        	test++;
        }

        Movement_setParameter(SENSOR_CONF1, sizeof(sensorConfig1), (uint8_t*)&sensorConfig1);

        appStateSet(APP_STATE_ACTIVE);

        // Activate task
        Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);

      }


    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF1, sensorConfig1, sizeof(sensorConfig1));
    }

    // Data initially zero
//    initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
//    initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
//    initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
    break;

  case SENSOR_CONF2:
    if ((SensorTag_testResult() & SENSOR_MOV_TEST_BM) == 0)
    {
      sensorConfig2 = ST_CFG_ERROR;
    }

    if (sensorConfig2 != ST_CFG_ERROR)
    {
      Movement_getParameter(SENSOR_CONF2, &newCfg);

      if ((newCfg ) == ST_CFG_SENSOR_DISABLE)
      {
        // All axes off, turn off device power
        sensorConfig2 = newCfg;
//        appStateSet(APP_STATE_OFF);
      }
      else
      {
        // Some axes on; power up and activate MPU
        sensorConfig2 = newCfg;
//        appStateSet(APP_STATE_ACTIVE);

      }

      Movement_setParameter(SENSOR_CONF2, sizeof(sensorConfig2), (uint8_t*)&sensorConfig2);
    }
    else
    {
      // Make sure the previous characteristics value is restored
      initCharacteristicValue(SENSOR_CONF2, sensorConfig2, sizeof(sensorConfig2));
    }

    // Data initially zero
    initCharacteristicValue(SENSOR_DATA1, 0, SENSOR_DATA1_LEN);
    initCharacteristicValue(SENSOR_DATA2, 0, SENSOR_DATA2_LEN);
    initCharacteristicValue(SENSOR_DATA3, 0, SENSOR_DATA3_LEN);
    break;

  case SENSOR_AXISMAP:
	  Movement_getParameter(SENSOR_AXISMAP, &newCfg);
	  if(newCfg != sensorAxisMap)
	  {
		  bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
		  bno055_set_axis_remap_value(newCfg & 0x3F);
		  bno055_set_remap_x_sign((newCfg & 0x0040) >> 6);
		  bno055_set_remap_y_sign((newCfg & 0x0080) >> 7);
		  bno055_set_remap_z_sign((newCfg & 0x0010) >> 8);
		  bno055_set_operation_mode(sensorConfig1 & BNO055_OPERATION_MODE_M);
		  sensorAxisMap = newCfg;
	  }
	  break;

  case SENSOR_PERI:
    Movement_getParameter(SENSOR_PERI, &newValue8);
    sensorPeriod = newValue8 * SENSOR_PERIOD_RESOLUTION;
//    Util_rescheduleClock(&periodicClock,sensorPeriod);
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
  initCharacteristicValue(SENSOR_CONF1, 0, SENSOR_CONF1_LEN);
  initCharacteristicValue(SENSOR_CONF2, 0, SENSOR_CONF2_LEN);
  initCharacteristicValue(SENSOR_AXISMAP, 0, SENSOR_AXISMAP_LEN);

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
	switch(paramID){
	case SENSOR_CONF1:
		  memset(&sensorConfig1,value,paramLen);
		  Movement_setParameter(paramID, paramLen, &sensorConfig1);
		break;
	case SENSOR_CONF2:
		  memset(&sensorConfig2,value,paramLen);
		  Movement_setParameter(paramID, paramLen, &sensorConfig2);
		break;
	case SENSOR_AXISMAP:
		  memset(&sensorAxisMap,value,paramLen);
		  Movement_setParameter(paramID, paramLen, &sensorAxisMap);
		break;
	case SENSOR_DATA1:
		  memset(sensorData1,value,paramLen);
		  Movement_setParameter(paramID, paramLen, sensorData1);
		break;
	case SENSOR_DATA2:
		  memset(sensorData2,value,paramLen);
		  Movement_setParameter(paramID, paramLen, sensorData2);
		break;
	case SENSOR_DATA3:
		  memset(sensorData3,value,paramLen);
		  Movement_setParameter(paramID, paramLen, sensorData3);
		break;
	default:
		break;
	}
}

/*******************************************************************************
 * @fn      appStateSet
 *
 * @brief   Set the application state
 *
 */
static void appStateSet(uint8_t newState)
{
	volatile uint8_t test;
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
//    nActivity = MOVEMENT_INACT_CYCLES;
//    movThreshold = WOM_THR;
//    mpuIntStatus = 0;
//    shakeDetected = false;
    bnoDataRdy = false;

//    SensorMpu9250_powerOn();
//    SensorMpu9250_enable(mpuConfig & 0xFF);

    //Reset Sensor, after reset device comes up in normal mode
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    DELAY_MS(650);
    //Set axis mapping
    if(bno055_set_operation_mode(sensorConfig1 & BNO055_OPERATION_MODE_M) == BNO055_SUCCESS){
    	test++;


		bno055_read_quaternion_wxyz((struct bno055_quaternion_t *)&sensorData2[0]);
    }




//    if (newState == APP_STATE_ACTIVE)
//    {
//      // Start scheduled data measurements
//      Util_startClock(&periodicClock);
//    }
//    else
//    {
//      // Stop scheduled data measurements
//      Util_stopClock(&periodicClock);
//    }
  }
}
#endif // EXCLUDE_MOV

/*********************************************************************
*********************************************************************/

