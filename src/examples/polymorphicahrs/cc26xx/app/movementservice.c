/******************************************************************************

 @file  movementservice.c

 @brief Movement Service

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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "string.h"

#include "movementservice.h"
#include "st_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* Service configuration values */
#define SENSOR_SERVICE_UUID     MOVEMENT_SERV_UUID
#define SENSOR_DATA1_UUID        MOVEMENT_DATA1_UUID
#define SENSOR_DATA2_UUID        MOVEMENT_DATA2_UUID
#define SENSOR_DATA3_UUID        MOVEMENT_DATA3_UUID
#define SENSOR_CONFIG1_UUID      MOVEMENT_CONF1_UUID
#define SENSOR_CONFIG2_UUID      MOVEMENT_CONF2_UUID
#define SENSOR_AXISMAP_UUID      MOVEMENT_AXISMAP_UUID
#define SENSOR_PERIOD_UUID      MOVEMENT_PERI_UUID

#define SENSOR_SERVICE          MOVEMENT_SERVICE
#define SENSOR_DATA1_LEN         MOVEMENT_DATA1_LEN
#define SENSOR_DATA2_LEN         MOVEMENT_DATA2_LEN
#define SENSOR_DATA3_LEN         MOVEMENT_DATA3_LEN

#define SENSOR_DATA1_DESCR       "Raw Mov Data"
#define SENSOR_DATA2_DESCR       "Quaternion Data"
#define SENSOR_DATA3_DESCR       "Grav/Lin Accel Data"
#define SENSOR_CONFIG1_DESCR     "Mov Mode"
#define SENSOR_CONFIG2_DESCR     "Mov Range/BW"
#define SENSOR_PERIOD_DESCR     "Mov Period"
#define SENSOR_AXISMAP_DESCR     "Axis Remap"

#define SENSOR_CONFIG1_LEN       1
#define SENSOR_CONFIG2_LEN       2
#define SENSOR_AXISMAP_LEN       2

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8_t sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_SERVICE_UUID),
};

// Characteristic UUID: data
static CONST uint8_t sensorData1UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA1_UUID),
};
static CONST uint8_t sensorData2UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA2_UUID),
};
static CONST uint8_t sensorData3UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA3_UUID),
};

// Characteristic UUID: config
static CONST uint8_t sensorCfg1UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG1_UUID),
};
static CONST uint8_t sensorCfg2UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG2_UUID),
};
static CONST uint8_t sensorAxisMapUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_AXISMAP_UUID),
};

// Characteristic UUID: period
static CONST uint8_t sensorPeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD_UUID),
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorCBs_t *sensor_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

// Characteristic Value: data1
static uint8_t sensorData1[SENSOR_DATA1_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0
                                             };

// Characteristic Properties: data
static uint8_t sensorData1Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t *sensorData1Config;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t sensorData1UserDescr[] = SENSOR_DATA1_DESCR;
#endif

// Characteristic Value: data2
static uint8_t sensorData2[SENSOR_DATA2_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0};


// Characteristic Properties: data
static uint8_t sensorData2Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t *sensorData2Config;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t sensorData2UserDescr[] = SENSOR_DATA2_DESCR;
#endif

// Characteristic Value: data3
static uint8_t sensorData3[SENSOR_DATA3_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0 };

// Characteristic Properties: data
static uint8_t sensorData3Props = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t *sensorData3Config;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t sensorData3UserDescr[] = SENSOR_DATA3_DESCR;
#endif

// Characteristic Properties: configuration1
static uint8_t sensorCfg1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t sensorCfg1[SENSOR_CONFIG1_LEN];

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t sensorCfg1UserDescr[] = SENSOR_CONFIG2_DESCR;
#endif

// Characteristic Properties: configuration2
static uint8_t sensorCfg2Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t sensorCfg2[SENSOR_CONFIG2_LEN];

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t sensorCfg2UserDescr[] = SENSOR_CONFIG2_DESCR;
#endif

// Characteristic Properties: axis remap
static uint8_t sensorAxisMapProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t sensorAxisMap[SENSOR_AXISMAP_LEN];

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t sensorAxisMapUserDescr[] = SENSOR_AXISMAP_DESCR;
#endif

// Characteristic Properties: period
static uint8_t sensorPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
static uint8_t sensorPeriod = SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION;

#ifdef USER_DESCRIPTION
// Characteristic User Description: period
static uint8_t sensorPeriodUserDescr[] = SENSOR_PERIOD_DESCR;
#endif

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&sensorService                 /* pValue */
  },

  // Characteristic Declaration
  {
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ,
    0,
    &sensorData1Props
  },

    // Characteristic Value "Data"
    {
      { TI_UUID_SIZE, sensorData1UUID },
      GATT_PERMIT_READ,
      0,
      sensorData1
    },

    // Characteristic configuration
    {
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE,
      0,
      (uint8_t *)&sensorData1Config
    },

#ifdef USER_DESCRIPTION
    // Characteristic User Description
    {
      { ATT_BT_UUID_SIZE, charUserDescUUID },
      GATT_PERMIT_READ,
      0,
      sensorData1UserDescr
    },
#endif

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorData2Props
    },

      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, sensorData2UUID },
        GATT_PERMIT_READ,
        0,
        sensorData2
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&sensorData2Config
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorData2UserDescr
      },
#endif

	    // Characteristic Declaration
	    {
	      { ATT_BT_UUID_SIZE, characterUUID },
	      GATT_PERMIT_READ,
	      0,
	      &sensorData3Props
	    },

	      // Characteristic Value "Data"
	      {
	        { TI_UUID_SIZE, sensorData3UUID },
	        GATT_PERMIT_READ,
	        0,
	        sensorData3
	      },

	      // Characteristic configuration
	      {
	        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
	        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	        0,
	        (uint8_t *)&sensorData3Config
	      },

	#ifdef USER_DESCRIPTION
	      // Characteristic User Description
	      {
	        { ATT_BT_UUID_SIZE, charUserDescUUID },
	        GATT_PERMIT_READ,
	        0,
	        sensorData3UserDescr
	      },
	#endif




    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfg1Props
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, sensorCfg1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t*)&sensorCfg1
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCfg1UserDescr
      },
#endif




	    // Characteristic Declaration
	    {
	      { ATT_BT_UUID_SIZE, characterUUID },
	      GATT_PERMIT_READ,
	      0,
	      &sensorCfg2Props
	    },

	      // Characteristic Value "Configuration"
	      {
	        { TI_UUID_SIZE, sensorCfg2UUID },
	        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
	        0,
	        (uint8_t*)&sensorCfg2
	      },

	#ifdef USER_DESCRIPTION
	      // Characteristic User Description
	      {
	        { ATT_BT_UUID_SIZE, charUserDescUUID },
	        GATT_PERMIT_READ,
	        0,
	        sensorCfg2UserDescr
	      },
	#endif


		    // Characteristic Declaration
		    {
		      { ATT_BT_UUID_SIZE, characterUUID },
		      GATT_PERMIT_READ,
		      0,
		      &sensorAxisMapProps
		    },

		      // Characteristic Value "Configuration"
		      {
		        { TI_UUID_SIZE, sensorAxisMapUUID },
		        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		        0,
		        (uint8_t*)&sensorAxisMap
		      },

		#ifdef USER_DESCRIPTION
		      // Characteristic User Description
		      {
		        { ATT_BT_UUID_SIZE, charUserDescUUID },
		        GATT_PERMIT_READ,
		        0,
		        sensorAxisMapUserDescr
		      },
		#endif

     // Characteristic Declaration "Period"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriodProps
    },

      // Characteristic Value "Period"
      {
        { TI_UUID_SIZE, sensorPeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorPeriod
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description "Period"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorPeriodUserDescr
      },
#endif
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len, 
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Movement Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and 
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the 
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an 
// operation on a characteristic requires authorization the Stack will call 
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be 
// made within these functions.
static CONST gattServiceCBs_t sensorCBs =
{
  sensor_ReadAttrCB,  // Read callback function pointer
  sensor_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Movement_addService
 *
 * @brief   Initializes the Sensor Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Movement_addService(void)
{
  // Allocate Client Characteristic Configuration table
  sensorData1Config = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                    linkDBNumConns);
  sensorData2Config = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                    linkDBNumConns);
  sensorData3Config = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                    linkDBNumConns);

  if ((sensorData1Config == NULL) || (sensorData2Config == NULL) || (sensorData3Config == NULL))
  {
    return (bleMemAllocError);
  }
  
  // Register with Link DB to receive link status change callback
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, sensorData1Config);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, sensorData2Config);
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, sensorData3Config);

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(sensorAttrTable,
                                      GATT_NUM_ATTRS (sensorAttrTable),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &sensorCBs);
}


/*********************************************************************
 * @fn      Movement_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Movement_registerAppCBs(sensorCBs_t *appCallbacks)
{
  if (sensor_AppCBs == NULL)
  {
    if (appCallbacks != NULL)
    {
      sensor_AppCBs = appCallbacks;
    }

    return (SUCCESS);
  }

  return (bleAlreadyInRequestedMode);
}

/*********************************************************************
 * @fn      Movement_setParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Movement_setParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SENSOR_DATA1:
    if (len == SENSOR_DATA1_LEN)
    {
      memcpy(sensorData1, value, SENSOR_DATA1_LEN);
      // See if Notification has been enabled
      ret = GATTServApp_ProcessCharCfg(sensorData1Config, sensorData1, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS(sensorAttrTable),
                                 INVALID_TASK_ID, sensor_ReadAttrCB);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_DATA2:
    if (len == SENSOR_DATA2_LEN)
    {
      memcpy(sensorData1, value, SENSOR_DATA2_LEN);
      // See if Notification has been enabled
      ret = GATTServApp_ProcessCharCfg(sensorData2Config, sensorData2, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS(sensorAttrTable),
                                 INVALID_TASK_ID, sensor_ReadAttrCB);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_DATA3:
    if (len == SENSOR_DATA3_LEN)
    {
      memcpy(sensorData1, value, SENSOR_DATA3_LEN);
      // See if Notification has been enabled
      ret = GATTServApp_ProcessCharCfg(sensorData3Config, sensorData3, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS(sensorAttrTable),
                                 INVALID_TASK_ID, sensor_ReadAttrCB);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_CONF1:
      if (len == SENSOR_CONFIG1_LEN)
      {
        memcpy(sensorCfg1, value, SENSOR_CONFIG1_LEN);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_CONF2:
      if (len == SENSOR_CONFIG2_LEN)
      {
        memcpy(sensorCfg2, value, SENSOR_CONFIG2_LEN);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_AXISMAP:
      if (len == SENSOR_AXISMAP_LEN)
      {
        memcpy(sensorAxisMap, value, SENSOR_AXISMAP_LEN);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_PERI:
      if (len == sizeof(uint8_t))
      {
        sensorPeriod = *((uint8_t*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      Movement_getParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Movement_getParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
	case SENSOR_DATA1:
	  memcpy(value, sensorData1, SENSOR_DATA1_LEN);
	  break;

	case SENSOR_DATA2:
	  memcpy(value, sensorData2, SENSOR_DATA2_LEN);
	  break;

	case SENSOR_DATA3:
	  memcpy(value, sensorData3, SENSOR_DATA3_LEN);
	  break;

    case SENSOR_CONF1:
      memcpy(value, sensorCfg1, SENSOR_CONFIG1_LEN);
      break;

    case SENSOR_CONF2:
      memcpy(value, sensorCfg2, SENSOR_CONFIG2_LEN);
      break;

    case SENSOR_AXISMAP:
      memcpy(value, sensorAxisMap, SENSOR_AXISMAP_LEN);
      break;

    case SENSOR_PERI:
      *((uint8_t*)value) = sensorPeriod;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          sensor_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method)
{
  uint16_t uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) 
  {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case SENSOR_DATA1_UUID:
      *pLen = SENSOR_DATA1_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_DATA1_LEN);
      break;

    case SENSOR_DATA2_UUID:
      *pLen = SENSOR_DATA2_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_DATA2_LEN);
      break;

    case SENSOR_DATA3_UUID:
      *pLen = SENSOR_DATA3_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_DATA3_LEN);
      break;

    case SENSOR_CONFIG1_UUID:
      *pLen = SENSOR_CONFIG1_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_CONFIG1_LEN);
      break;

    case SENSOR_CONFIG2_UUID:
      *pLen = SENSOR_CONFIG2_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_CONFIG2_LEN);
      break;

    case SENSOR_AXISMAP_UUID:
      *pLen = SENSOR_AXISMAP_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_AXISMAP_LEN);
      break;
      
    case SENSOR_PERIOD_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return (status);
}

/*********************************************************************
 * @fn      sensor_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len, 
                                    uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8_t notifyApp = 0xFF;
  uint16_t uuid;

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    case SENSOR_DATA1_UUID:
      // Should not get here
      break;

    case SENSOR_DATA2_UUID:
      // Should not get here
      break;

    case SENSOR_DATA3_UUID:
      // Should not get here
      break;

    case SENSOR_CONFIG1_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if (offset == 0)
      {
        if (len != SENSOR_CONFIG1_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if (status == SUCCESS)
      {
        memcpy(pAttr->pValue, pValue, SENSOR_CONFIG1_LEN);

        if (pAttr->pValue == (uint8_t*)&sensorCfg1)
        {
          notifyApp = SENSOR_CONF1;
        }
      }
      break;

    case SENSOR_CONFIG2_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if (offset == 0)
      {
        if (len != SENSOR_CONFIG2_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if (status == SUCCESS)
      {
        memcpy(pAttr->pValue, pValue, SENSOR_CONFIG2_LEN);

        if (pAttr->pValue == (uint8_t*)&sensorCfg2)
        {
          notifyApp = SENSOR_CONF2;
        }
      }
      break;

    case SENSOR_AXISMAP_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if (offset == 0)
      {
        if (len != SENSOR_AXISMAP_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if (status == SUCCESS)
      {
        memcpy(pAttr->pValue, pValue, SENSOR_AXISMAP_LEN);

        if (pAttr->pValue == (uint8_t*)&sensorAxisMap)
        {
          notifyApp = SENSOR_AXISMAP;
        }
      }
      break;

    case SENSOR_PERIOD_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if (offset == 0)
      {
        if (len != 1) 
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if (status == SUCCESS)
      {
        if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
        {

          uint8_t *pCurValue = (uint8_t *)pAttr->pValue;
          *pCurValue = pValue[0];

          if (pAttr->pValue == &sensorPeriod)
          {
            notifyApp = SENSOR_PERI;
          }
        }
        else
        {
           status = ATT_ERR_INVALID_VALUE;
        }
      }
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY);
      break;

    default:
      // Should never get here!
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a characteristic value changed then callback function 
  // to notify application of change
  if ((notifyApp != 0xFF) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange)
  {
    sensor_AppCBs->pfnSensorChange(notifyApp);
  }

  return (status);
}

/*********************************************************************
*********************************************************************/
