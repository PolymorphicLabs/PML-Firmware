/**************************************************************************************************
  Filename:       OSAL_ICallBle.c
  Revised:        $Date: 2014-03-04 00:38:54 +0100 (ti, 04 mar 2014) $
  Revision:       $Revision: 37490 $

  Description:    This file contains function that allows user setup tasks


  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include <ICall.h>
#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "osal_snv.h"


/* LL */
#include "ll.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbtimer.h"
#endif

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"

#if defined ( GAP_BOND_MGR )
  #include "gapbondmgr.h"
#endif

/* GATT */
#include "gatt.h"

/* Application */
#include "hci_tl.h"

#include "gattservapp.h"

#include "gapbondmgr.h"

#include "bleUserConfig.h"
#include "bleDispatch.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
  LL_ProcessEvent,                                                  // task 0
  HCI_ProcessEvent,                                                 // task 1
#if defined ( OSAL_CBTIMER_NUM_TASKS )
  OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),           // task 2
#endif
  L2CAP_ProcessEvent,                                               // task 3
  GAP_ProcessEvent,                                                 // task 4
  SM_ProcessEvent,                                                  // task 5
  GATT_ProcessEvent,                                                // task 6
  GATTServApp_ProcessEvent,                                         // task 7
#if defined ( GAP_BOND_MGR )
  GAPBondMgr_ProcessEvent,                                          // task 8
#endif
  bleDispatch_ProcessEvent                                          // task 9
};

const uint8_t tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16_t *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *6
 * @return  none
 */
void osalInitTasks( void )
{
  ICall_EntityID entity;
  ICall_Semaphore sem;
  uint8_t taskID = 0;
  uint8_t i;

  tasksEvents = (uint16_t *)osal_mem_alloc( sizeof( uint16_t ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16_t ) * tasksCnt));

  /* LL Task */
  LL_Init( taskID++ );

  /* HCI Task */
  HCI_Init( taskID++ );

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  /* Callback Timer Tasks */
  osal_CbTimerInit( taskID );
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* L2CAP Task */
  L2CAP_Init( taskID++ );

  /* GAP Task */
  GAP_Init( taskID++ );

  /* SM Task */
  SM_Init( taskID++ );

	/* GATT Task */
  GATT_Init( taskID++ );
	
  /* GATT Server App Task */
  GATTServApp_Init( taskID++ );
      
#if defined ( GAP_BOND_MGR )
  /* Bond Manager Task */
  GAPBondMgr_Init( taskID++ );
#endif
  
  /* ICall BLE Dispatcher Task */
  bleDispatch_Init( taskID );

  // ICall enrollment
  /* Enroll the service that this stack represents */
  ICall_enrollService(ICALL_SERVICE_CLASS_BLE, NULL, &entity, &sem);

  /* Enroll the obtained dispatcher entity and OSAL task ID of HCI Ext App
   * to OSAL so that OSAL can route the dispatcher message into
   * the appropriate OSAL task.
   */
  osal_enroll_dispatchid(taskID, entity);

  /* Register all other OSAL tasks to use the registered dispatcher entity
   * ID as the source of dispatcher messages, even though the other OSAL
   * tasks didn't register themselves to receive messages from application.
   */
  for (i = 0; i < taskID; i++)
  {
    osal_enroll_senderid(i, entity);
  }
}

/**
 * Main entry function for the stack image
 */
int stack_main( void *arg )
{
  /* User reconfiguration of BLE Controller and Host variables */
  setBleUserConfig( (bleUserCfg_t *)arg );
  
  /* Establish OSAL for a stack service that requires accompanying
   * messaging service */
  if (ICall_enrollService(ICALL_SERVICE_CLASS_BLE_MSG,
                          (ICall_ServiceFunc) osal_service_entry,
                          &osal_entity, &osal_semaphore) !=
      ICALL_ERRNO_SUCCESS)
  {
    /* abort */
    ICall_abort();
  }

  halIntState_t state;
  HAL_ENTER_CRITICAL_SECTION(state);
  
  // Turn off interrupts
  //osal_int_disable( INTS_ALL );

  // Initialize NV System
  osal_snv_init( );

  // Initialize the operating system
  osal_init_system();

  // Allow interrupts
  //osal_int_enable( INTS_ALL );
  HAL_EXIT_CRITICAL_SECTION(state);

  osal_start_system(); // No Return from here

  return 0;  // Shouldn't get here.
}

/*********************************************************************
*********************************************************************/
