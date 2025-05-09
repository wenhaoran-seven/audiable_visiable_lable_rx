/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       simpleBLEperipheral.h
    Revised:
    Revision:

    Description:    This file contains the Simple BLE Peripheral sample application
                  definitions and prototypes.


**************************************************************************************************/

#ifndef SIMPLEBLEPERIPHERAL_H
#define SIMPLEBLEPERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/
extern uint8 simpleBLEPeripheral_TaskID;

// Simple BLE Peripheral Task Events
#define SBP_START_DEVICE_EVT                           0x0001
#define SBP_DISABLE_BLE_EVT                            0x0002
#define SBP_NRF_SEND_DATA_EVT												   0x0004
#define SBP_ENABLE_BLE_EVT                             0x0008

#define SBP_NRF_PERIODIC_TX_EVT                        0x0100
#define SBP_NRF_START_RX_EVT                           0x0200
#define SBP_NRF_STOP_RX_EVT                            0x0400
#define SBP_NRF_HEART_BEAT_EVT												 0x0800

#define SBP_NRF_ENTER_SLEEP_EVT												 0x0010
/*********************************************************************
    MACROS
*/

#define SOFT_VERSION			0x1001
/*********************************************************************
    FUNCTIONS
*/

/*
    Task Initialization for the BLE Application
*/
extern void SimpleBLEPeripheral_Init( uint8 task_id );

/*
    Task Event Processor for the BLE Application
*/
extern uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
