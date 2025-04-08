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
    Filename:       simpleBLEPeripheral.c
    Revised:
    Revision:

    Description:    This file contains the Simple BLE Peripheral sample application


**************************************************************************************************/
/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
//#include "devinfoservice.h"
#include "sbpProfile_ota.h"
#include "ota_app_service.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "simpleBLEPeripheral.h"
#include "ll.h"
#include "ll_hw_drv.h"
#include "ll_def.h"
#include "hci_tl.h"
#include "phy_plus_phy.h"
#include "rf_phy_nrf.h"

#include "user_info.h"
#include "user_hal.h"
/*********************************************************************
    MACROS
*/
//#define LOG(...)
/*********************************************************************
    CONSTANTS
*/

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0


#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     24//32//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800//48//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Default passcode
#define DEFAULT_PASSCODE                      0//19655

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#define RESOLVING_LIST_ENTRY_NUM              10


/*********************************************************************
    build define
*/

#define APP_CFG_RPA_TEST                       0

#define DBG_RTC_TEST                           0

#define LATENCY_TEST                           0

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
perStatsByChan_t g_perStatsByChanTest;

/*********************************************************************
    EXTERNAL VARIABLES
*/
volatile uint8_t g_current_advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;
extern uint16_t  s_rf_netid;
extern uint16_t s_rf_target_netid;
extern uint8_t dst_pubAddr[B_ADDR_LEN];
/*********************************************************************
    EXTERNAL FUNCTIONS
*/


/*********************************************************************
    LOCAL VARIABLES
*/
uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing
static gaprole_States_t gapProfileState = GAPROLE_INIT;
static uint8 s_rf_dlen = 0;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x12,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'


    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
};


// advert data for iBeacon
static uint8 advertData[] =
{
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x1A, // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
    0x4c, // Company ID - Fixed
    0x00, // Company ID - Fixed
    0x02, // Data Type - Fixed
    0x15, // Data Length - Fixed
    0xFD, // UUID
    0xA5, // UUID
    0x06, // UUID
    0x93, // UUID
    0xA4, // UUID
    0xE2, // UUID
    0x4F, // UUID
    0xB1, // UUID
    0xAF, // UUID
    0xCF, // UUID
    0xC6, // UUID
    0xEB, // UUID
    0x07, // UUID
    0x64, // UUID
    0x78, // UUID
    0x25, // UUID
    0x27, // Major
    0x74, // Major
    0x6b,//0x04, // Minor
    0xed,//0xb0, // Minor
    0xc5 // Power - The 2's complement of the calibrated Tx Power
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "TAG_RX ";
/*********************************************************************
    LOCAL FUNCTIONS
*/
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void peripheralStateReadRssiCB( int8 rssi  );
static uint8_t Smart_nRF_data_process(phy_comm_evt_t* pdata);


char* bdAddr2Str( uint8* pAddr );
//static uint8_t simpleBLEPeripheral_ScanRequestFilterCBack(void);
/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
};
#if (DEF_GAPBOND_MGR_ENABLE==1)
//GAP Bond Manager Callbacks, add 2017-11-15
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
    NULL,                     // Passcode callback (not used by application)
    NULL                      // Pairing / Bonding state Callback (not used by application)
};
#endif
// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
    simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/
/*********************************************************************
    @fn      SimpleBLEPeripheral_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void SimpleBLEPeripheral_Init( uint8 task_id )
{
    simpleBLEPeripheral_TaskID = task_id;
    // Setup the GAP
    phy_cbfunc_regist(PHY_DATA_CB,Smart_nRF_data_process);
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
    // Setup the GAP Peripheral Role Profile
    {
        // device starts advertising upon initialization
        uint8 initial_advertising_enable = FALSE;
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39;
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        uint8 peerPublicAddr[] =
        {
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06
        };
        uint8 advType =g_current_advType;// LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
        GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
        GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
        // set adv channel map
        GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);
        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
        osal_memcpy(&scanRspData[2],attDeviceName,0x11);
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Set advertising interval
    {
        uint16 advInt = 800;//2400;//1600;//1600;//800;//1600;   // actual time = advInt * 625us
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
    #if(DEF_GAPBOND_MGR_ENABLE==1)
    // Setup the GAP Bond Manager, add 2017-11-15
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8 mitm = TRUE;
        uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
        uint8 bonding = TRUE;
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
    }
    #endif
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    //DevInfo_AddService();                           // Device Information Service
    ota_app_AddService();
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
    // Setup the SimpleProfile Characteristic Values
    {
        uint8 power = 0x0f;
        uint8 reset[IBEACON_ATT_LONG_PKT];

        for(uint8 i=0; i<IBEACON_ATT_LONG_PKT; i++)
        {
            reset[i]=(i<6) ? 0 : i;
        }

        // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, IBEACON_UUID_LEN, uuid_setting);
        // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint16 ), &major );
        // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint16 ), &minor );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &power );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, IBEACON_ATT_LONG_PKT, &reset );
    }

    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

		uint8_t mtuSet = 247;
		llInitFeatureSet2MPHY(TRUE);
		llInitFeatureSetDLE(TRUE);
		ATT_SetMTUSizeMax(mtuSet);
		LOG("[2Mbps | DLE | MTU %d] \r\n",mtuSet);

    // Setup a delayed profile startup
    osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
    // for receive HCI complete message
    GAP_RegisterForHCIMsgs(simpleBLEPeripheral_TaskID);

    LL_PLUS_PerStats_Init(&g_perStatsByChanTest);
    LOG("SimpleBLEPeripheral_Init Done====================RX version:0x%04X\r\n",SOFT_VERSION);
		
		User_Hal_Init();
		
		osal_start_timerEx(simpleBLEPeripheral_TaskID,SBP_NRF_HEART_BEAT_EVT,100);
}

/*********************************************************************
    @fn      SimpleBLEPeripheral_ProcessEvent

    @brief   Simple BLE Peripheral Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
        {
            simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );
        #if(DEF_GAPBOND_MGR_ENABLE==1)
        // Start Bond Manager, 2017-11-15
        VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );
        #endif
        HCI_LE_ReadResolvingListSizeCmd();
			
				osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DISABLE_BLE_EVT, 100 );
        return ( events ^ SBP_START_DEVICE_EVT );
    }

		
    if ( events & SBP_DISABLE_BLE_EVT )
    {
				LOG("disable ble\r\n");
				uint8_t initial_advertising_enable = FALSE;
				GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
			
        return ( events ^ SBP_DISABLE_BLE_EVT );
    }


    // enable adv
    if ( events & SBP_ENABLE_BLE_EVT )
    {
				LOG("enable ble\r\n");
        uint8 initial_advertising_enable = TRUE;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        return ( events ^ SBP_ENABLE_BLE_EVT );
    }


    if ( events & SBP_NRF_PERIODIC_TX_EVT )
    {
        static uint16_t advCnt=1;
        // advert data for iBeacon
        static uint8 advdata[] =
        {
            0x02,   // length of this data
            0x01,//GAP_ADTYPE_FLAGS,
            0x06,//DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
            // complete name
            0x09,   // length of this data
            0x09,//GAP_ADTYPE_LOCAL_NAME_COMPLETE,
            0x53,0x6D,0x61,0x72,0x54,0x6E,0x52,0x46,//SmarTnRF
            0x07, // length of this data including the data type byte
            0xff,//GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
            0x04, // Company ID - Fixed
            0x05, // Company ID - Fixed
            0x02, // Data Type - Fixed
            0x02, // Data Length - Fixed
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
            0x00, // cnt
            0x00, // cnt
        };
        uint8_t dlen = s_rf_dlen;
        uint8_t ret = 0;
        advdata[dlen-2] = advCnt>>8;
        advdata[dlen-1] = advCnt&0xff;
				
        ret = phy_rf_start_tx(advdata,dlen, 0, DEFAULT_TARGET_NETID);
        if(ret == PPlus_SUCCESS)
            advCnt++;

        LOG_DEBUG("%d %d %d\r\n",ret,advCnt,phy_rf_get_current_status());
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_PERIODIC_TX_EVT, 1000 );
        return ( events ^ SBP_NRF_PERIODIC_TX_EVT );
    }

    if ( events & SBP_NRF_START_RX_EVT )
    {
				uint8_t ret = phy_rf_start_rx(1*1000);
				if(ret == PPlus_SUCCESS)
				{
						//LOG("[start rx] Ok \r\n");
						osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_STOP_RX_EVT, 5);
				}
				else
				{
						LOG("[start rx] Err ret %x status %x\r\n",ret,phy_rf_get_current_status());
						osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_STOP_RX_EVT, 10);
				}
				
        return ( events ^ SBP_NRF_START_RX_EVT );
    }
		
		if ( events & SBP_NRF_STOP_RX_EVT )
		{
				uint8_t ret = phy_rf_stop_rx();
				if(ret == PPlus_SUCCESS)
				{
						//LOG("[stop rx] Ok\r\n");
						osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_START_RX_EVT, 1000);
				}
				else
				{
						LOG("[stop rx] Err ret %x status %x\r\n",ret,phy_rf_get_current_status());
				}
				
				return ( events ^ SBP_NRF_STOP_RX_EVT );
		}
		
		if(events & SBP_NRF_HEART_BEAT_EVT)
    {
				User_Beacon_Heart_Beat();
        osal_start_timerEx(simpleBLEPeripheral_TaskID,SBP_NRF_HEART_BEAT_EVT,60000);
        return(events ^ SBP_NRF_HEART_BEAT_EVT);
    }
		
		if(events & SBP_NRF_SEND_DATA_EVT)
    {
				uint8_t test_data[10] = {0,1,2,3,4,5,6,7,8,9};
				User_Beacon_Send_Data(DEFALUT_TX_DURATION,DEFALUT_TX_INTERVAL,DEFAULT_TARGET_NETID,test_data, sizeof(test_data));
			
				return(events ^ SBP_NRF_SEND_DATA_EVT);
		}

		if(events & SBP_NRF_ENTER_SLEEP_EVT)
    {
				hal_pwrmgr_unlock(MOD_USR1);
			
				return(events ^ SBP_NRF_ENTER_SLEEP_EVT);
		}
		
    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      simpleBLEPeripheral_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    hciEvt_CmdComplete_t* pHciMsg;

    switch ( pMsg->event )
    {
    case HCI_GAP_EVENT_EVENT:
    {
        switch( pMsg->status )
        {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
            pHciMsg = (hciEvt_CmdComplete_t*)pMsg;
            LOG("==> HCI_COMMAND_COMPLETE_EVENT_CODE: %x\r\n", pHciMsg->cmdOpcode);
            //safeToDealloc = gapProcessHCICmdCompleteEvt( (hciEvt_CmdComplete_t *)pMsg );
            break;

        default:
            //safeToDealloc = FALSE;  // Send to app
            break;
        }
    }
    }
}
/*********************************************************************
    @fn      peripheralStateReadRssiCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
*/
static void peripheralStateReadRssiCB( int8  rssi )
{

}

/*********************************************************************
    @fn      peripheralStateNotificationCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
*/
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
		LOG("peripheralStateNotificationCB newState:%d\r\n",newState);
    switch ( newState )
    {
    case GAPROLE_STARTED:
    {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 str_addr[14]= {0};
        uint8 initial_advertising_enable = FALSE;//true
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
        osal_memcpy(&scanRspData[11],&str_addr[6],8);
        osal_memcpy(&attDeviceName[9],&str_addr[6],8);
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
        // Set the GAP Characteristics
        GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
				
				GAPRole_GetParameter(GAPROLE_BD_ADDR,Device_Mac_Addr);
				osal_memcpy(dst_pubAddr,Device_Mac_Addr,B_ADDR_LEN);
				dst_pubAddr[0] = Device_Mac_Addr[5];
				dst_pubAddr[1] = Device_Mac_Addr[4];
				dst_pubAddr[2] = Device_Mac_Addr[3];
				dst_pubAddr[3] = Device_Mac_Addr[2];
				dst_pubAddr[4] = Device_Mac_Addr[1];
				dst_pubAddr[5] = Device_Mac_Addr[0];
				LOG("macADDR:0x%02X%02X%02X%02X%02X%02X\r\n",dst_pubAddr[0],dst_pubAddr[1],dst_pubAddr[2],dst_pubAddr[3],dst_pubAddr[4],dst_pubAddr[5]);
    }
    break;

    case GAPROLE_ADVERTISING:
    {

    }
    break;

    case GAPROLE_CONNECTED:
        HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, NULL);
        break;

    case GAPROLE_CONNECTED_ADV:
        break;

    case GAPROLE_WAITING:
        break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
        break;

    case GAPROLE_ERROR:
        break;

    default:
        break;
    }

    gapProfileState = newState;
    LOG("[GAP ROLE %d]\r\n",newState);
    VOID gapProfileState;
}


/*********************************************************************
    @fn      simpleProfileChangeCB

    @brief   Callback from SimpleBLEProfile indicating a value change

    @param   paramID - parameter ID of the value that was changed.

    @return  none
*/
static void simpleProfileChangeCB( uint8 paramID )
{
    uint8 newValue[IBEACON_ATT_LONG_PKT];
		LOG("simpleProfileChangeCB paramID %02x \r\n",paramID);
    switch( paramID )
    {
    case SIMPLEPROFILE_CHAR5:
        SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, newValue );
        LOG("[WRT_ATT] %02x \r\n",newValue[0]);
        break;

    default:
        break;
    }
}


/*********************************************************************
    @fn      bdAddr2Str

    @brief   Convert Bluetooth address to string. Only needed when
           LCD display is used.

    @return  none
*/
char* bdAddr2Str( uint8* pAddr )
{
    uint8       i;
    char        hex[] = "0123456789ABCDEF";
    static char str[B_ADDR_STR_LEN];
    char*        pStr = str;
    *pStr++ = '0';
    *pStr++ = 'x';
    // Start from end of addr
    pAddr += B_ADDR_LEN;

    for ( i = B_ADDR_LEN; i > 0; i-- )
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }

    *pStr = 0;
    return str;
}

uint8_t Smart_nRF_data_process(phy_comm_evt_t* pdata)
{
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT==1)
    //LOG_DEBUG("OPCODE=%x  datalen=%d\r\n",pdata->type,pdata->len);
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    uint8_t status = phy_rf_get_current_status();

    if(status == PHYPLUS_RFPHY_RX_ONLY)
    {
        LOG_DEBUG("It's nrf CB ack:");
        my_dump_byte(pdata->data,pdata->len);
    }
    else
    {
        LOG_DEBUG("It's nrf CB data:");
        my_dump_byte(pdata->data,pdata->len);
    }

    #else

    if(pdata->type == PHYPLUS_STX_DONE_TYPE)
    {
        LOG_DEBUG("STX Done Reporting\r\n");		
				phy_rf_stop_tx();
				phy_rf_stop_rx();
				osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_START_RX_EVT);
				osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_STOP_RX_EVT);
				osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_START_RX_EVT,1000);
    }
    else if(PHYPLUS_GET_ACK_BIT(pdata->type))
    {
        LOG_DEBUG("It's rf CB ack:");
				User_Process_Beacon_Data(pdata);
    }
    else if(PHYPLUS_GET_NEEDACK_BIT(pdata->type))
    {
        LOG_DEBUG("It's rf CB data:");
				User_Process_Beacon_Data(pdata);
    }
    else if(pdata->len != NULL)
    {
        LOG_DEBUG("It's rf broadcast data:");
				User_Process_Beacon_Data(pdata);
    }
		
		
    #endif
    #else
    #if(DEF_PHYPLUS_NRF_SUPPORT==PHYPLUS_NRF_ENABLE)
    LOG_DEBUG("It's noack nrf data:");
    my_dump_byte(pdata->data,pdata->len);
    #else
    LOG_DEBUG("It's noack rf data:");
    my_dump_byte(pdata->data,pdata->len);
    #endif
    #endif
    return PPlus_SUCCESS;
}

/*********************************************************************
*********************************************************************/
