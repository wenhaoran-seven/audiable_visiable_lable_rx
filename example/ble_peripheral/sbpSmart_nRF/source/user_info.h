#ifndef _APP_USER_INFO_H_
#define _APP_USER_INFO_H_
/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "phy_plus_phy.h"
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */
//---------------------------------SET -------------------------------------------------	
#define COMMAND_TYPE_BLE_SET									0x02
#define COMMAND_TYPE_BLE_SET_RSP							0x82

#define BLE_SET_SYS_RESET  										0x01
#define BLE_SET_RESET_TO_FN										0x02
#define BLE_SET_ADV_TX_POWER									0x03
#define BLE_SET_BOND_STATE  									0x04
#define BLE_SET_GROUP_ID    									0x05
#define BLE_SET_DISTRICT											0x06

#define BLE_SET_DISABLE_BLE										0xA0
#define BLE_SET_ENABLE_BLE										0xA1

//---------------------------------QUERY------------------------------------------------
#define COMMAND_TYPE_QUERY										0x03
#define COMMAND_TYPE_QUERY_RSP								0x83

#define BLE_QUERY_MAC_ADDR										0x01
#define BLE_QUERY_SOFT_VERSION								0x02
#define BLE_QUERY_TX_POWER										0x03
#define BLE_QUERY_BOND_STATE  							  0x04
#define BLE_QUERY_GROUP_ID    								0x05
#define BLE_QUERY_DISTRICT										0x06
	

//--------------------------------------------------------------------------------------
#define GROUP_ID_BROADCAST										0xFFFF
#define GROUP_ID_DEFAULT											0x0000

/*********************************************************************
 * TYPEDEFS
 */
#define DEFALUT_TX_DURATION			100 //us
#define DEFALUT_TX_INTERVAL			0 //us

#define DEFAULT_TARGET_NETID		0xFF00
/*********************************************************************
* EVENTS
*/

/*********************************************************************
 * VARIABLES
 */
extern uint8_t Device_Mac_Addr[B_ADDR_LEN];
/*********************************************************************
 * FUNCTIONS
 */
void User_Beacon_Heart_Beat(void);

uint8_t User_Beacon_Send_Data(uint32_t tx_duration,uint32_t tx_interval,uint16_t target_net_id,uint8_t *data,uint8_t len);

uint8_t User_Beacon_Send_Rsp(uint8_t cmd,uint8_t *data,uint8_t len);

void User_Process_Beacon_Data(phy_comm_evt_t *data);
#endif

