/*********************************************************************
 * INCLUDE
 */
#include "log.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "simpleBLEPeripheral.h"
#include "phy_plus_phy.h"
#include "bcomdef.h"
#include "user_info.h"
#include "pwrmgr.h"
#include "adc.h"
#include "user_hal.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern uint8_t dst_pubAddr[B_ADDR_LEN];
extern uint16_t s_rf_target_netid;
extern uint16_t  s_rf_netid;
uint8_t Device_Mac_Addr[B_ADDR_LEN];
/*********************************************************************
 * LOCAL VARIABLES
 */
uint32_t trace_seq = 0;
/***************************************************************************
* EVENTS
*/

/*********************************************************************
 * FUNCTIONS
 */
/***************************************************************
beacon send data 
****************************************************************/
void User_Beacon_Heart_Beat(void)
{
		uint8_t ret = 0;
		uint8_t send_data[31];
		uint8_t send_index = 0;
		uint8_t heartbeat_pdata[10] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A};
		
		hal_adc_start(INTERRUPT_MODE);
		heartbeat_pdata[0] = (Battery_Voltage >> 8);
		heartbeat_pdata[1] = (Battery_Voltage & 0xFF);
		heartbeat_pdata[2] = (s_rf_netid >> 8);
		heartbeat_pdata[3] = (s_rf_netid & 0xFF);
		
		osal_memcpy(send_data,heartbeat_pdata,sizeof(heartbeat_pdata));
		send_index += sizeof(heartbeat_pdata);
		send_data[send_index++] = ((trace_seq >> 24) & 0xFF);
		send_data[send_index++] = ((trace_seq >> 16) & 0xFF);
		send_data[send_index++] = ((trace_seq >> 8) & 0xFF);
		send_data[send_index++] = (trace_seq  & 0xFF);
		
		phy_set_tx_maxtime(DEFALUT_TX_DURATION);
		LOG_DEBUG("send heart beat trace_seq = %d\r\n",trace_seq);
		phy_rf_stop_tx();
		phy_rf_stop_rx();
		osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_START_RX_EVT);
		osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_STOP_RX_EVT);
			
		ret = phy_rf_start_tx( send_data, send_index, DEFALUT_TX_INTERVAL, DEFAULT_TARGET_NETID);
		if(ret == PPlus_SUCCESS)
		{
				trace_seq++;
		}
}

/***************************************************************
beacon send data 
****************************************************************/
uint8_t User_Beacon_Send_Data(uint32_t tx_duration,uint32_t tx_interval,uint16_t target_net_id,uint8_t *data,uint8_t len)
{
		uint8_t ret = 0;
		uint8_t send_data[31];
		uint8_t send_index = 0;
	
		if(len > 23)
		{
				len = 23;
				LOG_DEBUG("INVALID DATA LEN\r\n");
		}
		
		phy_set_tx_maxtime(tx_duration);
		phy_rf_stop_tx();
		phy_rf_stop_rx();
		osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_START_RX_EVT);
		osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_STOP_RX_EVT);
		osal_memcpy(send_data,data,len);
		send_index += len;
		send_data[send_index++] = ((trace_seq >> 24) & 0xFF);
		send_data[send_index++] = ((trace_seq >> 16) & 0xFF);
		send_data[send_index++] = ((trace_seq >> 8) & 0xFF);
		send_data[send_index++] = (trace_seq  & 0xFF);
	
		ret = phy_rf_start_tx( send_data, send_index, tx_interval, target_net_id);
		if(ret == PPlus_SUCCESS)
		{
				trace_seq++;
		}
		
		return ret;
}


/***************************************************************
beacon send response 
****************************************************************/
uint8_t User_Beacon_Send_Rsp(uint8_t cmd,uint8_t *data,uint8_t len)
{
		uint8_t ret = 0;
		uint8_t send_data[31];
		uint8_t send_index = 0;
	
		if(len > 23)
		{
				len = 23;
				LOG_DEBUG("INVALID DATA LEN\r\n");
		}
		
		phy_set_tx_maxtime(DEFALUT_TX_DURATION);
		phy_rf_stop_tx();
		phy_rf_stop_rx();
		osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_START_RX_EVT);
		osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_NRF_STOP_RX_EVT);
		
		send_data[send_index++] = cmd;
		osal_memcpy(send_data,data,len);
		send_index += len;
		send_data[send_index++] = ((trace_seq >> 24) & 0xFF);
		send_data[send_index++] = ((trace_seq >> 16) & 0xFF);
		send_data[send_index++] = ((trace_seq >> 8) & 0xFF);
		send_data[send_index++] = (trace_seq  & 0xFF);
	
		ret = phy_rf_start_tx( send_data, send_index, DEFALUT_TX_INTERVAL, DEFAULT_TARGET_NETID);
		if(ret == PPlus_SUCCESS)
		{
				trace_seq++;
		}
		
		return ret;
}
/***************************************************************
process received beacon data 
****************************************************************/
void User_Process_Beacon_Data(phy_comm_evt_t *data)
{
		LOG("recv data:");
		my_dump_byte (data->data,data->len);
		LOG("\r\n");
		LOG("RSSI = %d\r\n",data->rssi);
	
		uint8_t cmd_type = 0;
		uint8_t index = 0;
		uint8_t cmd_data_len = 0;
	
		index += 10;
		uint8_t cmd = data->data[index++];
		switch(cmd)
		{
				case COMMAND_TYPE_BLE_SET:
					cmd_type = data->data[index++];
					cmd_data_len = data->data[index++];
					if(BLE_SET_GROUP_ID == cmd_type)
					{
							s_rf_netid = (data->data[index++] << 8);
							s_rf_netid |= data->data[index++];
						
							LOG_DEBUG("set s_rf_netid = 0x%04X\r\n",s_rf_netid);
							//User_Beacon_Send_Rsp(COMMAND_TYPE_BLE_SET_RSP,data->data + 11,cmd_data_len + 2);
					}
					break;
				
				case COMMAND_TYPE_QUERY:
					break;
				
				default:
					break;
		}
		
}

