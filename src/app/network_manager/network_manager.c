/*
* File Name: network.c
* 
* Author: Thuan Le 
* 
* Description: This file contains declarations for functions related to handling BLE data.
* 
*/

/* Includes ----------------------------------------------------------- */
#include "network_manager.h"
#include "protocol.h"
#include "system_manager.h"
#include "ble_peripheral.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(network, CONFIG_LOG_DEFAULT_LEVEL);

typedef void network_data_process(uint8_t *p_data, uint8_t len);

typedef struct
{
    packet_type_t packet_type;
    network_data_process *func_process;
}
network_process_t;

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define NET_PROCESS_INFO(_pkt_type, _func)[_pkt_type] =         \
{                                                               \
    .packet_type  = _pkt_type,                                  \
    .func_process = _func                                       \
}

/* Public variables --------------------------------------------------- */
/* Private Constants -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Private variables -------------------------------------------------- */
static const network_process_t NET_PROCESS_DATA_TABLE[PACKET_CMD_MAX] = 
{
  //                 +======================================+==============================================+
  //                 |             Packet Type              |           Process Function                   |
  //                 +--------------------------------------+----------------------------------------------+
     NET_PROCESS_INFO(PACKET_CMD_NOTIFICATION               , NULL     )
  //                 +======================================+==============================================+
};

/* Function definitions ----------------------------------------------- */
void network_process_data(uint8_t *p_data, uint8_t len)
{
    for (uint8_t i = 0; i < PACKET_CMD_MAX; i++)
    {
        if (p_data[PACKET_HDR_CMD_POS] == i)
        {
            if (NET_PROCESS_DATA_TABLE[i].func_process != NULL)
            {
                NET_PROCESS_DATA_TABLE[i].func_process(p_data, len);
                LOG_INF("Received command %d from network", i);
            }
        }
    }
}

void network_send_notification(void)
{
    packet_cmd_notification_t notification_pkt;

    notification_pkt.header.cmd       = PACKET_CMD_NOTIFICATION;
    notification_pkt.header.len       = 0;
    notification_pkt.header.msg_index = 0;

    ble_peripheral_send_data((uint8_t *)&notification_pkt, sizeof(packet_cmd_notification_t));
}

/* Private function definitions ---------------------------------------- */
/* Command Process Function ----------------------------------------------- */
/* Response Function ----------------------------------------------- */
/* End of file --------------------------------------------------------- */
