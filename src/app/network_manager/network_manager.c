/*
* File Name: network.c
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: This file contains declarations for functions related to handling BLE data.
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Includes ----------------------------------------------------------- */
#include "network_manager.h"
#include "protocol.h"
#include "sensor_manager.h"
#include "system_manager.h"
#include "ble_peripheral.h"
#include "configuration_manager.h"

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
static void network_process_cmd_start_stop_study_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_start_stop_study_get(uint8_t *p_data, uint8_t len);

static void network_process_cmd_temp_configuration_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_temp_configuration_get(uint8_t *p_data, uint8_t len);

static void network_process_cmd_afe_configuration_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_afe_configuration_get(uint8_t *p_data, uint8_t len);

static void network_process_cmd_imu_configuration_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_imu_configuration_get(uint8_t *p_data, uint8_t len);

static void network_process_cmd_device_info_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_device_info_get(uint8_t *p_data, uint8_t len);

static void network_process_cmd_device_mode_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_device_mode_get(uint8_t *p_data, uint8_t len);

static void network_process_cmd_device_configuration_set(uint8_t *p_data, uint8_t len);
static void network_process_cmd_device_configuration_get(uint8_t *p_data, uint8_t len);

/* Private variables -------------------------------------------------- */
static const network_process_t NET_PROCESS_DATA_TABLE[PACKET_CMD_MAX] = 
{
  //                 +======================================+==============================================+
  //                 |             Packet Type              |           Process Function                   |
  //                 +--------------------------------------+----------------------------------------------+
     NET_PROCESS_INFO(PACKET_CMD_START_STOP_STUDY_SET       , network_process_cmd_start_stop_study_set     )
    ,NET_PROCESS_INFO(PACKET_CMD_START_STOP_STUDY_GET       , network_process_cmd_start_stop_study_get     )
    ,NET_PROCESS_INFO(PACKET_CMD_START_STOP_STUDY_RESP      , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_PPG_STREAMING_DATA         , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_IMU_STREAMING_DATA         , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_STUDY_DATA                 , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_TEMPERATURE_DATA           , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_PPG_PHYSICAL_DATA          , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_BATTERY_DATA               , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_CHARGING_DATA       , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_CONFIGURATION_SET   , network_process_cmd_device_configuration_set )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_CONFIGURATION_GET   , network_process_cmd_device_configuration_get )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_CONFIGURATION_RESP   , NULL                                        )
    ,NET_PROCESS_INFO(PACKET_CMD_TEMP_CONFIGURATION_SET     , network_process_cmd_temp_configuration_set   )
    ,NET_PROCESS_INFO(PACKET_CMD_TEMP_CONFIGURATION_GET     , network_process_cmd_temp_configuration_get   )
    ,NET_PROCESS_INFO(PACKET_CMD_TEMP_CONFIGURATION_RESP    , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_AFE_CONFIGURATION_SET      , network_process_cmd_afe_configuration_set    )
    ,NET_PROCESS_INFO(PACKET_CMD_AFE_CONFIGURATION_GET      , network_process_cmd_afe_configuration_get    )
    ,NET_PROCESS_INFO(PACKET_CMD_AFE_CONFIGURATION_RESP     , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_IMU_CONFIGURATION_SET      , network_process_cmd_imu_configuration_set    )
    ,NET_PROCESS_INFO(PACKET_CMD_IMU_CONFIGURATION_GET      , network_process_cmd_imu_configuration_get    )
    ,NET_PROCESS_INFO(PACKET_CMD_IMU_CONFIGURATION_RESP     , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_INFO_SET            , network_process_cmd_device_info_get          )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_INFO_GET            , network_process_cmd_device_info_set          )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_INFO_RESP           , NULL                                         )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_MODE_SET            , network_process_cmd_device_mode_set          )
    ,NET_PROCESS_INFO(PACKET_CMD_DEVICE_MODE_GET            , network_process_cmd_device_mode_get          )
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
                g_device.comm.server.communicated = true;
            }
        }
    }
}

/* Private function definitions ---------------------------------------- */
/* Command Process Function ----------------------------------------------- */
static void network_process_cmd_start_stop_study_set(uint8_t *p_data, uint8_t len)
{
    packet_start_stop_study_resp_t study_resp_pkt;
    packet_start_stop_study_t *study = (packet_start_stop_study_t *)p_data;

    system_manager_study_start(study->start);
    
    if (study->start)
    {
        LOG_INF("Start study with interval %d minutes", study->interval_min);
    }
    else
    {
        LOG_INF("Stop study");
    }

    g_device.study.interval_min = study->interval_min;
    g_device.study.start        = study->start;

    // Send the response
    study_resp_pkt.header.cmd       = PACKET_CMD_START_STOP_STUDY_RESP;
    study_resp_pkt.header.len       = sizeof(packet_start_stop_study_resp_t) - sizeof(packet_hdr_t);;
    study_resp_pkt.header.msg_index = 0;
    study_resp_pkt.study_status     = study->start;
    ble_peripheral_send_data((uint8_t *)&study_resp_pkt, sizeof(packet_start_stop_study_resp_t));
}

static void network_process_cmd_start_stop_study_get(uint8_t *p_data, uint8_t len)
{
    packet_start_stop_study_resp_t study_resp_pkt;

    // Send the response
    study_resp_pkt.header.cmd       = PACKET_CMD_START_STOP_STUDY_RESP;
    study_resp_pkt.header.len       = sizeof(packet_start_stop_study_resp_t) - sizeof(packet_hdr_t);;
    study_resp_pkt.header.msg_index = 0;
    study_resp_pkt.study_status     = g_device.study.start;
    ble_peripheral_send_data((uint8_t *)&study_resp_pkt, sizeof(packet_start_stop_study_resp_t));
}

static void network_process_cmd_temp_configuration_set(uint8_t *p_data, uint8_t len)
{
    packet_temp_configuration_t *temp_config = (packet_temp_configuration_t *)p_data;
}

static void network_process_cmd_temp_configuration_get(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_afe_configuration_set(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_afe_configuration_get(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_imu_configuration_set(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_imu_configuration_get(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_device_info_set(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_device_info_get(uint8_t *p_data, uint8_t len)
{
}

static void network_process_cmd_device_mode_set(uint8_t *p_data, uint8_t len)
{
    packet_device_mode_t *device_mode = (packet_device_mode_t *)p_data;

    g_device.mode = device_mode->mode;
}

static void network_process_cmd_device_mode_get(uint8_t *p_data, uint8_t len)
{

}

static void network_process_cmd_device_configuration_set(uint8_t *p_data, uint8_t len)
{
    device_configuration_t device_configuration;

    packet_device_configuration_set_t *device_config = (packet_device_configuration_set_t *)p_data;

    g_device.study.interval_min              = device_config->study_interval;
    g_device.study.temp_meas_interval_second = device_config->temp_measure_interval;
    g_device.oled_switch_off_period_seconds  = device_config->oled_switch_off_period;

    LOG_INF("Study interval                : %d (minutes)", g_device.study.interval_min);
    LOG_INF("Temperature measure interval  : %d (seconds)", g_device.study.temp_meas_interval_second);
    LOG_INF("OLED switch off period        : %d (seconds)", g_device.oled_switch_off_period_seconds);

    device_configuration.study_interval_min              = device_config->study_interval;
    device_configuration.study_temp_meas_interval_second = device_config->temp_measure_interval;
    device_configuration.oled_switch_off_period_second   = device_config->oled_switch_off_period;

    cfg_mgmt_set_device_configuration(device_configuration);
}

static void network_process_cmd_device_configuration_get(uint8_t *p_data, uint8_t len)
{
    packet_device_configuration_resp_t device_config_pkt;

    // Send the response
    device_config_pkt.header.cmd       = PACKET_CMD_DEVICE_CONFIGURATION_RESP;
    device_config_pkt.header.len       = sizeof(packet_device_configuration_resp_t) - sizeof(packet_hdr_t);;
    device_config_pkt.header.msg_index = 0;

    device_config_pkt.study_interval         = g_device.study.interval_min;
    device_config_pkt.temp_measure_interval  = g_device.study.temp_meas_interval_second;
    device_config_pkt.oled_switch_off_period = g_device.oled_switch_off_period_seconds;
    ble_peripheral_send_data((uint8_t *)&device_config_pkt, sizeof(packet_device_configuration_resp_t));
}

/* Response Function ----------------------------------------------- */
/* End of file --------------------------------------------------------- */
