/*
*  File Name: configuration_manager.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description:
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Includes ----------------------------------------------------------- */
#include "base_include.h"
#include "bsp_nvs.h"
#include "configuration_manager.h"
#include "system_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(configuration_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */

/* Private prototypes ------------------------------------------------------- */
base_status_t cfg_mgmt_init(void)
{
    base_status_t status = BS_OK;
    __ASSERT_NO_MSG(sizeof(nvs_data_t) <= NVS_MAX_DATA_SIZE);

    // Initialize the NVS module for configuration storage
    status = bsp_nvs_init();

    if (status != BS_OK)
    {
        // Log an error if NVS initialization fails
        LOG_ERR("NVS init failed");
        return BS_ERROR;
    }

    LOG_INF("Study interval                : %d (minutes)", g_nvs_setting_data.device_configuration.study_interval_min);
    LOG_INF("Temperature measure interval  : %d (seconds)", g_nvs_setting_data.device_configuration.study_temp_meas_interval_second);
    LOG_INF("OLED switch off period        : %d (seconds)", g_nvs_setting_data.device_configuration.oled_switch_off_period_second);

    cfg_mgmt_update_to_system_data();

    return BS_OK;
}

void cfg_mgmt_update_to_system_data(void)
{
    g_device.study.interval_min              = g_nvs_setting_data.device_configuration.study_interval_min;
    g_device.study.temp_meas_interval_second = g_nvs_setting_data.device_configuration.study_temp_meas_interval_second;
    g_device.oled_switch_off_period_seconds  = g_nvs_setting_data.device_configuration.oled_switch_off_period_second;
}

base_status_t cfg_mgmt_set_device_configuration(device_configuration_t device_config)
{
    g_nvs_setting_data.device_configuration.study_interval_min              = device_config.study_interval_min;
    g_nvs_setting_data.device_configuration.study_temp_meas_interval_second = device_config.study_temp_meas_interval_second;
    g_nvs_setting_data.device_configuration.oled_switch_off_period_second   = device_config.oled_switch_off_period_second;
    
    SYS_NVS_STORE(device_configuration);

    return BS_OK;
}

/* Private implementations -------------------------------------------------- */

/* End of file -------------------------------------------------------- */
