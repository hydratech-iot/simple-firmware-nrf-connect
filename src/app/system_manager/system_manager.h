/*
 * File Name: system_manager.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: System Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "base_type.h"
#include "sensor_manager.h"
#include "power_manager.h"

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
typedef enum
{
    SYSTEM_MANAGER_STATE_INIT         = 0,
    SYSTEM_MANAGER_STATE_POWERING_ON  = 1,
    SYSTEM_MANAGER_STATE_CONNECTING   = 2,
    SYSTEM_MANAGER_STATE_WAKE_UP      = 3,
    SYSTEM_MANAGER_STATE_COLLECT_DATA = 4,
    SYSTEM_MANAGER_STATE_SEND_DATA    = 5,
    SYSTEM_MANAGER_STATE_SLEEP        = 6,
    SYSTEM_MANAGER_STATE_STOPPED      = 7,
    SYSTEM_MANAGER_STATE_POWER_OFF    = 8,
    SYSTEM_MANAGER_STATE_MAX
}
system_manager_state_t;

typedef enum
{
    UI_MODE_SILENT = 0,
    UI_MODE_NORMAL = 1,
    UI_MODE_UNKNOW = 2
}
ui_mode_t;

typedef struct
{
    uint8_t start;
    uint32_t interval_min;
    uint32_t temp_meas_interval_second;
}
study_t;

typedef struct
{
    struct
    {
        bool connected;
        uint8_t rssi;
    }
    ble;

    struct
    {
        bool connected;
        uint8_t rssi;
    }
    zigbee;

    struct
    {
        bool connected;
        bool communicated;
    }
    server;
    
}
connectivity_t;

typedef struct
{
    ppg_phys_data_t ppg_phys;
    ppg_phys_data_t ppg_phys_prv;

    float temperature[5];

    float avr_skin_temp;

    battery_data_t battery;

    imu_phys_data_t imu_phys;

    device_mode_t mode;

    study_t study;

    connectivity_t comm;

    ui_mode_t ui_mode;

    uint16_t oled_switch_off_period_seconds;

    bool first_run_system_manager;
}
device_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
extern device_t g_device;

/* Public APIs -------------------------------------------------------------- */
void system_manager_init(void);
void system_manager_study_start(bool start);
void system_manager_state_set(system_manager_state_t state);
void system_manager_display_charging(void);
bool system_manager_check_device_is_remove_from_horse(void);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
