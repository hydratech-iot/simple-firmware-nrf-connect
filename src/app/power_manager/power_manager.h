/*
 * File Name: power_manager.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Power Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "base_type.h"
#include "sensor_manager.h"

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
typedef struct
{
    uint8_t level;
    uint8_t is_charging;
    float voltage;
}
battery_data_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void power_manager_init(void);
base_status_t power_manager_task(device_mode_t mode, battery_data_t *battery);
void power_manager_display_led(void);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
