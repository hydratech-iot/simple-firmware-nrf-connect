/*
 * File Name: user_interface_manager_oled.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: OLED Manager
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

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
typedef enum
{
    UI_BATT_0,
    UI_BATT_25,
    UI_BATT_50,
    UI_BATT_75,
    UI_BATT_100,
    UI_BLE,
    UI_BLE_ACTIVE,
    UI_BOLT,
    UI_CHECK_CIRCLE,
    UI_CIRCLE,
    UI_HEART_OUTLINE,
    UI_HEART_SOLID,
    UI_VECTOR,
    UI_SIGNAL_ACQ_1,
    UI_SIGNAL_ACQ_2,
    UI_TEMPERATURE,
    UI_ZIGBEE_SIGNAL_0,
    UI_ZIGBEE_SIGNAL_1,
    UI_ZIGBEE_SIGNAL_2,
    UI_ZIGBEE_SIGNAL_3,
    UI_ZIGBEE_SIGNAL_4,
    UI_CLEAR_12_12,
    UI_CLEAR_24_24
} ui_oled_item_t;

typedef enum
{
    UI_GENERAL_ERROR,
    UI_TEMP_ERROR,
    UI_IMU_ERROR,
    UI_AFE_ERROR,
    UI_AMBIENT_ERROR
} ui_error_type_t;

typedef enum
{
    UI_OLED_STATE_INIT             = 0,
    UI_OLED_STATE_CHECK_CONNECTION = 1,
    UI_OLED_STATE_MEASURING        = 2,
    UI_OLED_STATE_MEASURE_VALUE    = 3,
    UI_OLED_STATE_CHARGING         = 4,
    UI_OLED_STATE_ERROR            = 5,
    UI_OLED_STATE_WAIT_TO_CLEAR    = 6,
    UI_OLED_STATE_CLEAR            = 7,
    UI_OLED_STATE_MAC_DISPLAY      = 8,
    UI_OLED_STATE_IDLE             = 9,
    UI_OLED_STATE_POWER_OFF        = 10,
    UI_OLED_STATE_MAX
}
ui_oled_state_t;

/* Public Constants --------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Public macros ------------------------------------------------------------ */
/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_oled_init(void);
void user_interface_manager_oled_display_header(bool force_update);

void user_interface_manager_oled_update_state(ui_oled_state_t state);
void user_interface_manager_oled_display_init(void);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C" {
#endif

/* End of file ---------------------------------------------------------------- */
