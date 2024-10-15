/*
 * File Name: user_interface_manager_led.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: LED Manager
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
#include "bsp_led.h"

/* Public defines ----------------------------------------------------------- */
typedef enum
{
    UI_LED_POWER_ON_PROCESS = 0,
    UI_LED_POWER_OFF_PROCESS,
    UI_LED_POWERED_ON,
    UI_LED_ERROR,
    UI_LED_CHARGING_BATTERY_LOW,
    UI_LED_CHARGING_BATTERY_MID,
    UI_LED_CHARGING_BATTERY_HIGH,
    UI_LED_CHARGING_FULL,
    UI_LED_DISCHARGING_BATTERY_LOW,
    UI_LED_DISPLAY_MAC_ADDRESS,
    UI_LED_LOW_BATTERY,
    UI_LED_STUDY_ONGOING,
    UI_LED_SILENT_MODE,
    UI_LED_UNKNOWN
}
ui_led_id_t;

typedef struct
{
    ui_led_id_t id;
    led_mode_t mode;
    led_color_t color;
    uint8_t brightness;
}
ui_led_info_t;

/* Public enumerate/structure ----------------------------------------------- */
/* Public Constants --------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Public macros ------------------------------------------------------------ */
/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_led_init(void);
void user_interface_manager_led_write_status(ui_led_id_t led, bool status);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C" {
#endif

/* End of file ---------------------------------------------------------------- */
