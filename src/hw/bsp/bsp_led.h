/*
 * File Name: bsp_led.h
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

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
    LED_COLOR_GREEN = 0,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,
    LED_COLOR_CYAN,
    LED_COLOR_WHITE,
    LED_COLOR_RED,
    LED_COLOR_PINK,
    LED_COLOR_UNKNOWN
} led_color_t;

typedef enum
{
    LED_MODE_OFF = 0,
    LED_MODE_ON,
    LED_MODE_FLASH_FAST,
    LED_MODE_FLASH,
    LED_MODE_FLASH_ONE_EVERY_3_SECONDS,
    LED_MODE_PULSE,
    LED_MODE_UNKNOWN
} led_mode_t;

typedef enum
{
    LED_CHANNEL_RED = 0,
    LED_CHANNEL_GREEN,
    LED_CHANNEL_BLUE,
    LED_CHANNEL_UNKNOWN
} led_channel_t;

/* Public macros ------------------------------------------------------ */
void bsp_led_init(void);
void bsp_led_set_led(led_mode_t mode, led_color_t color, uint8_t brightness);
void bsp_led_update_led_state(void);

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/* End of file -------------------------------------------------------- */
