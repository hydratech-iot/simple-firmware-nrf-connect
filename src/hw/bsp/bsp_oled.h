/*
 * File Name: bsp_oled.c
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
#include "font.h"
#include "ssd1306.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t bsp_oled_init(void);
base_status_t bsp_oled_deinit(void);
base_status_t bsp_oled_control_vdd(bool enable);

base_status_t bsp_oled_clear(void);
base_status_t bsp_oled_update_screen(void);
base_status_t bsp_oled_write_string(uint8_t x, uint8_t y, char *str, font_def font);
base_status_t bsp_oled_draw_image(uint16_t x_pos, uint16_t y_pos, uint16_t w, uint16_t h, uint16_t color, const uint8_t bitmap[]);

/* End of file -------------------------------------------------------- */
