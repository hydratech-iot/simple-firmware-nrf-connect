/*
 * File Name: stts22h.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Driver for temperature sensor (STS22H)
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
#define STS22H_DEV_ADDRESS 0x3F

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
    uint8_t one_shot     : 1;
    uint8_t time_out_dis : 1;
    uint8_t free_run     : 1;
    uint8_t if_add_inc   : 1;
    uint8_t avg          : 2;
    uint8_t bdu          : 1;
    uint8_t low_odr_start: 1;
}
stts22h_ctrl_reg_t;

typedef struct
{
  uint8_t not_used_01         : 1;
  uint8_t sw_reset            : 1;
  uint8_t not_used_02         : 4;
  uint8_t low_odr_enable      : 1;
  uint8_t not_used_03         : 1;
}
stts22h_software_reset_t;

typedef union
{
    uint8_t byte;
    stts22h_ctrl_reg_t ctrl;
    stts22h_software_reset_t software_reset;
}
stts22h_reg_t;

typedef struct
{
    uint8_t dev_addr;
    base_status_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t size);
    base_status_t (*i2c_read)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t size);
}
stts22h_handle_t;

typedef enum
{
  STTS22H_POWER_DOWN   = 0x00,
  STTS22H_ONE_SHOT     = 0x01,
  STTS22H_1Hz          = 0x04,
  STTS22H_25Hz         = 0x02,
  STTS22H_50Hz         = 0x12,
  STTS22H_100Hz        = 0x22,
  STTS22H_200Hz        = 0x32,
} stts22h_odr_temp_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t stts22h_init(stts22h_handle_t *hdl);
base_status_t stts22h_single_shot_set(stts22h_handle_t *hdl);
base_status_t stts22h_temp_data_rate_set(stts22h_handle_t *hdl, stts22h_odr_temp_t val);
base_status_t stts22h_get_temp(stts22h_handle_t *hdl, float *value);

/* End of file -------------------------------------------------------- */
