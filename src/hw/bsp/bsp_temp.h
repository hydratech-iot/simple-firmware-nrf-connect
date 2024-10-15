/*
* File Name: bsp_temp.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Board Support Package for AS6221
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
#include "bsp_i2c.h"
#include "as6221.h"

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
typedef enum
{
  TEMP_OPERATION_MODE_CONTINUOUS = AS6221_CONTINUOUS,
  TEMP_OPERATION_MODE_SLEEP = AS6221_SLEEP,
}
bsp_temp_operation_mode_t;

typedef enum
{
    SKIN_TEMPERATURE_1 = 0,
    SKIN_TEMPERATURE_2 = 1,
    SKIN_TEMPERATURE_3 = 2,
    SKIN_TEMPERATURE_4 = 3,
    SKIN_TEMPERATURE_MAX
}
skin_temp_t;

typedef enum
{
    TEMPERATURE_SKIN_1  = 0,
    TEMPERATURE_SKIN_2  = 1,
    TEMPERATURE_SKIN_3  = 2,
    TEMPERATURE_SKIN_4  = 3,
    TEMPERATURE_AMBIENT = 4,
    TEMPERATURE_MAX,
}
bsp_temp_channel_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
base_status_t bsp_temp_init(void);
base_status_t bsp_temp_celsius_get(bsp_temp_channel_t channel, float *temp);
base_status_t bsp_temp_operation_mode_set(bsp_temp_channel_t channel, bsp_temp_operation_mode_t mode);
base_status_t bsp_temp_single_shot_set(bsp_temp_channel_t channel);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif


/* End of file -------------------------------------------------------------- */
