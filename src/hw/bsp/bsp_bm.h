/*
 * File Name: bsp_bm.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Board Support Package Battery Monitor
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  float voltage;
  uint8_t capacity;
}
bsp_bm_info_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t bsp_bm_init(void);
base_status_t bsp_bm_get_info(float *voltage, uint8_t *capacity);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */
