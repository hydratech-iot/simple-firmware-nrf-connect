/*
* File Name: bsp_i2c_mul.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Board support package for PCA9540 (2-channel I2C-bus multiplexer)
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
#include "pca9540.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t bsp_i2c_mul_init(void);
base_status_t bsp_i2c_mul_select_channel(i2c_channel_t channel);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */
