/*
* File Name: pca9540.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Driver support PCA9540 (2-channel I2C-bus multiplexer)
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
#define PCA9540_I2C_ADDR               (0x70) // 7 Bits 

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
    I2C_CHANNEL_1 = 0x04,
    I2C_CHANNEL_2 = 0x05
}
i2c_channel_t;

/**
 * @brief PCA9540 sensor struct
 */
typedef struct
{
    uint8_t device_address; // I2C device address

    // Read n-bytes from device's internal address <reg_addr> via I2C bus
    base_status_t (*i2c_write)(uint8_t slave_addr, uint8_t *p_data, uint16_t len);
}
pca9540_t;

/* Public function prototypes ----------------------------------------- */
base_status_t pca9540_init(pca9540_t *me);
base_status_t pca9540_select_channel(pca9540_t *me, i2c_channel_t channel);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */
