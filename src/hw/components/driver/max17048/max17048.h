/*
* File Name: max17048.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Driver support MAX17048 (Stand-Alone Fuel Gauge IC)
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
// Un-shifted I2C address. Becomes 0x6C for write and 0x6D for read.
#define MAX17048_I2C_ADDR               (0x36) // 7 Bits 

/* Public enumerate/structure ----------------------------------------- */
/**
 * @brief MAX17048 sensor struct
 */
typedef struct
{
    uint8_t device_address; // I2C device address

    // Read n-bytes from device's internal address <reg_addr> via I2C bus
    base_status_t (*i2c_read)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
} max17048_t;

/* Public function prototypes ----------------------------------------- */
/**
 * @brief         MAX17048 init
 *
 * @param[in]     me    Pointer to handle of MAX17048 module
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t max17048_init(max17048_t *me);

/**
 * @brief         MAX17048 get voltage
 *
 * @param[in]     me        Pointer to handle of MAX17048 module
 * @param[in]     voltage   Voltage
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t max17048_get_voltage(max17048_t *me, float *voltage);

/**
 * @brief         MAX17048 Get SOC
 *
 * @param[in]     me          Pointer to handle of MAX17048 module
 * @param[in]     capacity    Capacity
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t max17048_get_capacity(max17048_t *me, uint8_t *capacity);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */
