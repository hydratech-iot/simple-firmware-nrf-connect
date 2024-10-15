/*
 * File Name: as6221.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Driver for temperature sensor (AS6221)
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
#define AS6221_DEV_ADDRESS 0x49

/* Public enumerate/structure ----------------------------------------- */
enum as6212_register_enum
{
    // Internal Register Addresses
    AS6221_REG_T_VAL = 0x00,  // Temperature Register
    AS6221_REG_CONFIG = 0x01, // Configuration Register
    AS6221_REG_T_LOW = 0x02,  // Low Temperature Threshold
    AS6221_REG_T_HIGH = 0x03, // High Temperature Threshold

    // Helpful preset definitions for configuration register
    AS6221_REG_DEFAULT_M = 0x40A0,  // Default state
    AS6221_REG_SLEEP_MODE = 0x41A0, // Sleep Mode
    AS6221_REG_SLEEP_SS = 0xC1A0,   // Sleep Mode Single Shot
};

typedef enum
{
    AS6221_CV_RATE_0_25 = 0x00U,
    AS6221_CV_RATE_1 = 0x01U,
    AS6221_CV_RATE_4 = 0x02U,
    AS6221_CV_RATE_8 = 0x03U
} as6221_conversion_rate_t;

typedef enum
{
    AS6221_CONTINUOUS = 0x00U,
    AS6221_SLEEP = 0x01U,
} as6221_operation_mode_t;

typedef enum
{
    AS6221_COMPARATOR = 0x00U,
    AS6221_INTERRUPT = 0x01U,
} as6221_interrupt_mode_t;

typedef enum
{
    AS6221_POL_LOW = 0x00U,
    AS6221_POL_HIGH = 0x01U,
} as6221_polarity_t;

typedef struct
{
    uint16_t reserved_1 : 5;
    uint16_t alert_bit : 1;
    uint16_t conversion_rate : 2;
    uint16_t operation_mode : 1;
    uint16_t interrupt_mode : 1;
    uint16_t polarity : 1;
    uint16_t consecutive_faults : 2;
    uint16_t reserved_2 : 2;
    uint16_t single_shot : 1;
} as6221_config_reg_t;

typedef struct
{
    uint8_t dev_addr;
    base_status_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t size);
    base_status_t (*i2c_read)(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint16_t size);
} as6221_handle_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief Load configurations and start the sensor
 *
 * @param[in] hdl as6221 handle
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 */
base_status_t as6221_init(as6221_handle_t *hdl);

/**
 * @brief Load configurations and start the sensor
 *
 * @param[in] hdl as6221 handle
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 */
base_status_t as6212_verify_device_address(as6221_handle_t *hdl);

/**
 * @brief Read temperature in Celsius
 *
 * @param[in] hdl as6221 handle
 * @param[out] value temperature in Celsius
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_temp_celsius(as6221_handle_t *hdl, float *value);

/**
 * @brief Read temperature in Fahrenheit
 *
 * @param[in] hdl as6221 handle
 * @param[out] value temperature in Fahrenheit
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_temp_fahrenheit(as6221_handle_t *hdl, float *value);

/**
 * @brief Read Low temperature threshold value
 *
 * @param[in] hdl as6221 handle
 * @param[out] value
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_temp_low(as6221_handle_t *hdl, float *value);

/**
 * @brief Set Low temperature threshold value
 *
 * @param[in] hdl as6221 handle
 * @param[in] low_limit low limit input value
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_temp_low(as6221_handle_t *hdl, float low_limit);

/**
 * @brief Read High temperature threshold value
 *
 * @param[in] hdl as6221 handle
 * @param[out] value high limit output value
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_temp_high(as6221_handle_t *hdl, float *value);

/**
 * @brief Set High temperature threshold value
 *
 * @param[in] hdl as6221 handle
 * @param[in] high_limit  high limit input value
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_temp_high(as6221_handle_t *hdl, float high_limit);

/**
 * @brief Set conversion rate
 *
 * @param[in] hdl as6221 handle
 * @param[in] val conversion rate
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_conversion_rate(as6221_handle_t *hdl, as6221_conversion_rate_t val);

/**
 * @brief Get conversion rate
 *
 * @param[in] hdl as6221 handle
 * @param[out] val conversion rate
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_conversion_rate(as6221_handle_t *hdl, as6221_conversion_rate_t *val);

/**
 * @brief Set operation mode
 *
 * @param[in] hdl as6221 handle
 * @param[in] val operation mode
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_operation_mode(as6221_handle_t *hdl, as6221_operation_mode_t val);

/**
 * @brief Get operation mode
 *
 * @param[in] hdl as6221 handle
 * @param[out] val operation mode
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_operation_mode(as6221_handle_t *hdl, as6221_operation_mode_t *val);

/**
 * @brief Set interrupt mode
 *
 * @param[in] hdl as6221 handle
 * @param[in] val interrupt mode
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_interrupt_mode(as6221_handle_t *hdl, as6221_interrupt_mode_t val);

/**
 * @brief Get interrupt mode
 *
 * @param[in] hdl as6221 handle
 * @param[out] val interrupt mode
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_interrupt_mode(as6221_handle_t *hdl, as6221_interrupt_mode_t *val);

/**
 * @brief Set polarity
 *
 * @param[in] hdl as6221 handle
 * @param[in] val polarity
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_polarity(as6221_handle_t *hdl, as6221_polarity_t val);

/**
 * @brief Set polarity
 *
 * @param[in] hdl as6221 handle
 * @param[out] val polarity
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_polarity(as6221_handle_t *hdl, as6221_polarity_t *val);

/**
 * @brief Set consecutive fault value
 *
 * @param[in] hdl as6221 handle
 * @param[in] val consecutive fault value
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_set_consecutive_fault(as6221_handle_t *hdl, uint8_t val);

/**
 * @brief Get consecutive fault value
 *
 * @param[in] hdl as6221 handle
 * @param[in] val consecutive fault value
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_get_consecutive_fault(as6221_handle_t *hdl, uint8_t *val);

/**
 * @brief Enter Single Shot
 *
 * @param[in] hdl as6221 handle
 *
 * @return
 *  - AS6221_OK: Success
 *  - AS6221_ERR: Fail
 *
 */
base_status_t as6221_single_shot_set(as6221_handle_t *hdl);

/* End of file -------------------------------------------------------- */
