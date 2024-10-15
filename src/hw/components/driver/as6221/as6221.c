/*
 * File Name: as6221.c
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

/* Includes ----------------------------------------------------------- */
#include "as6221.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/**
 * @brief Write data to register of AS622
 *
 * @param[in] hdl AS6221 handler
 * @param[in] reg AS6221 register
 * @param[in] data Parameter to be written
 *
 * @return
 *  - BS_OK: Success
 *  - BS_ERROR: Fail
 *
 */
static base_status_t as6221_write_reg(as6221_handle_t *hdl, uint8_t reg, uint16_t *data);

/**
 * @brief Read data from register of AS622
 *
 * @param[in] hdl AS6221 handler
 * @param[in] reg AS6221 register
 * @param[out] value
 *
 * @return
 *  - BS_OK: Success
 *  - BS_ERROR: Fail
 *
 */
static base_status_t as6221_read_reg(as6221_handle_t *hdl, uint8_t reg, uint16_t *value);

/**
 * @brief Get temperature value from register of AS6221
 *
 * @param[in]  hdl       AS6221 handler
 * @param[in]  reg_addr  AS6221 register
 * @param[out] value     Temperature value
 *
 * @return
 *  - BS_OK: Success
 *  - BS_ERROR: Fail
 */
static inline base_status_t as6221_get_temp(as6221_handle_t *hdl, uint8_t reg_addr, float *value);

/* Function definitions ----------------------------------------------- */
base_status_t as6221_init(as6221_handle_t *hdl)
{
    assert(hdl != NULL);

    uint16_t dummy = 0;
    uint16_t config = AS6221_REG_DEFAULT_M;

    // Verify the Device Address
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_T_VAL, &dummy));

    // Load default config
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, &config));

    return BS_OK;
}

base_status_t as6212_verify_device_address(as6221_handle_t *hdl)
{
    assert(hdl != NULL);

    uint16_t dummy = 0;

    return as6221_read_reg(hdl, AS6221_REG_T_VAL, &dummy);
}

base_status_t as6221_get_temp_celsius(as6221_handle_t *hdl, float *value)
{
    return as6221_get_temp(hdl, AS6221_REG_T_VAL, value);
}

base_status_t as6221_get_temp_fahrenheit(as6221_handle_t *hdl, float *value)
{
    assert(hdl != NULL);

    CHECK_STATUS(as6221_get_temp(hdl, AS6221_REG_T_VAL, value));

    // Convert to Fahrenheit
    *value = *value * 9 / 5 + 32;

    return BS_OK;
}

base_status_t as6221_get_temp_low(as6221_handle_t *hdl, float *value)
{
    return as6221_get_temp(hdl, AS6221_REG_T_LOW, value);
}

base_status_t as6221_set_temp_low(as6221_handle_t *hdl, float low_limit)
{
    assert(hdl != NULL);

    float t_high_val, t_low_val;

    CHECK_STATUS(as6221_get_temp_low(hdl, &t_low_val));
    CHECK_STATUS(as6221_get_temp_high(hdl, &t_high_val));

    uint16_t low_temp = low_limit / 0.0078125f;
    return as6221_write_reg(hdl, AS6221_REG_T_LOW, &low_temp);
}

base_status_t as6221_get_temp_high(as6221_handle_t *hdl, float *value)
{
    return as6221_get_temp(hdl, AS6221_REG_T_HIGH, value);
}

base_status_t as6221_set_temp_high(as6221_handle_t *hdl, float high_limit)
{
    assert(hdl != NULL);

    float t_low_val;

    CHECK_STATUS(as6221_get_temp_low(hdl, &t_low_val));

    uint16_t high_temp = high_limit / 0.0078125f;

    return as6221_write_reg(hdl, AS6221_REG_T_LOW, &high_temp);
}

base_status_t as6221_set_conversion_rate(as6221_handle_t *hdl, as6221_conversion_rate_t val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    reg_config.conversion_rate = (uint16_t)val;
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));

    return BS_OK;
}

base_status_t as6221_get_conversion_rate(as6221_handle_t *hdl, as6221_conversion_rate_t *val)
{
    assert(hdl != NULL);
    assert(val != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    *val = reg_config.conversion_rate;

    return BS_OK;
}

base_status_t as6221_set_operation_mode(as6221_handle_t *hdl, as6221_operation_mode_t val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    reg_config.operation_mode = (uint16_t)val;
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));

    return BS_OK;
}

base_status_t as6221_get_operation_mode(as6221_handle_t *hdl, as6221_operation_mode_t *val)
{
    assert(hdl != NULL);
    assert(val != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    *val = reg_config.operation_mode;

    return BS_OK;
}

base_status_t as6221_set_interrupt_mode(as6221_handle_t *hdl, as6221_interrupt_mode_t val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    reg_config.interrupt_mode = (uint16_t)val;
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));

    return BS_OK;
}

base_status_t as6221_get_interrupt_mode(as6221_handle_t *hdl, as6221_interrupt_mode_t *val)
{
    assert(hdl != NULL);
    assert(val != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    *val = reg_config.interrupt_mode;

    return BS_OK;
}

base_status_t as6221_set_polarity(as6221_handle_t *hdl, as6221_polarity_t val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    reg_config.polarity = (uint16_t)val;
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));

    return BS_OK;
}

base_status_t as6221_get_polarity(as6221_handle_t *hdl, as6221_polarity_t *val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    *val = reg_config.polarity;

    return BS_OK;
}

base_status_t as6221_set_consecutive_fault(as6221_handle_t *hdl, uint8_t val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    reg_config.consecutive_faults = (uint16_t)val;
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    
    return BS_OK;
}

base_status_t as6221_get_consecutive_fault(as6221_handle_t *hdl, uint8_t *val)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    *val = reg_config.consecutive_faults;

    return BS_OK;
}

base_status_t as6221_single_shot_set(as6221_handle_t *hdl)
{
    assert(hdl != NULL);

    as6221_config_reg_t reg_config;

    CHECK_STATUS(as6221_read_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));
    reg_config.single_shot = 0x01;
    CHECK_STATUS(as6221_write_reg(hdl, AS6221_REG_CONFIG, (uint16_t *)&reg_config));

    return BS_OK;
}

static base_status_t as6221_write_reg(as6221_handle_t *hdl, uint8_t reg, uint16_t *data)
{
    assert(hdl != NULL);

    uint8_t write_buffer[2] = {0};

    write_buffer[0] = ((uint8_t)(*data >> 8) & 0xFF);
    write_buffer[1] = ((uint8_t)*data & 0xFF);

    CHECK_STATUS(hdl->i2c_write(hdl->dev_addr, reg, write_buffer, 2));

    return BS_OK;
}

static base_status_t as6221_read_reg(as6221_handle_t *hdl, uint8_t reg, uint16_t *value)
{
    assert(hdl != NULL);

    uint8_t read_buffer[2] = {0};

    CHECK_STATUS(hdl->i2c_read(hdl->dev_addr, reg, read_buffer, 2));

    *value = 0;
    *value |= ((uint16_t)read_buffer[0] << 8);
    *value |= ((uint16_t)read_buffer[1]);
    
    return BS_OK;
}

static inline base_status_t as6221_get_temp(as6221_handle_t *hdl, uint8_t reg_addr, float *value)
{
    assert(hdl != NULL);
    assert(value != NULL);
    assert(reg_addr == AS6221_REG_T_VAL || reg_addr == AS6221_REG_T_LOW || reg_addr == AS6221_REG_T_HIGH);

    uint16_t reg_val = 0;

    CHECK_STATUS(as6221_read_reg(hdl, reg_addr, &reg_val));

    // Calculate temperature value
    *value = (reg_val < 32768) ? (reg_val * 0.0078125f) : -((reg_val - 1) * 0.0078125f);

    return BS_OK;
}

/* End of file -------------------------------------------------------- */
