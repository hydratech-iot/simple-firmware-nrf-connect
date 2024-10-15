/*
 * File Name: stts22h.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Driver for temperature sensor (STTS22H)
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------- */
#include "stts22h.h"

/* Private defines ---------------------------------------------------- */
#define STTS22H_REG_WHOAMI                          0x01U
#define STTS22H_REG_TEMP_H_LIMIT                    0x02U
#define STTS22H_REG_TEMP_L_LIMIT                    0x03U
#define STTS22H_REG_CTRL                            0x04U
#define STTS22H_REG_STATUS                          0x05U
#define STTS22H_REG_TEMP_L_OUT                      0x06U
#define STTS22H_REG_TEMP_H_OUT                      0x07U
#define STTS22H_REG_SOFTWARE_RESET                  0x0CU
#define STTS22H_WHOAMI_VALUE                        0xA0U

#define PROPERTY_DISABLE                            (0U)
#define PROPERTY_ENABLE                             (1U)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t stts22h_write_reg(stts22h_handle_t *hdl, uint8_t reg, uint8_t data);
static base_status_t stts22h_read_reg(stts22h_handle_t *hdl, uint8_t reg, uint8_t *data);

/* Function definitions ----------------------------------------------- */
base_status_t stts22h_auto_increment_set(stts22h_handle_t *hdl, uint8_t val)
{
    stts22h_reg_t stts22h_reg;

    CHECK_STATUS(stts22h_read_reg(hdl, STTS22H_REG_CTRL, (uint8_t *)&stts22h_reg.byte));
    stts22h_reg.ctrl.if_add_inc = (uint8_t)val;
    CHECK_STATUS(stts22h_write_reg(hdl, STTS22H_REG_CTRL, (uint8_t)stts22h_reg.byte));

    return BS_OK;
}

base_status_t stts22h_block_data_update_set(stts22h_handle_t *hdl, uint8_t val)
{
    stts22h_reg_t stts22h_reg;

    CHECK_STATUS(stts22h_read_reg(hdl, STTS22H_REG_CTRL, (uint8_t *)&stts22h_reg.byte));
    stts22h_reg.ctrl.bdu = val;
    CHECK_STATUS(stts22h_write_reg(hdl, STTS22H_REG_CTRL, (uint8_t)stts22h_reg.byte));

    return BS_OK;
}

base_status_t stts22h_init(stts22h_handle_t *hdl)
{
    assert(hdl != NULL);

    uint8_t who_am_i;

    CHECK_STATUS(stts22h_read_reg(hdl, STTS22H_REG_WHOAMI, &who_am_i));

    // Verify the Who Am I
    if (who_am_i != STTS22H_WHOAMI_VALUE)
    {
        return BS_ERROR;
    }

    CHECK_STATUS(stts22h_block_data_update_set(hdl, PROPERTY_ENABLE));
    CHECK_STATUS(stts22h_auto_increment_set(hdl, PROPERTY_ENABLE));

    return BS_OK;
}

base_status_t stts22h_single_shot_set(stts22h_handle_t *hdl)
{
    assert(hdl != NULL);

    stts22h_reg_t stts22h_reg;

    CHECK_STATUS(stts22h_read_reg(hdl, STTS22H_REG_CTRL, (uint8_t *)&stts22h_reg.byte));
    stts22h_reg.ctrl.one_shot = 1;
    CHECK_STATUS(stts22h_write_reg(hdl, STTS22H_REG_CTRL, (uint8_t)stts22h_reg.byte));

    return BS_OK;
}

base_status_t stts22h_temp_data_rate_set(stts22h_handle_t *hdl, stts22h_odr_temp_t val)
{
    stts22h_reg_t stts22h_reg;

    CHECK_STATUS(stts22h_read_reg(hdl, STTS22H_REG_SOFTWARE_RESET, (uint8_t *)&stts22h_reg.byte));

    if (val == STTS22H_ONE_SHOT)
    {
        stts22h_reg.software_reset.sw_reset = PROPERTY_ENABLE;
        CHECK_STATUS(stts22h_write_reg(hdl, STTS22H_REG_SOFTWARE_RESET, (uint8_t)stts22h_reg.byte));

        stts22h_reg.software_reset.sw_reset = PROPERTY_DISABLE;
        CHECK_STATUS(stts22h_write_reg(hdl, STTS22H_REG_SOFTWARE_RESET, (uint8_t)stts22h_reg.byte));
    }

    CHECK_STATUS(stts22h_read_reg(hdl, STTS22H_REG_CTRL, (uint8_t *)&stts22h_reg.byte));
    stts22h_reg.ctrl.one_shot      = (uint8_t)val & 0x01U;
    stts22h_reg.ctrl.free_run      = ((uint8_t)val & 0x02U) >> 1;
    stts22h_reg.ctrl.low_odr_start = ((uint8_t)val & 0x04U) >> 2;
    stts22h_reg.ctrl.avg           = ((uint8_t)val & 0x30U) >> 4;
    CHECK_STATUS(stts22h_write_reg(hdl, STTS22H_REG_CTRL, (uint8_t)stts22h_reg.byte));

    return BS_OK;
}

float stts22h_from_lsb_to_celsius(int16_t lsb)
{
    return ((float)lsb / 100.0f);
}

base_status_t stts22h_get_temp(stts22h_handle_t *hdl, float *value)
{
    assert(hdl != NULL);
    assert(value != NULL);

    uint8_t buf[2];
    int16_t temperature;

    CHECK_STATUS(hdl->i2c_read(hdl->dev_addr, STTS22H_REG_TEMP_L_OUT, buf, 2));

    // Calculate temperature value
    temperature = (int16_t)((buf[1] << 8) + buf[0]);
    *value = stts22h_from_lsb_to_celsius(temperature);

    return BS_OK;
}

/* Private function definitions --------------------------------------- */
static base_status_t stts22h_write_reg(stts22h_handle_t *hdl, uint8_t reg, uint8_t data)
{
    assert(hdl != NULL);

    CHECK_STATUS(hdl->i2c_write(hdl->dev_addr, reg, &data, 1));

    return BS_OK;
}

static base_status_t stts22h_read_reg(stts22h_handle_t *hdl, uint8_t reg, uint8_t *data)
{
    assert(hdl != NULL);

    CHECK_STATUS(hdl->i2c_read(hdl->dev_addr, reg, data, 1));

    return BS_OK;
}

/* End of file -------------------------------------------------------- */
