/**
 * File Name: bsp_env.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Board Support Package Environment Sensor
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_bm.h"
#include "bsp_i2c.h"
#include "max17048.h"
#include "bsp_i2c_mul.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_bm, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static max17048_t max17048;

/* Private function prototypes ---------------------------------------- */
static base_status_t bsp_bm_read_data(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_bm_init(void)
{
    max17048.device_address = MAX17048_I2C_ADDR;
    max17048.i2c_read = bsp_bm_read_data;

    if (max17048_init(&max17048) != BS_OK)
    {
        LOG_ERR("Battery monitor (MAX17048) init failed");
        return BS_ERROR;
    }

    LOG_INF("Battery monitor (MAX17048) init success");
    return BS_OK;
}

base_status_t bsp_bm_get_info(float *voltage, uint8_t *capacity)
{
    CHECK_STATUS(max17048_get_voltage(&max17048, voltage));
    CHECK_STATUS(max17048_get_capacity(&max17048, capacity));

    return BS_OK;
}

/* Private function definitions ---------------------------------------- */
static base_status_t bsp_bm_read_data(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    // Select the I2C from MUX
    CHECK_STATUS(bsp_i2c_mul_select_channel(I2C_CHANNEL_2));

    CHECK_STATUS(bsp_i2c_0_read_mem(slave_addr, reg_addr, data, len));

    return BS_OK;
}

/* End of file -------------------------------------------------------- */
