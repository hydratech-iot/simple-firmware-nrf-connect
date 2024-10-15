/*
 * File Name: bsp_temp.c
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

/* Public includes ---------------------------------------------------------- */
#include "as6221.h"
#include "stts22h.h"
#include "bsp_temp.h"
#include "bsp_i2c.h"

/* Private includes --------------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_temp, CONFIG_LOG_DEFAULT_LEVEL);

/* Private defines ---------------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
static as6221_handle_t as6221[SKIN_TEMPERATURE_MAX];

static stts22h_handle_t stts22h = {
    .dev_addr  = STS22H_DEV_ADDRESS,
    .i2c_write = bsp_i2c_1_write_mem,
    .i2c_read  = bsp_i2c_1_read_mem
};

/* Private prototypes ------------------------------------------------------- */
/* Public implementations --------------------------------------------------- */
base_status_t bsp_temp_init(void)
{
    // bsp_temp_gpio_init();

    as6221[SKIN_TEMPERATURE_1].dev_addr = 0x48;

    as6221[SKIN_TEMPERATURE_2].dev_addr = 0x49;
    as6221[SKIN_TEMPERATURE_3].dev_addr = 0x4A;
    as6221[SKIN_TEMPERATURE_4].dev_addr = 0x4B;

    // Init skin sensor
    for (uint8_t i = 0; i < SKIN_TEMPERATURE_MAX; i++)
    {
        as6221[i].i2c_write = bsp_i2c_1_write_mem;
        as6221[i].i2c_read  = bsp_i2c_1_read_mem;

        if (as6221_init(&as6221[i]) != BS_OK)
        {
            LOG_ERR("Temperature sensor (AS6221) %d init failed", i);
            return BS_ERROR;
        }

        LOG_INF("Temperature sensor (AS6221) %d init success", i);
    }

    // Init ambient sensor
    if (stts22h_init(&stts22h) != BS_OK)
    {
        LOG_ERR("Ambient temperature sensor (STTS22H) init failed");
        return BS_ERROR;
    }

    LOG_INF("Ambient temperature sensor (STTS22H) init success");

    return BS_OK;
}

base_status_t bsp_temp_celsius_get(bsp_temp_channel_t channel, float *temp)
{
    if (channel == TEMPERATURE_AMBIENT)
    {
        return stts22h_get_temp(&stts22h, temp);
    }
    else
    {
        return as6221_get_temp_celsius(&as6221[channel], temp);
    }
}

base_status_t bsp_temp_operation_mode_set(bsp_temp_channel_t channel, bsp_temp_operation_mode_t mode)
{
    if (channel == TEMPERATURE_AMBIENT)
    {
        LOG_ERR("STTS22H no need to set the operation");
        return BS_ERROR;
    }
    else
    {
        return as6221_set_operation_mode(&as6221[channel], mode);
    }
}

base_status_t bsp_temp_single_shot_set(bsp_temp_channel_t channel)
{
    if (channel == TEMPERATURE_AMBIENT)
    {
        return stts22h_temp_data_rate_set(&stts22h, STTS22H_ONE_SHOT);
    }
    else
    {
        return as6221_single_shot_set(&as6221[channel]);
    }
}

/* Private implementations -------------------------------------------------- */
/* End of file -------------------------------------------------------------- */
