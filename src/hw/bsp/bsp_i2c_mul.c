/*
 * File Name: bsp_i2c_mul.c
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

/* Includes ----------------------------------------------------------- */
#include "bsp_i2c_mul.h"
#include "bsp_i2c.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_i2c_mul, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static pca9540_t pca9540;

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t bsp_i2c_mul_init(void)
{
    pca9540.device_address = PCA9540_I2C_ADDR;
    pca9540.i2c_write      = bsp_i2c_0_write_data;

    if (pca9540_init(&pca9540) != BS_OK)
    {
        LOG_ERR("I2C multiplexer (PCA9540) init failed");
        return BS_ERROR;
    }

    LOG_INF("I2C multiplexer (PCA9540) init success");
    return BS_OK;
}

base_status_t bsp_i2c_mul_select_channel(i2c_channel_t channel)
{
    CHECK_STATUS(pca9540_select_channel(&pca9540, channel));

    return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
