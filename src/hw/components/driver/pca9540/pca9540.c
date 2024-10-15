/*
* File Name: pca9540.c
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

/* Includes ----------------------------------------------------------- */
#include "pca9540.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t pca9540_init(pca9540_t *me)
{
    if ((me == NULL) || (me->i2c_write == NULL))
        return BS_ERROR;

    CHECK_STATUS(pca9540_select_channel(me, I2C_CHANNEL_1));

    return BS_OK;
}

base_status_t pca9540_select_channel(pca9540_t *me, i2c_channel_t channel)
{
    CHECK_STATUS(me->i2c_write(me->device_address, &channel, 1));

    return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
