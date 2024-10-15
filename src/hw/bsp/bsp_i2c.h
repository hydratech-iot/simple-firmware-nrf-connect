/*
* File Name: bsp_imu.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Board Support Package for BMI270
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------- */
#include "base_include.h"
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
    BSP_I2C_NUM_0,
    BSP_I2C_NUM_1,
    BSP_I2C_NUM_MAX,
}
bsp_i2c_num_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t bsp_i2c_init(void);

base_status_t bsp_i2c_0_read_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
base_status_t bsp_i2c_0_write_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
base_status_t bsp_i2c_0_write_data(uint8_t dev_addr, uint8_t *p_data, uint16_t len);

base_status_t bsp_i2c_1_read_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
base_status_t bsp_i2c_1_write_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
base_status_t bsp_i2c_1_write_data(uint8_t dev_addr, uint8_t *p_data, uint16_t len);

/* End of file -------------------------------------------------------- */
