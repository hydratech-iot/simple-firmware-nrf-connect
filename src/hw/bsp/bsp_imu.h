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

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "bsp_i2c.h"
#include "bmi323.h"

/* Public defines ----------------------------------------------------------- */
#define IMU_FIFO_MAX_SAMPLE        (250)

/* Public enumerate/structure ----------------------------------------------- */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}
imu_raw_data_t;

typedef struct
{
    float x;
    float y;
    float z;
}
imu_converted_data_t;
typedef struct
{
    imu_raw_data_t accel_raw;           // Raw
    imu_raw_data_t gyro_raw;            // Raw
    imu_converted_data_t accel_mps2;    // Meter per second squared
    imu_converted_data_t gyro_dps;      // Degree per second
}
imu_data_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
base_status_t bsp_imu_init(void);
base_status_t bsp_imu_get_data(imu_data_t *imu_data, uint16_t *fifo_frames);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
