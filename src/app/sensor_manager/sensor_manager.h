/**
 * @file       sensor_manager.h
 * @copyright  Copyright (C) 2020 HydraTech. All rights reserved.
 * @license    This project is released under the HydraTech License.
 * @version    01.00.00
 * @date       2020-09-21
 * @author     Thuan Le
 * @brief      Sensor Manager
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------------- */
typedef enum
{
  SM_TEMPERATURE_STATE_IDLE,
  SM_TEMPERATURE_STATE_TRIGGER_MEASURE,
  SM_TEMPERATURE_STATE_MEASURE,
  SM_TEMPERATURE_STATE_MAX
}
sm_temperature_state_t;

/* Public enumerate/structure ----------------------------------------------- */
typedef enum
{
    DEVICE_MODE_STREAMING            = 0,
    DEVICE_MODE_PRODUCTION           = 1,
    DEVICE_MODE_MAX
}
device_mode_t;
typedef struct
{
    uint8_t heart_rate;
    uint8_t spo2;
}
ppg_phys_data_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}
imu_phys_data_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void sensor_manager_init(void);
void sensor_manager_afe_init(void);

void sensor_manager_task(device_mode_t mode);
void sensor_manager_start(void);
void sensor_manager_stop(void);

base_status_t sensor_manager_afe_task(device_mode_t mode, ppg_phys_data_t *ppg_phys);
base_status_t sensor_manager_imu_task(device_mode_t mode, imu_phys_data_t *imu_phys);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
