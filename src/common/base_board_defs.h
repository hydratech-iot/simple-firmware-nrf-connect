/*
 * File Name: base_board_defs.h
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description:
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include "base_include.h"

/* Public defines ----------------------------------------------------- */
// Input Node
#define IO_PWR_SWITCH_DETECT_NODE_ID        DT_ALIAS(button_pwr_switch_detect)
#define IO_IMU_INT1_NODE_ID                 DT_ALIAS(imu_int_1)
#define IO_PPG_INT1_NODE_ID                 DT_ALIAS(ppg_int)
#define IO_BODY_TEMP_1_ALERT_NODE_ID        DT_ALIAS(body_temp_1_alert)
#define IO_BODY_TEMP_2_ALERT_NODE_ID        DT_ALIAS(body_temp_2_alert)
#define IO_BODY_TEMP_3_ALERT_NODE_ID        DT_ALIAS(body_temp_3_alert)
#define IO_BODY_TEMP_4_ALERT_NODE_ID        DT_ALIAS(body_temp_4_alert)
#define IO_AMB_TEMP_ALERT_NODE_ID           DT_ALIAS(amb_temp_alert)
#define IO_CHG_PGOOD_NODE_ID                DT_ALIAS(charge_pgood)

// Input
#define IO_PWR_SWITCH_DETECT                GPIO_DT_SPEC_GET_OR(IO_PWR_SWITCH_DETECT_NODE_ID, gpios, { 0 })
#define IO_IMU_INT1                         GPIO_DT_SPEC_GET_OR(IO_IMU_INT1_NODE_ID, gpios, { 0 })
#define IO_PPG_INT1                         GPIO_DT_SPEC_GET_OR(IO_PPG_INT1_NODE_ID, gpios, { 0 })
#define IO_BODY_TEMP_1_ALERT                GPIO_DT_SPEC_GET_OR(IO_BODY_TEMP_1_ALERT_NODE_ID, gpios, { 0 })
#define IO_BODY_TEMP_2_ALERT                GPIO_DT_SPEC_GET_OR(IO_BODY_TEMP_2_ALERT_NODE_ID, gpios, { 0 })
#define IO_BODY_TEMP_3_ALERT                GPIO_DT_SPEC_GET_OR(IO_BODY_TEMP_3_ALERT_NODE_ID, gpios, { 0 })
#define IO_BODY_TEMP_4_ALERT                GPIO_DT_SPEC_GET_OR(IO_BODY_TEMP_4_ALERT_NODE_ID, gpios, { 0 })
#define IO_AMB_TEMP_ALERT                   GPIO_DT_SPEC_GET_OR(IO_AMB_TEMP_ALERT_NODE_ID, gpios, { 0 })
#define IO_CHG_PGOOD                        GPIO_DT_SPEC_GET_OR(IO_CHG_PGOOD_NODE_ID, gpios, { 0 })

// Output Node
#define IO_MCU_PWR_EN_NODE_ID               DT_ALIAS(mcu_pwr_en)
#define IO_SYSTEM_PWR_EN_NODE_ID            DT_ALIAS(system_pwr_en)
#define IO_SENSOR_PWR_EN_NODE_ID            DT_ALIAS(sensor_pwr_en)
#define IO_OLED_PWR_EN_NODE_ID              DT_ALIAS(oled_pwr_en)
#define IO_I2C_BUFFER_EN_NODE_ID            DT_ALIAS(i2c_buffer_en)
#define IO_PPG_SPI_CS_NODE_ID               DT_ALIAS(ppg_spi_cs)
#define IO_LOGIC_SER_PWR_EN_NODE_ID         DT_ALIAS(logic_ser_pwr_en)

// Output
#define IO_MCU_PWR_EN                       GPIO_DT_SPEC_GET_OR(IO_MCU_PWR_EN_NODE_ID, gpios, { 0 })
#define IO_SYSTEM_PWR_EN                    GPIO_DT_SPEC_GET_OR(IO_SYSTEM_PWR_EN_NODE_ID, gpios, { 0 })
#define IO_OLED_PWR_EN                      GPIO_DT_SPEC_GET_OR(IO_OLED_PWR_EN_NODE_ID, gpios, { 0 })
#define IO_SENSOR_PWR_EN                    GPIO_DT_SPEC_GET_OR(IO_SENSOR_PWR_EN_NODE_ID, gpios, { 0 })
#define IO_I2C_BUFFER_EN                    GPIO_DT_SPEC_GET_OR(IO_I2C_BUFFER_EN_NODE_ID, gpios, { 0 })
#define IO_PPG_SPI_CS                       GPIO_DT_SPEC_GET_OR(IO_PPG_SPI_CS_NODE_ID, gpios, { 0 })
#define IO_LOGIC_SER_PWR_EN                 GPIO_DT_SPEC_GET_OR(IO_LOGIC_SER_PWR_EN_NODE_ID, gpios, { 0 })

// PWM
#define IO_RGB_RED_NODE_ID                  DT_ALIAS(led_status_red)
#define IO_RGB_GREEN_NODE_ID                DT_ALIAS(led_status_green)
#define IO_RGB_BLUE_NODE_ID                 DT_ALIAS(led_status_blue)

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/* -------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
