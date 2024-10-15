/*
 * File Name: bsp_io.h
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

/* Includes ----------------------------------------------------------- */
#include "base_type.h"
#include "base_board_defs.h"
#include "base_include.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
     IO_PIN_PWR_SWITCH_DETECT = 0
    ,IO_PIN_IMU_INT1
    ,IO_PIN_PPG_INT1
    ,IO_PIN_BODY_TEMP_1_ALERT
    ,IO_PIN_BODY_TEMP_2_ALERT
    ,IO_PIN_BODY_TEMP_3_ALERT
    ,IO_PIN_BODY_TEMP_4_ALERT
    ,IO_PIN_AMB_TEMP_ALERT
    ,IO_PIN_CHG_PGOOD
    ,IO_PIN_MCU_PWR_EN
    ,IO_PIN_SYSTEM_PWR_EN
    ,IO_PIN_OLED_PWR_EN
    ,IO_PIN_SENSOR_PWR_EN
    ,IO_PIN_I2C_BUFFER_EN
    ,IO_PIN_PPG_SPI_CS
    ,IO_PIN_LOGIC_SER_PWR_EN
    ,IO_PIN_MAX
}
io_pin_t;

/* Public macros ------------------------------------------------------ */
#define bsp_io_write_pin(io, level)\
    ({\
        gpio_pin_set_dt(io, level);\
    })

#define bsp_io_read_pin(io)\
    ({\
        gpio_pin_get_dt(io);\
    })

#define bsp_io_set_config_pin(io, direction)\
    ({\
        gpio_pin_configure_dt(io, direction);\
    })

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void bsp_io_init(void);
void bsp_io_write_specific_pin(io_pin_t pin, bool level);
bool bsp_io_read_specific_pin(io_pin_t pin);

base_status_t bsp_io_attach_interrupt(struct gpio_dt_spec gpio_dev, struct gpio_callback *gpio_callback_data, gpio_callback_handler_t gpio_callback_handler, gpio_flags_t flags);

/* End of file -------------------------------------------------------- */

