/*
 * File Name: bsp_led.c
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

/* Includes ----------------------------------------------------------- */
#include "bsp_pwm.h"
#include "base_board_defs.h"

/* Private defines ---------------------------------------------------- */
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(bsp_pwm, LOG_LEVEL);

#define LED_STATUS_RED_DUTY_CYCLE(x)        ((x * DT_PWMS_PERIOD(IO_RGB_RED_NODE_ID)    / 100))
#define LED_STATUS_GREEN_DUTY_CYCLE(x)      ((x * DT_PWMS_PERIOD(IO_RGB_GREEN_NODE_ID)  / 100))
#define LED_STATUS_BLUE_DUTY_CYCLE(x)       ((x * DT_PWMS_PERIOD(IO_RGB_BLUE_NODE_ID)   / 100))

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static const struct pwm_dt_spec led_status_red_dev   = PWM_DT_SPEC_GET(IO_RGB_RED_NODE_ID);
static const struct pwm_dt_spec led_status_green_dev = PWM_DT_SPEC_GET(IO_RGB_GREEN_NODE_ID);
static const struct pwm_dt_spec led_status_blue_dev  = PWM_DT_SPEC_GET(IO_RGB_BLUE_NODE_ID);

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
base_status_t bsp_pwm_init(void)
{
    LOG_DBG("PWM initialized succeeded");
    if (!device_is_ready(led_status_red_dev.dev))
    {
        LOG_ERR("led_status_red_dev is not ready");
        return BS_ERROR;
    }
    if (!device_is_ready(led_status_green_dev.dev))
    {
        LOG_ERR("led_status_green_dev is not ready");
        return BS_ERROR;
    }
    if (!device_is_ready(led_status_blue_dev.dev))
    {
        LOG_ERR("led_status_blue_dev is not ready");
        return BS_ERROR;
    }

    // Turn off all of LEDs when we just power on
    bsp_pwm_set_percent(PWM_CHANNEL_1, 0);
    bsp_pwm_set_percent(PWM_CHANNEL_2, 0);
    bsp_pwm_set_percent(PWM_CHANNEL_3, 0);

    return BS_OK;
}

base_status_t bsp_pwm_set_percent(pwm_channel_t channel, uint8_t percent)
{
    uint32_t pulse;

    if (channel == PWM_CHANNEL_1)
    {
        pulse = LED_STATUS_RED_DUTY_CYCLE(percent);
        if (pwm_set_pulse_dt(&led_status_red_dev, pulse) != 0)
        {
            return BS_ERROR;
        }
    }
    else if (channel == PWM_CHANNEL_2)
    {
        pulse = LED_STATUS_GREEN_DUTY_CYCLE(percent);
        if (pwm_set_pulse_dt(&led_status_green_dev, pulse) != 0)
        {
            return BS_ERROR;
        }
    }
    else if (channel == PWM_CHANNEL_3)
    {
        pulse = LED_STATUS_BLUE_DUTY_CYCLE(percent);
        if (pwm_set_pulse_dt(&led_status_blue_dev, pulse) != 0)
        {
            return BS_ERROR;
        }
    }

    return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */
