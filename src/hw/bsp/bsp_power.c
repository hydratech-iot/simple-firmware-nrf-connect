/*
 * File Name: bsp_power.c
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
#include "bsp_power.h"
#include "base_board_defs.h"
#include "bsp_io.h"
#include "user_interface_manager_led.h"
#include "user_interface_manager_oled.h"
#include "bsp_timer.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static tmr_t power_on_timer;

/* Private function prototypes ---------------------------------------- */
static void cold_boot(void);
static void power_off(struct k_timer *timer);

/* Function definitions ----------------------------------------------- */
void bsp_power_boot(void)
{
    cold_boot();
}

void bsp_power_off(void)
{
    user_interface_manager_led_write_status(UI_LED_POWER_OFF_PROCESS, true);
    user_interface_manager_oled_update_state(UI_OLED_STATE_POWER_OFF);

    SYSTEM_DELAY_MSEC(3000); // Wait for power stable
    power_off(NULL);
}

/* Private function definitions ---------------------------------------- */
static void cold_boot(void)
{
    bsp_tmr_start(&power_on_timer, 3000);

    while (!bsp_tmr_is_expired(&power_on_timer))
    {
        if (bsp_io_read_specific_pin(IO_PIN_PWR_SWITCH_DETECT) == 1)
        {
            power_off(NULL);

            while (1)
                ;
        }
    }

    user_interface_manager_led_write_status(UI_LED_POWERED_ON, true);
}

static void power_off(struct k_timer *timer)
{
    bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 0);
    bsp_io_write_specific_pin(IO_PIN_MCU_PWR_EN, 0);
    bsp_io_write_specific_pin(IO_PIN_SYSTEM_PWR_EN, 0);
}

/* End of file -------------------------------------------------------- */
