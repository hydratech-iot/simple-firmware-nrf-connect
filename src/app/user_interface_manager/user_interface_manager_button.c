/*
 * File Name: user_interface_manager_button.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Button Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------------- */
#include "user_interface_manager_button.h"
#include "user_interface_manager_led.h"
#include "user_interface_manager_oled.h"
#include "base_board_defs.h"
#include "bsp_power.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "system_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(user_interface_manager_button, CONFIG_LOG_DEFAULT_LEVEL);

#define BUTTON_MANAGER_THREAD_STACK_SIZE (2048)

/* Private enumerate/structure ---------------------------------------------- */
/* Private Constants -------------------------------------------------------- */
#define BUTTON_NUM            (1)
#define BTN_DEBOUNCE_TIME     (50)       // Unit: ms
#define BTN_CLICK_TIME        (1000)     // Unit: ms

/* Private variables -------------------------------------------------------- */
K_THREAD_STACK_DEFINE(button_manager_stack, BUTTON_MANAGER_THREAD_STACK_SIZE);
static struct k_thread button_manager_thread;
static k_tid_t button_manager_thread_id;

static one_button_t ob[BUTTON_NUM];

/* Private prototypes ------------------------------------------------------- */
static void user_interface_manager_button_callback(int pin, one_button_event_t evt);
static void user_interface_manager_button_task(void *p1, void *p2, void *p3);

int read_pin(int pin)
{
    return (int)bsp_io_read_specific_pin(pin);
}

uint32_t get_tick()
{
    return bsp_tmr_get_tick_ms();
}

/* Private macros ----------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_button_init(void)
{
    uint32_t pin_arr[] = {IO_PIN_PWR_SWITCH_DETECT};
    uint32_t press_time_arr[] = {2000};

    one_button_cfg_t ob_cfg;
    ob_cfg._evt_mask      = (OB_EVT_CLICK | OB_EVT_LONG_PRESS_START | OB_EVT_DOUBLE_CLICK);
    ob_cfg._activeLow     = true;
    ob_cfg._callback      = user_interface_manager_button_callback;
    ob_cfg._debounceTicks = BTN_DEBOUNCE_TIME;
    ob_cfg._clickTicks    = BTN_CLICK_TIME;
    ob_cfg.fp_read_pin    = read_pin;
    ob_cfg.fp_get_tick_ms = get_tick;

    for (int i = 0; i < BUTTON_NUM; i++)
    {
        ob_cfg._pin = pin_arr[i];
        ob_cfg._pressTicks = press_time_arr[i];
        one_button_init(&ob[i], &ob_cfg);
        one_button_reset(&ob[i]);
    }

    button_manager_thread_id = k_thread_create(&button_manager_thread,                      //!< Pointer to initialize thread
                                               button_manager_stack,                        //!< Pointer to stack space
                                               K_THREAD_STACK_SIZEOF(button_manager_stack), //!< Stack size in bytes
                                               user_interface_manager_button_task,          //!< Entry thread function
                                               NULL,                                        //!< 1st entry point parameter
                                               NULL,                                        //!< 2nd entry point parameter
                                               NULL,                                        //!< 3rd entry point parameter
                                               K_PRIO_COOP(4),                              //!< Thread priority
                                               0,                                           //!< Thread option
                                               K_NO_WAIT);                                  //!< Scheduling delay (in milliseconds)

    k_thread_name_set(&button_manager_thread, "user_interface_manager_button");
}

void user_interface_manager_button_callback(int pin, one_button_event_t evt)
{
    if (OB_EVT_CLICK == evt)
    {
        LOG_INF("Button event click \n");

        if (g_device.battery.is_charging)
        {
            system_manager_display_charging();
        }
        else
        {
            g_device.first_run_system_manager = true;
            system_manager_study_start(true);
        }
    }
    else if (OB_EVT_LONG_PRESS_START == evt)
    {
        LOG_INF("Button long press start \n");
        system_manager_state_set(SYSTEM_MANAGER_STATE_POWER_OFF);
    }
    else if (OB_EVT_DOUBLE_CLICK == evt)
    {
        LOG_INF("Button event double click \n");
        user_interface_manager_oled_update_state(UI_OLED_STATE_MAC_DISPLAY);
    }
}

/* Private functions -------------------------------------------------------- */
static void user_interface_manager_button_task(void *p1, void *p2, void *p3)
{
    while (1)
    {
        for (int i = 0; i < BUTTON_NUM; i++)
        {
            one_button_tick(&ob[i]);
        }

        SYSTEM_DELAY_MSEC(50);
    }
}

/* Private implementations -------------------------------------------------- */
/* End of file -------------------------------------------------------------- */
