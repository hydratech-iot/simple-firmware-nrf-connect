/*
 * File Name: user_interface_manager_led.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: LED Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------------- */
#include "user_interface_manager_led.h"

/* Private defines ---------------------------------------------------- */
#define LED_TASK_INTERVAL     APP_TIMER_TICKS(10)       // Unit: ms
#define LED_MANAGER_THREAD_STACK_SIZE (2048)

/* Private enumerate/structure ---------------------------------------------- */
enum
{
    LED_STATE_ON,
    LED_STATE_OFF
};

#define LED(_id, _color, _mode, _br)      \
    {                                     \
        .id         = UI_LED_##_id,       \
        .color      = LED_COLOR_##_color, \
        .mode       = LED_MODE_##_mode,   \
        .brightness = _br,                \
    }

const ui_led_info_t LED_LIST[] =
{
  //   +=================================================+=========+
  //   |                     |           |            |            |
  //   |       LED ID        | LED Color | LED Mode   | Brightness |
  //   +---------------------+-----------+------------+------------+
  //____ VERY HIGH _________________________________________________
    LED( POWER_ON_PROCESS     , BLUE      , FLASH_FAST ,         2 )
  , LED( POWER_OFF_PROCESS    , GREEN     , FLASH_FAST ,         2 )
  , LED( POWERED_ON           , GREEN     , ON         ,         2 )
  , LED( ERROR                , YELLOW    , FLASH      ,         4 )

  //____ HIGH ______________________________________________________

  //____ MEDIUM ____________________________________________________
  , LED( CHARGING_BATTERY_LOW           , RED       , FLASH      ,        2 )
  , LED( CHARGING_BATTERY_MID           , GREEN      , ON        ,        2 )
  , LED( CHARGING_BATTERY_HIGH          , GREEN      , FLASH     ,        2 )
  , LED( CHARGING_FULL                  , BLUE      , ON         ,         2 )
  , LED( DISCHARGING_BATTERY_LOW        , RED       , FLASH_ONE_EVERY_3_SECONDS         ,         2 )
  , LED( DISPLAY_MAC_ADDRESS            , GREEN       , FLASH         ,         2 )

  //____ LOW _______________________________________________________
  , LED( LOW_BATTERY         , BLUE      , FLASH      ,         2 )
  , LED( STUDY_ONGOING       , GREEN     , PULSE      ,        10 )
  , LED( SILENT_MODE         , GREEN     , OFF        ,         0 )

  //____ VERY LOW __________________________________________________

  //   +---------------------+-----------+------------+------------+
};

struct
{
  uint8_t cur;
  uint8_t next;
}
ui_led_id;

/* Private Constants -------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
K_THREAD_STACK_DEFINE(led_manager_stack, LED_MANAGER_THREAD_STACK_SIZE);
static struct k_thread led_manager_thread;
static k_tid_t led_manager_thread_id;

static bool ui_led_status[UI_LED_UNKNOWN] = {false};

/* Private prototypes ------------------------------------------------------- */
static void ui_update_led(ui_led_id_t led);
static void user_interface_manager_led_task(void *p1, void *p2, void *p3);

/* Private macros ----------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_led_init(void)
{
    bsp_led_init();

    for (uint_fast32_t i = 0; i < UI_LED_UNKNOWN; i++)
    {
        user_interface_manager_led_write_status((ui_led_id_t)i, false);
    }

    ui_led_id.cur = UI_LED_UNKNOWN;
    ui_led_id.next = UI_LED_UNKNOWN;

    led_manager_thread_id = k_thread_create(&led_manager_thread,                      //!< Pointer to initialize thread
                                            led_manager_stack,                        //!< Pointer to stack space
                                            K_THREAD_STACK_SIZEOF(led_manager_stack), //!< Stack size in bytes
                                            user_interface_manager_led_task,          //!< Entry thread function
                                            NULL,                                     //!< 1st entry point parameter
                                            NULL,                                     //!< 2nd entry point parameter
                                            NULL,                                     //!< 3rd entry point parameter
                                            K_PRIO_COOP(4),                           //!< Thread priority
                                            0,                                        //!< Thread option
                                            K_NO_WAIT);                               //!< Scheduling delay (in milliseconds)

    k_thread_name_set(&led_manager_thread, "user_interface_manager_led");

    user_interface_manager_led_write_status(UI_LED_POWER_ON_PROCESS, true);
}

void user_interface_manager_led_write_status(ui_led_id_t led, bool status)
{
    for (uint_fast32_t i = 0; i < UI_LED_UNKNOWN; i++)
    {
        ui_led_status[i] = false;
    }

    ui_led_status[led] = status;
}

/* Private functions -------------------------------------------------------- */
static void user_interface_manager_led_task(void *p1, void *p2, void *p3)
{
    while (1)
    {
        for (uint_fast32_t i = 0; i < UI_LED_UNKNOWN; i++)
        {
            if ((ui_led_status[i]))
            {
                ui_led_id.next = (ui_led_id_t)i;
                break;
            }
        }

        ui_update_led(ui_led_id.next);
        bsp_led_update_led_state();

        SYSTEM_DELAY_MSEC(100);
    }
}

static void ui_update_led(ui_led_id_t led)
{
    // Led ID
    if (ui_led_id.cur != led)
    {
        bsp_led_set_led(LED_LIST[led].mode, LED_LIST[led].color, LED_LIST[led].brightness);
    }

    // Update information
    ui_led_id.cur = led;
}

/* Private implementations -------------------------------------------------- */
/* End of file -------------------------------------------------------------- */
