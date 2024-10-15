/*
 * File Name: user_interface_manager_oled.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: OLED Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------------- */
#include "user_interface_manager_oled.h"
#include "user_interface_manager_led.h"
#include "bsp_oled.h"
#include "icon.h"
#include "system_manager.h"
#include "ble_peripheral.h"
#include "bsp_timer.h"

/* Private defines ---------------------------------------------------- */
#define OLED_MANAGER_THREAD_STACK_SIZE (2048)
K_THREAD_STACK_DEFINE(oled_manager_stack, OLED_MANAGER_THREAD_STACK_SIZE);
static struct k_thread oled_manager_thread;
static k_tid_t oled_manager_thread_id;

/* Private enumerate/structure ---------------------------------------------- */
typedef struct
{
    uint8_t width;
    uint8_t height;
    uint16_t color;
    const uint8_t *data;
} ui_img_t;

typedef struct
{
    uint8_t x_pos;
    uint8_t y_pos;
    ui_img_t img;
} ui_oled_t;

/* Private macros ----------------------------------------------------- */
#define ITEM_INFO(item, _x_pos, _y_pos, _w, _h, _c, _data) [item] = {   \
    .x_pos      = _x_pos, \
    .y_pos      = _y_pos, \
    .img.width  = _w,     \
    .img.height = _h,     \
    .img.color  = _c,     \
    .img.data   = _data   \
}

/* Private variables -------------------------------------------------------- */
static ui_oled_t ITEMS_TABLE[] = 
{
  //          +=====================+=======+=======+==============================+===========================+============================+
  //          |ITEMS                | X-Pos | Y-Pos | Width                        | Height                    | Data                       |
  //          +---------------------+-------+-------+------------------------------+---------------------------+----------------------------+
     ITEM_INFO(UI_BATT_0            ,     94,      0,   BATTERY_LEVEL_0_WIDTH,     BATTERY_LEVEL_0_HEIGHT,     SSD1306_COLOR_WHITE,  battery_level_0_data        )
    ,ITEM_INFO(UI_BATT_25           ,     94,      0,   BATTERY_LEVEL_25_WIDTH,    BATTERY_LEVEL_0_HEIGHT,     SSD1306_COLOR_WHITE,  battery_level_25_data       )
    ,ITEM_INFO(UI_BATT_50           ,     94,      0,   BATTERY_LEVEL_50_WIDTH,    BATTERY_LEVEL_50_HEIGHT,    SSD1306_COLOR_WHITE,  battery_level_50_data       )
    ,ITEM_INFO(UI_BATT_75           ,     94,      0,   BATTERY_LEVEL_75_WIDTH,    BATTERY_LEVEL_75_HEIGHT,    SSD1306_COLOR_WHITE,  battery_level_75_data       )
    ,ITEM_INFO(UI_BATT_100          ,     94,      0,   BATTERY_LEVEL_100_WIDTH,   BATTERY_LEVEL_100_HEIGHT,   SSD1306_COLOR_WHITE,  battery_level_100_data      )
    ,ITEM_INFO(UI_BLE               ,     66,      0,   BLE_WIDTH,                 BLE_HEIGHT,                 SSD1306_COLOR_WHITE,  ble_data                    )
    ,ITEM_INFO(UI_BLE_ACTIVE        ,     66,      0,   BLE_ACTIVE_WIDTH,          BLE_ACTIVE_HEIGHT,          SSD1306_COLOR_WHITE,  ble_active_data             )
    ,ITEM_INFO(UI_BOLT              ,     18,      0,   BOLT_WIDTH,                BOLT_HEIGHT,                SSD1306_COLOR_WHITE,  bolt_data                   )
    ,ITEM_INFO(UI_CHECK_CIRCLE      ,      4,      0,   CHECK_CIRCLE_WIDTH,        CHECK_CIRCLE_HEIGHT,        SSD1306_COLOR_WHITE,  check_circle_data           )
    ,ITEM_INFO(UI_CIRCLE            ,      4,      0,   CIRCLE_WIDTH,              CIRCLE_HEIGHT,              SSD1306_COLOR_WHITE,  circle_data                 )
    ,ITEM_INFO(UI_HEART_OUTLINE     ,      8,     50,   HEART_OUTLINE_WIDTH,       HEART_OUTLINE_HEIGHT,       SSD1306_COLOR_WHITE,  heart_outline_data          )
    ,ITEM_INFO(UI_HEART_SOLID       ,      8,     50,   HEART_SOLID_WIDTH,         HEART_SOLID_HEIGHT,         SSD1306_COLOR_WHITE,  heart_solid_data            )
    ,ITEM_INFO(UI_VECTOR            ,     20,      0,   VECTOR_WIDTH,              VECTOR_HEIGHT,              SSD1306_COLOR_WHITE,  vector_data                 )
    ,ITEM_INFO(UI_SIGNAL_ACQ_1      ,      0,      0,   SIGNAL_ACQ_1_WIDTH,        SIGNAL_ACQ_1_HEIGHT,        SSD1306_COLOR_WHITE,  signal_acq_1_data           )
    ,ITEM_INFO(UI_SIGNAL_ACQ_2      ,      0,      0,   SIGNAL_ACQ_2_WIDTH,        SIGNAL_ACQ_2_HEIGHT,        SSD1306_COLOR_WHITE,  signal_acq_2_data           )
    ,ITEM_INFO(UI_TEMPERATURE       ,     50,     50,   TEMPERATURE_WIDTH,         TEMPERATURE_HEIGHT,         SSD1306_COLOR_WHITE,  temperature_data            )
    ,ITEM_INFO(UI_ZIGBEE_SIGNAL_0   ,     80,      0,   ZIGBEE_SIGNAL_0_WIDTH,     ZIGBEE_SIGNAL_0_HEIGHT,     SSD1306_COLOR_WHITE,  zigbee_signal_0_data        )
    ,ITEM_INFO(UI_ZIGBEE_SIGNAL_1   ,     80,      0,   ZIGBEE_SIGNAL_1_WIDTH,     ZIGBEE_SIGNAL_1_HEIGHT,     SSD1306_COLOR_WHITE,  zigbee_signal_1_data        )
    ,ITEM_INFO(UI_ZIGBEE_SIGNAL_2   ,     80,      0,   ZIGBEE_SIGNAL_2_WIDTH,     ZIGBEE_SIGNAL_2_HEIGHT,     SSD1306_COLOR_WHITE,  zigbee_signal_2_data        )
    ,ITEM_INFO(UI_ZIGBEE_SIGNAL_3   ,     80,      0,   ZIGBEE_SIGNAL_3_WIDTH,     ZIGBEE_SIGNAL_3_HEIGHT,     SSD1306_COLOR_WHITE,  zigbee_signal_3_data        )
    ,ITEM_INFO(UI_ZIGBEE_SIGNAL_4   ,     80,      0,   ZIGBEE_SIGNAL_4_WIDTH,     ZIGBEE_SIGNAL_4_HEIGHT,     SSD1306_COLOR_WHITE,  zigbee_signal_4_data        )
    ,ITEM_INFO(UI_CLEAR_12_12       ,      0,      0,   CLEAR_12_12_WIDTH,         CLEAR_12_12_HEIGHT,         SSD1306_COLOR_BLACK,  clear_12_12_data            )
    ,ITEM_INFO(UI_CLEAR_24_24       ,      0,      0,   CLEAR_24_24_WIDTH,         CLEAR_24_24_HEIGHT,         SSD1306_COLOR_BLACK,  clear_24_24_data            )
  //          +=====================+=======+=======+===============================+===========================+===========================+
};

static ui_oled_state_t oled_state = UI_OLED_STATE_IDLE;

static tmr_t oled_task_timer;

/* Private Constants -------------------------------------------------------- */
/* Private prototypes ------------------------------------------------------- */
static void user_interface_manager_oled_display_no_connection(void);
static void user_interface_manager_oled_display_measuring(void);
static void user_interface_manager_oled_display_measure_data(void);
static void user_interface_manager_oled_display_charging(void);
static void user_interface_manager_oled_display_error(ui_error_type_t error);
static void user_interface_manager_oled_display_mac_address(void);
static void user_interface_manager_oled_display_power_off(void);

static void user_interface_manager_oled_display_server_connect_status(void);
static void user_interface_manager_oled_display_comm_with_server(void);
static void user_interface_manager_oled_display_ble_signal(void);
static void user_interface_manager_oled_display_zigbee_signal(void);
static void user_interface_manager_oled_display_battery_level(void);
static void user_interface_manager_oled_display_measure_interval(void);

static void user_interface_manager_oled_display_image_position(uint16_t x_pos, uint16_t y_pos, ui_oled_item_t item);
static void user_interface_manager_oled_display_image(ui_oled_item_t item);

static void user_interface_manager_oled_task(void *p1, void *p2, void *p3);

/* Private macros ----------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_oled_init(void)
{
    bsp_oled_init();

    // Create the thread
    oled_manager_thread_id = k_thread_create(&oled_manager_thread,                      //!< Pointer to initialize thread
                                             oled_manager_stack,                        //!< Pointer to stack space
                                             K_THREAD_STACK_SIZEOF(oled_manager_stack), //!< Stack size in bytes
                                             user_interface_manager_oled_task,          //!< Entry thread function
                                             NULL,                                      //!< 1st entry point parameter
                                             NULL,                                      //!< 2nd entry point parameter
                                             NULL,                                      //!< 3rd entry point parameter
                                             K_PRIO_PREEMPT(7),                            //!< Thread priority
                                             0,                                         //!< Thread option
                                             K_NO_WAIT);                                //!< Scheduling delay (in milliseconds)

    k_thread_name_set(&oled_manager_thread, "user_interface_manager_oled");
}

void user_interface_manager_oled_update_state(ui_oled_state_t state)
{
    oled_state = state;

    // Event that need to update immediately
    switch (oled_state)
    {
    case UI_OLED_STATE_CHARGING:
        bsp_tmr_restart(&oled_task_timer, 5000);
        user_interface_manager_oled_display_charging();
        break;
    
    case UI_OLED_STATE_POWER_OFF:
        user_interface_manager_oled_display_power_off();
        break;
    
    case UI_OLED_STATE_MEASURING:
        bsp_tmr_start(&oled_task_timer, 1000);
        break;

    default:
        break;
    }
}

static void user_interface_manager_oled_task(void *p1, void *p2, void *p3)
{
    while (1)
    {
        switch (oled_state)
        {
        case UI_OLED_STATE_INIT:
            user_interface_manager_oled_display_init();
            oled_state = UI_OLED_STATE_IDLE;
            break;

        case UI_OLED_STATE_CHECK_CONNECTION:
            user_interface_manager_oled_display_no_connection();
            oled_state = UI_OLED_STATE_IDLE;
            break;

        case UI_OLED_STATE_MEASURING:
            if (bsp_tmr_is_expired(&oled_task_timer))
            {
                user_interface_manager_oled_display_measuring();
                bsp_tmr_restart(&oled_task_timer, 1000);
            }
            break;

        case UI_OLED_STATE_MEASURE_VALUE:
            user_interface_manager_oled_display_measure_data();
            oled_state = UI_OLED_STATE_WAIT_TO_CLEAR;
            bsp_tmr_restart(&oled_task_timer, g_device.oled_switch_off_period_seconds * 1000);
            break;

        case UI_OLED_STATE_WAIT_TO_CLEAR:
            if (bsp_tmr_is_expired(&oled_task_timer))
            {
                bsp_oled_clear();
                bsp_oled_update_screen();
                user_interface_manager_led_write_status(UI_LED_SILENT_MODE, true);
                oled_state = UI_OLED_STATE_CLEAR;
            }
            break;

        case UI_OLED_STATE_CLEAR:
            bsp_oled_clear();
            bsp_oled_update_screen();
            oled_state = UI_OLED_STATE_IDLE;
            break;

        case UI_OLED_STATE_CHARGING:
            if (bsp_tmr_is_expired(&oled_task_timer))
            {
                user_interface_manager_oled_display_charging();
                bsp_tmr_restart(&oled_task_timer, 5000);
            }
            break;

        case UI_OLED_STATE_ERROR:
            user_interface_manager_oled_display_error(UI_GENERAL_ERROR);
            break;

        case UI_OLED_STATE_MAC_DISPLAY:
            user_interface_manager_oled_display_mac_address();
            user_interface_manager_led_write_status(UI_LED_DISPLAY_MAC_ADDRESS, true);

            oled_state = UI_OLED_STATE_WAIT_TO_CLEAR;
            bsp_tmr_restart(&oled_task_timer, g_device.oled_switch_off_period_seconds * 1000);
            break;

        case UI_OLED_STATE_IDLE:
            break;

        default:
            break;
        }

        SYSTEM_DELAY_MSEC(100);
    }
}

void user_interface_manager_oled_display_header(bool force_update)
{
    user_interface_manager_oled_display_server_connect_status();
    user_interface_manager_oled_display_comm_with_server();
    user_interface_manager_oled_display_ble_signal();
    user_interface_manager_oled_display_zigbee_signal();
    user_interface_manager_oled_display_battery_level();
    user_interface_manager_oled_display_measure_interval();

    if (force_update)
    {
        bsp_oled_update_screen();
    }
}

void user_interface_manager_oled_display_init(void)
{
    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);

    bsp_oled_write_string(28, 22, "Salus", ssd1306_font_16x26);
    bsp_oled_write_string(20, 48, "Equine Monitor", ssd1306_font_7x10);

    bsp_oled_update_screen();
}

/* Private implementations -------------------------------------------------- */
static void user_interface_manager_oled_display_power_off(void)
{
    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);

    bsp_oled_write_string(28, 22, "Salus", ssd1306_font_16x26);
    bsp_oled_write_string(22, 48, "Shutting down", ssd1306_font_7x10);

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_no_connection(void)
{
    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);

    bsp_oled_write_string(10, 22, "No Network", ssd13315_font_11x18);
    bsp_oled_write_string(25, 48, "Searching ...", ssd1306_font_7x10);

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_mac_address(void)
{
    char mac_add[20];

    bsp_oled_clear();

    ble_peripheral_get_mac_address(mac_add);

    user_interface_manager_oled_display_header(false);

    bsp_oled_write_string(25, 22, "MAC Address", ssd1306_font_7x10);
    bsp_oled_write_string(5, 40, mac_add, ssd1306_font_7x10);

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_measuring(void)
{
    static bool wavey_line = false;
    char buf[10];

    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);

    user_interface_manager_oled_display_image(UI_HEART_OUTLINE);

    snprintf(buf, sizeof(buf), "%.2f", g_device.avr_skin_temp);
    bsp_oled_write_string(38, 28, buf, ssd13315_font_11x18);
    user_interface_manager_oled_display_image(UI_TEMPERATURE);
    bsp_oled_write_string(65, 52, "C", ssd1306_font_7x10);

    bsp_oled_write_string(100, 52, "SPO2", ssd1306_font_7x10);

    if (!system_manager_check_device_is_remove_from_horse())
    {
        // Display previous heart rate and SPO2
        snprintf(buf, sizeof(buf), "%d", g_device.ppg_phys_prv.heart_rate);
        bsp_oled_write_string(4, 28, buf, ssd13315_font_11x18);

        snprintf(buf, sizeof(buf), "%d", g_device.ppg_phys_prv.spo2);
        bsp_oled_write_string(106, 28, buf, ssd13315_font_11x18);
    }

    // Display wavey lines
    if (wavey_line)
    {
        if (system_manager_check_device_is_remove_from_horse())
        {
            user_interface_manager_oled_display_image_position(3, 20, UI_CLEAR_24_24);
            user_interface_manager_oled_display_image_position(3, 20, UI_SIGNAL_ACQ_1);

            user_interface_manager_oled_display_image_position(102, 20, UI_CLEAR_24_24);
            user_interface_manager_oled_display_image_position(102, 20, UI_SIGNAL_ACQ_1);
        }
        else
        {
            user_interface_manager_oled_display_image_position(8, 50, UI_CLEAR_12_12);
            user_interface_manager_oled_display_image(UI_HEART_OUTLINE);
        }

        wavey_line = false;
    }
    else
    {
        if (system_manager_check_device_is_remove_from_horse())
        {
            user_interface_manager_oled_display_image_position(3, 20, UI_CLEAR_24_24);
            user_interface_manager_oled_display_image_position(3, 20, UI_SIGNAL_ACQ_2);

            user_interface_manager_oled_display_image_position(102, 20, UI_CLEAR_24_24);
            user_interface_manager_oled_display_image_position(102, 20, UI_SIGNAL_ACQ_2);
        }
        else
        {
            user_interface_manager_oled_display_image_position(8, 50, UI_CLEAR_12_12);
            user_interface_manager_oled_display_image(UI_HEART_SOLID);
        }

        wavey_line = true;
    }

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_measure_data(void)
{
    char buf[10];

    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);

    snprintf(buf, sizeof(buf), "%d", g_device.ppg_phys.heart_rate);
    bsp_oled_write_string(4, 28, buf, ssd13315_font_11x18);
    user_interface_manager_oled_display_image(UI_HEART_OUTLINE);

    snprintf(buf, sizeof(buf), "%.2f", g_device.avr_skin_temp);
    bsp_oled_write_string(38, 28, buf, ssd13315_font_11x18);
    user_interface_manager_oled_display_image(UI_TEMPERATURE);
    bsp_oled_write_string(65, 52, "C", ssd1306_font_7x10);

    snprintf(buf, sizeof(buf), "%d", g_device.ppg_phys.spo2);
    bsp_oled_write_string(106, 28, buf, ssd13315_font_11x18);
    bsp_oled_write_string(100, 52, "SPO2", ssd1306_font_7x10);

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_charging(void)
{
    char buf[10];
    
    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);
    snprintf(buf, sizeof(buf), "%d%%", g_device.battery.level);

    bsp_oled_write_string(48, 28, buf, ssd13315_font_11x18);
    bsp_oled_write_string(38, 52, "Charging", ssd1306_font_7x10);

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_error(ui_error_type_t error)
{
    bsp_oled_clear();

    user_interface_manager_oled_display_header(false);

    switch (error)
    {
    case UI_GENERAL_ERROR:
        bsp_oled_write_string(6, 26, "General Err", ssd13315_font_11x18);
        break;

    case UI_TEMP_ERROR:
        bsp_oled_write_string(12, 26, "Temp Error", ssd13315_font_11x18);
        break;

    case UI_IMU_ERROR:
        bsp_oled_write_string(18, 26, "IMU Error", ssd13315_font_11x18);
        break;

    case UI_AFE_ERROR:
        bsp_oled_write_string(18, 26, "AFE Error", ssd13315_font_11x18);
        break;

    case UI_AMBIENT_ERROR:
        bsp_oled_write_string(4, 26, "Ambient Err", ssd13315_font_11x18);
        break;

    default:
        break;
    }

    bsp_oled_write_string(14, 52, "Contact Support", ssd1306_font_7x10);

    bsp_oled_update_screen();
}

static void user_interface_manager_oled_display_server_connect_status(void)
{
    user_interface_manager_oled_display_image_position(ITEMS_TABLE[UI_CHECK_CIRCLE].x_pos, ITEMS_TABLE[UI_CHECK_CIRCLE].y_pos, UI_CLEAR_12_12);

    if (g_device.comm.ble.connected == BLE_CONNECTED)
    {
        user_interface_manager_oled_display_image(UI_CHECK_CIRCLE);
    }
    else
    {
        user_interface_manager_oled_display_image(UI_CIRCLE);
    }
}

static void user_interface_manager_oled_display_comm_with_server(void)
{
    if (g_device.comm.server.communicated)
    {
        user_interface_manager_oled_display_image(UI_BOLT);
    }
}

static void user_interface_manager_oled_display_ble_signal(void)
{
    user_interface_manager_oled_display_image(UI_BLE_ACTIVE);
}

static void user_interface_manager_oled_display_zigbee_signal(void)
{
    user_interface_manager_oled_display_image(UI_ZIGBEE_SIGNAL_0);
}

static void user_interface_manager_oled_display_battery_level(void)
{
    char buf[5];

    user_interface_manager_oled_display_image_position(ITEMS_TABLE[UI_BATT_0].x_pos, ITEMS_TABLE[UI_BATT_0].y_pos, UI_CLEAR_12_12);

    if (g_device.battery.level < 5)
    {
        user_interface_manager_oled_display_image(UI_BATT_0);
    }
    else if ((g_device.battery.level >= 5) && (g_device.battery.level < 25))
    {
        user_interface_manager_oled_display_image(UI_BATT_25);
    }
    else if ((g_device.battery.level >= 25) && (g_device.battery.level < 50))
    {
        user_interface_manager_oled_display_image(UI_BATT_50);
    }
    else if ((g_device.battery.level >= 50) && (g_device.battery.level < 75))
    {
        user_interface_manager_oled_display_image(UI_BATT_75);
    }
    else if ((g_device.battery.level >= 75) && (g_device.battery.level < 100))
    {
        user_interface_manager_oled_display_image(UI_BATT_100);
    }

    snprintf(buf, sizeof(buf), "%d", g_device.battery.level);
    bsp_oled_write_string(110, 2, buf, ssd1306_font_6x8);
}

static void user_interface_manager_oled_display_measure_interval(void)
{
    char buf[5];

    snprintf(buf, sizeof(buf), "i-%d", g_device.study.interval_min);
    bsp_oled_write_string(32, 2, buf, ssd1306_font_6x8);
}

static void user_interface_manager_oled_display_image(ui_oled_item_t item)
{
    bsp_oled_draw_image(ITEMS_TABLE[item].x_pos, ITEMS_TABLE[item].y_pos,
                        ITEMS_TABLE[item].img.width, ITEMS_TABLE[item].img.height, ITEMS_TABLE[item].img.color, ITEMS_TABLE[item].img.data);
}

static void user_interface_manager_oled_display_image_position(uint16_t x_pos, uint16_t y_pos, ui_oled_item_t item)
{
    bsp_oled_draw_image(x_pos, y_pos, ITEMS_TABLE[item].img.width, ITEMS_TABLE[item].img.height, ITEMS_TABLE[item].img.color, ITEMS_TABLE[item].img.data);
}

/* End of file -------------------------------------------------------------- */
