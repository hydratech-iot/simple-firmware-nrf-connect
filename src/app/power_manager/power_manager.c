/*
 * File Name: power_manager.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Power Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Public includes ---------------------------------------------------------- */
#include "base_board_defs.h"
#include "protocol.h"
#include "bsp_bm.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "power_manager.h"
#include "bsp_wireless_charger.h"
#include "ble_peripheral.h"
#include "user_interface_manager_led.h"
#include "user_interface_manager_oled.h"
#include "system_manager.h"

/* Private includes --------------------------------------------------------- */
LOG_MODULE_REGISTER(power_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Private defines ---------------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */

/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
static struct gpio_dt_spec io_charge_pgood = IO_CHG_PGOOD;
static struct gpio_callback io_charge_pgood_cb_data;

static packet_battery_data_t battery_pkt = {
    .header.cmd       = PACKET_CMD_BATTERY_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .level            = 0,
    .is_charging      = 0,
    .voltage          = 0,
};

/* Private prototypes ------------------------------------------------------- */
static void power_manager_get_data(battery_data_t *battery);
static void power_good_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void power_manager_check_pwr_good_pin(void);

/* Public implementations --------------------------------------------------- */
void power_manager_init(void)
{
    bsp_bm_init();

    bsp_wireless_charger_init();

    bsp_io_attach_interrupt(io_charge_pgood, &io_charge_pgood_cb_data, power_good_interrupt_callback, GPIO_INT_EDGE_BOTH);

    power_manager_get_data(&g_device.battery);

    power_manager_check_pwr_good_pin();

    power_manager_display_led();
}

base_status_t power_manager_task(device_mode_t mode, battery_data_t *battery)
{
    if (mode == DEVICE_MODE_STREAMING)
    {
        static uint32_t tick_start = 0;

        if (bsp_tmr_get_tick_ms() - tick_start >= 5000)
        {
            tick_start = bsp_tmr_get_tick_ms();

            power_manager_get_data(battery);
            power_manager_display_led();

            battery_pkt.header.len  = sizeof(packet_battery_data_t) - sizeof(packet_hdr_t);
            battery_pkt.level       = battery->level;
            battery_pkt.voltage     = battery->voltage;
            ble_peripheral_send_data((uint8_t *)&battery_pkt, sizeof(packet_battery_data_t));
        }
    }
    else
    {
        power_manager_get_data(battery);
        power_manager_display_led();
    }

    return BS_OK;
}

void power_manager_display_led(void)
{
    power_manager_get_data(&g_device.battery);

    if (g_device.battery.is_charging)
    {
        if (g_device.battery.level < 5)
        {
            user_interface_manager_led_write_status(UI_LED_CHARGING_BATTERY_LOW, true);
        }
        else if ((g_device.battery.level > 5) && (g_device.battery.level < 80))
        {
            user_interface_manager_led_write_status(UI_LED_CHARGING_BATTERY_MID, true);
        }
        else if (g_device.battery.level > 80)
        {
            user_interface_manager_led_write_status(UI_LED_CHARGING_BATTERY_HIGH, true);
        }
    }
    else
    {
        if (g_device.battery.level < 5)
        {
            user_interface_manager_led_write_status(UI_LED_DISCHARGING_BATTERY_LOW, true);
        }
        else
        {
            user_interface_manager_led_write_status(UI_LED_SILENT_MODE, true);
        }
    }
}

/* Private implementations -------------------------------------------------- */
static void power_manager_get_data(battery_data_t *battery)
{
    // Get battery information
    bsp_bm_get_info(&battery->voltage, &battery->level);
    battery->is_charging = battery_pkt.is_charging;

    LOG_INF("Battery Voltage: %f", battery->voltage);
    LOG_INF("Battery Level  : %d", battery->level);
}

static void power_good_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
   power_manager_check_pwr_good_pin();
}

static void power_manager_check_pwr_good_pin(void)
{
    // Check device is charging or not (Power Good pin LOW indicates a good input source)
    if (bsp_io_read_specific_pin(IO_PIN_CHG_PGOOD) == 0)
    {
        battery_pkt.is_charging = 1;
        g_device.battery.is_charging = 1;

        LOG_INF("Charging");
        system_manager_state_set(SYSTEM_MANAGER_STATE_INIT);
    }
    else
    {
        battery_pkt.is_charging = 0;
        g_device.battery.is_charging = 0;
        g_device.first_run_system_manager = true;

        LOG_INF("Discharging");
        user_interface_manager_oled_update_state(UI_OLED_STATE_INIT);

        system_manager_state_set(SYSTEM_MANAGER_STATE_INIT);
    }
}

/* End of file -------------------------------------------------------------- */