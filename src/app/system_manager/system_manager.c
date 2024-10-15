/*
 * File Name: system_manager.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: System Manager
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Public includes ---------------------------------------------------------- */
// Board Support Packages
#include "bsp_afe.h"
#include "bsp_i2c.h"
#include "bsp_i2c_mul.h"
#include "bsp_bm.h"
#include "bsp_spi.h"
#include "bsp_io.h"
#include "bsp_oled.h"
#include "bsp_wireless_charger.h"
#include "bsp_power.h"
#include "bsp_temp.h"
#include "bsp_imu.h"
#include "bsp_led.h"
#include "bsp_timer.h"

// Applications
#include "sensor_manager.h"
#include "configuration_manager.h"
#include "power_manager.h"
#include "user_interface_manager_button.h"
#include "user_interface_manager_led.h"
#include "user_interface_manager_oled.h"
#include "system_manager.h"
#include "protocol.h"
#include "base_board_defs.h"

// Bluetooth
#include "ble_manager.h"

// Algorithms
#include "ppg_data.h"

/* Private includes --------------------------------------------------------- */
LOG_MODULE_REGISTER(system_manager, CONFIG_LOG_DEFAULT_LEVEL);

#define SYSTEM_MANAGER_THREAD_STACK_SIZE (8192)
K_THREAD_STACK_DEFINE(system_manager_stack, SYSTEM_MANAGER_THREAD_STACK_SIZE);
static struct k_thread system_manager_thread;
static k_tid_t system_manager_thread_id;

/* Private defines ---------------------------------------------------------- */
#define ONE_MINUTE_IN_MILLISECOND           (1000 * 60) // 1 minute
#define MAX_TIME_COLLECT_PPG                (1000 * 60) // 1 minute
#define MAX_TIME_COLLECT_IMU                (1000 * 10) // 10 seconds

/* Private enumerate/structure ---------------------------------------------- */

typedef enum
{
    COLLECT_STUDY_STATE_ONE_TIME_DATA        = 0,
    COLLECT_STUDY_STATE_INIT_PPG             = 1,
    COLLECT_STUDY_STATE_COLLECT_PPG_AND_TEMP = 2,
    COLLECT_STUDY_STATE_INIT_IMU             = 3,
    COLLECT_STUDY_STATE_COLLECT_IMU          = 4,
    COLLECT_STUDY_STATE_MAX
}
collect_study_state_t;

/* Private macros ----------------------------------------------------------- */

/* Public variables --------------------------------------------------------- */
device_t g_device;

/* Private variables -------------------------------------------------------- */
static system_manager_state_t system_manager_state = SYSTEM_MANAGER_STATE_INIT;
static collect_study_state_t collect_study_state = COLLECT_STUDY_STATE_INIT_PPG;

static packet_study_data_t study_pkt = {
    .header.cmd       = PACKET_CMD_STUDY_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .heart_rate       = 0,
    .spo2             = 0,
    .battery_level    = 0,
    .battery_voltage  = 0,
    .temperature      = {0, 0, 0, 0, 0},
    .roll             = 0,
    .pitch            = 0,
    .yaw              = 0,
};

static packet_temperature_data_t temperature_pkt = {
    .header.cmd       = PACKET_CMD_TEMPERATURE_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .value            = {0},
};

static packet_device_charging_data_t device_charging_data_pkt = {
    .header.cmd       = PACKET_CMD_DEVICE_CHARGING_DATA,
    .header.msg_index = 0,
    .header.len       = 0,
    .battery_level    = 0,
    .is_charging      = 0,
    .battery_voltage  = 0,
    .temperature      = {0, 0, 0, 0, 0},
    .roll             = 0,
    .pitch            = 0,
    .yaw              = 0,
};

static tmr_t system_manager_task_timer;
static tmr_t system_manager_ui_display_timer;
static tmr_t system_manager_collect_study_timer;
static tmr_t system_manager_collect_temperature_timer;

/* Private prototypes ------------------------------------------------------- */
static base_status_t system_manager_collect_study_data(void);
static void system_manager_collect_temperature_data(float *temperature, float *avr_skin_temp);
static void system_manager_sending_study_data(void);
static void system_manager_sending_temperature_data(void);
static void system_manager_data_init(void);
static void system_manager_data_reset(void);
static bool system_manager_is_silent(void);
static void system_manager_sending_device_charging_data(void);

static void system_manager_task(void *p1, void *p2, void *p3);

/* Public implementations --------------------------------------------------- */
void system_manager_init(void)
{
    system_manager_data_init();

    // Board support packages
    user_interface_manager_led_init();

    bsp_io_init();
    bsp_power_boot();

    bsp_i2c_init();
    bsp_i2c_mul_init();
    bsp_spi_init();

    // Bluetooth
    ble_manager_init();

    // Applications
    cfg_mgmt_init();
    user_interface_manager_button_init();

    user_interface_manager_oled_init();
    
    // Workaround: The power manager should be init after OLED init
    // If the power init before OLED init, the OLED will init failed
    power_manager_init();
    user_interface_manager_oled_display_init();

    if (g_device.mode == DEVICE_MODE_STREAMING)
    {
        bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 1);
        SYSTEM_DELAY_MSEC(100); // Wait for power stable
        sensor_manager_init();
    }

    // Algorithms
    ppg_data_init();

    system_manager_state = SYSTEM_MANAGER_STATE_INIT;

    // Create the thread
    system_manager_thread_id = k_thread_create(&system_manager_thread,                      //!< Pointer to initialize thread
                                               system_manager_stack,                        //!< Pointer to stack space
                                               K_THREAD_STACK_SIZEOF(system_manager_stack), //!< Stack size in bytes
                                               system_manager_task,                         //!< Entry thread function
                                               NULL,                                        //!< 1st entry point parameter
                                               NULL,                                        //!< 2nd entry point parameter
                                               NULL,                                        //!< 3rd entry point parameter
                                               K_PRIO_COOP(4),                              //!< Thread priority
                                               0,                                           //!< Thread option
                                               K_NO_WAIT);                                  //!< Scheduling delay (in milliseconds)

    k_thread_name_set(&system_manager_thread, "system_manager");
}

static void system_manager_task(void *p1, void *p2, void *p3)
{
    while (1)
    {
        if (g_device.battery.is_charging)
        {
            switch (system_manager_state)
            {
            case SYSTEM_MANAGER_STATE_INIT:
            {
                system_manager_study_start(false);
                system_manager_display_charging();

                bsp_tmr_start(&system_manager_task_timer, g_device.study.interval_min * ONE_MINUTE_IN_MILLISECOND);
                bsp_tmr_restart(&system_manager_ui_display_timer, 10000);
                
                system_manager_state = SYSTEM_MANAGER_STATE_WAKE_UP;
                break;
            }

            case SYSTEM_MANAGER_STATE_WAKE_UP:
            {
                bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 1);
                SYSTEM_DELAY_MSEC(100); // Wait for power stable

                // Collect temperature data
                bsp_temp_init();
                system_manager_collect_temperature_data(g_device.temperature, &g_device.avr_skin_temp);
                power_manager_task(g_device.mode, &g_device.battery);
                bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 0);

                system_manager_sending_device_charging_data();
                system_manager_state = SYSTEM_MANAGER_STATE_SLEEP;

                break;
            }

            case SYSTEM_MANAGER_STATE_SLEEP:
            {
                if (bsp_tmr_is_expired(&system_manager_task_timer))
                {
                    system_manager_state = SYSTEM_MANAGER_STATE_WAKE_UP;
                    bsp_tmr_restart(&system_manager_task_timer, g_device.study.interval_min * ONE_MINUTE_IN_MILLISECOND);
                }

                if (bsp_tmr_is_expired(&system_manager_ui_display_timer))
                {
                    bsp_tmr_restart(&system_manager_ui_display_timer, 10000);
                    user_interface_manager_oled_update_state(UI_OLED_STATE_CLEAR);
                    user_interface_manager_led_write_status(UI_LED_SILENT_MODE, true);
                }

                break;
            }

            default:
                break;
            }
        }
        else
        {
            if (g_device.mode == DEVICE_MODE_STREAMING)
            {
                sensor_manager_task(g_device.mode);
                power_manager_task(g_device.mode, &g_device.battery);
            }
            else
            {
                // Production mode
                switch (system_manager_state)
                {
                case SYSTEM_MANAGER_STATE_INIT:
                {
                    bsp_tmr_start(&system_manager_task_timer, 5000);
                    system_manager_state = SYSTEM_MANAGER_STATE_POWERING_ON;
                    user_interface_manager_led_write_status(UI_LED_SILENT_MODE, true);

                    power_manager_task(g_device.mode, &g_device.battery);
                    break;
                }

                case SYSTEM_MANAGER_STATE_POWERING_ON:
                {
                    if (bsp_tmr_is_expired(&system_manager_task_timer))
                    {
                        system_manager_state = SYSTEM_MANAGER_STATE_CONNECTING;
                        if (g_device.comm.ble.connected == BLE_CONNECTED)
                        {
                            system_manager_state = SYSTEM_MANAGER_STATE_WAKE_UP;
                            system_manager_is_silent();
                        }
                        else
                        {
                            user_interface_manager_oled_update_state(UI_OLED_STATE_CHECK_CONNECTION);
                            bsp_tmr_restart(&system_manager_task_timer, ONE_MINUTE_IN_MILLISECOND);
                        }
                    }
                    break;
                }

                case SYSTEM_MANAGER_STATE_CONNECTING:
                {
                    if (g_device.comm.ble.connected == BLE_CONNECTED)
                    {
                        system_manager_state = SYSTEM_MANAGER_STATE_WAKE_UP;
                        system_manager_is_silent();
                    }
                    else if (bsp_tmr_is_expired(&system_manager_task_timer))
                    {
                        system_manager_state = SYSTEM_MANAGER_STATE_POWER_OFF;
                    }
                    break;
                }

                case SYSTEM_MANAGER_STATE_WAKE_UP:
                {
                    system_manager_data_reset();

                    bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 1);
                    SYSTEM_DELAY_MSEC(100); // Wait for power stable

                    system_manager_state = SYSTEM_MANAGER_STATE_COLLECT_DATA;
                    collect_study_state = COLLECT_STUDY_STATE_ONE_TIME_DATA;
                    break;
                }

                case SYSTEM_MANAGER_STATE_COLLECT_DATA:
                {
                    if (system_manager_collect_study_data() == BS_OK)
                    {
                        if (!system_manager_is_silent())
                        {
                            user_interface_manager_oled_update_state(UI_OLED_STATE_MEASURE_VALUE);
                            g_device.first_run_system_manager = false;
                        }

                        system_manager_state = SYSTEM_MANAGER_STATE_SEND_DATA;
                        bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 0);
                        user_interface_manager_led_write_status(UI_LED_SILENT_MODE, true);
                    }

                    break;
                }

                case SYSTEM_MANAGER_STATE_SEND_DATA:
                {
                    system_manager_sending_study_data();
                    system_manager_state = SYSTEM_MANAGER_STATE_SLEEP;
                    bsp_tmr_restart(&system_manager_task_timer, g_device.study.interval_min * ONE_MINUTE_IN_MILLISECOND);

                    break;
                }

                case SYSTEM_MANAGER_STATE_SLEEP:
                {
                    if (bsp_tmr_is_expired(&system_manager_task_timer))
                    {
                        system_manager_state = SYSTEM_MANAGER_STATE_WAKE_UP;
                    }

                    break;
                }

                case SYSTEM_MANAGER_STATE_STOPPED:
                {
                    // Do nothings
                    break;
                }

                case SYSTEM_MANAGER_STATE_POWER_OFF:
                {
                    bsp_power_off();
                    break;
                }

                default:
                    break;
                }
            }
        }

        SYSTEM_DELAY_MSEC(10);
    }
}

void system_manager_study_start(bool start)
{
    if (start)
    {
        // Power on the sensors
        bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 1);
        system_manager_state = SYSTEM_MANAGER_STATE_WAKE_UP;
    }
    else
    {
        // Power off the sensors 
        bsp_io_write_specific_pin(IO_PIN_SENSOR_PWR_EN, 0);
        SYSTEM_DELAY_MSEC(100); // Wait for power off
        system_manager_state = SYSTEM_MANAGER_STATE_STOPPED;
    }
}

void system_manager_state_set(system_manager_state_t state)
{
    system_manager_state = state;
}

void system_manager_display_charging(void)
{
    bsp_tmr_start(&system_manager_ui_display_timer, 10000);
    user_interface_manager_oled_update_state(UI_OLED_STATE_CHARGING);
    power_manager_display_led();
}

bool system_manager_check_device_is_remove_from_horse(void)
{
    if ((g_device.avr_skin_temp < 37) || (g_device.avr_skin_temp > 39))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* Private implementations -------------------------------------------------- */
static base_status_t system_manager_collect_study_data(void)
{
    base_status_t result = BS_ERROR;

    switch (collect_study_state)
    {
    case COLLECT_STUDY_STATE_ONE_TIME_DATA:
    {
        // 1. Collect battery information
        power_manager_task(g_device.mode, &g_device.battery);

        // 2. Collect temperature data
        bsp_temp_init();
        system_manager_collect_temperature_data(g_device.temperature, &g_device.avr_skin_temp);

        if (!system_manager_is_silent())
        {
            user_interface_manager_oled_update_state(UI_OLED_STATE_MEASURING);
        }

        collect_study_state = COLLECT_STUDY_STATE_INIT_PPG;
        break;
    }

    case COLLECT_STUDY_STATE_INIT_PPG:
    {
        sensor_manager_afe_init();
        SYSTEM_DELAY_MSEC(10); 
        
        collect_study_state = COLLECT_STUDY_STATE_COLLECT_PPG_AND_TEMP;
        bsp_tmr_start(&system_manager_collect_study_timer, MAX_TIME_COLLECT_PPG);
        bsp_tmr_start(&system_manager_collect_temperature_timer, g_device.study.temp_meas_interval_second * 1000);
        break;
    }

    case COLLECT_STUDY_STATE_COLLECT_PPG_AND_TEMP:
    {
        if (!system_manager_check_device_is_remove_from_horse())
        {
            // 3. Collect temperature with the interval
            if (bsp_tmr_is_expired(&system_manager_collect_temperature_timer))
            {
                system_manager_collect_temperature_data(g_device.temperature, &g_device.avr_skin_temp);
                bsp_tmr_restart(&system_manager_collect_temperature_timer, g_device.study.temp_meas_interval_second * 1000);

                // Send temperature data to server
                system_manager_sending_temperature_data();
            }
        }

        // 4. Collect PPG physical data
        result = sensor_manager_afe_task(g_device.mode, &g_device.ppg_phys);

        if (result == BS_OK)
        {
            LOG_INF("Collect PPG physical data success");
            collect_study_state = COLLECT_STUDY_STATE_INIT_IMU;
        }
        else
        {
            if (bsp_tmr_is_expired(&system_manager_collect_study_timer))
            {
                LOG_INF("Collect PPG physical data failure, timeout");
                collect_study_state = COLLECT_STUDY_STATE_INIT_IMU;
            }
        }
        break;
    }

    case COLLECT_STUDY_STATE_INIT_IMU:
    {
        bsp_imu_init();
        collect_study_state = COLLECT_STUDY_STATE_COLLECT_IMU;
        bsp_tmr_restart(&system_manager_collect_study_timer, MAX_TIME_COLLECT_IMU);
        break;
    }

    case COLLECT_STUDY_STATE_COLLECT_IMU:
    {
        // 5. Collect IMU physical data
        result = sensor_manager_imu_task(g_device.mode, &g_device.imu_phys);
        if (result == BS_OK)
        {
            LOG_INF("Collect IMU physical data success");
            return BS_OK;
        }
        else
        {
            if (bsp_tmr_is_expired(&system_manager_collect_study_timer))
            {
                LOG_INF("Collect IMU physical data failure, timeout");
                return BS_OK;
            }
        }
        break;
    }

    default:
        break;
    }

    return BS_ERROR;
}

static void system_manager_collect_temperature_data(float *temperature, float *avr_skin_temp)
{
    for (uint8_t i = 0; i < TEMPERATURE_MAX; i++)
    {
        bsp_temp_single_shot_set(i);
    }

    // Wait for sensor wakeup
    SYSTEM_DELAY_MSEC(50);

    for (uint8_t i = 0; i < TEMPERATURE_MAX; i++)
    {
        bsp_temp_celsius_get(i, &temperature[i]);
        
        LOG_INF("Temperature %d equal %f", i, temperature[i]);
    }

    *avr_skin_temp = temperature[TEMPERATURE_SKIN_1] + temperature[TEMPERATURE_SKIN_2] +
                     temperature[TEMPERATURE_SKIN_3] + temperature[TEMPERATURE_SKIN_4];

    *avr_skin_temp = (*avr_skin_temp / 4);
}

static void system_manager_sending_study_data(void)
{
    g_device.imu_phys.roll       = 20;
    g_device.imu_phys.pitch      = 30;
    g_device.imu_phys.yaw        = 40;

    study_pkt.header.msg_index = 0;
    study_pkt.header.len       = sizeof(packet_study_data_t) - sizeof(packet_hdr_t);
    study_pkt.heart_rate       = g_device.ppg_phys.heart_rate;
    study_pkt.spo2             = g_device.ppg_phys.spo2;
    study_pkt.battery_level    = g_device.battery.level;
    study_pkt.battery_voltage  = g_device.battery.voltage;
    study_pkt.roll             = g_device.imu_phys.roll;
    study_pkt.pitch            = g_device.imu_phys.pitch;
    study_pkt.yaw              = g_device.imu_phys.yaw;

    memcpy(study_pkt.temperature, g_device.temperature, sizeof(g_device.temperature));

    ble_peripheral_send_data((uint8_t *)&study_pkt, sizeof(packet_study_data_t));
}

static void system_manager_sending_device_charging_data(void)
{
    g_device.imu_phys.roll       = 20;
    g_device.imu_phys.pitch      = 30;
    g_device.imu_phys.yaw        = 40;

    device_charging_data_pkt.header.msg_index = 0;
    device_charging_data_pkt.header.len       = sizeof(packet_device_charging_data_t) - sizeof(packet_hdr_t);
    device_charging_data_pkt.battery_level    = g_device.battery.level;
    device_charging_data_pkt.battery_voltage  = g_device.battery.voltage;
    device_charging_data_pkt.is_charging      = g_device.battery.is_charging;
    device_charging_data_pkt.roll             = g_device.imu_phys.roll;
    device_charging_data_pkt.pitch            = g_device.imu_phys.pitch;
    device_charging_data_pkt.yaw              = g_device.imu_phys.yaw;

    memcpy(device_charging_data_pkt.temperature, g_device.temperature, sizeof(g_device.temperature));

    ble_peripheral_send_data((uint8_t *)&device_charging_data_pkt, sizeof(packet_device_charging_data_t));

    LOG_INF("Send device charging data");
}

static void system_manager_sending_temperature_data(void)
{
    temperature_pkt.header.msg_index = 0;
    temperature_pkt.header.len       = sizeof(packet_temperature_data_t) - sizeof(packet_hdr_t);

    memcpy(temperature_pkt.value, g_device.temperature, sizeof(g_device.temperature));

    ble_peripheral_send_data((uint8_t *)&temperature_pkt, sizeof(packet_temperature_data_t));
}

static void system_manager_data_init()
{
    memset(&g_device.ppg_phys, 0, sizeof(g_device.ppg_phys));
    memset(g_device.temperature, 0, sizeof(g_device.temperature));
    memset(&g_device.battery, 0, sizeof(g_device.battery));
    memset(&g_device.imu_phys, 0, sizeof(g_device.imu_phys));

    g_device.mode    = DEVICE_MODE_PRODUCTION;
    g_device.ui_mode = UI_MODE_SILENT;

    g_device.study.interval_min              = 1;
    g_device.study.temp_meas_interval_second = 5;
    g_device.first_run_system_manager        = true;
    g_device.oled_switch_off_period_seconds  = 30;
}

static void system_manager_data_reset()
{
    memset(&g_device.ppg_phys, 0, sizeof(g_device.ppg_phys));
    memset(g_device.temperature, 0, sizeof(g_device.temperature));
    memset(&g_device.imu_phys, 0, sizeof(g_device.imu_phys));
}

static bool system_manager_is_silent(void)
{
    if ((g_device.ui_mode == UI_MODE_SILENT) && (!g_device.first_run_system_manager))
    {
        user_interface_manager_oled_update_state(UI_OLED_STATE_CLEAR);
        return true;
    }
    else if ((g_device.ui_mode == UI_MODE_NORMAL) || (g_device.first_run_system_manager))
    {
        return false;
    }

    return false;
}

/* End of file -------------------------------------------------------------- */