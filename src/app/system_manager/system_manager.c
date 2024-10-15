/*
 * File Name: system_manager.c
 *
 * Author: Thuan Le
 *
 * Description: System Manager
 *
 */

/* Public includes ---------------------------------------------------------- */
// Board Support Packages
#include "bsp_io.h"
#include "bsp_timer.h"

// Applications
#include "user_interface_manager_button.h"
#include "system_manager.h"
#include "base_board_defs.h"
#include "network_manager.h"

// Bluetooth
#include "ble_manager.h"

/* Private includes --------------------------------------------------------- */
LOG_MODULE_REGISTER(system_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Private defines ---------------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
/* Private prototypes ------------------------------------------------------- */
static void system_manager_button_event_callback(button_event_t event);

/* Public implementations --------------------------------------------------- */
void system_manager_init(void)
{
    bsp_io_init();

    // Applications
    user_interface_manager_button_init(system_manager_button_event_callback);
}

/* Private implementations -------------------------------------------------- */
static void system_manager_button_event_callback(button_event_t event)
{
    static int button_click_cnt = 0;

    if (event ==  BTN_EVT_CLICK)
    {
        button_click_cnt++;
        LOG_INF("Button press %d time", button_click_cnt);

        if (button_click_cnt == 2)
        {
            // Enable BLE
            ble_manager_init();
            LOG_INF("Enable BLE");
        }
        else if (button_click_cnt == 5)
        {
            LOG_INF("Turn on LED and send notification");

            // Turn on LED
            bsp_io_write_specific_pin(IO_PIN_LED, 1);

            // Send notification
            network_send_notification();
        }
        else
        {
            // Do nothing
        }
    }
    else if (event == BTM_EVT_LONG_PRESS)
    {
        LOG_INF("Turn off BLE and put the system into the sleep mode");

        // Turn off LED
        bsp_io_write_specific_pin(IO_PIN_LED, 0);
        button_click_cnt = 0;

        // Disable BLE
        ble_manager_deinit();
    }
    else
    {
        // Do nothing
    }
}

/* End of file -------------------------------------------------------------- */