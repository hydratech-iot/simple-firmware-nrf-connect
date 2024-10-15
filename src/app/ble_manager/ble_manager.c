/*
 * File Name: ble_manager.c
 *
 * Author: Thuan Le
 *
 * Description: BLE Manager
 *
 */

/* Includes ----------------------------------------------------------- */
#include "ble_manager.h"
#include "ble_peripheral.h"
#include "network_manager.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(ble_manager, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */

/* Public variables --------------------------------------------------- */
/* Private Constants -------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void ble_manager_received_handler(uint8_t *p_data, uint8_t data_len);
static void ble_manager_connection_status_handler(ble_connection_status_t connection_status);

/* Function definitions ------------------------------------------------ */
void ble_manager_init(void)
{
    ble_peripheral_init(ble_manager_received_handler, ble_manager_connection_status_handler);
}

void ble_manager_deinit(void)
{
    ble_peripheral_deinit();
}

/* Private function definitions ---------------------------------------- */
static void ble_manager_received_handler(uint8_t *p_data, uint8_t data_len)
{
    LOG_INF("Received data");
    network_process_data(p_data, data_len);
}

static void ble_manager_connection_status_handler(ble_connection_status_t connection_status)
{
    LOG_INF("Connection status: %d", connection_status);
}

/* Command Process Function -------------------------------------------- */
/* End of file --------------------------------------------------------- */
