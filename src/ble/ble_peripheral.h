/*
 * File Name: ble_peripheral.h
 *
 * Author: Thuan Le
 *
 * Description:
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include <stdint.h>

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
    BLE_DISCONNECTED = 0,
    BLE_CONNECTED
}
ble_connection_status_t;

/* Public defines ----------------------------------------------------- */
typedef void (*ble_receive_cb_t)(uint8_t *p_data, uint8_t data_len);
typedef void (*ble_connection_status_cb_t)(ble_connection_status_t connection_status);

/* Public function prototypes ----------------------------------------- */
void ble_peripheral_init(ble_receive_cb_t receive_cb, ble_connection_status_cb_t connection_status_cb);
void ble_peripheral_deinit(void);

void ble_peripheral_send_data(const uint8_t *data, uint16_t len);
void ble_peripheral_get_mac_address(char *mac_addr);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */
