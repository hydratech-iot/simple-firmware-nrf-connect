/*
 * File Name: ble_manager.h
 *
 * Author: Thuan Le
 *
 * Description: BLE Manager
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------- */
#include "ble_peripheral.h"

/* Public defines ----------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void ble_manager_init(void);
void ble_manager_deinit(void);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */
