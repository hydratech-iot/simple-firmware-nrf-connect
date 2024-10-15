/**
 * @file       network_manager.h
 * @copyright  Copyright (C) 2020 HydraTech. All rights reserved.
 * @license    This project is released under the HydraTech License.
 * @version    1.0.0
 * @date       2021-10-24
 * @author     Thuan Le
 * @brief      BLE data handler
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __NETWORK_H
#define __NETWORK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void network_process_data(uint8_t *p_data, uint8_t len);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __NETWORK_H

/* End of file -------------------------------------------------------- */