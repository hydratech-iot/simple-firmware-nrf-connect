/*
* File Name: network_manager.h
* 
* Author: Thuan Le 
* 
* Description: Network Manager
* 
*/

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void network_process_data(uint8_t *p_data, uint8_t len);
void network_send_notification(void);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif

/* End of file -------------------------------------------------------- */