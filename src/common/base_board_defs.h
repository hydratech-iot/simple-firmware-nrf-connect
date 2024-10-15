/*
 * File Name: base_board_defs.h
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
#include <stdbool.h>
#include <stdint.h>
#include "base_include.h"

/* Public defines ----------------------------------------------------- */
// Input Node
#define IO_BUTTON_NODE_ID           DT_ALIAS(user_button)

// Input
#define IO_BUTTON                   GPIO_DT_SPEC_GET_OR(IO_BUTTON_NODE_ID, gpios, { 0 })

// Output Node
#define IO_LED_NODE_ID              DT_ALIAS(user_led)

// Output
#define IO_LED                      GPIO_DT_SPEC_GET_OR(IO_LED_NODE_ID, gpios, { 0 })

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/* -------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
