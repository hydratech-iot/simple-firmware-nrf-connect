/*
 * File Name: user_interface_manager_button.h
 *
 * Author: Thuan Le
 *
 * Description: Button Manager
 *
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
/* Includes ----------------------------------------------------------------- */

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
typedef enum
{
    BTN_EVT_CLICK = 0,
    BTM_EVT_LONG_PRESS,
    BTN_UNKNOW,
}
button_event_t;

/* Public Constants --------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Public macros ------------------------------------------------------------ */
typedef void (*button_event_cb_t)(button_event_t event);

/* Public APIs -------------------------------------------------------------- */
void user_interface_manager_button_init(button_event_cb_t callback);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C" {
#endif

/* End of file ---------------------------------------------------------------- */
