/*
 * File Name: bsp_timer.h
 *
 * Author: Thuan Le
 *
 * Description:
 *
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "base_type.h"
#include "base_include.h"

/* Public defines ----------------------------------------------------------- */
#define bsp_tmr_get_tick_ms() k_uptime_get()

typedef uint32_t tick_t;    //!< Count of system tick

typedef void (*timer_timeout_handler_t)(struct k_timer *timer);
typedef struct k_timer * timer_id_t;

/* Public enumerate/structure ----------------------------------------------- */

/**
 * @brief Simple timer
 */
typedef struct
{
  tick_t start;
  tick_t interval;
}
tmr_t;

/**
 * @brief Auto timer structure
 */
typedef struct
{
  struct k_timer *handle;
  tmr_t timer;
  timer_timeout_handler_t callback;
}
auto_timer_t;

/* Public Constants --------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Public macros ------------------------------------------------------------ */
/* Public APIs -------------------------------------------------------------- */
void bsp_tmr_start     (tmr_t *tm, tick_t interval);
void bsp_tmr_restart   (tmr_t *tm, tick_t interval);
void bsp_tmr_stop      (tmr_t *tm);
bool bsp_tmr_is_expired(tmr_t *tm);

void bsp_tmr_auto_init   (auto_timer_t *atm, timer_timeout_handler_t callback);
void bsp_tmr_auto_start  (auto_timer_t *atm, tick_t interval);
void bsp_tmr_auto_restart(auto_timer_t *atm);
void bsp_tmr_auto_stop   (auto_timer_t *atm);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C" {
#endif

/* End of file ---------------------------------------------------------------- */
