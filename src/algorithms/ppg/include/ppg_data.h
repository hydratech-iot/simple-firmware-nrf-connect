/**
 * @file       ppg_data.h
 * @brief      PPG data process
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __PPG_DATA_H
#define __PPG_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h> 
#include "custom_filter.h"
#include "sqi_cal.h"

/* Public defines ----------------------------------------------------------- */
#define THR_SBP_MAX 180
#define THR_SBP_MIN 80
#define THR_DBP_MAX 100
#define THR_DBP_MIN 55
#define THR_SD_MAX 100
#define THR_SD_MIN 35

/* Public enumerate/structure ----------------------------------------------- */
typedef enum
{
  SP_NOT_ENOUGH_SAMPLE,
  SP_PROCESS_FAILED,
  SP_PROCESS_OK
}
sp_status_t;

typedef enum
{
    PPG_LED_CHANNEL_RED,
    PPG_LED_CHANNEL_IR,
    PPG_LED_CHANNEL_GREEN,
    PPG_LED_CHANNEL_MAX
}
ppg_led_channel_t;

typedef struct
{
    float heart_rate;
    float spo2;
    float sbp;
    float dbp;
    uint8_t sqi_red_percent;
    uint8_t sqi_red_percent_for_spo2;
    uint8_t sqi_ir_percent_for_spo2;
}
measured_values_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void ppg_data_init(void);
sp_status_t ppg_data_process(float sample, ppg_led_channel_t channel, float *ppg_bp, measured_values_t *meas_value);
void ppg_data_process_clear(void);

#endif  // __PPG_DATA_H