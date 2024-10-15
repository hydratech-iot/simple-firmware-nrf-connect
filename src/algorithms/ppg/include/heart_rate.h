/**
 * @file       heart_rate.h
 * @brief      PPG data process
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __HEART_RATE_H
#define __HEART_RATE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "spo2.h"
#include "sqi_cal.h"
#include "custom_filter.h"
#include "ppg_data.h"

/* Public defines ----------------------------------------------------------- */
#define PI      (3.141592653589793)
#define LEN_FFT (7680)  // 128Hz*60

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
float measure_hr(float *ppg);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

#endif  // __HEART_RATE_H

/* End of file -------------------------------------------------------------- */
