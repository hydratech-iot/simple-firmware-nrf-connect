/**
 * @file       sqi_cal
 * @brief      SQI
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __SQI_H
#define __SQI_H

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

/* Public defines ----------------------------------------------------------- */\
#define LEN_SIG_128HZ   (1024)  // 128*8
#define LEN_SIG_SQI     (1024)  // 128*8
#define THR_SKEWNESS    (-0.5)
#define THR_KURTOSIS    (2.38)
#define THR_STD         (0.3)
#define THR_PERCENT_SQI (50)    // 50%

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
float sqi_cal_percentage(float kurt);
bool sqi_cal_good_signal_quality(float *ppg, uint8_t *sqi_percent);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

#endif  // __SQI_H

/* End of file -------------------------------------------------------------- */