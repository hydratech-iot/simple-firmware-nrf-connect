/**
 * @file       spo2.h
 * @brief      SPO2 Algorithm
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __SPO2_H
#define __SPO2_H

#ifdef __cplusplus
extern "C" 
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
#define IND_GET_RED_IR          (576)// 4.5*128Hz
#define MX_A                    (-8.33)
#define MX_B                    (14.44)
#define MX_C                    (93.24)
#define FS_128HZ                (50)

#define PPG_BUF_LEN             (1664) // 13s*128Hz
#define PPG_BUF_LEN_OVERLAP     (640) // 5s*128Hz
#define LEN_4S_128HZ            (512) // 4s*128Hz
#define IND_GET                 (640)
#define NUM_CHANNELS            (2)
#define ADC_GAIN                (32768) // 2^20/32

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
float spo2_measure(double *ac, double *dc);
bool sqi_for_spo2(float *ppg, uint8_t *sqi_percent);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

#endif  // __SPO2_H

/* End of file -------------------------------------------------------------- */
