/**
 * @file       heart_rate_analyzer.h
 * @brief      Heart rate analyzer
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __HEART_RATE_ANALYZER_H
#define __HEART_RATE_ANALYZER_H

/* Includes ----------------------------------------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
 
#ifdef __cplusplus
extern "C" {
#endif

/* Public defines ----------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
bool heart_rate_analyzer_runner(int32_t *ppg_signal, size_t len, uint8_t *p_heart_rate);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __HEART_RATE_ANALYZER_H

/* End of file -------------------------------------------------------- */
