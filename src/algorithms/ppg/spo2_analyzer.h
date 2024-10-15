/**
 * @file       spo2_analyzer.h
 * @brief      SPO2 analyzer
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __SPO2_ANALYZER_H
#define __SPO2_ANALYZER_H

/* Includes ----------------------------------------------------------- */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
 
#ifdef __cplusplus
extern "C" {
#endif

/* Public defines ----------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void spo2_analyzer_runner(int32_t *ppg_signal, size_t len, uint8_t *p_spo2);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __SPO2_ANALYZER_H

/* End of file -------------------------------------------------------- */
