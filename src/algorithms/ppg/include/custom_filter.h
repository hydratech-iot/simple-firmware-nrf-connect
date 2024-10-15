/**
 * @file       custom_filter.h
 * @brief      Filter
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __CUSTOM_FILTER_H
#define __CUSTOM_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "stdio.h"
#include "stdint.h"

/* Public defines ----------------------------------------------------------- */
#define FILTER_GAIN (2)
#define FILTER_LEN  (5)

/* Public enumerate/structure ----------------------------------------------- */
typedef struct
{
    double *ca_sh;
    double *cb_sh;
    double  x[FILTER_LEN];
    double  y[FILTER_LEN];
} data_float_poly_sh_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
void custom_filter_init(data_float_poly_sh_t *poly, double *ca_sh, double *cb_sh);
float custom_filter(data_float_poly_sh_t *poly, float signal);
float custom_hlp_filter(data_float_poly_sh_t *poly, float signal);
void custom_filter_clear_buffer(data_float_poly_sh_t *poly);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

#endif // __CUSTOM_FILTER_H

/* End of file -------------------------------------------------------------- */