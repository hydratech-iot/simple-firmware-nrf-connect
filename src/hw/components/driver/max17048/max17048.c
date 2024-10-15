/*
* File Name: max17048.c
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Driver support MAX17048 (Stand-Alone Fuel Gauge IC)
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Includes ----------------------------------------------------------- */
#include "max17048.h"

/* Private defines ---------------------------------------------------- */
// All registers contain two bytes of data and span two addresses.
// Registers which are present on the MAX17048/49 only are prefixed with MAX17048
#define MAX17048_VCELL        (0x02)    // R - 12-bit A/D measurement of battery voltage
#define MAX17048_SOC          (0x04)    // R - 16-bit state of charge (SOC)
#define MAX17048_MODE         (0x06)    // W - Sends special commands to IC
#define MAX17048_VERSION      (0x08)    // R - Returns IC version
#define MAX17048_HIBRT        (0x0A)    // R/W - (MAX17048/49) Thresholds for entering hibernate
#define MAX17048_CONFIG       (0x0C)    // R/W - Battery compensation (default 0x971C)
#define MAX17048_CVALRT       (0x14)    // R/W - (MAX17048/49) Configures adc_value range to generate alerts (default 0x00FF)
#define MAX17048_CRATE        (0x16)    // R - (MAX17048/49) Charge rate 0.208%/hr
#define MAX17048_VRESET_ID    (0x18)    // R/W - (MAX17048/49) Reset voltage and ID (default 0x96__)
#define MAX17048_STATUS       (0x1A)    // R/W - (MAX17048/49) Status of ID (default 0x01__)
#define MAX17048_COMMAND      (0xFE)    // W - Sends special comands to IC
#define MAX17048_MODE_ENSLEEP (0x2000)  // W - _Enables_ sleep mode (the SLEEP bit in the CONFIG reg engages sleep)
#define MAX17048_MODE_HIBSTAT (0x1000)  // R - indicates when the IC is in hibernate mode

#define MAX17048_IC_VERSION   (0x0012)  // IC production version // 18

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t m_max17048_read_reg(max17048_t *me, uint8_t reg, uint16_t *p_data);
static base_status_t m_max17048_is_available(max17048_t *me);

/* Function definitions ----------------------------------------------- */
base_status_t max17048_init(max17048_t *me)
{
    if ((me == NULL) || (me->i2c_read == NULL))
        return BS_ERROR;

    CHECK_STATUS(m_max17048_is_available(me));

    return BS_OK;
}

base_status_t max17048_get_voltage(max17048_t *me, float *voltage)
{
    uint16_t adc_value;

    CHECK_STATUS(m_max17048_read_reg(me, MAX17048_VCELL, &adc_value));

    // On the MAX17048/49: vCell is a 16-bit register where each bit represents 78.125uV/cell per LSB
    // i.e. 78.125uV per LSB on the MAX17048
    // i.e. 156.25uV per LSB on the MAX17049
    *voltage = (adc_value * 78.125) / 1000000.0;

    return BS_OK;
}

base_status_t max17048_get_capacity(max17048_t *me, uint8_t *capacity)
{
    uint16_t adc_value;

    CHECK_STATUS(m_max17048_read_reg(me, MAX17048_SOC, &adc_value));

    *capacity = adc_value / 256;

    return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief          Read register
 *
 * @param[in]     me      Pointer to handle of  module.
 * @param[in]     reg     Register
 * @param[in]     p_data  Pointer to handle of data
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
static base_status_t m_max17048_read_reg(max17048_t *me, uint8_t reg, uint16_t *p_data)
{
    uint8_t buf[2];

    CHECK_STATUS(me->i2c_read(me->device_address, reg, buf, 2));

    *p_data = (buf[0] << 8) | buf[1];

    return BS_OK;
}

/**
 * @brief          MAX17048 is available
 *
 * @param[in]     me      Pointer to handle of  module.
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
static base_status_t m_max17048_is_available(max17048_t *me)
{
  uint16_t ic_version;

  CHECK_STATUS(m_max17048_read_reg(me, MAX17048_VERSION, &ic_version));

  if (ic_version != MAX17048_IC_VERSION)
    return BS_ERROR;
  
  return BS_OK;
}

/* End of file -------------------------------------------------------- */
