/*
* File Name: max86176.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Driver for Ultra-Low-Power, Optical PPG and Single-Lead ECG AFE (MAX86176)
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include "base_type.h"
#include "max86176_regs.h"

/* Public defines ----------------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------------- */
/**
 * @brief MAX86176 struct
 */
typedef struct
{
  // Function pointers ---------------------------------------------------------
  base_status_t (*spi_transmit)        (uint8_t *p_data, uint16_t len);
  base_status_t (*spi_transmit_receive)(uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
  void (*spi_select)(void);
  void (*spi_deselect)(void);
}
max86176_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */
base_status_t max86176_init(max86176_t *me, const max86176_config_t *config);
base_status_t max86176_check_device_id(max86176_t *me);
base_status_t max86176_pll_enable(max86176_t *me, bool enable);
base_status_t max86176_ecg_enable(max86176_t *me, bool enable);
base_status_t max86176_set_software_power_down_mode(max86176_t *me, bool enable);

base_status_t max86176_ppg_enable(max86176_t *me, max86176_ppg_channel_t channel, bool enable);
base_status_t max86176_ppg_set_led_rge(max86176_t *me, max86176_ppg_measure_t measure, max86176_ppg_led_drive_rge_t led_drive_current);
base_status_t max86176_ppg_set_led_current(max86176_t *me, max86176_ppg_measure_t measure, max86176_led_drv_t led_drv, uint8_t value);
base_status_t max86176_ppg_frame_rate_config(max86176_t *me, const max86176_ppg_frame_rate_config_t *config);

base_status_t max86176_read_fifo(max86176_t *me, uint32_t *p_data, uint16_t num_samples);
base_status_t max86176_get_fifo_count(max86176_t *me, uint16_t *fifo_cnt);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
