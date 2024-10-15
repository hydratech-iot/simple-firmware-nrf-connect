/*
* File Name: bsp_afe.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Board Support Package for MAX86176
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
#include "bsp_spi.h"
#include "max86176.h"

/* Public defines ----------------------------------------------------------- */
#define AFE_FIFO_THRESHOLD       (60) // FIFO threshold (watermark level) to trigger FIFO_RDY interrupt
#define AFE_FIFO_MAX             (256)
#define AFE_SAMPLING_RATE        (128)

/* Public enumerate/structure ----------------------------------------------- */
typedef void (*afe_interrupt_callback_t)(uint8_t pin);

typedef enum
{
  AFE_PPG_CHANNEL_1 = MAX86176_PPG_CHANNEL_1,
  AFE_PPG_CHANNEL_2 = MAX86176_PPG_CHANNEL_2,
}
afe_ppg_channel_t;

typedef enum
{
  AFE_PPG_LED_RED,
  AFE_PPG_LED_IR,
  AFE_PPG_LED_GREEN,
  AFE_PPG_LED_MAX
} 
afe_ppg_led_t;

typedef enum
{
  AFE_PPG_MEAS1 = MAX86176_PPG_MEAS1,
  AFE_PPG_MEAS2 = MAX86176_PPG_MEAS2,
  AFE_PPG_MEAS3 = MAX86176_PPG_MEAS3,
  AFE_PPG_MEAS4 = MAX86176_PPG_MEAS4,
  AFE_PPG_MEAS5 = MAX86176_PPG_MEAS5,
  AFE_PPG_MEAS6 = MAX86176_PPG_MEAS6,
  AFE_PPG_MEAS7 = MAX86176_PPG_MEAS7,
  AFE_PPG_MEAS8 = MAX86176_PPG_MEAS8,
  AFE_PPG_MEAS9 = MAX86176_PPG_MEAS9,
  AFE_PPG_MEAS_MAX = MAX86176_PPG_MEAS_MAX
} 
afe_ppg_measure_t;

typedef enum
{
  AFE_PPG_LED_CURRENT_STEP_0_125 = MAX86176_PPG_LED_DRIVE_RGE_32,
  AFE_PPG_LED_CURRENT_STEP_0_25  = MAX86176_PPG_LED_DRIVE_RGE_64,
  AFE_PPG_LED_CURRENT_STEP_0_375 = MAX86176_PPG_LED_DRIVE_RGE_96,
  AFE_PPG_LED_CURRENT_STEP_0_5   = MAX86176_PPG_LED_DRIVE_RGE_128,
  AFE_PPG_LED_CURRENT_STEP_MAX
} 
afe_ppg_led_current_step_t;

/* Public macros ------------------------------------------------------------ */
typedef struct
{
  uint8_t  ppg_led_current_step[AFE_PPG_LED_MAX];
  float    ppg_led_current[AFE_PPG_LED_MAX];
  uint32_t ppg_sample_rate;
}
afe_sensor_setting_t;

/* Public variables --------------------------------------------------------- */
extern afe_sensor_setting_t g_afe_setting;

/* Public APIs -------------------------------------------------------------- */
base_status_t bsp_afe_init(afe_interrupt_callback_t callback);
base_status_t bsp_afe_deinit(void);
base_status_t bsp_afe_pll_enable(bool enable);
base_status_t bsp_afe_ppg_enable(afe_ppg_channel_t channel, bool enable);
base_status_t bsp_afe_ppg_set_current(afe_ppg_led_t led, afe_ppg_led_current_step_t current_range, float current);
base_status_t bsp_afe_ppg_set_sample_rate(float sample_rate);

base_status_t bsp_afe_read_data(uint32_t *p_data, uint16_t *num_samples);

base_status_t bsp_afe_post(void);
base_status_t bsp_afe_bist(void);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
