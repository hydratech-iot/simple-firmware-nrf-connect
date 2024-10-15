/*
* File Name: max86176_regs.c
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

/* Public defines ----------------------------------------------------------- */
/* Status Registers */
#define MAX86176_REG_STATUS_1                     0x00
#define MAX86176_REG_STATUS_2                     0x01
#define MAX86176_REG_STATUS_3                     0x02
#define MAX86176_REG_STATUS_4                     0x03
#define MAX86176_REG_STATUS_5                     0x04
#define MAX86176_REG_STATUS_6                     0x05

/* FIFO Registers */
#define MAX86176_REG_FIFO_WRITE_POINTER           0x08
#define MAX86176_REG_FIFO_READ_POINTER            0x09
#define MAX86176_REG_FIFO_COUNTER_1               0x0A
#define MAX86176_REG_FIFO_COUNTER_2               0x0B
#define MAX86176_REG_FIFO_DATA_REGISTER           0x0C
#define MAX86176_REG_FIFO_CONFIGURATION_1         0x0D
#define MAX86176_REG_FIFO_CONFIGURATION_2         0x0E

/* System Control Registers */
#define MAX86176_REG_SYSTEM_CONFIGURATION_1       0x10
#define MAX86176_REG_SYSTEM_CONFIGURATION_2       0x11
#define MAX86176_REG_SYSTEM_CONFIGURATION_3       0x12
#define MAX86176_REG_SYSTEM_CONFIGURATION_4       0x13
#define MAX86176_REG_PHOTODIODE_BIAS              0x14
#define MAX86176_REG_PIN_FUNCTIONAL_CONFIGURATION 0x15
#define MAX86176_REG_OUTPUT_PIN_CONFIGURATION     0x16
#define MAX86176_REG_I2C_BROADCAST_ADDRESS        0x17

/* PLL Registers */
#define MAX86176_REG_PLL_CONFIGURATION_1          0x18
#define MAX86176_REG_PLL_CONFIGURATION_2          0x19
#define MAX86176_REG_PLL_CONFIGURATION_3          0x1A

/* PPG Frame Rate Clock Registers */
#define MAX86176_REG_FR_CLOCK_FREQUENCY_SELECT    0x1C
#define MAX86176_REG_FR_CLOCK_DIVIDER_MSB         0x1D
#define MAX86176_REG_FR_CLOCK_DIVIDER_LSB         0x1E

/* PPG MEAS1 Setup Registers */
#define MAX86176_REG_MEAS1_SELECTS                0x20
#define MAX86176_REG_MEAS1_CONFIGURATION_1        0x21
#define MAX86176_REG_MEAS1_CONFIGURATION_2        0x22
#define MAX86176_REG_MEAS1_CONFIGURATION_3        0x23
#define MAX86176_REG_MEAS1_CONFIGURATION_4        0x24
#define MAX86176_REG_MEAS1_LED_A_CURRENT          0x25
#define MAX86176_REG_MEAS1_LED_B_CURRENT          0x26

/* PPG MEAS2 Setup Registers */
#define MAX86176_REG_MEAS2_SELECTS                0x28
#define MAX86176_REG_MEAS2_CONFIGURATION_1        0x29
#define MAX86176_REG_MEAS2_CONFIGURATION_2        0x2A
#define MAX86176_REG_MEAS2_CONFIGURATION_3        0x2B
#define MAX86176_REG_MEAS2_CONFIGURATION_4        0x2C
#define MAX86176_REG_MEAS2_LED_A_CURRENT          0x2D
#define MAX86176_REG_MEAS2_LED_B_CURRENT          0x2E

/* PPG MEAS3 Setup Registers */
#define MAX86176_REG_MEAS3_SELECTS                0x30
#define MAX86176_REG_MEAS3_CONFIGURATION_1        0x31
#define MAX86176_REG_MEAS3_CONFIGURATION_2        0x32
#define MAX86176_REG_MEAS3_CONFIGURATION_3        0x33
#define MAX86176_REG_MEAS3_CONFIGURATION_4        0x34
#define MAX86176_REG_MEAS3_LED_A_CURRENT          0x35
#define MAX86176_REG_MEAS3_LED_B_CURRENT          0x36

/* PPG MEAS4 Setup Registers */
#define MAX86176_REG_MEAS4_SELECTS                0x38
#define MAX86176_REG_MEAS4_CONFIGURATION_1        0x39
#define MAX86176_REG_MEAS4_CONFIGURATION_2        0x3A
#define MAX86176_REG_MEAS4_CONFIGURATION_3        0x3B
#define MAX86176_REG_MEAS4_CONFIGURATION_4        0x3C
#define MAX86176_REG_MEAS4_LED_A_CURRENT          0x3D
#define MAX86176_REG_MEAS4_LED_B_CURRENT          0x3E

/* PPG MEAS5 Setup Registers */
#define MAX86176_REG_MEAS5_SELECTS                0x40
#define MAX86176_REG_MEAS5_CONFIGURATION_1        0x41
#define MAX86176_REG_MEAS5_CONFIGURATION_2        0x42
#define MAX86176_REG_MEAS5_CONFIGURATION_3        0x43
#define MAX86176_REG_MEAS5_CONFIGURATION_4        0x44
#define MAX86176_REG_MEAS5_LED_A_CURRENT          0x45
#define MAX86176_REG_MEAS5_LED_B_CURRENT          0x46

/* PPG MEAS6 Setup Registers */
#define MAX86176_REG_MEAS6_SELECTS                0x48
#define MAX86176_REG_MEAS6_CONFIGURATION_1        0x49
#define MAX86176_REG_MEAS6_CONFIGURATION_2        0x4A
#define MAX86176_REG_MEAS6_CONFIGURATION_3        0x4B
#define MAX86176_REG_MEAS6_CONFIGURATION_4        0x4C
#define MAX86176_REG_MEAS6_LED_A_CURRENT          0x4D
#define MAX86176_REG_MEAS6_LED_B_CURRENT          0x4E

/* PPG MEAS7 Setup Registers */
#define MAX86176_REG_MEAS7_SELECTS                0x50
#define MAX86176_REG_MEAS7_CONFIGURATION_1        0x51
#define MAX86176_REG_MEAS7_CONFIGURATION_2        0x52
#define MAX86176_REG_MEAS7_CONFIGURATION_3        0x53
#define MAX86176_REG_MEAS7_CONFIGURATION_4        0x54
#define MAX86176_REG_MEAS7_LED_A_CURRENT          0x55
#define MAX86176_REG_MEAS7_LED_B_CURRENT          0x56

/* PPG MEAS8 Setup Registers */
#define MAX86176_REG_MEAS8_SELECTS                0x58
#define MAX86176_REG_MEAS8_CONFIGURATION_1        0x59
#define MAX86176_REG_MEAS8_CONFIGURATION_2        0x5A
#define MAX86176_REG_MEAS8_CONFIGURATION_3        0x5B
#define MAX86176_REG_MEAS8_CONFIGURATION_4        0x5C
#define MAX86176_REG_MEAS8_LED_A_CURRENT          0x5D
#define MAX86176_REG_MEAS8_LED_B_CURRENT          0x5E

/* PPG MEAS9 Setup Registers */
#define MAX86176_REG_MEAS9_SELECTS                0x60
#define MAX86176_REG_MEAS9_CONFIGURATION_1        0x61
#define MAX86176_REG_MEAS9_CONFIGURATION_2        0x62
#define MAX86176_REG_MEAS9_CONFIGURATION_3        0x63
#define MAX86176_REG_MEAS9_CONFIGURATION_4        0x64
#define MAX86176_REG_MEAS9_LED_A_CURRENT          0x65
#define MAX86176_REG_MEAS9_LED_B_CURRENT          0x66

/* PPG Threshold Interrupts Registers */
#define MAX86176_REG_THRESHOLD_MEAS_SEL           0x70
#define MAX86176_REG_THRESHOLD_HYST               0x71
#define MAX86176_REG_PPG_HI_THRESHOLD1            0x72
#define MAX86176_REG_PPG_LO_THRESHOLD1            0x73
#define MAX86176_REG_PPG_HI_THRESHOLD2            0x74
#define MAX86176_REG_PPG_LO_THRESHOLD2            0x75

/* Interrupt Enables Registers */
#define MAX86176_REG_INTERRUPT_1_ENABLE_1         0x80
#define MAX86176_REG_INTERRUPT_1_ENABLE_2         0x81
#define MAX86176_REG_INTERRUPT_1_ENABLE_3         0x82
#define MAX86176_REG_INTERRUPT_1_ENABLE_4         0x83
#define MAX86176_REG_INTERRUPT_1_ENABLE_5         0x84
#define MAX86176_REG_INTERRUPT_1_ENABLE_6         0x85
#define MAX86176_REG_INTERRUPT_2_ENABLE_1         0x86
#define MAX86176_REG_INTERRUPT_2_ENABLE_2         0x87
#define MAX86176_REG_INTERRUPT_2_ENABLE_3         0x88
#define MAX86176_REG_INTERRUPT_2_ENABLE_4         0x89
#define MAX86176_REG_INTERRUPT_2_ENABLE_5         0x8A
#define MAX86176_REG_INTERRUPT_2_ENABLE_6         0x8B

/* ECG Setup Registers */
#define MAX86176_REG_ECG_CONFIGURATION_1          0x90
#define MAX86176_REG_ECG_CONFIGURATION_2          0x91
#define MAX86176_REG_ECG_CONFIGURATION_3          0x92

/* BioZ Calibration */
/* BioZ Lead Detect */
/* BioZ Lead Bias */
/* Respiration Setup */

/* Part ID Registers */
#define MAX86176_REG_PART_ID                      0xFF

/* Part ID Value */
#define MAX86176_ID                               0x39

/* FIFO Tag Value */
#define MAX86176_FIFO_TAG_PPG_MEAS_1              0x00
#define MAX86176_FIFO_TAG_PPG_MEAS_2              0x01
#define MAX86176_FIFO_TAG_PPG_MEAS_3              0x02
#define MAX86176_FIFO_TAG_PPG_MEAS_4              0x03
#define MAX86176_FIFO_TAG_PPG_MEAS_5              0x04
#define MAX86176_FIFO_TAG_PPG_MEAS_6              0x05
#define MAX86176_FIFO_TAG_PPG_DARK                0x06
#define MAX86176_FIFO_TAG_PPG_ALC_OVF             0x07
#define MAX86176_FIFO_TAG_PPG_EXP_OVF             0x08
#define MAX86176_FIFO_TAG_BIOZ_I                  0x09
#define MAX86176_FIFO_TAG_BIOZ_Q                  0x0A
#define MAX86176_FIFO_TAG_ECG_AND_FAST_RECOVER    0x0B
#define MAX86176_FIFO_TAG_ECG_P_N_DIFFERENTIAL    0x0C
#define MAX86176_FIFO_TAG_CAP_P_N_DIFFERENTIAL    0x0D
#define MAX86176_FIFO_TAG_ECG_TO_PPG_TIMING       0x0E
#define MAX86176_FIFO_TAG_BIOZ_TO_PPG_TIMING      0x0E
#define MAX86176_FIFO_TAG_ECG_TO_BIOZ_TIMING      0x0E
#define MAX86176_FIFO_TAG_MARKER                  0x0F
#define MAX86176_FIFO_TAG_INVALID_DATA            0x0F

/* FIFO Information */
#define MAX86176_FIFO_CNT_MAX                     256

/* Public enumerate/structure ----------------------------------------------- */
/**
 *  Struct of bits is LITTLE_ENDIAN
 */
typedef struct
{
    uint8_t bit0: 1;
    uint8_t bit1: 1;
    uint8_t bit2: 1;
    uint8_t bit3: 1;
    uint8_t bit4: 1;
    uint8_t bit5: 1;
    uint8_t bit6: 1;
    uint8_t bit7: 1;
} max86176_bits_t;

/* Status Registers */
typedef struct
{
    uint8_t pwr_rdy      : 1;
    uint8_t thresh1_hilo : 1;
    uint8_t thresh2_hilo : 1;
    uint8_t exp_ovf      : 1;
    uint8_t alc_ovf      : 1;
    uint8_t fifo_data_rdy: 1;
    uint8_t ppg_frame_rdy: 1;
    uint8_t a_full       : 1;
} max86176_reg_status_1_t;

typedef struct
{
    uint8_t not_used_01: 1;
    uint8_t not_used_02: 1;
    uint8_t invalid_cfg: 1;
    uint8_t not_used_03: 1;
    uint8_t not_used_04: 1;
    uint8_t not_used_05: 1;
    uint8_t vdd_oor    : 1;
    uint8_t not_used_06: 1;
} max86176_reg_status_2_t;

typedef struct
{
    uint8_t led1_compb     : 1;
    uint8_t led2_compb     : 1;
    uint8_t led3_compb     : 1;
    uint8_t led4_compb     : 1;
    uint8_t led5_compb     : 1;
    uint8_t led6_compb     : 1;
    uint8_t not_used_01    : 1;
    uint8_t not_used_02    : 1;
} max86176_reg_status_3_t;

typedef struct
{
    uint8_t pd1_short_t : 1;
    uint8_t pd2_short_t : 1;
    uint8_t pd3_short_t : 1;
    uint8_t pd4_short_t : 1;
    uint8_t not_used_01 : 1;
    uint8_t not_used_02 : 1;
    uint8_t not_used_03 : 1;
    uint8_t not_used_04 : 1;
} max86176_reg_status_4_t;

typedef struct
{
    uint8_t pll_lock    : 1;
    uint8_t pll_unlock  : 1;
    uint8_t rld_oor     : 1;
    uint8_t not_used_01 : 1;
    uint8_t not_used_02 : 1;
    uint8_t not_used_03 : 1;
    uint8_t ecg_fast_rec: 1;
} max86176_reg_status_5_t;

/* FIFO Registers */
typedef struct
{
    uint8_t fifo_wr_ptr: 8;
} max86176_reg_fifo_write_pointer_t;

typedef struct
{
    uint8_t fifo_rd_ptr: 8;
} max86176_reg_fifo_read_pointer_t;

typedef struct
{
    uint8_t ovf_counter    : 7;
    uint8_t fifo_data_count: 1;
} max86176_reg_fifo_counter_1_t;

typedef struct
{
    uint8_t fifo_data_count: 8;
} max86176_reg_fifo_counter_2_t;

typedef struct
{
    uint8_t fifo_data: 8;
} max86176_reg_fifo_data_register_t;

typedef struct
{
    uint8_t fifo_a_full: 8;
} max86176_reg_fifo_configuration_1_t;

typedef struct
{
    uint8_t not_used_01  : 1;
    uint8_t fifo_ro      : 1;
    uint8_t a_full_type  : 1;
    uint8_t fifo_stat_clr: 1;
    uint8_t flush_fifo   : 1;
    uint8_t fifo_mark    : 1;
    uint8_t not_used_02  : 2;
} max86176_reg_fifo_configuration_2_t;

typedef struct
{
    uint8_t reset         : 1;
    uint8_t shutdown      : 1;
    uint8_t ppg1_pwr_dn   : 1;
    uint8_t ppg2_pwr_dn   : 1;
    uint8_t sync_mode     : 2;
    uint8_t sw_force_sync : 1;
    uint8_t time_sys_reset: 1;
} max86176_reg_system_configuration_1_t;

typedef struct
{
  uint8_t meas1_en: 1;
  uint8_t meas2_en: 1;
  uint8_t meas3_en: 1;
  uint8_t meas4_en: 1;
  uint8_t meas5_en: 1;
  uint8_t meas6_en: 1;
  uint8_t meas7_en: 1;
  uint8_t meas8_en: 1;
} max86176_reg_system_configuration_2_t;

typedef struct
{
  uint8_t meas1_config_sel: 1;
  uint8_t collect_raw_data: 1;
  uint8_t ppg_timing_data : 1;
  uint8_t master          : 1;
  uint8_t alc_disable     : 1;
  uint8_t not_used_01     : 1;
  uint8_t en_vdd_oor      : 1;
  uint8_t meas9_en        : 1;
} max86176_reg_system_configuration_3_t;

typedef struct
{
  uint8_t prox_auto     : 1;
  uint8_t prox_data_en  : 1;
  uint8_t samp_sync_freq: 5;
  uint8_t not_used_01   : 1;
} max86176_reg_system_configuration_4_t;

typedef struct
{
    uint8_t pd1_bias : 2;
    uint8_t pd2_bias : 2;
    uint8_t pd3_bias : 2;
    uint8_t pd4_bias : 2;
} max86176_reg_photodiode_bias_t;

typedef struct
{
    uint8_t trig_icfg  : 1;
    uint8_t int1_fcfg  : 2;
    uint8_t int2_fcfg  : 2;
    uint8_t not_used_01: 3;
} max86176_reg_pin_functional_configuration_t;

typedef struct
{
    uint8_t not_used_01: 1;
    uint8_t int1_ocfg  : 2;
    uint8_t int2_ocfg  : 2;
    uint8_t not_used_02: 3;
} max86176_reg_output_pin_configuration_t;

typedef struct
{
    uint8_t i2c_bcast_e   : 1;
    uint8_t i2c_bcast_addr: 7;
} max86176_reg_i2c_broadcast_address_t;

/* PLL Registers */
typedef struct
{
    uint8_t pll_en       : 1;
    uint8_t not_used_01  : 6;
    uint8_t pll_lock_wndw: 1;
} max86176_reg_pll_configuration_1_t;

typedef struct
{
    uint8_t mdiv       : 6;
    uint8_t not_used_01: 1;
    uint8_t ndiv_8     : 1;
} max86176_reg_pll_configuration_2_t;

typedef struct
{
    uint8_t ndiv_7_0: 8;
} max86176_reg_pll_configuration_3_t;

typedef struct
{
    uint8_t fr_clk_find_tune: 5;
    uint8_t fr_clk_sel      : 1;
    uint8_t not_used_01     : 2;
} max86176_fr_clock_frequency_select_t;

typedef struct
{
    uint8_t fr_clk_div_14_8: 7;
    uint8_t not_used_01    : 1;
} max86176_reg_fr_clock_divider_msb_t;

typedef struct
{
    uint8_t fr_clk_div_7_0: 8;
} max86176_reg_fr_clock_divider_lsb_t;

/* PPG MEAS1 - MEAS9 Setup Registers */
typedef struct
{
    uint8_t meas_x_drva: 3;
    uint8_t meas_x_drvb: 3;
    uint8_t meas_x_amb : 1;
    uint8_t not_used_01: 1;
} max86176_reg_meas_x_selects_t;

typedef struct
{
    uint8_t meas_x_aver: 3;
    uint8_t meas_x_tint: 2;
    uint8_t not_used_01: 3;
} max86176_reg_meas_x_configuration_1_t;

typedef struct
{
    uint8_t meas_x_ppg1_adc_rge: 2;
    uint8_t meas_x_ppg2_adc_rge: 2;
    uint8_t meas_x_led_rge     : 2;
    uint8_t meas_x_filt_sel    : 1;
    uint8_t meas_x_sinc3_sel   : 1;
} max86176_reg_meas_x_configuration_2_t;

typedef struct
{
    uint8_t meas_x_ppg1_dacoff: 2;
    uint8_t meas_x_ppg2_dacoff: 2;
    uint8_t meas_x_led_setlng : 2;
    uint8_t meas_x_pd_setlng  : 2;
} max86176_reg_meas_x_configuration_3_t;

typedef struct
{
    uint8_t meas_x_ppg1_pd_sel: 2;
    uint8_t meas_x_ppg2_pd_sel: 2;
    uint8_t meas_x_ppg_gain   : 2;
    uint8_t meas_x_bufchan    : 1;
    uint8_t not_used_01       : 1;
} max86176_reg_meas_x_configuration_4_t;

typedef struct
{
    uint8_t meas_x_drva_pa: 8;
} max86176_reg_meas_x_leda_current_t;

typedef struct
{
    uint8_t meas_x_drvb_pa: 8;
} max86176_reg_meas_x_ledb_current_t;

/* PPG Threshold Interrupts Registers */
typedef struct
{
    uint8_t thresh1_meas_sel: 4;
    uint8_t thresh2_meas_sel: 4;
} max86176_reg_threshold_meas_sel_t;

typedef struct
{
    uint8_t level_hyst     : 3;
    uint8_t time_hyst      : 2;
    uint8_t not_used_01    : 1;
    uint8_t thresh1_ppg_sel: 1;
    uint8_t thresh2_ppg_sel: 1;
} max86176_reg_threshold_hyst_t;

typedef struct
{
    uint8_t threshold1_upper: 8;
} max86176_reg_ppg_hi_threshold1_t;

typedef struct
{
    uint8_t threshold1_lower: 8;
} max86176_reg_ppg_lo_threshold1_t;

typedef struct
{
    uint8_t threshold2_upper: 8;
} max86176_reg_ppg_hi_threshold2_t;

typedef struct
{
    uint8_t threshold2_lower: 8;
} max86176_reg_ppg_lo_threshold2_t;

/* ECG Setup Registers */
typedef struct
{
    uint8_t ecg_en      : 1;
    uint8_t ecg_dec_rate: 3;
    uint8_t not_used_01 : 4;
} max86176_reg_ecg_configuration_1_t;

/* Interrupt Enables Registers */
typedef struct
{
    uint8_t led_tx_en1        : 1;
    uint8_t ppg_thresh1_hilo_en: 1;
    uint8_t ppg_thresh2_hilo_en: 1;
    uint8_t exp_ovf_en         : 1;
    uint8_t alc_ovf_en         : 1;
    uint8_t fifo_data_rdy_en   : 1;
    uint8_t frame_rdy_en       : 1;
    uint8_t a_full_en          : 1;
} max86176_reg_interrupt_x_enable_1_t;

typedef struct
{
    uint8_t not_used_01         : 2;
    uint8_t invalid_cfg_en1     : 1;
    uint8_t not_used_02         : 3;
    uint8_t vdd_oor_en1         : 1;
    uint8_t timing_sys_reset_en1: 1;
} max86176_reg_interrupt_x_enable_2_t;

typedef struct
{
    uint8_t led1_compb_en     : 1;
    uint8_t led2_compb_en     : 1;
    uint8_t led3_compb_en     : 1;
    uint8_t led4_compb_en     : 1;
    uint8_t led5_compb_en     : 1;
    uint8_t led6_compb_en     : 1;
    uint8_t not_used_01       : 2;
} max86176_reg_interrupt_x_enable_3_t;

typedef struct
{
    uint8_t pd1_short_en1: 1;
    uint8_t pd2_short_en1: 1;
    uint8_t pd3_short_en1: 1;
    uint8_t pd4_short_en1: 1;
    uint8_t not_used_01  : 4;
} max86176_reg_interrupt_x_enable_4_t;

typedef struct
{
    uint8_t samp_sync_en1   : 1;
    uint8_t pll_lock_en1    : 1;
    uint8_t pll_unlock_en1  : 1;
    uint8_t rld_oor_en1     : 3;
    uint8_t not_used_01     : 1;
    uint8_t ecg_fast_rec_en1: 1;
} max86176_reg_interrupt_x_enable_5_t;

typedef struct
{
    uint8_t dc_loff_nl_en1: 1;
    uint8_t dc_loff_nh_en1: 1;
    uint8_t dc_loff_pl_en1: 1;
    uint8_t dc_loff_ph_en1: 1;
    uint8_t not_used_01   : 1;
    uint8_t ac_loff_en1   : 1;
    uint8_t not_used_02   : 1;
    uint8_t lon_en1       : 1;
} max86176_reg_interrupt_x_enable_6_t;

/* Part ID Registers */
typedef struct
{
    uint8_t part_id: 8;
} max86176_reg_part_id_t;

/* Registers */
typedef union
{
    uint8_t                                      byte;
    max86176_bits_t                              bits;
    max86176_reg_status_1_t                      status_1;
    max86176_reg_status_2_t                      status_2;
    max86176_reg_status_3_t                      status_3;
    max86176_reg_status_4_t                      status_4;
    max86176_reg_status_5_t                      status_5;
    max86176_reg_fifo_write_pointer_t            fifo_write_pointer;
    max86176_reg_fifo_read_pointer_t             fifo_read_pointer;
    max86176_reg_fifo_counter_1_t                fifo_counter_1;
    max86176_reg_fifo_counter_2_t                fifo_counter_2;
    max86176_reg_fifo_data_register_t            fifo_data_register;
    max86176_reg_fifo_configuration_1_t          fifo_configuration_1;
    max86176_reg_fifo_configuration_2_t          fifo_configuration_2;
    max86176_reg_system_configuration_1_t        system_configuration_1;
    max86176_reg_system_configuration_2_t        system_configuration_2;
    max86176_reg_system_configuration_3_t        system_configuration_3;
    max86176_reg_system_configuration_4_t        system_configuration_4;
    max86176_reg_photodiode_bias_t               photodiode_bias;
    max86176_reg_pin_functional_configuration_t  pin_functional_configuration;
    max86176_reg_output_pin_configuration_t      output_pin_configuration;
    max86176_reg_i2c_broadcast_address_t         i2c_broadcast_address;
    max86176_reg_pll_configuration_1_t           pll_configuration_1;
    max86176_reg_pll_configuration_2_t           pll_configuration_2;
    max86176_reg_pll_configuration_3_t           pll_configuration_3;
    max86176_fr_clock_frequency_select_t         fr_clock_frequency_select;
    max86176_reg_fr_clock_divider_msb_t          fr_clock_divider_msb;
    max86176_reg_fr_clock_divider_lsb_t          fr_clock_divider_lsb;
    max86176_reg_meas_x_selects_t                meas_x_selects;
    max86176_reg_meas_x_configuration_1_t        meas_x_configuration_1;
    max86176_reg_meas_x_configuration_2_t        meas_x_configuration_2;
    max86176_reg_meas_x_configuration_3_t        meas_x_configuration_3;
    max86176_reg_meas_x_configuration_4_t        meas_x_configuration_4;
    max86176_reg_meas_x_leda_current_t           meas_x_leda_current;
    max86176_reg_meas_x_ledb_current_t           meas_x_ledb_current;
    max86176_reg_threshold_meas_sel_t            threshold_meas_sel;
    max86176_reg_threshold_hyst_t                threshold_hyst;
    max86176_reg_ppg_hi_threshold1_t             ppg_hi_threshold1;
    max86176_reg_ppg_lo_threshold1_t             ppg_lo_threshold1;
    max86176_reg_ppg_hi_threshold2_t             ppg_hi_threshold2;
    max86176_reg_ppg_lo_threshold2_t             ppg_lo_threshold2;
    max86176_reg_ecg_configuration_1_t           ecg_configuration_1;
    max86176_reg_interrupt_x_enable_1_t          interrupt_x_enable_1;
    max86176_reg_interrupt_x_enable_2_t          interrupt_x_enable_2;
    max86176_reg_interrupt_x_enable_3_t          interrupt_x_enable_3;
    max86176_reg_interrupt_x_enable_4_t          interrupt_x_enable_4;
    max86176_reg_interrupt_x_enable_5_t          interrupt_x_enable_5;
    max86176_reg_part_id_t                       part_id;
} max86176_reg_t;


/* System configuration */
typedef enum
{
  MAX86176_INT_CONFIG_DISABLE = 0,
  MAX86176_INT_CONFIG_ENABLE_CLEARED_BY_READ_STATUS_FIFO,
  MAX86176_INT_CONFIG_ENABLE_CLEARED_AFTER_30_5_US,
  MAX86176_INT_CONFIG_ENABLE_CLEARED_AFTER_244_US
}
max86176_interrupt_func_config_t;

typedef enum
{
  MAX86176_PPG_CAPACITANCE_NOT_RECOMMENDED = 0,
  MAX86176_PPG_CAPACITANCE_0_125,
  MAX86176_PPG_CAPACITANCE_125_150,
  MAX86176_PPG_CAPACITANCE_250_500
}
max86176_ppg_bias_t;
typedef enum
{
  MAX86176_OUTPUT_CONFIG_OPEN_DRAIN_ACTIVE_LOW = 0,
  MAX86176_OUTPUT_CONFIG_ACTIVE_HIGH,
  MAX86176_OUTPUT_CONFIG_ACTIVE_LOW,
  MAX86176_OUTPUT_CONFIG_DO_NOT_USE
}
max86176_output_pin_config_t;

typedef struct
{
  max86176_reg_system_configuration_1_t config_1;
  max86176_reg_system_configuration_2_t config_2;
  max86176_reg_system_configuration_3_t config_3;
  max86176_reg_system_configuration_4_t config_4;
  max86176_ppg_bias_t pd1_bias;
  max86176_ppg_bias_t pd2_bias;
  max86176_ppg_bias_t pd3_bias;
  max86176_ppg_bias_t pd4_bias;
  max86176_interrupt_func_config_t int1_func_config;
  max86176_interrupt_func_config_t int2_func_config;
  max86176_output_pin_config_t int1_output_config;
  max86176_output_pin_config_t int2_output_config;
}
max86176_sys_config_t;

/* PLL configuration */
typedef enum
{
  MAX86176_INTERNAL_OSCILLATOR = 0,
  MAX86176_EXTERNAL_OSCILLATOR
}
max86176_ref_clk_sel_t;

typedef enum
{
  MAX86176_REF_CLK_32_KHZ = 0,
  MAX86176_REF_CLK_32_768_KHZ
}
max86176_clk_freq_sel_t;

typedef enum
{
  MAX86176_ECG_FDIV_ADC_CLK_DISABLE = 0,
  MAX86176_ECG_FDIV_1,
  MAX86176_ECG_FDIV_2,
  MAX86176_ECG_FDIV_4,
  MAX86176_ECG_FDIV_8,
  MAX86176_ECG_FDIV_16
}
max86176_ecg_fdiv_t;

typedef enum
{
  MAX86176_BIOZ_NDIV_256 = 0,
  MAX86176_BIOZ_NDIV_512,
  MAX86176_BIOZ_NDIV_1024
}
max86176_bioz_ndiv_t;

typedef enum
{
  MAX86176_BIOZ_KDIV_1 = 0,
  MAX86176_BIOZ_KDIV_2,
  MAX86176_BIOZ_KDIV_4,
  MAX86176_BIOZ_KDIV_8,
  MAX86176_BIOZ_KDIV_16,
  MAX86176_BIOZ_KDIV_32,
  MAX86176_BIOZ_KDIV_64,
  MAX86176_BIOZ_KDIV_128,
  MAX86176_BIOZ_KDIV_256,
  MAX86176_BIOZ_KDIV_512,
  MAX86176_BIOZ_KDIV_1024,
  MAX86176_BIOZ_KDIV_2048,
  MAX86176_BIOZ_KDIV_4096,
  MAX86176_BIOZ_KDIV_8192
}
max86176_bioz_kdiv_t;

typedef struct
{
  bool pll_enable;
  uint16_t pll_mdiv;
  uint16_t pll_ndiv;
}
max86176_pll_config_t;

typedef enum
{
  MAX86176_CLOCK_FREQUENCY_32000 = 0,
  MAX86176_CLOCK_FREQUENCY_32768
}
max86176_clock_frequency_t;

/* PPG Frame Rate Configuration */
typedef struct
{
  max86176_clock_frequency_t clk_select;
  uint16_t clk_div;
}
max86176_ppg_frame_rate_config_t;

/* PPG configuration */
typedef enum
{
  MAX86176_PPG_CHANNEL_1 = 0,
  MAX86176_PPG_CHANNEL_2
}
max86176_ppg_channel_t;

/* PPG measure configuration */
typedef enum
{
  MAX86176_PPG_MEAS1 = 0,
  MAX86176_PPG_MEAS2,
  MAX86176_PPG_MEAS3,
  MAX86176_PPG_MEAS4,
  MAX86176_PPG_MEAS5,
  MAX86176_PPG_MEAS6,
  MAX86176_PPG_MEAS7,
  MAX86176_PPG_MEAS8,
  MAX86176_PPG_MEAS9,
  MAX86176_PPG_MEAS_MAX
}
max86176_ppg_measure_t;


typedef enum
{
  MAX86176_PPG_LED_DRVA = 0,
  MAX86176_PPG_LED_DRVB
}
max86176_led_drv_t;

typedef enum
{
  MAX86176_PPG_AMB_NORMAL_MODE = 0,
  MAX86176_PPG_AMB_DIRECT_AMBIENT_CONVERSION
}
max86176_ppg_ambient_t;

typedef enum
{
  MAX86176_PPG_LED1_DRV = 0,
  MAX86176_PPG_LED2_DRV,
  MAX86176_PPG_LED3_DRV,
  MAX86176_PPG_LED4_DRV,
  MAX86176_PPG_LED5_DRV,
  MAX86176_PPG_LED6_DRV
}
max86176_led_drv_pin_t;

typedef enum
{
  MAX86176_PPG_LED_PULSES_1 = 0,
  MAX86176_PPG_LED_PULSES_2,
  MAX86176_PPG_LED_PULSES_4,
  MAX86176_PPG_LED_PULSES_8,
  MAX86176_PPG_LED_PULSES_16,
  MAX86176_PPG_LED_PULSES_32,
  MAX86176_PPG_LED_PULSES_64,
  MAX86176_PPG_LED_PULSES_128
}
max86176_num_led_pulses_t;

typedef enum
{
  MAX86176_PPG_MEAS_INTEGRATION_14_6 = 0,
  MAX86176_PPG_MEAS_INTEGRATION_29_2,
  MAX86176_PPG_MEAS_INTEGRATION_58_6,
  MAX86176_PPG_MEAS_INTEGRATION_117,
}
max86176_ppg_meas_integration_t;

typedef enum
{
  MAX86176_PPG_COI3_FILTER_IS_USED = 0,
  MAX86176_PPG_SINC3_FILTER_IS_USED,
}
max86176_ppg_sinc3_select_t;

typedef enum
{
  MAX86176_PPG_FILTER_SELECT_CENTRAL = 0,
  MAX86176_PPG_FILTER_SELECT_FORWARD,
}
max86176_ppg_filter_select_t;

typedef enum
{
  MAX86176_PPG_ADC_RANGE_4 = 0,
  MAX86176_PPG_ADC_RANGE_8,
  MAX86176_PPG_ADC_RANGE_16,
  MAX86176_PPG_ADC_RANGE_32
}
max86176_ppg_range_t;

typedef enum
{
  MAX86176_PPG_SETLNG_TIME_8 = 0,
  MAX86176_PPG_SETLNG_TIME_12,
  MAX86176_PPG_SETLNG_TIME_16,
  MAX86176_PPG_SETLNG_TIME_24
}
max86176_ppg_settling_time_t;

typedef enum
{
  MAX86176_PPG_LED_DRIVE_RGE_32 = 0,
  MAX86176_PPG_LED_DRIVE_RGE_64,
  MAX86176_PPG_LED_DRIVE_RGE_96,
  MAX86176_PPG_LED_DRIVE_RGE_128
}
max86176_ppg_led_drive_rge_t;

typedef enum
{
  MAX86176_PPG_PD1_IS_INPUT = 0,
  MAX86176_PPG_PD2_IS_INPUT,
  MAX86176_PPG_PD3_IS_INPUT,
  MAX86176_PPG_PD4_IS_INPUT
}
max86176_ppg_pd_select_t;

typedef enum
{
  MAX86176_PPG_GAIN_1 = 0,
  MAX86176_PPG_GAIN_2,
  MAX86176_PPG_GAIN_4
}
max86176_ppg_gain_t;


typedef struct
{
  max86176_ppg_ambient_t ambient;
  max86176_led_drv_pin_t led_drv_a;
  max86176_led_drv_pin_t led_drv_b;
  max86176_num_led_pulses_t num_led_pulses;
  max86176_ppg_meas_integration_t meas_integration;

  max86176_ppg_range_t ppg1_range;
  max86176_ppg_range_t ppg2_range;
  max86176_ppg_led_drive_rge_t led_range;
  max86176_ppg_filter_select_t filter_select;
  max86176_ppg_sinc3_select_t sinc3_select;
  
  uint8_t ppg1_dac_offset; // [0:30]
  uint8_t ppg2_dac_offset; // [0:30]
  max86176_ppg_settling_time_t pd_setlng_time;
  max86176_ppg_settling_time_t led_setlng_time;

  max86176_ppg_pd_select_t ppg1_pd_sel;
  max86176_ppg_pd_select_t ppg2_pd_sel;
  max86176_ppg_gain_t ppg_gain;

  uint8_t led_a_current; // [0:255]
  uint8_t led_b_current; // [0:255]
}
max86176_ppg_measure_config_t;

/* ECG configuration */
typedef enum
{
  MAX86176_ECG_DEC_RATE_16 = 0,
  MAX86176_ECG_DEC_RATE_32,
  MAX86176_ECG_DEC_RATE_64,
  MAX86176_ECG_DEC_RATE_128,
  MAX86176_ECG_DEC_RATE_256,
  MAX86176_ECG_DEC_RATE_512
}
max86176_ecg_dec_rate_t;

typedef enum
{
  MAX86176_ECG_INPUT_NON_INVERTED = 0,
  MAX86176_ECG_INPUT_INVERTED
}
max86176_ecg_input_polarity_t;

typedef enum
{
  MAX86176_ECG_PGA_GAIN_1 = 0,
  MAX86176_ECG_PGA_GAIN_2 = 1,
  MAX86176_ECG_PGA_GAIN_4 = 2,
  MAX86176_ECG_PGA_GAIN_8 = 3,
  MAX86176_ECG_PGA_GAIN_16 = 7,
}
max86176_ecg_pga_gain_t;

typedef enum
{
  MAX86176_ECG_AUTO_REC_DISABLE = 0,
  MAX86176_ECG_AUTO_REC_ENABLE = 1,
}
max86176_ecg_auto_rec_t;

typedef enum
{
  MAX86176_ECG_MUX_SEL_NOT_CONNECTED = 0,
  MAX86176_ECG_MUX_SEL_P_EL1_N_EL2,
  MAX86176_ECG_MUX_SEL_P_EL3_N_EL1,
  MAX86176_ECG_MUX_SEL_P_EL2_N_EL3,
}
max86176_ecg_mux_sel_t;

typedef enum
{
  MAX86176_ECG_FAST_REC_NORMAL_MODE = 0,
  MAX86176_ECG_FAST_REC_MANUAL_FAST_RECOVER_MODE,
  MAX86176_ECG_FAST_REC_DIGITAL_FAST_RECOVER_MODE
}
max86176_ecg_fast_rec_t;

typedef enum
{
  MAX86176_ECG_INA_GAIN_RGE_0 = 0,
  MAX86176_ECG_INA_GAIN_RGE_1,
  MAX86176_ECG_INA_GAIN_RGE_2,
  MAX86176_ECG_INA_GAIN_RGE_3
}
max86176_ecg_ina_gain_rge_t;

typedef enum
{
  MAX86176_ECG_INA_GAIN_10 = 0,
  MAX86176_ECG_INA_GAIN_20,
  MAX86176_ECG_INA_GAIN_40,
  MAX86176_ECG_INA_GAIN_60
}
max86176_ecg_ina_gain_t;

typedef enum
{
  MAX86176_RLD_MODE_OPEN_LOOP_BODY_BIAS = 0,
  MAX86176_RLD_MODE_CLOSE_LOOP_RIGHT_LEG_DRIVE
}
max86176_rld_mode_t;

typedef enum
{
  MAX86176_RLD_GAIN_12 = 0,
  MAX86176_RLD_GAIN_24,
  MAX86176_RLD_GAIN_48,
  MAX86176_RLD_GAIN_97
}
max86176_rld_gain_t;

typedef struct
{
  bool enable;
}
max86176_ecg_config_t;

/* Interrupt configuration */
typedef enum
{
  MAX86176_INTR_1 = 0,
  MAX86176_INTR_2,
  MAX86176_INTR_MAX,
}
max86176_interrupt_type_t;

typedef struct
{
  max86176_reg_interrupt_x_enable_1_t intr_en_1;
  max86176_reg_interrupt_x_enable_2_t intr_en_2;
  max86176_reg_interrupt_x_enable_3_t intr_en_3;
  max86176_reg_interrupt_x_enable_4_t intr_en_4;
  max86176_reg_interrupt_x_enable_5_t intr_en_5;
}
max86176_interrupt_config_t;

/* FIFO configuration */
typedef enum
{
  MAX86176_FIFO_STAT_CLR_BY_STATUS_REGISTER = 0,
  MAX86176_FIFO_STAT_CLR_BY_STATUS_FIFO_REGISTER
}
max86176_fifo_status_clear_t;

typedef struct
{
  uint16_t fifo_threshold;
  max86176_fifo_status_clear_t fifo_status_clear;
}
max86176_fifo_config_t;

/* MAX86176 configuration */
typedef struct
{
  max86176_sys_config_t system;
  max86176_pll_config_t pll;
  max86176_ppg_frame_rate_config_t ppg_frame_rate;
  max86176_ppg_measure_config_t ppg_meas[MAX86176_PPG_MEAS_MAX];
  max86176_ecg_config_t ecg;
  max86176_interrupt_config_t interrupt[MAX86176_INTR_MAX];
  max86176_fifo_config_t fifo;
}
max86176_config_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
/* Public APIs -------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */
