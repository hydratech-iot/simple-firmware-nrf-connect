/*
* File Name: max86176.c
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

/* Public includes ---------------------------------------------------------- */
#include "max86176.h"

/* Private includes --------------------------------------------------------- */
/* Private defines ---------------------------------------------------------- */
#define MAX86176_READ_CMD        (0x80)
#define MAX86176_WRITE_CMD       (0x00)

#define MAX86176_SPI_SELECT()     me->spi_select()
#define MAX86176_SPI_DESELECT()   me->spi_deselect()

/* Private enumerate/structure ---------------------------------------------- */
/* Private macros ----------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
/* Private variables -------------------------------------------------------- */
static max86176_reg_t  g_max86176_reg;

/* Private prototypes ------------------------------------------------------- */
static base_status_t max86176_write_reg(max86176_t *me, uint8_t reg, uint8_t data);
static base_status_t max86176_read_reg(max86176_t *me, uint8_t reg, uint8_t *p_data);
static base_status_t max86176_read_burst(max86176_t *me, uint8_t reg, uint32_t *p_data, uint16_t num_samples);

static base_status_t max86176_software_reset(max86176_t *me);
static base_status_t max86176_system_config(max86176_t *me, const max86176_sys_config_t *config);
static base_status_t max86176_fifo_config(max86176_t *me, const max86176_fifo_config_t *config);
static base_status_t max86176_ppg_measure_config(max86176_t *me, const max86176_ppg_measure_config_t *config);
static base_status_t max86176_pll_config(max86176_t *me, const max86176_pll_config_t *config);

static base_status_t max86176_ecg_config(max86176_t *me, const max86176_ecg_config_t *config);
static base_status_t max86176_interrupt_config(max86176_t *me, const max86176_interrupt_config_t *config);

/* Public implementations --------------------------------------------------- */
base_status_t max86176_init(max86176_t *me, const max86176_config_t *config)
{
  assert(me != NULL && me->spi_transmit != NULL && me->spi_transmit_receive != NULL);

  // Check device ID
  CHECK_STATUS(max86176_check_device_id(me));

  // Software reset
  CHECK_STATUS(max86176_software_reset(me));

  // System Configuration
  CHECK_STATUS(max86176_system_config(me, &config->system));

  // FIFO Configuration
  CHECK_STATUS(max86176_fifo_config(me, &config->fifo));

  // PLL Configuration
  CHECK_STATUS(max86176_pll_config(me, &config->pll));

  // PPG Frame Rate Configuration
  CHECK_STATUS(max86176_ppg_frame_rate_config(me, &config->ppg_frame_rate));

  // PPG Measure Configuration
  CHECK_STATUS(max86176_ppg_measure_config(me, config->ppg_meas));

  // ECG Configuration
  CHECK_STATUS(max86176_ecg_config(me, &config->ecg));

  // Interrupt Configuration
  CHECK_STATUS(max86176_interrupt_config(me, config->interrupt));

  return BS_OK;
}

base_status_t max86176_ppg_set_led_rge(max86176_t *me, max86176_ppg_measure_t measure, max86176_ppg_led_drive_rge_t led_drive_current)
{
#define NEXT_REG_ADDR (measure * 8)

  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_MEAS1_CONFIGURATION_4 + NEXT_REG_ADDR, &g_max86176_reg.byte));
  g_max86176_reg.meas_x_configuration_2.meas_x_led_rge  = led_drive_current;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_CONFIGURATION_4 + NEXT_REG_ADDR, g_max86176_reg.byte));

  return BS_OK;

#undef NEXT_REG_ADDR
}

base_status_t max86176_ppg_set_led_current(max86176_t *me, max86176_ppg_measure_t measure, max86176_led_drv_t led_drv, uint8_t value)
{
#define NEXT_REG_ADDR (measure * 8)

  if (led_drv == MAX86176_PPG_LED_DRVA)
  {
    CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_MEAS1_LED_A_CURRENT + NEXT_REG_ADDR, &g_max86176_reg.byte));
    g_max86176_reg.meas_x_leda_current.meas_x_drva_pa = value;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_LED_A_CURRENT + NEXT_REG_ADDR, g_max86176_reg.byte));
  }
  else if (led_drv == MAX86176_PPG_LED_DRVB)
  {
    CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_MEAS1_LED_B_CURRENT + NEXT_REG_ADDR, &g_max86176_reg.byte));
    g_max86176_reg.meas_x_ledb_current.meas_x_drvb_pa = value;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_LED_B_CURRENT + NEXT_REG_ADDR, g_max86176_reg.byte));
  }

  return BS_OK;

#undef NEXT_REG_ADDR
}

base_status_t max86176_pll_enable(max86176_t *me, bool enable)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_PLL_CONFIGURATION_1, &g_max86176_reg.byte));
  g_max86176_reg.pll_configuration_1.pll_en = enable;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_PLL_CONFIGURATION_1, g_max86176_reg.byte));

  return BS_OK;
}

base_status_t max86176_ppg_enable(max86176_t *me, max86176_ppg_channel_t channel, bool enable)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, &g_max86176_reg.byte));

  if (channel == MAX86176_PPG_CHANNEL_1)
  {
    g_max86176_reg.system_configuration_1.ppg1_pwr_dn = !enable;
  }
  else if (channel == MAX86176_PPG_CHANNEL_2)
  {
    g_max86176_reg.system_configuration_1.ppg2_pwr_dn = !enable;
  }
  else
  {
    return BS_ERROR;
  }

  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, g_max86176_reg.byte));

  return BS_OK;
}

base_status_t max86176_ecg_enable(max86176_t *me, bool enable)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_ECG_CONFIGURATION_1, &g_max86176_reg.byte));
  g_max86176_reg.ecg_configuration_1.ecg_en = enable;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_ECG_CONFIGURATION_1, g_max86176_reg.byte));

  return BS_OK;
}

base_status_t max86176_set_software_power_down_mode(max86176_t *me, bool enable)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, &g_max86176_reg.byte));
  g_max86176_reg.system_configuration_1.shutdown = enable;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, g_max86176_reg.byte));

  return BS_OK;
}

base_status_t max86176_read_fifo(max86176_t *me, uint32_t *p_data, uint16_t num_samples)
{
  uint8_t status;
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_STATUS_1, &status));
  CHECK_STATUS(max86176_read_burst(me, MAX86176_REG_FIFO_DATA_REGISTER, p_data, num_samples));

  return BS_OK;
}

base_status_t max86176_get_fifo_count(max86176_t *me, uint16_t *fifo_cnt)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_FIFO_COUNTER_1, &g_max86176_reg.byte));

  if ((g_max86176_reg.fifo_counter_1.fifo_data_count) || (g_max86176_reg.fifo_counter_1.ovf_counter))
  {
    *fifo_cnt = MAX86176_FIFO_CNT_MAX;
    return BS_OK;
  }

  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_FIFO_COUNTER_2, &g_max86176_reg.byte));

  *fifo_cnt = g_max86176_reg.byte;

  return BS_OK;
}

base_status_t max86176_check_device_id(max86176_t *me)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_PART_ID, &g_max86176_reg.byte));
  if (g_max86176_reg.byte != MAX86176_ID)
  {
    return BS_ERROR;
  }

  return BS_OK;
}

base_status_t max86176_ppg_frame_rate_config(max86176_t *me, const max86176_ppg_frame_rate_config_t *config)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_FR_CLOCK_FREQUENCY_SELECT, &g_max86176_reg.byte));
  g_max86176_reg.fr_clock_frequency_select.fr_clk_sel = config->clk_select >> 5;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_FR_CLOCK_FREQUENCY_SELECT, g_max86176_reg.byte));

  g_max86176_reg.fr_clock_divider_msb.fr_clk_div_14_8 = config->clk_div >> 8;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_FR_CLOCK_DIVIDER_MSB, g_max86176_reg.byte));

  g_max86176_reg.fr_clock_divider_lsb.fr_clk_div_7_0 = config->clk_div;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_FR_CLOCK_DIVIDER_LSB, g_max86176_reg.byte));

  return BS_OK;
}

/* Private implementations -------------------------------------------------- */
static base_status_t max86176_software_reset(max86176_t *me)
{
  g_max86176_reg.system_configuration_1.reset = true;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, g_max86176_reg.byte));

  return BS_OK;
}

static base_status_t max86176_system_config(max86176_t *me, const max86176_sys_config_t *config)
{
  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, &g_max86176_reg.byte));
  g_max86176_reg.system_configuration_1 = config->config_1;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_1, g_max86176_reg.byte));

  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_2, &g_max86176_reg.byte));
  g_max86176_reg.system_configuration_2 = config->config_2;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_2, g_max86176_reg.byte));

  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_3, &g_max86176_reg.byte));
  g_max86176_reg.system_configuration_3 = config->config_3;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_3, g_max86176_reg.byte));

  CHECK_STATUS(max86176_read_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_4, &g_max86176_reg.byte));
  g_max86176_reg.system_configuration_4 = config->config_4;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_SYSTEM_CONFIGURATION_4, g_max86176_reg.byte));

  g_max86176_reg.photodiode_bias.pd1_bias = config->pd1_bias;
  g_max86176_reg.photodiode_bias.pd2_bias = config->pd2_bias;
  g_max86176_reg.photodiode_bias.pd3_bias = config->pd3_bias;
  g_max86176_reg.photodiode_bias.pd4_bias = config->pd4_bias;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_PHOTODIODE_BIAS, g_max86176_reg.byte));

  g_max86176_reg.pin_functional_configuration.int1_fcfg = config->int1_func_config;
  g_max86176_reg.pin_functional_configuration.int2_fcfg = config->int2_func_config;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_PIN_FUNCTIONAL_CONFIGURATION, g_max86176_reg.byte));

  g_max86176_reg.output_pin_configuration.int1_ocfg = config->int1_output_config;
  g_max86176_reg.output_pin_configuration.int2_ocfg = config->int2_output_config;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_OUTPUT_PIN_CONFIGURATION, g_max86176_reg.byte));

  return BS_OK;
}

static base_status_t max86176_fifo_config(max86176_t *me, const max86176_fifo_config_t *config)
{
  g_max86176_reg.fifo_configuration_1.fifo_a_full = MAX86176_FIFO_CNT_MAX - config->fifo_threshold;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_FIFO_CONFIGURATION_1, g_max86176_reg.byte));

  g_max86176_reg.fifo_configuration_2.fifo_stat_clr = config->fifo_status_clear;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_FIFO_CONFIGURATION_2, g_max86176_reg.byte));

  return BS_OK;
}

static base_status_t max86176_ppg_measure_config(max86176_t *me, const max86176_ppg_measure_config_t *config)
{
#define NEXT_REG_ADDR (i * 8)

  for (uint8_t i = 0; i < MAX86176_PPG_MEAS_MAX; i++)
  {
    g_max86176_reg.meas_x_selects.meas_x_amb  = config[i].ambient;
    g_max86176_reg.meas_x_selects.meas_x_drva = config[i].led_drv_a;
    g_max86176_reg.meas_x_selects.meas_x_drvb = config[i].led_drv_b;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_SELECTS + NEXT_REG_ADDR, g_max86176_reg.byte));

    g_max86176_reg.meas_x_configuration_1.meas_x_aver = config[i].num_led_pulses;
    g_max86176_reg.meas_x_configuration_1.meas_x_tint = config[i].meas_integration;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_CONFIGURATION_1 + NEXT_REG_ADDR, g_max86176_reg.byte));

    g_max86176_reg.meas_x_configuration_2.meas_x_ppg1_adc_rge = config[i].ppg1_range;
    g_max86176_reg.meas_x_configuration_2.meas_x_ppg2_adc_rge = config[i].ppg1_range;
    g_max86176_reg.meas_x_configuration_2.meas_x_led_rge      = config[i].led_range;
    g_max86176_reg.meas_x_configuration_2.meas_x_filt_sel     = config[i].filter_select;
    g_max86176_reg.meas_x_configuration_2.meas_x_sinc3_sel    = config[i].sinc3_select;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_CONFIGURATION_2 + NEXT_REG_ADDR, g_max86176_reg.byte));
    
    g_max86176_reg.meas_x_configuration_3.meas_x_ppg1_dacoff = config[i].ppg1_dac_offset;
    g_max86176_reg.meas_x_configuration_3.meas_x_ppg1_dacoff = config[i].ppg2_dac_offset;
    g_max86176_reg.meas_x_configuration_3.meas_x_led_setlng  = config[i].led_setlng_time;
    g_max86176_reg.meas_x_configuration_3.meas_x_pd_setlng   = config[i].pd_setlng_time;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_CONFIGURATION_3 + NEXT_REG_ADDR, g_max86176_reg.byte));

    g_max86176_reg.meas_x_configuration_4.meas_x_ppg1_pd_sel  = config[i].ppg1_pd_sel;
    g_max86176_reg.meas_x_configuration_4.meas_x_ppg2_pd_sel  = config[i].ppg2_pd_sel;
    g_max86176_reg.meas_x_configuration_4.meas_x_ppg_gain = config[i].ppg_gain;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_CONFIGURATION_4 + NEXT_REG_ADDR, g_max86176_reg.byte));
    
    g_max86176_reg.meas_x_leda_current.meas_x_drva_pa = config[i].led_a_current;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_LED_A_CURRENT + NEXT_REG_ADDR, g_max86176_reg.byte));
    
    g_max86176_reg.meas_x_ledb_current.meas_x_drvb_pa = config[i].led_b_current;
    CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_MEAS1_LED_B_CURRENT + NEXT_REG_ADDR, g_max86176_reg.byte));
  }

  return BS_OK;

#undef NEXT_REG_ADDR
}

static base_status_t max86176_pll_config(max86176_t *me, const max86176_pll_config_t *config)
{
  // PLL divider ----------------- {
  g_max86176_reg.pll_configuration_1.pll_en        = config->pll_enable;
  g_max86176_reg.pll_configuration_1.pll_lock_wndw = 0;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_PLL_CONFIGURATION_1, g_max86176_reg.byte));

  g_max86176_reg.pll_configuration_2.mdiv   = config->pll_mdiv;
  g_max86176_reg.pll_configuration_2.ndiv_8 = config->pll_ndiv >> 8;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_PLL_CONFIGURATION_2, g_max86176_reg.byte));

  g_max86176_reg.pll_configuration_3.ndiv_7_0 = config->pll_ndiv;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_PLL_CONFIGURATION_3, g_max86176_reg.byte));
  // }

  return BS_OK;
}

static base_status_t max86176_ecg_config(max86176_t *me, const max86176_ecg_config_t *config)
{
  g_max86176_reg.ecg_configuration_1.ecg_en = config->enable;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_ECG_CONFIGURATION_1, g_max86176_reg.byte));

  return BS_OK;
}

static base_status_t max86176_interrupt_config(max86176_t *me, const max86176_interrupt_config_t *config)
{
  // Interrupt 1
  g_max86176_reg.interrupt_x_enable_1 = config[0].intr_en_1;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_1_ENABLE_1, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_2 = config[0].intr_en_2;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_1_ENABLE_2, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_3 = config[0].intr_en_3;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_1_ENABLE_3, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_4 = config[0].intr_en_4;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_1_ENABLE_4, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_5 = config[0].intr_en_5;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_1_ENABLE_5, g_max86176_reg.byte));

  // Interrupt 2
  g_max86176_reg.interrupt_x_enable_1 = config[1].intr_en_1;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_2_ENABLE_1, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_2 = config[1].intr_en_2;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_2_ENABLE_2, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_3 = config[1].intr_en_3;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_2_ENABLE_3, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_4 = config[1].intr_en_4;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_2_ENABLE_4, g_max86176_reg.byte));

  g_max86176_reg.interrupt_x_enable_5 = config[1].intr_en_5;
  CHECK_STATUS(max86176_write_reg(me, MAX86176_REG_INTERRUPT_2_ENABLE_5, g_max86176_reg.byte));

  return BS_OK;
}

/**
 * @brief         MAX86176 write register
 * 
 * @param[in]     me          Pointer to handler of MAX86176 struct
 * @param[in]     reg         Register address
 * @param[in]     data        Data to write
 *
 * @return        base_status_t
 */
static base_status_t max86176_write_reg(max86176_t *me, uint8_t reg, uint8_t data)
{
  uint8_t buf_send[3] ={reg, MAX86176_WRITE_CMD, data};

  MAX86176_SPI_SELECT();
  CHECK_STATUS(me->spi_transmit(buf_send, 3));
  MAX86176_SPI_DESELECT();

  g_max86176_reg.byte = 0;

  return BS_OK;
}

/**
 * @brief         MAX86176 read register
 * 
 * @param[in]     me          Pointer to handler of MAX86176 struct
 * @param[in]     reg         Register address
 * @param[in]     p_data      Pointer to read data
 *
 * @return        base_status_t
 */
static base_status_t max86176_read_reg(max86176_t *me, uint8_t reg, uint8_t *p_data)
{
  uint8_t buf_send[3] ={reg, MAX86176_READ_CMD, 0xFF};
  uint8_t buf_receive[3];

  MAX86176_SPI_SELECT();
  CHECK_STATUS(me->spi_transmit_receive(buf_send, buf_receive, 3));
  MAX86176_SPI_DESELECT();

  *p_data = buf_receive[2];

  return BS_OK;
}

/**
 * @brief         MAX86176 read burst
 *                Each FIFO sample consists of 3 bytes per sample
 * 
 * @param[in]     me            Pointer to handler of MAX86176 struct
 * @param[in]     reg           Register address
 * @param[in]     p_data        Pointer to read data
 * @param[in]     num_samples   Number of sample (24 bit per sample)
 *
 * @return        base_status_t
 */
static base_status_t max86176_read_burst(max86176_t *me, uint8_t reg, uint32_t *p_data, uint16_t num_samples)
{
  uint8_t data_send[2] = {reg, MAX86176_READ_CMD};
  uint8_t data_read[MAX86176_FIFO_CNT_MAX * 3] = {0};
  uint16_t len = num_samples * 3;
  uint16_t count = 0;

  MAX86176_SPI_SELECT();
  CHECK_STATUS(me->spi_transmit(data_send, 2));
  CHECK_STATUS(me->spi_transmit_receive(data_read, data_read, len));
  MAX86176_SPI_DESELECT();

  for (int i = 0; i < num_samples; i++)
  {
    p_data[i] = 0;
    p_data[i] |= ((uint32_t)data_read[count] << 16);
    p_data[i] |= ((uint32_t)data_read[count + 1] << 8);
    p_data[i] |= ((uint32_t)data_read[count + 2]);
    count += 3;
  }

  return BS_OK;
}

/* End of file -------------------------------------------------------------- */
