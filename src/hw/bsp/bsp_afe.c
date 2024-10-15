/*
* File Name: bsp_afe.c
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

/* Public includes ---------------------------------------------------------- */
#include "bsp_afe.h"
#include "bsp_io.h"
#include "base_board_defs.h"

/* Private includes --------------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_afe, CONFIG_LOG_DEFAULT_LEVEL);

/* Private defines ---------------------------------------------------------- */
#define LED_INFO(_led, _measure) [_led] = { \
        .led     = _led,                    \
        .measure = _measure                 \
}

#define LED_CURRENT_INFO(_step, _min_val, _max_val, _step_val) [_step] = {  \
        .step     = _step,                                                  \
        .min_val  = _min_val,                                               \
        .max_val  = _max_val,                                               \
        .step_val = _step_val                                               \
}

/* Private enumerate/structure ---------------------------------------------- */
typedef struct
{
    uint8_t step;
    float min_val;
    float max_val;
    float step_val;
}
ppg_led_current_config_t;

typedef struct
{
    uint8_t led;
    uint8_t measure;
}
ppg_led_config_t;

/* Private macros ----------------------------------------------------------- */
/* Public variables --------------------------------------------------------- */
afe_sensor_setting_t g_afe_setting = {
    .ppg_sample_rate                       = 128.0f,
    .ppg_led_current_step[AFE_PPG_LED_RED] = AFE_PPG_LED_CURRENT_STEP_0_5,
    .ppg_led_current[AFE_PPG_LED_RED]      = 10.0f,
    .ppg_led_current_step[AFE_PPG_LED_IR]  = AFE_PPG_LED_CURRENT_STEP_0_5,
    .ppg_led_current[AFE_PPG_LED_IR]       = 10.0f
};

/* Private prototypes ------------------------------------------------------- */
static void bsp_afe_spi_select(void);
static void bsp_afe_spi_deselect(void);
static void bsp_afe_io_init(void);
static void bsp_afe_io_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* Private variables -------------------------------------------------------- */
static struct gpio_dt_spec io_ppg_int1 = IO_PPG_INT1;
static struct gpio_callback io_ppg_int1_cb_data;

static const ppg_led_current_config_t LED_CURRENT[AFE_PPG_LED_CURRENT_STEP_MAX] =
{
    //                +=================================+===========+============+========+
    //                | Current step                    | Min Value | Max Value  | Step   |
    //                |                                 |           |            |        |
    //                +---------------------------------+-----------+------------+--------+
     LED_CURRENT_INFO ( AFE_PPG_LED_CURRENT_STEP_0_125  , 0.0f      , 31.875f    , 0.125f )
    ,LED_CURRENT_INFO ( AFE_PPG_LED_CURRENT_STEP_0_25   , 0.0f      , 63.750f    , 0.25f  )
    ,LED_CURRENT_INFO ( AFE_PPG_LED_CURRENT_STEP_0_375  , 0.0f      , 95.625f    , 0.375f )
    ,LED_CURRENT_INFO ( AFE_PPG_LED_CURRENT_STEP_0_5    , 0.0f      , 127.5f     , 0.5f   )
    //                +=================================+===========+============+========+
};

static const ppg_led_config_t LED_CONFIG[AFE_PPG_LED_MAX] =
{
    //        +==================+================+
    //        | LED              | Measure        |
    //        |                  |                |
    //        +------------------+----------------+
     LED_INFO ( AFE_PPG_LED_RED  , AFE_PPG_MEAS1  )
    ,LED_INFO ( AFE_PPG_LED_IR   , AFE_PPG_MEAS2  )
    ,LED_INFO ( AFE_PPG_LED_GREEN, AFE_PPG_MEAS3  )
    //        +==================+================+
};

static max86176_t g_max86176 =
{
    .spi_transmit         = bsp_spi_2_transmit,
    .spi_transmit_receive = bsp_spi_2_transmit_receive,
    .spi_select           = bsp_afe_spi_select,
    .spi_deselect         = bsp_afe_spi_deselect
};

static const max86176_config_t AFE_CONFIG = 
{
  // System Configuration
    .system.config_1.reset       = false,
    .system.config_1.shutdown    = false,
    .system.config_1.ppg1_pwr_dn = true,
    .system.config_1.ppg2_pwr_dn = true,

    .system.config_2.meas1_en     = true,
    .system.config_2.meas2_en     = true,
    .system.config_2.meas3_en     = true,
    .system.config_2.meas4_en     = false,
    .system.config_2.meas5_en     = false,
    .system.config_2.meas6_en     = false,
    .system.config_2.meas7_en     = false,
    .system.config_2.meas8_en     = false,
    .system.config_3.meas9_en     = false,
    .system.config_4.prox_auto    = false,
    .system.config_4.prox_data_en = false,

    .system.pd1_bias = MAX86176_PPG_CAPACITANCE_0_125,
    .system.pd2_bias = MAX86176_PPG_CAPACITANCE_0_125,
    .system.pd3_bias = MAX86176_PPG_CAPACITANCE_0_125,
    .system.pd4_bias = MAX86176_PPG_CAPACITANCE_0_125,

    .system.int1_func_config   = MAX86176_INT_CONFIG_ENABLE_CLEARED_BY_READ_STATUS_FIFO,
    .system.int1_output_config = MAX86176_OUTPUT_CONFIG_ACTIVE_LOW,

    .system.int2_func_config   = MAX86176_INT_CONFIG_DISABLE,
    .system.int2_output_config = MAX86176_OUTPUT_CONFIG_DO_NOT_USE,

    // PLL Configuration
    // PLL_CLK = CLK_FREQ_SEL * MDIV = 32.768kHz * (255 + 1)  = 8.388608MHz
    // ECG_ADC_CLK = PLL_CLK / (ECG_FDIV * ECG_NDIV) = 8.388608MHz / (4 * 64) = 32.768kHz
    .pll.pll_enable   = false,
    .pll.pll_mdiv     = 255,
    .pll.pll_ndiv     = 255,

    // FIFO Configuration
    .fifo.fifo_threshold    = AFE_FIFO_THRESHOLD,
    .fifo.fifo_status_clear = MAX86176_FIFO_STAT_CLR_BY_STATUS_FIFO_REGISTER,

    // PPG Frame Rate Configuration
    // PPG_FR_CLK = CLK_FREQ_SEL = 32.768kHz
    // PPG Frame Rate = PPG_FR_CLK / FR_CLK_DIV = 32.768kHz / 256 = 128Hz
    .ppg_frame_rate.clk_select = MAX86176_CLOCK_FREQUENCY_32768,
    .ppg_frame_rate.clk_div    = 256,                              // 128Hz

    // PPG Measure Configuration ------------ {
    // LED Red
    .ppg_meas[MAX86176_PPG_MEAS1].ambient           = MAX86176_PPG_AMB_NORMAL_MODE,
    .ppg_meas[MAX86176_PPG_MEAS1].led_drv_a         = MAX86176_PPG_LED6_DRV,
    .ppg_meas[MAX86176_PPG_MEAS1].led_drv_b         = MAX86176_PPG_LED6_DRV,

    .ppg_meas[MAX86176_PPG_MEAS1].num_led_pulses    = MAX86176_PPG_LED_PULSES_1,
    .ppg_meas[MAX86176_PPG_MEAS1].meas_integration  = MAX86176_PPG_MEAS_INTEGRATION_117,

    .ppg_meas[MAX86176_PPG_MEAS1].ppg1_range        = MAX86176_PPG_ADC_RANGE_32,
    .ppg_meas[MAX86176_PPG_MEAS1].ppg1_range        = MAX86176_PPG_ADC_RANGE_32,
    .ppg_meas[MAX86176_PPG_MEAS1].led_range         = MAX86176_PPG_LED_DRIVE_RGE_128,
    .ppg_meas[MAX86176_PPG_MEAS1].filter_select     = MAX86176_PPG_FILTER_SELECT_CENTRAL,     // ALC method central
    .ppg_meas[MAX86176_PPG_MEAS1].sinc3_select      = MAX86176_PPG_SINC3_FILTER_IS_USED,

    .ppg_meas[MAX86176_PPG_MEAS1].ppg1_dac_offset   = 0,
    .ppg_meas[MAX86176_PPG_MEAS1].ppg2_dac_offset   = 0,
    .ppg_meas[MAX86176_PPG_MEAS1].pd_setlng_time    = MAX86176_PPG_SETLNG_TIME_24,
    .ppg_meas[MAX86176_PPG_MEAS1].led_setlng_time   = MAX86176_PPG_SETLNG_TIME_8,

    .ppg_meas[MAX86176_PPG_MEAS1].ppg1_pd_sel       = MAX86176_PPG_PD1_IS_INPUT,
    .ppg_meas[MAX86176_PPG_MEAS1].ppg2_pd_sel       = MAX86176_PPG_PD2_IS_INPUT,
    .ppg_meas[MAX86176_PPG_MEAS1].ppg_gain          = MAX86176_PPG_GAIN_1,

    .ppg_meas[MAX86176_PPG_MEAS1].led_a_current     = 0x00,
    .ppg_meas[MAX86176_PPG_MEAS1].led_b_current     = 0x00,

    // LED IR
    .ppg_meas[MAX86176_PPG_MEAS2].ambient           = MAX86176_PPG_AMB_NORMAL_MODE,
    .ppg_meas[MAX86176_PPG_MEAS2].led_drv_a         = MAX86176_PPG_LED2_DRV,
    .ppg_meas[MAX86176_PPG_MEAS2].led_drv_b         = MAX86176_PPG_LED2_DRV,

    .ppg_meas[MAX86176_PPG_MEAS2].num_led_pulses    = MAX86176_PPG_LED_PULSES_1,
    .ppg_meas[MAX86176_PPG_MEAS2].meas_integration  = MAX86176_PPG_MEAS_INTEGRATION_117,

    .ppg_meas[MAX86176_PPG_MEAS2].ppg1_range        = MAX86176_PPG_ADC_RANGE_32,
    .ppg_meas[MAX86176_PPG_MEAS2].ppg1_range        = MAX86176_PPG_ADC_RANGE_32,
    .ppg_meas[MAX86176_PPG_MEAS2].led_range         = MAX86176_PPG_LED_DRIVE_RGE_128,
    .ppg_meas[MAX86176_PPG_MEAS2].filter_select     = MAX86176_PPG_FILTER_SELECT_CENTRAL,     // ALC method central
    .ppg_meas[MAX86176_PPG_MEAS2].sinc3_select      = MAX86176_PPG_SINC3_FILTER_IS_USED,

    .ppg_meas[MAX86176_PPG_MEAS2].ppg1_dac_offset   = 0,
    .ppg_meas[MAX86176_PPG_MEAS2].ppg2_dac_offset   = 0,
    .ppg_meas[MAX86176_PPG_MEAS2].pd_setlng_time    = MAX86176_PPG_SETLNG_TIME_24,
    .ppg_meas[MAX86176_PPG_MEAS2].led_setlng_time   = MAX86176_PPG_SETLNG_TIME_8,

    .ppg_meas[MAX86176_PPG_MEAS2].ppg1_pd_sel       = MAX86176_PPG_PD1_IS_INPUT,
    .ppg_meas[MAX86176_PPG_MEAS2].ppg2_pd_sel       = MAX86176_PPG_PD2_IS_INPUT,
    .ppg_meas[MAX86176_PPG_MEAS2].ppg_gain          = MAX86176_PPG_GAIN_1,

    .ppg_meas[MAX86176_PPG_MEAS2].led_a_current     = 0x00,
    .ppg_meas[MAX86176_PPG_MEAS2].led_b_current     = 0x00,

    // LED GREEN
    .ppg_meas[MAX86176_PPG_MEAS3].ambient           = MAX86176_PPG_AMB_NORMAL_MODE,
    .ppg_meas[MAX86176_PPG_MEAS3].led_drv_a         = MAX86176_PPG_LED4_DRV,
    .ppg_meas[MAX86176_PPG_MEAS3].led_drv_b         = MAX86176_PPG_LED4_DRV,

    .ppg_meas[MAX86176_PPG_MEAS3].num_led_pulses    = MAX86176_PPG_LED_PULSES_1,
    .ppg_meas[MAX86176_PPG_MEAS3].meas_integration  = MAX86176_PPG_MEAS_INTEGRATION_117,

    .ppg_meas[MAX86176_PPG_MEAS3].ppg1_range        = MAX86176_PPG_ADC_RANGE_32,
    .ppg_meas[MAX86176_PPG_MEAS3].ppg2_range        = MAX86176_PPG_ADC_RANGE_32,
    .ppg_meas[MAX86176_PPG_MEAS3].led_range         = MAX86176_PPG_LED_DRIVE_RGE_128,
    .ppg_meas[MAX86176_PPG_MEAS3].filter_select     = MAX86176_PPG_FILTER_SELECT_CENTRAL,     // ALC method central
    .ppg_meas[MAX86176_PPG_MEAS3].sinc3_select      = MAX86176_PPG_SINC3_FILTER_IS_USED,

    .ppg_meas[MAX86176_PPG_MEAS3].ppg1_dac_offset   = 0,
    .ppg_meas[MAX86176_PPG_MEAS3].ppg2_dac_offset   = 0,
    .ppg_meas[MAX86176_PPG_MEAS3].pd_setlng_time    = MAX86176_PPG_SETLNG_TIME_24,
    .ppg_meas[MAX86176_PPG_MEAS3].led_setlng_time   = MAX86176_PPG_SETLNG_TIME_24,

    .ppg_meas[MAX86176_PPG_MEAS3].ppg1_pd_sel       = MAX86176_PPG_PD2_IS_INPUT,
    .ppg_meas[MAX86176_PPG_MEAS3].ppg2_pd_sel       = MAX86176_PPG_PD2_IS_INPUT,
    .ppg_meas[MAX86176_PPG_MEAS3].ppg_gain          = MAX86176_PPG_GAIN_1,

    .ppg_meas[MAX86176_PPG_MEAS3].led_a_current     = 0x00,
    .ppg_meas[MAX86176_PPG_MEAS3].led_b_current     = 0x00,
    // ------------------------------ }

    // ECG Configuration (No need ECG in the Application --> Turn Off)
    .ecg.enable = false,

    // Interrupt Configuration
    .interrupt[MAX86176_INTR_1].intr_en_1.a_full_en = true
};

static afe_interrupt_callback_t g_afe_callback = NULL;

/* Public implementations --------------------------------------------------- */
base_status_t bsp_afe_init(afe_interrupt_callback_t callback)
{
    g_afe_callback = callback;
    bsp_afe_io_init();

    if (max86176_init(&g_max86176, &AFE_CONFIG) != BS_OK)
    {
        LOG_ERR("AFE sensor (MAX86176) init failed");
        return BS_ERROR;
    }

    CHECK_STATUS(bsp_afe_ppg_set_current(AFE_PPG_LED_RED, AFE_PPG_LED_CURRENT_STEP_0_5, 3.0f));     // 4 mA
    CHECK_STATUS(bsp_afe_ppg_set_current(AFE_PPG_LED_IR, AFE_PPG_LED_CURRENT_STEP_0_5, 3.0f));      // 4 mA
    CHECK_STATUS(bsp_afe_ppg_set_current(AFE_PPG_LED_GREEN, AFE_PPG_LED_CURRENT_STEP_0_5, 3.0f));   // 2 mA
    CHECK_STATUS(bsp_afe_ppg_set_sample_rate(AFE_SAMPLING_RATE));                                              // Hz
    LOG_INF("AFE sensor (MAX86176) init success");
    return BS_OK;
}

base_status_t bsp_afe_deinit(void)
{
    CHECK_STATUS(bsp_afe_ppg_enable(AFE_PPG_CHANNEL_1, false));
    CHECK_STATUS(bsp_afe_ppg_enable(AFE_PPG_CHANNEL_2, false));
    CHECK_STATUS(bsp_afe_pll_enable(false));

    return max86176_set_software_power_down_mode(&g_max86176, true);
}

base_status_t bsp_afe_pll_enable(bool enable)
{
    return max86176_pll_enable(&g_max86176, enable);
}

base_status_t bsp_afe_ppg_enable(afe_ppg_channel_t channel, bool enable)
{
    return max86176_ppg_enable(&g_max86176, (max86176_ppg_channel_t)channel, enable);
}

base_status_t bsp_afe_read_data(uint32_t *p_data, uint16_t *num_samples)
{
    CHECK_STATUS(max86176_get_fifo_count(&g_max86176, num_samples));
    if (*num_samples > 0)
    {
        CHECK_STATUS(max86176_read_fifo(&g_max86176, p_data, *num_samples));
    }
    else
    {
        *num_samples = 0;
    }

    return BS_OK;
}

base_status_t bsp_afe_post(void)
{
    return max86176_check_device_id(&g_max86176);
}

base_status_t bsp_afe_bist(void)
{
    return bsp_afe_post();
}

base_status_t bsp_afe_ppg_set_current(afe_ppg_led_t led, afe_ppg_led_current_step_t current_step, float current)
{
    assert(current_step < AFE_PPG_LED_CURRENT_STEP_MAX);
    assert(led < AFE_PPG_LED_MAX);
    assert(current >= LED_CURRENT[current_step].min_val);
    assert(current <= LED_CURRENT[current_step].max_val);

    // Calculate register value in DEC
    uint8_t reg_value = (uint8_t)(current / LED_CURRENT[current_step].step_val);

    // PPG LED current range configuration
    CHECK_STATUS(max86176_ppg_set_led_rge(&g_max86176, LED_CONFIG[led].measure, (max86176_ppg_led_drive_rge_t)current_step));

    // PPG LED current configuration
    CHECK_STATUS(max86176_ppg_set_led_current(&g_max86176, LED_CONFIG[led].measure, MAX86176_PPG_LED_DRVA, reg_value));

    g_afe_setting.ppg_led_current_step[led] = current_step;
    g_afe_setting.ppg_led_current[led] = current;

    return BS_OK;
}

base_status_t bsp_afe_ppg_set_sample_rate(float sample_rate)
{
    assert(sample_rate >= 1.000061f);
    assert(sample_rate <= 2048.0f);

    max86176_ppg_frame_rate_config_t ppg_frame_rate;

    // PPG Frame Rate Configuration
    // PPG_FR_CLK = CLK_FREQ_SEL = 32.768kHz
    // PPG Frame Rate = PPG_FR_CLK / FR_CLK_DIV --> FR_CLK_DIV = 32.768 / PPG Frame Rate
    ppg_frame_rate.clk_div = (uint16_t)(32768.0 / sample_rate);

    CHECK_STATUS(max86176_ppg_frame_rate_config(&g_max86176, &ppg_frame_rate));

    g_afe_setting.ppg_sample_rate = sample_rate;

    return BS_OK;
}

/* Private implementations -------------------------------------------------- */
static void bsp_afe_io_init(void)
{
    // Interrupt
    bsp_io_attach_interrupt(io_ppg_int1, &io_ppg_int1_cb_data, bsp_afe_io_interrupt_callback, GPIO_INT_EDGE_FALLING);
}

static void bsp_afe_io_interrupt_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // printf("AFE Interrupt callback\n");
    g_afe_callback(IO_PIN_PPG_INT1);
}

static void bsp_afe_spi_select(void)
{
    bsp_io_write_specific_pin(IO_PIN_PPG_SPI_CS, 0);
}

static void bsp_afe_spi_deselect(void)
{
    bsp_io_write_specific_pin(IO_PIN_PPG_SPI_CS, 1);
}

/* End of file -------------------------------------------------------------- */
