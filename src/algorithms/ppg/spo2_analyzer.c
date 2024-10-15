/**
 * @file       spo2_analyzer.c
 * @brief      SPO2 analyzer
 * @note       None
 * @example    None
 */

 /* Includes ----------------------------------------------------------- */
#include "spo2_analyzer.h"
#include "bsp_afe.h"
#include "ppg_data.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private Constants -------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static measured_values_t measured_value;
// float resample_signal[LEN_SIG_128HZ];

/* Private function prototypes ---------------------------------------- */
/* Function definitions ------------------------------------------------ */
/* Private function definitions ---------------------------------------- */
void spo2_analyzer_runner(int32_t *ppg_signal, size_t len, uint8_t *p_spo2)
{
    //for (size_t i = 0; i < len; i += AFE4950_PHASE_NUM)
    //{
    //    NRF_LOG_DEBUG("ppg_signal[red]: %d", ppg_signal[i + 1]);
    //    NRF_LOG_DEBUG("ppg_signal[ir]: %d", ppg_signal[i + 2]);

    //    //  Using PD2: LED-RED and LED IR
    //    ppg_data_process(ppg_signal[i + 4], PPG_LED_CHANNEL_RED, resample_signal, &measured_value);
    //    ppg_data_process(ppg_signal[i + 5], PPG_LED_CHANNEL_IR, resample_signal, &measured_value);

    //    if (measured_value.spo2 == -1)
    //    {
    //        *p_spo2 = 0;
    //    }
    //    else
    //    {
    //        *p_spo2 = (uint8_t)measured_value.spo2;
    //    }
    //}
}

/* End of file --------------------------------------------------------- */
