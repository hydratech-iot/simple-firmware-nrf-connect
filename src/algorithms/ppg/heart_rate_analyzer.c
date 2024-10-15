/**
 * @file       heart_rate_analyzer.c
 * @brief      Heart rate analyzer
 * @note       None
 * @example    None
 */

 /* Includes ----------------------------------------------------------- */
#include <stdbool.h>
#include "heart_rate_analyzer.h"
#include "bsp_afe.h"

/* Private defines ---------------------------------------------------- */
#define HEART_RATE_ANALYZER_WINDOW_SIZE (10 * (AFE_SAMPLING_RATE)) // 10s

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private Constants -------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static int peaks[2000] = { 0 };
static float ibi[2000] = { 0 };

/* Private function prototypes ---------------------------------------- */
static void heart_rate_calculate(float *ppg_signal, int signal_length, int sampling_rate, int *peaks, float *heart_rate);
static bool heart_rate_detect_object(float *ppg_signal_window, uint32_t len);

/* Function definitions ----------------------------------------------- */
bool heart_rate_analyzer_runner(int32_t* ppg_signal_green, size_t len, uint8_t* p_heart_rate)
{
    static float ppg_signal_window[HEART_RATE_ANALYZER_WINDOW_SIZE] = { 0 };
    static int ppg_signal_window_index = 0;
    float heart_rate;

    for (size_t i = 0; i < len; i++)
    {
       // Get ppg green, pd1
       ppg_signal_window[ppg_signal_window_index++] = ppg_signal_green[i];

       if (ppg_signal_window_index >= HEART_RATE_ANALYZER_WINDOW_SIZE) 
       {
           heart_rate_calculate(ppg_signal_window, HEART_RATE_ANALYZER_WINDOW_SIZE, AFE_SAMPLING_RATE, peaks, &heart_rate);

           *p_heart_rate = (uint8_t)heart_rate;
           if (!heart_rate_detect_object(ppg_signal_window, HEART_RATE_ANALYZER_WINDOW_SIZE))
           {
                *p_heart_rate = 0;
           }

           memset(ppg_signal_window, 0, sizeof(ppg_signal_window));
           ppg_signal_window_index = 0;

           return true;
       }
    }

    return false;
}

/* Private function definitions --------------------------------------- */
static bool heart_rate_detect_object(float *ppg_signal_window, uint32_t len)
{
    uint16_t object_not_available_cnt = 0;

    for (size_t i = 0; i < len; i++)
    {
        if (ppg_signal_window[i] < 100000)
        {
            object_not_available_cnt++;
        }
    }

    if (object_not_available_cnt > 10)
    {
        return false;
    }
    else
    {
        return true;
    }
}

static void heart_rate_calculate(float *ppg_signal, int signal_length, int sampling_rate, int *peaks, float *heart_rate)
{
    // Step 2: Preprocess the signal (You may need more advanced preprocessing depending on your data)
    // TODO: Preprocessing the signal, filter [0.5:5] Hz

    // Step 3: Peak Detection
    int peak_count = 0;
    int distance = 0;
    for (int i = 1; i < signal_length - 1; i++) 
    {
       if (i < sampling_rate) 
       {
           distance = sampling_rate / 2.5;
           i += distance - 1;
       }
       else
       {
           distance = sampling_rate / 1.5;
       }

       if (ppg_signal[i] > ppg_signal[i - 1] && ppg_signal[i] > ppg_signal[i + 1] &&
           ppg_signal[i] > 0.5) 
       {
           // TODO: Make sure that peak_count value no more than length of peaks
           peaks[peak_count++] = i;

           // Skip peaks within the specified distance
           i += distance - 1;
       }
    }

    // Step 4: Calculate Inter-Beat Intervals (IBIs)
    for (int i = 1; i < peak_count; i++) 
    {
       ibi[i - 1] = (float)(peaks[i] - peaks[i - 1]) / sampling_rate;
    }

    // Step 5: Calculate Heart Rate
    float ibi_sum = 0;
    for (int i = 0; i < peak_count - 1; i++)
    {
       ibi_sum += ibi[i];
    }

    if (ibi_sum == 0 || peak_count == 1)
    {
       *heart_rate = 0;
       return;
    }

    *heart_rate = 60 / (ibi_sum / (peak_count - 1));
}

/* End of file --------------------------------------------------------- */
