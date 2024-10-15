/*
* File Name: bsp_pwm.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: 
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
     PWM_CHANNEL_1 = 0
    ,PWM_CHANNEL_2
    ,PWM_CHANNEL_3
}
pwm_channel_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t bsp_pwm_init(void);
base_status_t bsp_pwm_set_percent(pwm_channel_t channel, uint8_t percent);

/* End of file -------------------------------------------------------- */
