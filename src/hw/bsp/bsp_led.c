/*
 * File Name: bsp_led.c
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

/* Includes ----------------------------------------------------------- */
#include "bsp_led.h"
#include "bsp_pwm.h"
// #include "bsp_timer.h"
#include "bsp_timer.h"

/* Private defines ---------------------------------------------------- */
#define LED_FLASH_NORMAL_INTERVAL   (500)
#define LED_FLASH_FAST_INTERVAL     (150)
#define LED_PULSE_INTERVAL          (50)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_info_t;

#define COLOR_LED_INFO(c, r, g, b)[c] = {.red = r, .green = g, .blue = b}
const color_info_t COLOR_LIST[LED_COLOR_UNKNOWN] = 
{
   //               +===================+=====+=======+======+
   //               |COLOR              | Red | Green | Blue |
   //               +-------------------+-----+-------+------+
      COLOR_LED_INFO(LED_COLOR_GREEN    ,    0,    255,     0)
     ,COLOR_LED_INFO(LED_COLOR_BLUE     ,    0,      0,   255)
     ,COLOR_LED_INFO(LED_COLOR_YELLOW   ,  255,    100,     0)
     ,COLOR_LED_INFO(LED_COLOR_CYAN     ,    0,    255,   255)
     ,COLOR_LED_INFO(LED_COLOR_WHITE    ,  255,    255,   255)
     ,COLOR_LED_INFO(LED_COLOR_RED      ,  255,      0,     0)
     ,COLOR_LED_INFO(LED_COLOR_PINK     ,  255,     51,   255)
   //               +===================+=====+=======+======+
};

/* Private macros ----------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static led_mode_t current_mode    = LED_MODE_OFF;
static led_color_t current_color  = LED_COLOR_UNKNOWN;
static uint8_t current_brightness = 0;
static bool pulse_increasing      = true;
static uint8_t pulse_brightness   = 0;
static bool led_state             = false;
static uint32_t flash_interval    = 0;

static tmr_t bsp_led_timer;

/* Private function prototypes ---------------------------------------- */
static void bsp_led_set_color(led_color_t color, uint8_t brightness);

/* Function definitions ----------------------------------------------- */
void bsp_led_init(void)
{
    bsp_pwm_init();

    bsp_tmr_start(&bsp_led_timer, 1000);
}

void bsp_led_set_led(led_mode_t mode, led_color_t color, uint8_t brightness)
{
    current_mode = mode;
    current_color = color;
    current_brightness = brightness;
}

// Function to update the LED state based on the mode
void bsp_led_update_led_state(void)
{
    if ((current_mode == LED_MODE_FLASH) || (current_mode == LED_MODE_FLASH_FAST))
    {
        if (current_mode == LED_MODE_FLASH)
        {
            flash_interval = LED_FLASH_NORMAL_INTERVAL;
        }
        else if ((current_mode == LED_MODE_FLASH_FAST))
        {
            flash_interval = LED_FLASH_FAST_INTERVAL;
        }
    
        if (bsp_tmr_is_expired(&bsp_led_timer))
        {
            led_state = !led_state;
            if (led_state)
            {
                bsp_led_set_color(current_color, current_brightness);
            }
            else
            {
                bsp_led_set_color(current_color, 0);
            }
            bsp_tmr_restart(&bsp_led_timer, flash_interval);
        }
    }
    else if (current_mode == LED_MODE_PULSE)
    {
        if (bsp_tmr_is_expired(&bsp_led_timer))
        {
            if (pulse_increasing)
            {
                pulse_brightness++;
                if (pulse_brightness >= current_brightness)
                {
                    pulse_increasing = false;
                }
            }
            else
            {
                pulse_brightness--;
                if (pulse_brightness == 0)
                {
                    pulse_increasing = true;
                }
            }
            bsp_led_set_color(current_color, pulse_brightness);
            bsp_tmr_restart(&bsp_led_timer, LED_PULSE_INTERVAL);
        }
    }
    else if (current_mode == LED_MODE_FLASH_ONE_EVERY_3_SECONDS)
    {
        if (bsp_tmr_is_expired(&bsp_led_timer))
        {
            led_state = !led_state;
            if (led_state)
            {
                bsp_led_set_color(current_color, current_brightness);
                bsp_tmr_restart(&bsp_led_timer, 500);
            }
            else
            {
                bsp_led_set_color(current_color, 0);
                bsp_tmr_restart(&bsp_led_timer, 3000);
            }
        }
    }
    else if (current_mode == LED_MODE_ON)
    {
        bsp_led_set_color(current_color, current_brightness);
    }
    else if (current_mode == LED_MODE_OFF)
    {
        bsp_led_set_color(current_color, 0);
    }
}

/* Private function definitions ---------------------------------------- */
static void bsp_led_set_color(led_color_t color, uint8_t brightness)
{
    color_info_t color_info;

    color_info       = COLOR_LIST[color];
    color_info.red   = (uint8_t)(((color_info.red   / (255 / 100)) * brightness) / 100);
    color_info.green = (uint8_t)(((color_info.green / (255 / 100)) * brightness) / 100);
    color_info.blue  = (uint8_t)(((color_info.blue  / (255 / 100)) * brightness) / 100);

    // Set current for the LED pin
    bsp_pwm_set_percent(PWM_CHANNEL_1, color_info.red);
    bsp_pwm_set_percent(PWM_CHANNEL_2, color_info.green);
    bsp_pwm_set_percent(PWM_CHANNEL_3, color_info.blue);
}

/* End of file -------------------------------------------------------- */
