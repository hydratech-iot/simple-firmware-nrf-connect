/*
* File Name: font.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Driver for OLED
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __FONT_H
#define __FONT_H

/* Includes ----------------------------------------------------------- */
#include "base_type.h"

/* Public defines ----------------------------------------------------- */
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26
#define SSD1306_INCLUDE_FONT_16x24

/* Public enumerate/structure ----------------------------------------- */
typedef struct 
{
  const uint8_t font_width;   /*!< Font width in pixels */
  uint8_t font_height;        /*!< Font height in pixels */
  const uint16_t *data;       /*!< Pointer to data font data array */
} 
font_def;

/* Public macros ------------------------------------------------------ */
#ifdef SSD1306_INCLUDE_FONT_6x8
extern font_def ssd1306_font_6x8;
#endif
#ifdef SSD1306_INCLUDE_FONT_7x10
extern font_def ssd1306_font_7x10;
#endif
#ifdef SSD1306_INCLUDE_FONT_11x18
extern font_def ssd13315_font_11x18;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x26
extern font_def ssd1306_font_16x26;
#endif
#ifdef SSD1306_INCLUDE_FONT_16x24
extern font_def ssd1306_font_16x24;
#endif

/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */

#endif // __FONT_H

/* End of file -------------------------------------------------------- */
