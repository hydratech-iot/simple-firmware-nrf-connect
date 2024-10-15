/*
* File Name: ssd1306.h
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


/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ----------------------------------------------------------------- */
#include "font.h"
#include "bsp_oled.h"

/* Public defines ----------------------------------------------------------- */
/* All following bytes will contain commands */
#define SSD1306_CONTROL_ALL_BYTES_CMD       (0x00)
/* All following bytes will contain data */
#define SSD1306_CONTROL_ALL_BYTES_DATA      (0x40)
/* The next byte will contain a command */
#define SSD1306_CONTROL_BYTE_CMD            (0x80)
/* The next byte will contain data */
#define SSD1306_CONTROL_BYTE_DATA           (0xC0)

/**
* @brief  ssd1306 Size
*/
#define SSD1306_LCD_PIXEL_WIDTH                 ((uint16_t)128)
#define SSD1306_LCD_PIXEL_HEIGHT                ((uint16_t)64)

#define SSD1306_LCD_COLUMN_NUMBER               ((uint16_t)128)
#define SSD1306_LCD_PAGE_NUMBER                 ((uint16_t)8)

/**
* @brief  SSD1306 Scrolling
*/
#define SSD1306_CMD_ACTIVATE_SCROLL             (0x2F)
#define SSD1306_CMD_DEACTIVATE_SCROLL           (0x2E)

#define SSD1306_CMD_SCROLL_FREQ_2FRAMES         (0x07)
#define SSD1306_CMD_SCROLL_FREQ_3FRAMES         (0x04)
#define SSD1306_CMD_SCROLL_FREQ_4FRAMES         (0x05)
#define SSD1306_CMD_SCROLL_FREQ_5FRAMES         (0x00)
#define SSD1306_CMD_SCROLL_FREQ_25FRAMES        (0x06)
#define SSD1306_CMD_SCROLL_FREQ_64FRAMES        (0x01)
#define SSD1306_CMD_SCROLL_FREQ_128FRAMES       (0x02)
#define SSD1306_CMD_SCROLL_FREQ_256FRAMES       (0x03)

/**
 * @brief  SSD1306 Commands
 */
#define SSD1306_CMD_CHARGE_PUMP_SETTING             (0x8D)
#define SSD1306_CMD_READWRITE_CMD                   (0x80)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_1      (0x10)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_2      (0x11)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_3      (0x12)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_4      (0x13)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_5      (0x14)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_6      (0x15)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_7      (0x16)
#define SSD1306_CMD_HIGHER_COLUMN_START_ADDR_8      (0x17)
#define SSD1306_CMD_LOWER_COLUMN_START_ADDR         (0x00)
#define SSD1306_CMD_LOWER_COLUMN_START_ADDR_15      (0x0F)
#define SSD1306_CMD_MEMORY_ADDR_MODE                (0x20)
#define SSD1306_CMD_SET_COLUMN_ADDR                 (0x21)
#define SSD1306_CMD_SET_PAGE_ADDR                   (0x22)
#define SSD1306_CMD_DISPLAY_START_LINE_1            (0x40)
#define SSD1306_CMD_DISPLAY_START_LINE_32           (0x1F)
#define SSD1306_CMD_DISPLAY_START_LINE_64           (0x7F)
#define SSD1306_CMD_REMAPPED_MODE                   (0xC8)
#define SSD1306_CMD_SEG_REMAP                       (0xA0)
#define SSD1306_CMD_CONTRAST_CONTROL                (0xA1)
#define SSD1306_CMD_CONTRAST_CONTROL_2              (0xFF)
#define SSD1306_CMD_DISPLAY_ON                      (0xAF)
#define SSD1306_CMD_DISPLAY_OFF                     (0xAE)
#define SSD1306_CMD_SET_PAGE_START_ADDR             (0xB0)
#define SSD1306_CMD_DISPLAY_NORMAL                  (0xA6)  // Normal display, 0 in RAM: OFF in display panel, 1 in RAM: ON in display panel
#define SSD1306_CMD_DISPLAY_INVERSE                 (0xA7)  // Inverse display, 0 in RAM: ON in display panel, 1 in RAM: OFF in display panel

#define SSD1306_I2C_ADDR                            (0x3C)

#define SSD1306_X_OFFSET_LOWER                      (0x00)
#define SSD1306_X_OFFSET_UPPER                      (0x00)

/* Public enumerate/structure ----------------------------------------------- */
/**
 * @brief SSD1306 scroll mode enum
 */
typedef enum
{
    SSD1306_SCROLL_RIGHT = 0x26,
    SSD1306_SCROLL_LEFT = 0x27
}
ssd1306_scroll_mode_t;

/**
 * @brief  SSD1306 Colors
 */
typedef enum
{
    SSD1306_COLOR_WHITE = 0xFF,
    SSD1306_COLOR_BLACK = 0x00
}
ssd1306_color_t;

/**
 * @brief SSD1306 struct
 */
typedef struct
{
    uint8_t device_address; // I2C device address
    uint16_t current_x;
    uint16_t current_y;

    // Write n-bytes from device's internal address <reg_addr> via I2C bus
    base_status_t (*i2c_write)(uint8_t dev_addr, uint8_t reg, uint8_t *p_data, uint16_t len);
    // Write n-bytes from device's internal address <reg_addr> via I2C bus
    base_status_t (*i2c_mul_write)(uint8_t dev_addr, uint8_t start_addr, uint8_t *w_data, uint16_t len);
}
ssd1306_t;

/* Public macros ------------------------------------------------------------ */
/* Public variables --------------------------------------------------------- */
extern ssd1306_t ssd1306;

/* Public function prototypes ----------------------------------------------- */
base_status_t ssd1306_init(ssd1306_t *me);
base_status_t ssd1306_deinit(ssd1306_t *me);
base_status_t ssd1306_update_screen(ssd1306_t *me);
base_status_t ssd1306_display_on(ssd1306_t *me);
base_status_t ssd1306_display_off(ssd1306_t *me);
base_status_t ssd1306_clear(ssd1306_t *me, ssd1306_color_t color_code);
base_status_t ssd1306_set_cursor(ssd1306_t *me, uint8_t x, uint8_t y);
base_status_t ssd1306_refresh(ssd1306_t *me);
base_status_t ssd1306_draw_image(ssd1306_t *me,uint16_t x_pos, uint16_t y_pos, uint16_t w, uint16_t h, const uint8_t bitmap[], uint16_t color);
base_status_t ssd1306_write_pixel(ssd1306_t *me, uint16_t x_pos, uint16_t y_pos, ssd1306_color_t color_code);
char          ssd1306_write_char(ssd1306_t *me, char ch, font_def font, ssd1306_color_t color_code);
char          ssd1306_write_string(ssd1306_t *me, char* str, font_def font, ssd1306_color_t color);
base_status_t ssd1306_draw_line(ssd1306_t *me, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color);
base_status_t ssd1306_draw_rectangle(ssd1306_t *me, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color_code);
base_status_t ssd1306_fill_rectangle(ssd1306_t *me, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color);
base_status_t ssd1306_set_page(ssd1306_t *me, uint16_t page);
base_status_t ssd1306_set_column(ssd1306_t *me, uint16_t column);
base_status_t ssd1306_draw_hline(ssd1306_t *me, ssd1306_color_t rgb_code, uint16_t x_pos, uint16_t y_pos, uint16_t length);
base_status_t ssd1306_draw_vline(ssd1306_t *me, ssd1306_color_t rgb_code, uint16_t x_pos, uint16_t y_pos, uint16_t length);
base_status_t ssd1306_scrolling_setup(ssd1306_t *me, ssd1306_scroll_mode_t scroll_mode, uint16_t start_page, uint16_t end_page, uint16_t frequency);
base_status_t ssd1306_scrolling_start(ssd1306_t *me);
base_status_t ssd1306_scrolling_stop(ssd1306_t *me);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} /* extern "C" { */
#endif

/* End of file -------------------------------------------------------------- */