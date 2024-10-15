/*
* File Name: ssd1306.c
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

/* Includes ----------------------------------------------------------- */
#include "ssd1306.h"

/* Private defines ---------------------------------------------------- */
#define SSD1306_GDDRAM_BUFFER (SSD1306_LCD_COLUMN_NUMBER * SSD1306_LCD_PAGE_NUMBER)
#define SSD1306_MAXIMUM_BUFFER (254)

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static uint8_t phys_frame_buffer[SSD1306_GDDRAM_BUFFER];

/* Private function prototypes ---------------------------------------- */
static base_status_t m_ssd1306_write_cmd(ssd1306_t *me, uint8_t reg, uint8_t p_data);
static base_status_t m_ssd1306_write_mul(ssd1306_t *me, uint8_t start_addr, uint8_t *p_data, uint16_t length);

/* Function definitions ----------------------------------------------- */
base_status_t ssd1306_init(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_OFF, 0xAE));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_READWRITE_CMD, 0x80));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_CHARGE_PUMP_SETTING, 0x8D));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_HIGHER_COLUMN_START_ADDR_5, 0x14));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_MEMORY_ADDR_MODE, 0x20));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_START_LINE_1, 0x40));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_REMAPPED_MODE, 0xC8));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_CONTRAST_CONTROL, 0xA1));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_ON, 0xAF));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SEG_REMAP, SSD1306_CMD_SEG_REMAP));
    CHECK_STATUS(ssd1306_clear(me, SSD1306_COLOR_BLACK));


    CHECK_STATUS(m_ssd1306_write_mul(me, SSD1306_CONTROL_ALL_BYTES_DATA, (uint8_t *)phys_frame_buffer, SSD1306_MAXIMUM_BUFFER));

    return BS_OK;
}

base_status_t ssd1306_update_screen(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    // Write data to each page of RAM. Number of pages
    for (uint8_t i = 0; i < (SSD1306_LCD_PIXEL_HEIGHT / 8); i++)
    {
        m_ssd1306_write_cmd(me, SSD1306_CMD_READWRITE_CMD, 0x00 + SSD1306_X_OFFSET_LOWER);
        m_ssd1306_write_cmd(me, SSD1306_CMD_READWRITE_CMD, 0x10 + SSD1306_X_OFFSET_UPPER);
        m_ssd1306_write_cmd(me, SSD1306_CMD_READWRITE_CMD, 0xB0 + i); // Set the current RAM page address.

        // Write the screen buffer with changed to the screen
        if (me->i2c_mul_write(me->device_address, SSD1306_CONTROL_ALL_BYTES_DATA, &phys_frame_buffer[SSD1306_LCD_PIXEL_WIDTH * i], SSD1306_LCD_PIXEL_WIDTH) != BS_OK)
        {
            return BS_ERROR;
        }
    }

    return BS_OK;
}

base_status_t ssd1306_deinit(ssd1306_t *me)
{
    if (ssd1306_display_off(me) != BS_OK)
        return BS_ERROR;

    return BS_OK;
}

base_status_t ssd1306_display_on(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_CHARGE_PUMP_SETTING, 0x8D));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_HIGHER_COLUMN_START_ADDR_5, 0x14));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_ON, 0xAF));

    return BS_OK;
}

base_status_t ssd1306_display_off(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_CHARGE_PUMP_SETTING, 0x8D));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_HIGHER_COLUMN_START_ADDR_1, 0x10));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_OFF, 0xAE));

    return BS_OK;
}

base_status_t ssd1306_clear(ssd1306_t *me, ssd1306_color_t color_code)
{
    assert(me != NULL);

    // Check color
    if (color_code == SSD1306_COLOR_WHITE)
    {
        memset(phys_frame_buffer, SSD1306_COLOR_WHITE, sizeof(phys_frame_buffer));
    }
    else
    {
        memset(phys_frame_buffer, SSD1306_COLOR_BLACK, sizeof(phys_frame_buffer));
    }

    return BS_OK;
}

base_status_t ssd1306_refresh(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_START_LINE_1, 0x40));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SET_COLUMN_ADDR, 0x21));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_START_LINE_64, 0x7F));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SET_PAGE_ADDR, 0x22));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR_15, 0x0F));

    // Fill Buffer in GDDRAM of LCD
    if (me->i2c_write(me->device_address, SSD1306_CMD_LOWER_COLUMN_START_ADDR, phys_frame_buffer, SSD1306_GDDRAM_BUFFER) != BS_OK)
    {
        return BS_ERROR;
    }

    return BS_OK;
}

base_status_t ssd1306_draw_image(ssd1306_t *me, uint16_t x_pos, uint16_t y_pos, uint16_t w, uint16_t h, const uint8_t bitmap[], uint16_t color)
{
    int16_t byte_width = (w + 7) / 8;
    uint8_t b = 0;

    for (int16_t j = 0; j < h; j++, y_pos++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7)
            {
                b <<= 1;
            }
            else
            {
                b = pgm_read_byte(&bitmap[j * byte_width + i / 8]);
            }

            if (b & 0x80)
            {
                ssd1306_write_pixel(me, x_pos + i, y_pos, color);
            }
        }
    }

    return BS_OK;
}

base_status_t ssd1306_write_pixel(ssd1306_t *me, uint16_t x_pos, uint16_t y_pos, ssd1306_color_t color_code)
{
    assert(me != NULL && me->i2c_write != NULL);

    if (x_pos >= SSD1306_LCD_PIXEL_WIDTH || y_pos >= SSD1306_LCD_PIXEL_HEIGHT)
    {
        // Don't write outside the buffer
        return BS_ERROR;
    }

    // Set color
    if (color_code == SSD1306_COLOR_WHITE)
    {
        phys_frame_buffer[x_pos + (y_pos / 8) * SSD1306_LCD_PIXEL_WIDTH] |= 1 << (y_pos & 7);
    }
    else
    {
        phys_frame_buffer[x_pos + (y_pos / 8) * SSD1306_LCD_PIXEL_WIDTH] &= ~(1 << (y_pos & 7));
    }

    return BS_OK;
}

char ssd1306_write_char(ssd1306_t *me, char ch, font_def font, ssd1306_color_t color_code)
{
    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;

    // Check remaining space on current line
    if (SSD1306_LCD_PIXEL_WIDTH < (me->current_x + font.font_width) ||
        SSD1306_LCD_PIXEL_HEIGHT < (me->current_y + font.font_height))
    {
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for (uint32_t i = 0; i < font.font_height; i++)
    {
        uint32_t b = font.data[(ch - 32) * font.font_height + i];

        for (uint32_t j = 0; j < font.font_width; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306_write_pixel(me, me->current_x + j, me->current_y + i, (ssd1306_color_t)color_code);
            }
            else
            {
                ssd1306_write_pixel(me, me->current_x + j, me->current_y + i, (ssd1306_color_t)!color_code);
            }
        }
    }

    // The current space is now taken
    me->current_x += font.font_width;

    return ch;
}

// Write full string to screen buffer
char ssd1306_write_string(ssd1306_t *me, char *str, font_def font, ssd1306_color_t color)
{
    while (*str)
    {
        if (ssd1306_write_char(me, *str, font, color) != *str)
        {
            // Char could not be written
            return *str;
        }
        str++;
    }

    // Everything ok
    return *str;
}

// Position the cursor
base_status_t ssd1306_set_cursor(ssd1306_t *me, uint8_t x, uint8_t y)
{
    me->current_x = x;
    me->current_y = y;

    return BS_OK;
}

base_status_t ssd1306_set_page(ssd1306_t *me, uint16_t page)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SET_PAGE_START_ADDR, 0xB0 | page));

    return BS_OK;
}

base_status_t ssd1306_set_column(ssd1306_t *me, uint16_t column)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00 | column));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DISPLAY_START_LINE_32, 0x1F));

    return BS_OK;
}

base_status_t ssd1306_draw_hline(ssd1306_t *me, ssd1306_color_t rgb_code, uint16_t x_pos, uint16_t y_pos, uint16_t length)
{
    assert(me != NULL);

    // Sent a complete horizontal line
    for (uint32_t i = x_pos; i < (x_pos + length); i++)
    {
        if (ssd1306_write_pixel(me, i, y_pos, rgb_code) != BS_OK)
            return BS_ERROR;
    }

    return BS_OK;
}

base_status_t ssd1306_draw_vline(ssd1306_t *me, ssd1306_color_t rgb_code, uint16_t x_pos, uint16_t y_pos, uint16_t length)
{
    assert(me != NULL);

    // Sent a complete horizontal line
    for (uint32_t i = y_pos; i < (y_pos + length); i++)
    {
        if (ssd1306_write_pixel(me, i, x_pos, rgb_code) != BS_OK)
        {
            return BS_ERROR;
        }
    }

    return BS_OK;
}

base_status_t ssd1306_scrolling_setup(ssd1306_t *me, ssd1306_scroll_mode_t scroll_mode, uint16_t start_page, uint16_t end_page, uint16_t frequency)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, 0x00, scroll_mode));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SCROLL_FREQ_5FRAMES, start_page));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SCROLL_FREQ_5FRAMES, frequency));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_SCROLL_FREQ_5FRAMES, end_page));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_LOWER_COLUMN_START_ADDR, 0x00));
    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_CONTRAST_CONTROL_2, 0xFF));

    return BS_OK;
}

base_status_t ssd1306_scrolling_start(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_ACTIVATE_SCROLL, 0x2F));

    return BS_OK;
}

base_status_t ssd1306_scrolling_stop(ssd1306_t *me)
{
    assert(me != NULL && me->i2c_write != NULL);

    CHECK_STATUS(m_ssd1306_write_cmd(me, SSD1306_CMD_DEACTIVATE_SCROLL, 0x2E));

    return BS_OK;
}

base_status_t ssd1306_draw_line(ssd1306_t *me, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color)
{
    int32_t delta_x = abs(x2 - x1);
    int32_t delta_y = abs(y2 - y1);
    int32_t sign_x = ((x1 < x2) ? 1 : -1);
    int32_t sign_y = ((y1 < y2) ? 1 : -1);
    int32_t error = delta_x - delta_y;
    int32_t error2;

    CHECK_STATUS(ssd1306_write_pixel(me, x2, y2, color));

    while ((x1 != x2) || (y1 != y2))
    {
        CHECK_STATUS(ssd1306_write_pixel(me, x1, y1, color));

        error2 = error * 2;

        if (error2 > -delta_y)
        {
            error -= delta_y;
            x1 += sign_x;
        }

        if (error2 < delta_x)
        {
            error += delta_x;
            y1 += sign_y;
        }
    }

    return BS_OK;
}

base_status_t ssd1306_draw_rectangle(ssd1306_t *me, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color_code)
{
    CHECK_STATUS(ssd1306_draw_line(me, x1, y1, x2, y1, color_code));
    CHECK_STATUS(ssd1306_draw_line(me, x2, y1, x2, y2, color_code));
    CHECK_STATUS(ssd1306_draw_line(me, x2, y2, x1, y2, color_code));
    CHECK_STATUS(ssd1306_draw_line(me, x1, y2, x1, y1, color_code));

    return BS_OK;
}

base_status_t ssd1306_fill_rectangle(ssd1306_t *me, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, ssd1306_color_t color)
{
    uint8_t x_start = ((x1 <= x2) ? x1 : x2);
    uint8_t x_end = ((x1 <= x2) ? x2 : x1);
    uint8_t y_start = ((y1 <= y2) ? y1 : y2);
    uint8_t y_end = ((y1 <= y2) ? y2 : y1);

    for (uint8_t y = y_start; (y <= y_end) && (y < SSD1306_LCD_PIXEL_HEIGHT); y++)
    {
        for (uint8_t x = x_start; (x <= x_end) && (x < SSD1306_LCD_PIXEL_WIDTH); x++)
        {
            CHECK_STATUS(ssd1306_write_pixel(me, x, y, color));
        }
    }

    return BS_OK;
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief          Write command
 *
 * @param[in]     me      Pointer to handle of  module.
 * @param[in]     cmd     Command
 * @param[in]     p_data  Pointer to handle of data
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
static base_status_t m_ssd1306_write_cmd(ssd1306_t *me, uint8_t cmd, uint8_t p_data)
{
    CHECK_STATUS(me->i2c_write(me->device_address, cmd, (uint8_t *)&p_data, 1));

    return BS_OK;
}

static base_status_t m_ssd1306_write_mul(ssd1306_t *me, uint8_t start_addr, uint8_t *p_data, uint16_t length)
{
    uint16_t t = 254;
    while (t--)
    {
        CHECK_STATUS(me->i2c_mul_write(me->device_address, start_addr, p_data, length));
    }

    return BS_OK;
}

/* End of file -------------------------------------------------------- */
