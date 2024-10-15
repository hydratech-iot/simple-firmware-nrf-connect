/*
 * File Name: bsp_oled.c
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
#include "bsp_oled.h"
#include "bsp_i2c.h"

/* Private defines ----------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_oled, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ----------------------------------------- */
/* Private macros ------------------------------------------------------ */
/* Private variables --------------------------------------------------- */
ssd1306_t ssd1306;

/* Private function prototypes ----------------------------------------- */
static base_status_t bsp_oled_write_cmd(uint8_t slave_addr, uint8_t cmd, uint8_t *data, uint16_t len);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_oled_init(void)
{
    ssd1306.device_address = SSD1306_I2C_ADDR;
    ssd1306.i2c_write      = bsp_oled_write_cmd;
    ssd1306.i2c_mul_write  = bsp_oled_write_cmd;
    ssd1306.current_x      = 0;
    ssd1306.current_y      = 0;

    if (ssd1306_init(&ssd1306) != BS_OK)
    {
        LOG_ERR("OLED (SSD1306) init failed");
        return BS_ERROR;
    }

    LOG_INF("OLED (SSD1306) init success");

    return BS_OK;
}

base_status_t bsp_oled_deinit(void)
{
    CHECK_STATUS(ssd1306_clear(&ssd1306, SSD1306_COLOR_BLACK));
    CHECK_STATUS(ssd1306_update_screen(&ssd1306));
    CHECK_STATUS(ssd1306_deinit(&ssd1306));

    return BS_OK;
}

base_status_t bsp_oled_write_string(uint8_t x, uint8_t y, char *str, font_def font)
{
    CHECK_STATUS(ssd1306_set_cursor(&ssd1306, x, y));
    CHECK_STATUS(ssd1306_write_string(&ssd1306, str, font, SSD1306_COLOR_WHITE));

    return BS_OK;
}

base_status_t bsp_oled_draw_image(uint16_t x_pos, uint16_t y_pos, uint16_t w, uint16_t h, uint16_t color, const uint8_t bitmap[])
{
    return ssd1306_draw_image(&ssd1306, x_pos, y_pos, w, h, bitmap, color);
}

base_status_t bsp_oled_update_screen(void)
{
    return ssd1306_update_screen(&ssd1306);
}

base_status_t bsp_oled_clear(void)
{
    return ssd1306_clear(&ssd1306, SSD1306_COLOR_BLACK);
}

base_status_t bsp_oled_control_vdd(bool enable)
{
    return BS_OK;
}

/* Private function --------------------------------------------------- */
static base_status_t bsp_oled_write_cmd(uint8_t slave_addr, uint8_t cmd, uint8_t *data, uint16_t len)
{
    return bsp_i2c_0_write_mem(slave_addr, cmd, data, (uint16_t)len);
}

/* End of file -------------------------------------------------------- */
