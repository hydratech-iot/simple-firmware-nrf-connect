/*
 * File Name: bsp_wireless_charger.c
 *
 * Author: Thuan Le (thuanle@hydratech-iot.com)
 *
 * Description: Board Support Package Wireless Charger
 *
 * Copyright 2024, HydraTech. All rights reserved.
 * You may use this file only in accordance with the license, terms, conditions,
 * disclaimers, and limitations in the end user license agreement accompanying
 * the software package with which this file was provided.
 */

/* Includes ----------------------------------------------------------- */
#include "bsp_wireless_charger.h"
#include "bsp_i2c.h"
#include "bsp_i2c_mul.h"

/* Private defines ---------------------------------------------------- */
LOG_MODULE_REGISTER(bsp_wireless_charger, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static struct stwlc38_dev stwlc38 = { 0 };
static uint8_t stwlc_alloc_buf[258] = { 0x00 };

/* Private function prototypes ---------------------------------------- */
static int32_t platform_write(void *phandle, uint8_t *wbuf, int32_t wlen);
static int32_t platform_write_read(void *phandle, uint8_t *wbuf, int32_t wlen, uint8_t *rbuf, int32_t rlen);
static void platform_delay(uint32_t millisec);
static void *platform_alloc_mem(size_t size);
static void platform_free_mem(void *ptr);
static void platform_log(void *phandle, int32_t level, const char *msg, int32_t len);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_wireless_charger_init(void)
{
    struct stwlc38_chip_info info = { 0 };

    stwlc38.bus_write      = platform_write;
    stwlc38.bus_write_read = platform_write_read;
    stwlc38.mdelay         = platform_delay;
    stwlc38.alloc_mem      = platform_alloc_mem;
    stwlc38.free_mem       = platform_free_mem;
    stwlc38.log            = platform_log;

    if (stwlc38_get_chip_info(&stwlc38, &info) != STWLC38_OK)
    {
        LOG_ERR("Wireless charger (STMLC38) init failed");
        return BS_ERROR;
    }
    LOG_INF("Wireless charger (STMLC38) init success");

    // if (stwlc38_fw_update(&stwlc38, STWLC38_FW_PATCH_CFG, 1) != STWLC38_OK)
    // {
    //     LOG_ERR("Wireless charger firmware update failed");
    //     return BS_ERROR;
    // }

    // LOG_INF("Wireless charger firmware update success");
    return BS_OK;
}


/* Private function definitions ---------------------------------------- */
static int32_t platform_write(void *phandle, uint8_t *wbuf, int32_t wlen)
{
    // Select the I2C from MUX
    if (bsp_i2c_mul_select_channel(I2C_CHANNEL_1) != BS_OK)
        return STWLC38_ERR_BUS_W;

    if (bsp_i2c_0_write_data(STWLC38_I2C_ADDR, wbuf, (uint16_t)wlen) != BS_OK)
        return STWLC38_ERR_BUS_W;

    return STWLC38_OK;
}

static int32_t platform_write_read(void *phandle, uint8_t *wbuf, int32_t wlen, uint8_t *rbuf, int32_t rlen)
{
    // Select the I2C from MUX
    if (bsp_i2c_mul_select_channel(I2C_CHANNEL_1) != BS_OK)
        return STWLC38_ERR_BUS_WR;

    // if (bsp_i2c_1_write_read(STWLC38_I2C_ADDR, wbuf, (uint8_t)wlen, rbuf, (uint8_t)rlen) != BS_OK)
    //     return STWLC38_ERR_BUS_WR;

    return STWLC38_OK;
}

static void platform_delay(uint32_t millisec)
{
    SYSTEM_DELAY_MSEC(millisec);
}

static void *platform_alloc_mem(size_t size)
{
    return (void*) stwlc_alloc_buf;
}

static void platform_free_mem(void *ptr)
{
    // Do not things
}

static void platform_log(void *phandle, int32_t level, const char *msg, int32_t len)
{
    // LOG_INF(msg);
}

/* End of file -------------------------------------------------------- */
