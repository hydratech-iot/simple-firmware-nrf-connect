/*
* File Name: bsp_imu.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Board Support Package for BMI270
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Includes ----------------------------------------------------------- */
#include "bsp_i2c.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
K_MUTEX_DEFINE(bsp_i2c0_mutex);
#define BSP_I2C0_CS_ENTER(timeout_ms)              \
    do                                             \
    {                                              \
        k_mutex_lock(&bsp_i2c0_mutex, timeout_ms); \
    } while (0)

#define BSP_I2C0_CS_EXIT()               \
    do                                   \
    {                                    \
        k_mutex_unlock(&bsp_i2c0_mutex); \
    } while (0)

K_MUTEX_DEFINE(bsp_i2c1_mutex);
#define BSP_I2C1_CS_ENTER(timeout_ms)              \
    do                                             \
    {                                              \
        k_mutex_lock(&bsp_i2c1_mutex, timeout_ms); \
    } while (0)

#define BSP_I2C1_CS_EXIT()               \
    do                                   \
    {                                    \
        k_mutex_unlock(&bsp_i2c1_mutex); \
    } while (0)

/* Public variables --------------------------------------------------- */
const struct device *i2c0_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device *i2c1_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static base_status_t bsp_i2c_read_mem(bsp_i2c_num_t i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
static base_status_t bsp_i2c_write_mem(bsp_i2c_num_t i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
static base_status_t bsp_i2c_write_data(bsp_i2c_num_t i2c_num, uint8_t dev_addr, uint8_t *p_data, uint16_t len);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_i2c_init(void)
{
    if (!device_is_ready(i2c0_dev))
    {
        return BS_ERROR;
    }

    if (!device_is_ready(i2c1_dev))
    {
        return BS_ERROR;
    }
    return BS_OK;
}

base_status_t bsp_i2c_0_read_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len)
{
    return bsp_i2c_read_mem(BSP_I2C_NUM_0, dev_addr, reg_addr, p_data, len);
}

base_status_t bsp_i2c_0_write_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len)
{
    return bsp_i2c_write_mem(BSP_I2C_NUM_0, dev_addr, reg_addr, p_data, len);
}

base_status_t bsp_i2c_0_write_data(uint8_t dev_addr, uint8_t *p_data, uint16_t len)
{
    return bsp_i2c_write_data(BSP_I2C_NUM_0, dev_addr, p_data, len);
}

base_status_t bsp_i2c_1_read_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len)
{
    return bsp_i2c_read_mem(BSP_I2C_NUM_1, dev_addr, reg_addr, p_data, len);
}

base_status_t bsp_i2c_1_write_mem(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len)
{
    return bsp_i2c_write_mem(BSP_I2C_NUM_1, dev_addr, reg_addr, p_data, len);
}

base_status_t bsp_i2c_1_write_data(uint8_t dev_addr, uint8_t *p_data, uint16_t len)
{
    return bsp_i2c_write_data(BSP_I2C_NUM_1, dev_addr, p_data, len);
}

/* Private function ---------------------------------------------------- */
static base_status_t bsp_i2c_read_mem(bsp_i2c_num_t i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len)
{
    int ret = 0;

    __ASSERT_NO_MSG(i2c_num < BSP_I2C_NUM_MAX);

    switch (i2c_num)
    {
    case BSP_I2C_NUM_0:
    {
        BSP_I2C0_CS_ENTER(K_FOREVER);
        ret = i2c_burst_read(i2c0_dev, dev_addr, reg_addr, p_data, len);
        BSP_I2C0_CS_EXIT();
        break;
    }

    case BSP_I2C_NUM_1:
    {
        BSP_I2C1_CS_ENTER(K_FOREVER);
        ret = i2c_burst_read(i2c1_dev, dev_addr, reg_addr, p_data, len);
        BSP_I2C1_CS_EXIT();
        break;
    }

    default:
        break;
    }

    if (0 == ret)
    {
        return BS_OK;
    }

    return BS_ERROR;
}

static base_status_t bsp_i2c_write_mem(bsp_i2c_num_t i2c_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len)
{
    int ret = 0;
    struct i2c_msg msg[2];

    __ASSERT_NO_MSG(i2c_num < BSP_I2C_NUM_MAX);

    /* Send the address to write to */
    msg[0].buf   = &reg_addr;
    msg[0].len   = 1U;
    msg[0].flags = I2C_MSG_WRITE;

    /* Data to be written, and STOP after this. */
    msg[1].buf = (uint8_t *)p_data;
    msg[1].len = len;
    msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    switch (i2c_num)
    {
    case BSP_I2C_NUM_0:
    {
        BSP_I2C1_CS_ENTER(K_FOREVER);
        ret = i2c_transfer(i2c0_dev, msg, 2, dev_addr);
        BSP_I2C1_CS_EXIT();
        break;
    }

    case BSP_I2C_NUM_1:
    {
        BSP_I2C1_CS_ENTER(K_FOREVER);
        ret = i2c_transfer(i2c1_dev, msg, 2, dev_addr);
        BSP_I2C1_CS_EXIT();
        break;
    }

    default:
        break;
    }

    if (0 == ret)
    {
        return BS_OK;
    }

    return BS_ERROR;
}

static base_status_t bsp_i2c_write_data(bsp_i2c_num_t i2c_num, uint8_t dev_addr, uint8_t *p_data, uint16_t len)
{
    struct i2c_msg msgs;
    int ret = 0;

    __ASSERT_NO_MSG(i2c_num < BSP_I2C_NUM_MAX);

    /* Data to be written, and STOP after this. */
    msgs.buf   = p_data;
    msgs.len   = len;
    msgs.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

    switch (i2c_num)
    {
    case BSP_I2C_NUM_0:
    {
        BSP_I2C0_CS_ENTER(K_FOREVER);
        ret = i2c_transfer(i2c0_dev, &msgs, 1, dev_addr);
        BSP_I2C0_CS_EXIT();
        break;
    }

    case BSP_I2C_NUM_1:
    {
        BSP_I2C1_CS_ENTER(K_FOREVER);
        ret = i2c_transfer(i2c1_dev, &msgs, 1, dev_addr);
        BSP_I2C1_CS_EXIT();
        break;
    }

    default:
        break;
    }

    if (0 == ret)
    {
        return BS_OK;
    }

    return BS_ERROR;
}

/* End of file -------------------------------------------------------- */
