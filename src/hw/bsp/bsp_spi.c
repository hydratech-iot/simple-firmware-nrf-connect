/*
 * File Name: bsp_spi.h
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
#include "bsp_spi.h"
#include "bsp_io.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
K_MUTEX_DEFINE(bsp_spi2_mutex);
#define BSP_SPI2_CS_ENTER(timeout_ms)              \
    do                                             \
    {                                              \
        k_mutex_lock(&bsp_spi2_mutex, timeout_ms); \
    } while (0)

#define BSP_SPI2_CS_EXIT()               \
    do                                   \
    {                                    \
        k_mutex_unlock(&bsp_spi2_mutex); \
    } while (0)

/* Public variables --------------------------------------------------- */

/* Private variables -------------------------------------------------- */
static const struct device *spi2_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));

/* Private function prototypes ---------------------------------------- */
static base_status_t bsp_spi_transfer_data(bsp_spi_num_t spi_num, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_spi_init(void)
{
    if (!device_is_ready(spi2_dev))
    {
        return BS_ERROR;
    }

    return BS_OK;
}

base_status_t bsp_spi_2_transmit(uint8_t *tx_data, uint16_t len)
{
    return bsp_spi_transfer_data(BSP_SPI_NUM_2, tx_data, NULL, len);
}

base_status_t bsp_spi_2_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    return bsp_spi_transfer_data(BSP_SPI_NUM_2, tx_data, rx_data, len);
}

/* Private function definitions ---------------------------------------- */
static base_status_t bsp_spi_transfer_data(bsp_spi_num_t spi_num, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    int ret = 0;
    struct spi_config spi2_cfg = {
        .frequency = DT_PROP(DT_NODELABEL(spi2), max_frequency),
        .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8U),
    };

    struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = len,
    };

    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = len,
    };

    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    switch (spi_num)
    {
    case BSP_SPI_NUM_2:
        BSP_SPI2_CS_ENTER(K_FOREVER);
        ret = spi_transceive(spi2_dev, &spi2_cfg, &tx, &rx);
        BSP_SPI2_CS_EXIT();
        break;
    default:
        // LOG_ERR("Read: unsupported SPI device %u", spi_num);
        break;
    }

    if (0 == ret)
    {
        return BS_OK;
    }
    else
    {
        return BS_ERROR;
    }
}

/* End of file -------------------------------------------------------- */
