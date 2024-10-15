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

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------- */
#include "base_include.h"

/* Public defines ----------------------------------------------------- */
typedef enum
{
    BSP_SPI_NUM_0 = 0,
    BSP_SPI_NUM_1,
    BSP_SPI_NUM_2,
    BSP_SPI_NUM_MAX,
} bsp_spi_num_t;

/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
base_status_t bsp_spi_init(void);
base_status_t bsp_spi_2_transmit(uint8_t *tx_data, uint16_t len);
base_status_t bsp_spi_2_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/* End of file -------------------------------------------------------- */
