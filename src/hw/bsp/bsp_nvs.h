/*
* File Name: bsp_nvs.h
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
// IMPORTANT: The revision of nvs_data_t. Every time nvs_data_t is changed,
// the NVS_DATA_VERSION value must be updated too.
// #define NVS_DATA_VERSION        (uint32_t)(CONFIG_CFG_MGMT_NVS_VERSION)
#define NVS_DATA_VERSION        (uint32_t)(1)
#define FLASH_PAGE_SIZE         (4096)
#define NVS_SECTOR_COUNT        (3)
#define NVS_SECTOR_SIZE         (FLASH_PAGE_SIZE)
#define NVS_MAX_DATA_SIZE       ((NVS_SECTOR_COUNT - 1) * NVS_SECTOR_SIZE)

/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
    bool passed;
    uint8_t led_drive_current_decimal[6];
}
afe_calib_t;

typedef struct
{
    uint32_t study_interval_min;
    uint32_t study_temp_meas_interval_second;
    uint32_t oled_switch_off_period_second;
}
device_configuration_t;

/**
 * @brief Structure representing the NVS data layout.
 */
typedef struct nvs_data_struct {
    uint32_t version; ///< NVS data version.

    device_configuration_t device_configuration;
}
nvs_data_t;

/* Public macros ------------------------------------------------------ */

/**
 * @brief Macro to look up the NVS key for a specific data member.
 */
#define SYS_NVS_LOOKUP_KEY(name)                                                                    \
    bsp_nvs_lookup_key(offsetof(struct nvs_data_struct, name),                                 \
    sizeof(g_nvs_setting_data.name))

/**
 * @brief Macro to store a specific data member in NVS.
 */
#define SYS_NVS_STORE(member)                                                                       \
    do                                                                                              \
    {                                                                                               \
            bsp_nvs_store(SYS_NVS_LOOKUP_KEY(member), &g_nvs_setting_data.member,              \
                sizeof(g_nvs_setting_data.member));                                                 \
    } while (0)

/**
 * @brief Macro to load a specific data member from NVS.
 */
#define SYS_NVS_LOAD(member)                                                                        \
    do                                                                                              \
    {                                                                                               \
        return bsp_nvs_load(SYS_NVS_LOOKUP_KEY(member), &g_nvs_setting_data.member,            \
            sizeof(g_nvs_setting_data.member));                                                     \
    } while (0)

/* Public variables --------------------------------------------------- */
extern nvs_data_t g_nvs_setting_data; ///< Global NVS data structure.

/* Public function prototypes ----------------------------------------- */

/**
 * @brief Initialize NVS storage and automatically load data to RAM if the data version is valid.
 *        In case of data version difference, all data will be set to default values both in NVS and RAM.
 *
 * @return Status indicating the success or failure of the operation.
 */
base_status_t bsp_nvs_init(void);

/**
 * @brief Store specific data into NVS storage.
 *
 * @param[in] key   Key for the data.
 * @param[in] p_src Pointer to the buffer containing data.
 * @param[in] len   Length of data in bytes.
 *
 * @return Status indicating the success or failure of the operation.
 */
base_status_t bsp_nvs_store(uint32_t key, void *p_src, uint32_t len);

/**
 * @brief Load specific data from NVS storage to the destination buffer.
 *
 * @param[in]  key   Key for the data.
 * @param[out] p_des Pointer to the buffer that will contain data.
 * @param[in]  len   Length of the buffer in bytes.
 *
 * @return Status indicating the success or failure of the operation.
 */
base_status_t bsp_nvs_load(uint32_t key, void *p_des, uint32_t len);

/**
 * @brief Automatically look up the ID entry based on input parameters: offset and size of variable.
 *
 * @param[in] offset Offset of the variable.
 * @param[in] size   Size of data in bytes.
 *
 * @attention The lookup table is @ref VAR_DATA_ID_LIST[]
 * 
 * @return Key value.
 */
uint32_t bsp_nvs_lookup_key(uint32_t offset, uint32_t size);

/* End of file -------------------------------------------------------- */
