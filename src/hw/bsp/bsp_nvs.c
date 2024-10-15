/*
* File Name: bsp_nvs.c
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
#include "base_type.h"
#include "bsp_nvs.h"

/* Public variables --------------------------------------------------- */
nvs_data_t g_nvs_setting_data;

/* Private defines ---------------------------------------------------- */
#define NVS_DATA_PAIR(key_id, name)                                             \
    {                                                                           \
        .key = key_id, .offset = offsetof(struct nvs_data_struct, name),        \
        .size = sizeof(g_nvs_setting_data.name)                                 \
    }

LOG_MODULE_REGISTER(bsp_nvs, CONFIG_LOG_DEFAULT_LEVEL);

/* Private enumerate/structure ---------------------------------------- */
enum {
    NVS_VERSION_KEY_NAME              = 0,
    NVS_DEVICE_CONFIGURATION_KEY_NAME = 1,
};

typedef struct {
    uint32_t key; // This is the key-pair of data stored in NVS
    uint32_t offset; // The offset of variable in @ref nvs_data_struct
    uint32_t size; // The size of variable in bytes
} nvs_key_data_t;

/**
 * @brief List of key-pair data stored in NVS
 *
 * This list defines the keys used to store and retrieve specific data
 * in the NVS storage. Each entry in the list represents a key-pair,
 * where 'key' is the identifier used for storage, 'offset' is the
 * offset of the variable in the @ref nvs_data_struct, and 'size' is
 * the size of the variable in bytes.
 */
static const nvs_key_data_t nvs_data_list[] = {
    NVS_DATA_PAIR(NVS_DEVICE_CONFIGURATION_KEY_NAME, device_configuration),
};

/* Private macros ----------------------------------------------------- */
/* Private Constants -------------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static struct nvs_fs fs_handle;

static const nvs_data_t default_configurations = {
    .version                                              = NVS_DATA_VERSION,
    .device_configuration.study_interval_min              = 1,
    .device_configuration.study_temp_meas_interval_second = 5,
    .device_configuration.oled_switch_off_period_second   = 30
};

/* Private function prototypes ---------------------------------------- */
static base_status_t bsp_nvs_store_all(void);
static base_status_t bsp_nvs_reset_data(void);
static bool bsp_nvs_setup(void);
static void bsp_nvs_load_all(void);

/* Function definitions ----------------------------------------------- */
base_status_t bsp_nvs_init(void)
{
    uint32_t nvs_ver;
    const uint8_t size_nvs_ver = sizeof(nvs_ver);

    if (!bsp_nvs_setup())
    {
        LOG_ERR("NVS init failed");
        goto __LBL_END__;
    }

    // Get NVS data version
    (void)nvs_read(&fs_handle, NVS_VERSION_KEY_NAME, &nvs_ver, sizeof(nvs_ver));

    LOG_INF("Data version in NVS storage: %d", nvs_ver);

    // Check NVS data version
    if (nvs_ver != NVS_DATA_VERSION)
    {
        LOG_INF("NVS data version is different, all current data in NVS will be erased");
        bsp_nvs_reset_data();

        // Update new NVS data version
        if (nvs_write(&fs_handle, NVS_VERSION_KEY_NAME, &g_nvs_setting_data.version, size_nvs_ver) < 0)
        {
            LOG_ERR("NVS store new data version error");
            goto __LBL_END__;
        }
    }
    else
    {
        // Update NVS data version into RAM
        g_nvs_setting_data.version = nvs_ver;

        // Load all data from NVS to RAM structure data
        bsp_nvs_load_all();
    }

    LOG_INF("System NVS storage init successfully");
    return BS_OK;

__LBL_END__:
    LOG_ERR("System NVS storage init failed");
    return BS_ERROR;
}

base_status_t bsp_nvs_store(uint32_t key, void *p_src, uint32_t len)
{
    __ASSERT_NO_MSG(p_src != NULL);

    if (nvs_write(&fs_handle, key, p_src, len) < 0)
    {
        LOG_ERR("NVS store data error");
        return BS_ERROR;
    }

    return BS_OK;
}

base_status_t bsp_nvs_load(uint32_t key, void *p_des, uint32_t len)
{
    __ASSERT_NO_MSG(p_des != NULL);

    if (nvs_read(&fs_handle, key, p_des, len) < 0)
    {
        LOG_ERR("NVS load data error");
        return BS_ERROR;
    }

    return BS_OK;
}

uint32_t bsp_nvs_lookup_key(uint32_t offset, uint32_t size)
{
    for (uint_fast16_t i = 0; i < (sizeof(nvs_data_list) / sizeof(nvs_key_data_t)); i++)
    {
        if ((nvs_data_list[i].offset == offset) && (nvs_data_list[i].size == size))
        {
            return nvs_data_list[i].key;
        }
    }

    // In case there are no key in table, return NULL. Please refer @nvs_data_list
    return 0;
}

/* Private implementations -------------------------------------------------- */
static base_status_t bsp_nvs_reset_data(void)
{
    g_nvs_setting_data = default_configurations;
    return bsp_nvs_store_all();
}

static bool bsp_nvs_setup(void)
{
    struct flash_pages_info info;

    fs_handle.flash_device = FLASH_AREA_DEVICE(storage);
    __ASSERT_NO_MSG(fs_handle.flash_device);

    // Get page info by offset
    fs_handle.offset = FLASH_AREA_OFFSET(storage);
    if (flash_get_page_info_by_offs(fs_handle.flash_device, fs_handle.offset, &info))
    {
        LOG_ERR("Unable to get page info");
        return false;
    }

    // Mount NVS
    fs_handle.sector_size = info.size;
    fs_handle.sector_count = 3U;
    if (nvs_mount(&fs_handle))
    {
        LOG_ERR("Flash Init failed");
        return false;
    }

    return true;
}

base_status_t bsp_nvs_store_all(void)
{
    uint16_t sizeof_nvs_data_list;
    uint32_t addr;
    void *p_data;
    size_t var_len;

    // Automatically looking into the nvs data list in order to get data information and store to NVS
    sizeof_nvs_data_list = (uint16_t)(sizeof(nvs_data_list) / sizeof(nvs_data_list[0]));
    addr = (uint32_t)&g_nvs_setting_data;

    for (uint_fast16_t i = 0; i < sizeof_nvs_data_list; i++)
    {
        p_data = (void *)(addr + nvs_data_list[i].offset);
        var_len = (size_t)nvs_data_list[i].size;

        if (nvs_write(&fs_handle, nvs_data_list[i].key, p_data, var_len) < 0)
        {
            LOG_ERR("NVS store all data error");
            return BS_ERROR;
        }
    }

    return BS_OK;
}

static void bsp_nvs_load_all(void)
{
    uint16_t sizeof_nvs_data_list;
    uint32_t addr;
    void *p_data;
    size_t var_len;

    // Load variable data from ID List Table
    addr = (uint32_t)&g_nvs_setting_data;
    sizeof_nvs_data_list = (uint16_t)(sizeof(nvs_data_list) / sizeof(nvs_data_list[0]));

    for (uint_fast16_t i = 0; i < sizeof_nvs_data_list; i++)
    {
        p_data = (void *)(addr + nvs_data_list[i].offset);
        var_len = (size_t)nvs_data_list[i].size;

        if (nvs_read(&fs_handle, nvs_data_list[i].key, p_data, var_len) < 0)
        {
            LOG_ERR("NVS load all data error");
            break;
        }
    }
}

/* End of file -------------------------------------------------------- */
