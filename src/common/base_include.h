/*
 * File Name: base_include.h
 *
 * Author: Thuan Le
 *
 * Description:
 *
 */

/* Define to prevent recursive inclusion ------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree/io-channels.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/irq_offload.h>
#include <zephyr/sys/reboot.h>
#include "base_type.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <shell/shell_bt_nus.h>

#include <zephyr/bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>

#include <zephyr/settings/settings.h>

#include <stddef.h>

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/* End of file -------------------------------------------------------- */
