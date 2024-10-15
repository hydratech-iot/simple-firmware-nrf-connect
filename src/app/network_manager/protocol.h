/*
* File Name: protocol.h
* 
* Author: Thuan Le (thuanle@hydratech-iot.com)
* 
* Description: Network Protocol
* 
* Copyright 2024, HydraTech. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include "bsp_afe.h"
#include "bsp_imu.h"
#include "bsp_temp.h"

/* Public defines ----------------------------------------------------- */
#define MAX_CHANNEL         (9)
#define PACKET_HDR_CMD_POS  (0)
#define PACKET_HDR_LEN_POS  (1)
#define PACKET_HDR_DATA_POS (2)

#define PACKET_ACK_RESPONSE_NONE                (0)
#define PACKET_ACK_RESPONSE_ACK                 (1)
#define PACKET_ACK_RESPONSE_NACK_BAD_CRC        (2)
#define PACKET_ACK_RESPONSE_NACK_UNIMPLEMENTED  (3)
#define PACKET_ACK_RESPONSE_NACK_TIMED_OUT      (4)
#define PACKET_ACK_RESPONSE_NACK_BUSY           (5)
#define PACKET_ACK_RESPONSE_NACK_CMD_FAILED     (6)
#define PACKET_ACK_RESPONSE_NACK_INVALID_TYPE   (7)

/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
    PACKET_CMD_START_STOP_STUDY_SET      = 0,
    PACKET_CMD_START_STOP_STUDY_GET      = 1,
    PACKET_CMD_START_STOP_STUDY_RESP     = 2,
    PACKET_CMD_PPG_STREAMING_DATA        = 3,    // Only for streaming mode
    PACKET_CMD_IMU_STREAMING_DATA        = 4,    // Only for streaming mode
    PACKET_CMD_STUDY_DATA                = 5,    // Data will be sent to the cloud each every interval
    PACKET_CMD_TEMPERATURE_DATA          = 6,    // Only for streaming mode
    PACKET_CMD_PPG_PHYSICAL_DATA         = 7,    // Only for streaming mode
    PACKET_CMD_BATTERY_DATA              = 8,    // Only for streaming mode
    PACKET_CMD_DEVICE_CHARGING_DATA      = 9,    // Data will be sent to the cloud each every interval
    PACKET_CMD_DEVICE_CONFIGURATION_SET  = 10,
    PACKET_CMD_DEVICE_CONFIGURATION_GET  = 11,
    PACKET_CMD_DEVICE_CONFIGURATION_RESP = 12,
    PACKET_CMD_TEMP_CONFIGURATION_SET    = 13,
    PACKET_CMD_TEMP_CONFIGURATION_GET    = 14,
    PACKET_CMD_TEMP_CONFIGURATION_RESP   = 15,
    PACKET_CMD_AFE_CONFIGURATION_SET     = 16,
    PACKET_CMD_AFE_CONFIGURATION_GET     = 17,
    PACKET_CMD_AFE_CONFIGURATION_RESP    = 18,
    PACKET_CMD_IMU_CONFIGURATION_SET     = 19,
    PACKET_CMD_IMU_CONFIGURATION_GET     = 20,
    PACKET_CMD_IMU_CONFIGURATION_RESP    = 21,
    PACKET_CMD_DEVICE_INFO_SET           = 22,
    PACKET_CMD_DEVICE_INFO_GET           = 23,
    PACKET_CMD_DEVICE_INFO_RESP          = 24,
    PACKET_CMD_DEVICE_MODE_SET           = 25,
    PACKET_CMD_DEVICE_MODE_GET           = 26,
    PACKET_CMD_DEVICE_MODE_RESP          = 27,
    PACKET_CMD_MAX
}
packet_type_t;

/*
 * Header for all packets.
 */
typedef struct __attribute__((packed))
{
    uint8_t cmd; // Packet CMD
    uint8_t len; // Data length
    uint16_t msg_index;
}
packet_hdr_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t start; // 0: Stop; 1: Start
    uint32_t interval_min; // Study measure interval (Minutes)
}
packet_start_stop_study_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t study_status; // 0: Stop; 1: Start
}
packet_start_stop_study_resp_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    int32_t data[AFE_FIFO_MAX];
}
packet_ppg_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    int16_t data[IMU_FIFO_MAX_SAMPLE];
}
packet_imu_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t heart_rate;
    uint8_t spo2;
}
packet_ppg_phys_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    float value[5];
}
packet_temperature_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t level;
    uint8_t is_charging;
    float voltage;
}
packet_battery_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t heart_rate;
    uint8_t spo2;
    uint8_t battery_level;
    uint8_t is_charging;
    float battery_voltage;
    float temperature[5];
    float roll;
    float pitch;
    float yaw;
}
packet_study_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t battery_level;
    uint8_t is_charging;
    float battery_voltage;
    float temperature[5];
    float roll;
    float pitch;
    float yaw;
}
packet_device_charging_data_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    float temp_high_limit;
    float temp_low_limit;
    uint8_t operation_mode; // 0: Continuous; 1: Single shot
}
packet_temp_configuration_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint16_t sampling_rate;
    float current[AFE_PPG_LED_MAX]; // 0 -> 127.5 mA. AFE_PPG_LED_MAX = 3
    // 3 LEDs; 0: LED RED; 1: LED IR; 2: LED GREEN
}
packet_afe_configuration_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t operation_mode; // 0: Low power; 1: Normal
    uint16_t sampling_rate;
}
packet_imu_configuration_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    char mac_addr[12];
}
packet_device_info_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint8_t mode;
}
packet_device_mode_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint16_t study_interval;            // Minutes
    uint16_t temp_measure_interval;     // Seconds
    uint16_t oled_switch_off_period;    // Seconds
}
packet_device_configuration_set_t;

typedef struct __attribute__((packed))
{
    packet_hdr_t header;
    uint16_t study_interval;            // Minutes
    uint16_t temp_measure_interval;     // Seconds
    uint16_t oled_switch_off_period;    // Seconds
}
packet_device_configuration_resp_t;

/* End of file --------------------------------------------------------- */
