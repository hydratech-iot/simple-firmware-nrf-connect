/*
* File Name: protocol.h
* 
* Author: Thuan Le 
* 
* Description: Network Protocol
* 
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#pragma once

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>

/* Public defines ----------------------------------------------------- */
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
    PACKET_CMD_NOTIFICATION = 1,
    PACKET_CMD_MAX
} packet_type_t;

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
}
packet_cmd_notification_t;

/* End of file --------------------------------------------------------- */
