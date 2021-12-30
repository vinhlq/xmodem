/*
 * Copyright (c) 2021 Vinh Le.
 * All Rights Reserved.
 * Implementation of xmodem protocol
 * https://web.mit.edu/6.115/www/amulet/xmodem.htm
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "xmodem.h"

#define DEBUG_PRINT(msg)  xm->send(msg, sizeof(msg));

#if defined(__GNUC__)
/** @brief A GCC style macro for handling packed structures. */
#define XM_ATTRIBUTE_PACKED __attribute__ ((packed))

/** @brief A macro for handling packed structures.
 *  @n Use this macro before the structure definition.
 *  @n X denotes the maximum alignment of structure members. X is not supported with
 *  GCC. GCC always uses 1 byte maximum alignment.
 */
#define XM_PACK_START(x)

/** @brief A macro for handling packed structures.
 *  @n Use this macro after the structure definition.
 *  @n With GCC, add XM_ATTRIBUTE_PACKED after the closing curly braces of the structure
 *  definition.
 */
#define XM_PACK_END()

/** @brief GCC style macro for aligning a variable. */
#define XM_ATTRIBUTE_ALIGN(X) __attribute__ ((aligned(X)))

/** @brief A macro for aligning a variable.
 *  @n Use this macro before the variable definition.
 *  @n X denotes the storage alignment value in bytes.
 *  @n To be GCC-compatible, use XM_ATTRIBUTE_ALIGN(X) before the semicolon on normal
 *  variables. Use XM_ATTRIBUTE_ALIGN(X) before the opening curly brace on structure variables.
 */
#define XM_ALIGN(X)

/** @brief A macro for defining a weak symbol. */
#define XM_WEAK __attribute__ ((weak))

/** @brief A macro for handling non-returning functions. */
#define XM_NORETURN __attribute__ ((noreturn))

/** A macro for placing a variable in a section.
 *  @n Use this macro after the variable definition, before the equal sign or a semicolon.
 *  @n X denotes the section to place the variable in.
 */
#define XM_ATTRIBUTE_SECTION(X) __attribute__ ((section(X)))

/** @brief A macro for notifying the compiler of an intended
 *  switch case fallthrough. */
#if __GNUC__ >= 7
  #define XM_FALLTHROUGH __attribute__ ((fallthrough));
#else
  #define XM_FALLTHROUGH
#endif
#endif

/// Size of an XMODEM packet
#define XMODEM_DATA_SIZE              128

/***************************************************************************//**
 * @addtogroup Commands
 * @{
 ******************************************************************************/

/// Start of Header
#define XMODEM_CMD_SOH                (0x01)
/// End of Transmission
#define XMODEM_CMD_EOT                (0x04)
/// Acknowledge
#define XMODEM_CMD_ACK                (0x06)
/// Not Acknowledge
#define XMODEM_CMD_NAK                (0x15)
/// Cancel
#define XMODEM_CMD_CAN                (0x18)
/// Ctrl+C
#define XMODEM_CMD_CTRL_C             (0x03)
/// ASCII 'C'
#define XMODEM_CMD_C                  (0x43)

/** @} addtogroup Commands */

XM_PACK_START(1)
/// XMODEM packet
typedef struct
{
    uint8_t header;                   ///< Packet header (@ref XMODEM_CMD_SOH)
    uint8_t packet_number;             ///< Packet sequence number
    uint8_t packet_number_c;            ///< Complement of packet sequence number
    uint8_t data[XMODEM_DATA_SIZE];   ///< Payload
    uint8_t crc_h;                     ///< CRC high byte
    uint8_t crc_l;                     ///< CRC low byte
} XM_ATTRIBUTE_PACKED xmodem_packet_t;
XM_PACK_END()

static xmodem_packet_t xmodem_packet;

static uint16_t crc16(const uint8_t new_byte, uint16_t prev_result)
{
    prev_result = (prev_result >> 8) | (prev_result << 8);
    prev_result ^= new_byte;
    prev_result ^= (prev_result & 0xff) >> 4;
    prev_result ^= (prev_result << 8) << 4;

    prev_result ^= ((uint8_t) ((uint8_t) ((uint8_t) (prev_result & 0xff)) << 5))
        | ((uint16_t) ((uint8_t) ((uint8_t) (prev_result & 0xff))
                           >> 3) << 8);

    return prev_result;
}

uint16_t crc16_stream(const uint8_t *buffer,
                     size_t        length,
                     uint16_t      prev_result)
{
    size_t position = 0;
    for (; position < length; position++) {
        prev_result = crc16(buffer[position], prev_result);
    }

    return prev_result;
}

static int32_t send_packet(xmodem_callback_t *xm, uint8_t packet)
{
    uint8_t packet_buf[3];

    if (packet == XMODEM_CMD_CAN)
    {
        packet_buf[0] = XMODEM_CMD_CAN;
        packet_buf[1] = XMODEM_CMD_CAN;
        packet_buf[2] = XMODEM_CMD_CAN;
        return xm->send(packet_buf, 3);
    }
    else
    {
        packet_buf[0] = packet;
        return xm->send(packet_buf, 1);
    }
}

static int receive_packet(xmodem_callback_t *xm, xmodem_packet_t *packet, int timeout)
{
    int requested_bytes;
    int received_bytes;
    uint8_t *buf = (uint8_t *)packet;

    // Read the first byte
    received_bytes = xm->receive(buf, 1, timeout);
    if(received_bytes == 0)
    {
        // Timeout
        return -1;
    }
    if (packet->header != XMODEM_CMD_SOH)
    {
        // All packets except XMODEM_CMD_SOH are single-byte
        return 1;
    }

    requested_bytes = sizeof(xmodem_packet_t) - 1;
    received_bytes = xm->receive(buf + 1, requested_bytes, timeout);
    if (received_bytes != requested_bytes)
    {
        return 0;
    }
    return received_bytes;
}

static int xmodem_receive2(xmodem_callback_t *xm, xmodem_packet_t *packet, void *args)
{
    bool started = false;
    uint8_t packet_count = 0;
    uint8_t packet_number = 0;
    uint16_t crc16;
    int length;

    // Start synchronization
    send_packet(xm, XMODEM_CMD_C);
    while(1)
    {
        length = receive_packet(xm, packet, 3000);
        if (length == -1)
        {
            // Timeout
            return -1;
        }
        else if (length == 0)
        {
            DEBUG_PRINT("rcverr");
            send_packet(xm, XMODEM_CMD_NAK);
            return -1;
        }
        switch (packet->header)
        {
            case XMODEM_CMD_SOH:
                // Packet number must start at 1, and must monotonically increase
                if (!started)
                {
                    if (packet->packet_number != 0x01)
                    {
                        DEBUG_PRINT("pktnum");
                        send_packet(xm, XMODEM_CMD_NAK);
                        return -1;
                    }
                    started = true;
                }
                else
                {
                    if (packet->packet_number == packet_number)
                    {
                        DEBUG_PRINT("replay");
                        send_packet(xm, XMODEM_CMD_ACK);
                        return XMODEM_ERROR_PKTDUP;
                    }
                    else if (packet->packet_number != (uint8_t)(packet_number + 1))
                    {
                        DEBUG_PRINT("ooseq");
                        send_packet(xm, XMODEM_CMD_NAK);
                        return XMODEM_ERROR_PKTSEQ;
                    }
                }

                // Byte 3 is the two's complement of the packet number in the second byte
                if (packet->packet_number + packet->packet_number_c != 0xFF)
                {
                    DEBUG_PRINT("compl");
                    send_packet(xm, XMODEM_CMD_NAK);
                    return XMODEM_ERROR_PKTNUM;
                }

                // Bytes 132-133 contain a 16-bit CRC over the data bytes
                crc16 = crc16_stream(packet->data, XMODEM_DATA_SIZE, 0);

                if (((crc16 >> 8) & 0xFF) != packet->crc_h)
                {
                    DEBUG_PRINT("crch");
                    send_packet(xm, XMODEM_CMD_NAK);
                    return XMODEM_ERROR_CRCH;
                }

                if ((crc16 & 0xFF) != packet->crc_l)
                {
                    DEBUG_PRINT("crcl");
                    send_packet(xm, XMODEM_CMD_NAK);
                    return XMODEM_ERROR_CRCL;
                }

                xm->write(packet->data, packet_count, args);
                packet_count++;
                packet_number = packet->packet_number;
                send_packet(xm, XMODEM_CMD_ACK);
                break;

            case XMODEM_CMD_EOT:
//                DEBUG_PRINT("EOT");
                send_packet(xm, XMODEM_CMD_ACK);
                return XMODEM_ERROR_DONE;

            case XMODEM_CMD_CAN:
            case XMODEM_CMD_C:
            case XMODEM_CMD_CTRL_C:
                DEBUG_PRINT("CAN");
                send_packet(xm, XMODEM_CMD_CAN);
                return XMODEM_ERROR_CANCEL;

            default:
                DEBUG_PRINT("UNK");
                send_packet(xm, XMODEM_CMD_CAN);
                return XMODEM_ERROR_NO_SOH;
        }
    }
}

int xmodem_receive(xmodem_callback_t *xm, void *args)
{
    return xmodem_receive2(xm, &xmodem_packet, args);
}

/*
 * EOF
 */
