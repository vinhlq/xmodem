/*
 * Copyright (c) 2021 Vinh Le.
 * All Rights Reserved.
 */

#ifndef XMODEM_H_
#define XMODEM_H_

enum XMODEM_ERROR
{
/// Could not verify lower CRC byte
    XMODEM_ERROR_CRCL = -1,
/// Could not verify upper CRC byte
    XMODEM_ERROR_CRCH = -2,
/// No start of header found
    XMODEM_ERROR_NO_SOH = -3,
/// Packet number doesn't match its inverse
    XMODEM_ERROR_PKTNUM = -4,
/// Packet number error (unexpected sequence)
    XMODEM_ERROR_PKTSEQ =-5,
/// Packet number error (duplicate)
    XMODEM_ERROR_PKTDUP = -6,
/// Transfer is canceled
    XMODEM_ERROR_CANCEL =-8,
    /// Transfer is done (Technically not an error)
    XMODEM_ERROR_DONE = 0
};

typedef struct
{
    /**
     * Write the packet callback.
     *
     * @param[in] buf          Buffer to write data, size=128 bytes.
     * @param[in] index        Index of the packet.
     * @param[in] args         Callback argument.
     *
     * @return
     * >=0  Length of received data\n
     * -1   Receive failed.
     *
     */
    void (*write)(uint8_t *buf, uint16_t index, void *args);
    /**
     * Receiving the data callback.
     *
     * @param[in] buf          Buffer to receive data.
     * @param[in] size         Buffer size.
     * @param[in] timeout      Read timeout.
     *
     * @return
     * >=0  Length of received data\n
     * -1   Receive failed.
     *
     */
    int (*receive)(uint8_t *buf, int size, int timeout);
    /**
     * Sending the data callback.
     *
     * @param[in] buf          Buffer to send data.
     * @param[in] size         Buffer size.
     *
     * @return
     * 0    SUCCESS\n
     * -1   FAILED.
     *
     */
    int (*send)(uint8_t *buf, int size);
}xmodem_callback_t;

int xmodem_receive(xmodem_callback_t *xm, void *args);

#endif /* XMODEM_H_ */
