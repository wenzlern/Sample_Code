/****************************************************************************
 * apps/orcactrl/bt_lmx9830.h
 *
 *   Copyright (C) 2012 Sandro Merkli, Nils Wenzler. All rights reserved.
 *   Author: Sandro Merkli <smerkli@ee.ethz.ch>
 *           Nils Wenzler  <wenzlern@ee.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#ifndef DRV_LMX9830_H
#define DRV_LMX9830_H 1


#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <fcntl.h>

#include <nuttx/spi.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include <stdio.h>




/* header for the driver for the LMX9830 Bluetooth module.
 *
 * The declarations shown here might change while the driver is 
 * further being developed. The constants ought to stay the same,
 * though.
 */



/**
 * Enum with the possible packet types. 
 *
 * This describes the possible packet types. For further indormation see the 
 * Software user guide. http://www.ti.com/tool/lmx9830-sw
 */
enum 
{
  PACK_REQ = 0x52, /**< request */
  PACK_CFM = 0x43, /**< confirm */
  PACK_IND = 0x69, /**< indication */
  PACK_RES = 0x72, /**< response */
};


/**
 * Enum with the OPCodes for the Bluetooth protocol.
 * 
 * They are copied from the software user guide and thoroughly documented
 * there. http://www.ti.com/tool/lmx9830-sw
 */
enum 
{
  GAP_INQUIRY = 0x00,
  GAP_DEVICE_FOUND = 0x01,
  GAP_REMOTE_DEVICE_NAME = 0x02,
  GAP_READ_LOCAL_NAME = 0x03,
  GAP_WRITE_LOCAL_NAME = 0x04,
  GAP_READ_LOCAL_BDA = 0x05,
  GAP_SET_SCANMODE = 0x06,
  GAP_GET_FIXED_PIN = 0x16,
  GAP_SET_FIXED_PIN = 0x17,
  GAP_GET_PIN = 0x75,
  GAP_GET_SECURITY_MODE = 0x18,
  GAP_SET_SECURITY_MODE = 0x19,
  GAP_REMOVE_PAIRING = 0x1B,
  GAP_LIST_PAIRED_DEVICES = 0x1C,
  GAP_ENTER_SNIFF_MODE = 0x21,
  GAP_EXIT_SNIFF_MODE = 0x37,
  GAP_ENTER_PARK_MODE = 0x38,
  GAP_EXIT_PARK_MODE = 0x39,
  GAP_ENTER_HOLD_MODE = 0x3A,
  GAP_SET_LINK_POLICY = 0x3B,
  GAP_GET_LINK_POLICY = 0x3C,
  GAP_POWER_SAVE_MODE_CHANGED = 0x3D,
  GAP_ACL_ESTABLISHED = 0x50,
  GAP_ACL_TERMINATED = 0x51,
  GAP_SET_AUDIO_CONFIG = 0x59,
  GAP_GET_AUDIO_CONFIG = 0x5A,
  GAP_ESTABLISH_SCO_LINK = 0x5D,
  GAP_RELEASE_SCO_LINK = 0x5E,
  GAP_MUTE_MIC = 0x5F,
  GAP_SET_VOLUME = 0x60,
  GAP_GET_VOLUME = 0x61,
  GAP_CHANGE_SCO_PACKET_TYPE = 0x62,
  SPP_SET_PORT_CONFIG = 0x07,
  SPP_GET_PORT_CONFIG = 0x08,
  SPP_PORT_CONFIG_CHANGED = 0x09,
  SPP_ESTABLISH_LINK = 0x0A,
  SPP_LINK_ESTABLISHED = 0x0B,
  SPP_INCOMING_LINK_ESTABLISHED = 0x0C,
  SPP_RELEASE_LINK = 0x0D,
  SPP_LINK_RELEASED = 0x0E,
  SPP_SEND_DATA = 0x0F,
  SPP_INCOMING_DATA = 0x10,
  SPP_TRANSPARENT_MODE = 0x11,
  SPP_CONNECT_DEFAULT_CON = 0x12,
  SPP_STORE_DEFAULT_CON = 0x13,
  SPP_GET_LIST_DEFAULT_CON = 0x14,
  SPP_DELETE_DEFAULT_CON = 0x15,
  SPP_SET_LINK_TIMEOUT = 0x57,
  SPP_GET_LINK_TIMEOUT = 0x58,
  SPP_PORT_STATUS_CHANGED = 0x3E,
  SPP_GET_PORT_STATUS = 0x40,
  SPP_PORT_SET_DTR = 0x41,
  SPP_PORT_SET_RTS = 0x42,
  SPP_PORT_BREAK = 0x43,
  SPP_PORT_OVERRUN_ERROR = 0x44,
  SPP_PORT_PARITY_ERROR = 0x45,
  SPP_PORT_FRAMING_ERROR = 0x46,
  SDAP_CONNECT = 0x32,
  SDAP_DISCONNECT = 0x33,
  SDAP_CONNECTION_LOST = 0x34,
  SDAP_SERVICE_BROWSE = 0x35,
  SDAP_SERVICE_SEARCH = 0x36,
  SDAP_SERVICE_REQUEST = 0x1E,
  SDAP_ATTRIBUTE_REQUEST = 0x3F,
  CHANGE_LOCAL_BDADDRESS = 0x27,
  CHANGE_NVS_UART_SPEED = 0x23,
  CHANGE_UART_SETTINGS = 0x48,
  SET_PORTS_TO_OPEN = 0x22,
  GET_PORTS_TO_OPEN = 0x1F,
  RESTORE_FACTORY_SETTINGS = 0x1A,
  STORE_CLASS_OF_DEVICE = 0x28,
  FORCE_MASTER_ROLE = 0x1D,
  READ_OPERATION_MODE = 0x49,
  WRITE_OPERATION_MODE = 0x4A,
  SET_DEFAULT_LINK_POLICY = 0x4C,
  GET_DEFAULT_LINK_POLICY = 0x4D,
  SET_EVENT_FILTER = 0x4E,
  GET_EVENT_FILTER = 0x4F,
  SET_DEFAULT_LINK_TIMEOUT = 0x55,
  GET_DEFAULT_LINK_TIMEOUT = 0x56,
  SET_DEFAULT_AUDIO_CONFIG = 0x5B,
  GET_DEFAULT_AUDIO_CONFIG = 0x5C,
  SET_DEFAULT_LINK_LATENCY = 0x63,
  GET_DEFAULT_LINK_LATENCY = 0x64,
  SET_CLOCK_FREQUENCY = 0x67,
  GET_CLOCK_FREQUENCY = 0x68,
  SET_PCM_SLAVE_CONFIG = 0x74,
  ENABLE_SDP_RECORD = 0x29,
  DELETE_SDP_RECORDS = 0x2A,
  STORE_SDP_RECORD = 0x31,
  RESET = 0x26,
  LMX9830_READY = 0x25,
  TEST_MODE = 0x24,
  WRITE_ROM_PATCH = 0x47,
  READ_RSSI = 0x20,
  RF_TEST_MODE = 0x4B,
  DISABLE_TL = 0x52,
  TL_ENABLED = 0x53,
  HCI_COMMAND = 0x65,
  AWAIT_INITIALIZATION_EVENT = 0x66,
  ENTER_BLUETOOTH_MODE = 0x66,
  SET_CLOCK_AND_BAUDRATE = 0x69,
  SET_GPIO_WPU = 0x6B,
  GET_GPIO_STATE = 0x6C,
  SET_GPIO_DIRECTION = 0x6D,
  SET_GPIO_OUTPUT_HIGH = 0x6E,
  SET_GPIO_OUTPUT_LOW = 0x6F,
  READ_NVS = 0x72,
  WRITE_NVS = 0x73,
};


/**
 * Packet delimiter
 * 
 * The start and stop delimiters, as defined in the datasheet.
 */
enum
{
  STX = 0x02, /**< start delimiter */
  ETX = 0x03, /**< stop delimiter */
};


/** Initialization routine for the bluetooth module
 *
 * This opens the UART belonging to the bluetooth module
 * (this is specified using const_lmx9830_uartfile[] internally
 * and the value for the uartfile is set in config.h)
 * and puts the bluetooth module in the right mode.
 *
 * @return -1 for error opening UART or 0 for success.
 */
uint8_t 
lmx9830_init( void );

/**
 * Send packet
 * 
 * Send routine that does framing internally. 
 * @param rfcomm_port specifies the port to use. The port has to be
                      configured as open to use it.
 * @param data pointer to the data buffer
 * @param len length of the data to be sent (# of bytes)
 * @return The value that gets returned by the POSIX write() function
 */
uint8_t
lmx9830_send( uint8_t rfcomm_port, uint8_t *data, uint16_t len  );

/**
 * Receive data
 *
 * Receive routine for arbitrary packet length (max. LMX9830_MAX_PACKET_LENGTH)
 * @param recv_to Buffer for the data. Should be large enough for the data +
                  1 byte for the rfcomm channel
 * @return -1 on failure else length of the data (+1 byte for the rfcomm chan).
 */
int8_t
lmx9830_receive( uint8_t *recv_to );

int8_t
lmx9830_receive_windowing( uint8_t *recv_to );

/**
 * Connect to device
 *
 * Connect to a device actively. It is recommended to set the SELFCONNECT flag
 * in the config.h file when using this function. With this on, the device
 * tries to become the master device in the connection, allowing up to 7
 * connections simultanious. Additionally the const_lmx9830_ports[] variable
 * needs to be adjusted to open several rfcomm ports. Each bit that is on
 * or of represents a port that is open or not (portrange: rfcomm1 - rfcomm32).
 * The values are in byte swapped order, so opening port 1 (default) is:
 *  const uint8_t const_lmx9830_ports[] = { 0x03,0x00,0x00,0x00};
 * 
 *@param local_rfc The RFCOMM port to be used by the BT module
 *@param remote_rfc The RFCOMM port of the device we want to connect to
 *@param bt_addr Pointer to the array that stores the BT address of the remote
         device. The address needs to be stored byte swapped.
         AB:::::CD => bt_addr[0] = CD,...., bt_addr[5] = AB
 *@return -1 if it fails, 0 on success
 */
uint8_t
lmx9830_connect(uint8_t local_rfc, uint8_t remote_rfc,
                        const uint8_t *bt_addr);

/**
 * Change the BT pin
 *
 * Sets the pin the module uses to pair with the remote device.
 *
 *@param  bt_pin The desired pin as an array of hex values of the
 *               corresponding ascii entry of the pin.
 *               Pin 1234 => const uint8_t bt_pin[] = {0x31, 0x32, 0x33, 0x34}
 */
void lmx9830_setpin(const uint8_t *bt_pin );

#endif /* DRV_LMX9830_H */
