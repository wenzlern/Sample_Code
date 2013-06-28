/****************************************************************************
 * apps/orcactrl/bt_lmx9830.c
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


/* include driver header and config file. As soon as this gets moved around, 
 * change the " signs to < and >.
 */
#include "bt_lmx9830.h"
#include "config.h"



/** Local packet buffer for sending 
 *
 * +2 to catch unfortunate typos that will probably made sometime.
 */
static uint8_t local_sendbuf[ LMX9830_PACK_MAXSIZE+2 ] = {0}; 
static uint8_t local_sppdatabuf[ LMX9830_PACK_MAXSIZE+2 ] = {0}; 

/*Local buffers for receiving
 *
 * Buffer needs to be pack_maxsize plus one for the rfcomm_port value
 */
static uint8_t local_recbuf[ LMX9830_PACK_MAXSIZE + 1] = {0}; 

/** Uart File handler
 */
static int uartfile = 0;




/** Packet setup helper
 *
 * Setup a packet with checksum and framing
 *
 *@param type Type of package (see bt_lmx9830.h)
 *@param opcode The Command do send (see bt_lmx9830.h) 
 *@param len The length of the data (in bytes)
 *@param data Pointer to the data
 */
inline static void 
lmx9830_helper_setuppack( uint8_t type, 
  uint8_t opcode,
  uint16_t len,
  const uint8_t *data )
{
  /* auto storage */
  uint16_t checksum = 0;
  uint8_t cnt;

  /* start byte, always the same */
  local_sendbuf[0] = STX;

  /* set type and opcode */
  local_sendbuf[1] = type;
  local_sendbuf[2] = opcode,
  
  /* set size */
  local_sendbuf[3] = len & 0xff;
  local_sendbuf[4] = (len >> 8) & 0xff;

  /* calculate sum of bytes number 2 through 4 (index starts at 0...) */
  for( cnt = 1; cnt < 5; ++cnt )
    checksum += local_sendbuf[cnt];

  /* set sixth byte (which is the checksum byte) to lower byte of sum */
  local_sendbuf[5] = (checksum & 0xff);

  /* set data */
  if( len > 0 && data != NULL )
    memcpy( local_sendbuf+6, data, len );

  /* set end byte */
  local_sendbuf[6+len] = ETX;
}

/** Delayread helper
 * Read on uartfile until all bytes arrived, with a small delay to wait for
 * the new data to arrive
 *
 *@param len How many bytes we want to read
 *@param readto Receive buffer of size len
 *@param delay_ms How long we wait for new data to arrive
 *@return How many bytes we read, if it fails: -1
 */
inline static uint16_t
lmx9830_helper_delayedread( uint16_t len, 
  uint8_t *readto,
  uint8_t delay_ms )
{
  /* auto variables */
  uint16_t cnt = 0;
  int8_t res = 0;

  /* loop until bytes read match 'len', the number of bytes to be read */
  while( cnt < len )
  { /* wait for new data to arrive */
    up_mdelay(delay_ms);

    /* try reading the rest, which is 'len-cnt' long. Read to */
    res = read( uartfile, readto+cnt, len-cnt );
    if( res <= 0 ) return -1;   /* reading failed, return error */
    else           cnt += res;  /* reading ok, add bytes to count */
  }
  
  /* if all is fine, we should now have cnt=len */
  return cnt;
}


/** Initialization routine 
 *
 * Sets up the module
 *
 *@return -1 on failure, 0 on success
 */
uint8_t 
lmx9830_init( void )
{ 
  /* wait for module to be ready */
  up_mdelay(1000);

  /* open uart file */
  uartfile = open( LMX9830_UART_FILE, O_RDWR | O_NOCTTY | O_NONBLOCK );
  if (uartfile < 0)
    /* return error */
    return -1;

  /* This init sequence has to be sent once at the beginning. */
  uint8_t tmp = 0x01;
  write( uartfile, &tmp, 1 );
  up_mdelay(1000);

  /* setup the change bt address packet */
  lmx9830_helper_setuppack( PACK_REQ, CHANGE_LOCAL_BDADDRESS,
    6, const_lmx9830_btaddr );

  /* send packet */
  write( uartfile, local_sendbuf, 13 );

  /* wait. The proper thing to do here would be to wait for
   * the CFM packets and see if things went fine, but since
   * we cannot do any real error handling anyway, we'll just 
   * wait until the LMX surely is ready.
   */
  up_mdelay(1000);

#if SELFCONNECT
  /* Force Master mode, in order to accept several connections efficiently
   */
  tmp = 1;
  lmx9830_helper_setuppack( PACK_REQ, FORCE_MASTER_ROLE, 1, &tmp );
  write( uartfile, local_sendbuf, 8 );
  up_mdelay(1000);
#endif /* SELFCONNECT */

#if PACKET_MODE
  /* disable automatic mode. Do not remove this. If the lmx is in auto
   * mode, it will go to transparent mode after connection is established.
   * we don't want that.
   */
  tmp = 0;
  lmx9830_helper_setuppack( PACK_REQ, WRITE_OPERATION_MODE, 1, &tmp );
  write( uartfile, local_sendbuf, 8 );
  up_mdelay(1000);
#endif /* PACKET_MODE */

  /* setup req to go to BT mode */
  lmx9830_helper_setuppack( PACK_REQ, ENTER_BLUETOOTH_MODE, 0, NULL );
  write( uartfile, local_sendbuf, 7 );
  up_mdelay(1000);

  /* another CFM packet will now be sent by the LMX indicating 
   * that things went fine. It will have to be dropped by the 
   * user of the driver since it will be at the beginning of the 
   * buffer after this completes.
   */
  printf("Initialized Bluetooth with Adress %X\n", const_lmx9830_btaddr);

  return 0;
}

/* Send packet routine 
 *
 * Send a packet over the bluetooth module
 *
 *@param rfcomm_port The RFCOMM port to send on
 *@param data Pointer to the data to be sent
 *@param len The length of the data (in bytes)
 *@return What the POSIX write() function return^^
 */
uint8_t
lmx9830_send( uint8_t rfcomm_port, uint8_t *data, uint16_t len  )
{
  /* the payload of a SPP_SEND_DATA pack looks like this:
   *
   * | rfcomm port | payload data size | payload                   |
   * | 1 byte      | 2 bytes           | <payload data size> bytes |
   *
   * and hence has to be assembled as such. This is done here. No
   * idea why the length has to be specified again, since this adds
   * 2 bytes to the packet size for no apparent reason (this length is 
   * always just the packet data length + 3 ......). Whatever.
   */
  local_sppdatabuf[0] = rfcomm_port;

  local_sppdatabuf[1] = (len & 0xff);
  local_sppdatabuf[2] = ((len>>8) & 0xff);

  /* copy actual data after the payload header */
  memcpy( local_sppdatabuf+3, data, len );

  /* setup the packet with the prepared payload. len+3 means length of payload 
   * +3 for length of payload header. 
   */
  lmx9830_helper_setuppack( PACK_REQ, SPP_SEND_DATA, len+3, local_sppdatabuf );

  /* actually send. 10 bytes are added by framing. (7 normally, 3 payload 
   * header)
   */
  return write( uartfile, local_sendbuf, len+10 );
}

/* Get packet routine  
 *
 * Since reading from the file happens in a 
 * buffered manner, this method here will need to pick apart
 * the incoming byte stream and identify the individual packets.
 * 
 * This method currently skips bytes until it finds an actual packet
 * start byte. It then interprets the packet and returns the rfcomm_port
 * in the first byte and the data in the following.
 *
 * Note that the recv_to buffer supplied has to be at least
 * LMX9830_PACK_MAXSIZE  + 1 long to make sure no buffer overflows occur.
 * 
 * The return value indicates the length of the received data. It does not
 * consist with the buffer length, since it omits the rf_comm byte.
 * => return = lmx9830_receive(buf);
 *    return == 5 indicates 5 data bytes at buf[1] to buf[6] with buf[0] beeing
*     the rfcomm channel
 */

int8_t
lmx9830_receive( uint8_t *recv_to )
{
  /* The character-by-character reading is limited to 
   * finding the start of the packet, after that, we 
   * read in bulk.
   */

  /* Find start of packet */
  int8_t ret = 0;
  uint8_t tmp = 0, buf[5] = {0};
  while( 1 ) 
  {
    ret = read( uartfile, &tmp, 1 );

    if( tmp == STX ) /* found packet */
      break; /* hence stop looping */
    else if( ret == -1 || ret == -EAGAIN ) /* nothing to read */
      return 0;
  }

    /* if none of the above, we have a random character. Skip it
     * by reading the next byte.
     */


  /* Read rest of header, data length will now be stored in bytes
   * buf[2] and buf[3]. buf[4] contains the checksum, we read that 
   * as well to skip it.
   */
  lmx9830_helper_delayedread( 5, buf, 1 );
   
  /* get data length */
  uint16_t len = buf[2] | (((uint16_t)buf[3]) << 8);
 // printf("Length, reported by BT: %i\n", len);
  
  /* limit length to maximum packet size */
  if( len > LMX9830_PACK_MAXSIZE )
    len = LMX9830_PACK_MAXSIZE;

  /* read out the actual data */
  lmx9830_helper_delayedread( len, local_recbuf, 1 );
 
  /* look at packet type. We want to ignore everything but 
   * regular data for the moment. The switch statement seems
   * overkill for just deciding between IND/SPPDATA and other,
   * but it will become required later on when we start 
   * differentiating between more types of packets.
   */
  switch( buf[0] ) /* buf[0] is the packet type byte */
  {
    /* indicator package */
    case PACK_IND: 
      if( buf[1] == SPP_INCOMING_DATA ) /* buf[1] is opcode */
      { /* actual data */
        /* the +3 skips rfcomm chan (1 byte) and length (2 bytes)*/
        printf( "Got %i bytes from port %i, %s\n", *(local_recbuf+1), 
            *local_recbuf, local_recbuf+3 );
        /*In the first byte of the recv_to buffer we want the rfcomm_port*/
        memcpy( recv_to, local_recbuf, 1);
        memcpy( recv_to + 1, local_recbuf+3, len-2 );
      }
      break;
    case PACK_REQ: /* request */
    case PACK_CFM: /* confirmation */
      if(buf[1] == SPP_LINK_ESTABLISHED )
        {
          /*If the error Code is not 0 indicate that somethin went wrong*/
          if(buf[4] != 0)
            len = -1;
        }
        break;
    case PACK_RES: /*  */
    default:
      break; /* nothing more to do */
  }

  /* reset buffer. This might be inefficient since we'll mostly use
   * small packets, then this could be change to erase only 
   * len-3 bytes or so.
   */
  memset( local_recbuf, 0x0, LMX9830_PACK_MAXSIZE + 1);

  /* yes, packet here */
  return len-2;
}

/** Receive with windowing for transparent mode
 *
 * This routine will receive and parse the continous data stream we receive
 * in transparent mode.
 * @param recv_to pointer to the receive struct
 *                Note that the recv_to buffer supplied has to be at least
 *                LMX9830_PACK_MAXSIZE long to make sure no buffer 
 *                overflows occur.
 * @return  length of the received packet 
 */
int8_t
lmx9830_receive_windowing( uint8_t *recv_to )
{
  /* Find start of packet */
  int8_t ret = 0;
  uint8_t received = 0;
  uint8_t tmp = 0 ;
  uint8_t i;
  uint8_t buf[RECV_PACKET_LENGTH] = {0};
  while( 1 ) 
  {
    ret = read( uartfile, &tmp, 1 );

    if( tmp == STARTBYTE ) /* found packet */
    {
      break; /* hence stop looping */
    }
    else if( ret == -1 || ret == -EAGAIN ) /* nothing to read */
      return 0;

    /* if none of the above, we have a random character. Skip it
     * by reading the next byte.
     */
  }

  /* Read the rest of the packet */
  received = read( uartfile, &local_recbuf, RECV_PACKET_LENGTH - 1);

  for(i=0; i < RECV_TIMEOUT; i++)
  {
    if( received == (RECV_PACKET_LENGTH - 1) && local_recbuf[RECV_END - 1] == ENDBYTE)
    {  
      memcpy(recv_to, &local_recbuf, received - 1);
      break; /* hence stop looping */
    }
    else if( ret >= 0)
    {
      ret = read( uartfile, &buf, RECV_PACKET_LENGTH - received - 1);
      memcpy(&local_recbuf + received, &buf, ret);
      received += ret;
    }
  }
  return received - 1;

}



/*
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
                        const uint8_t *bt_addr)
{
  /*Setup payload packet (Remember the packet is byte-swapped)*/
  uint8_t temp[8];
  temp[0] = remote_rfc;
  /*Copy the bt_addr into the buffer (6 bytes)*/
  memcpy(temp + 1, bt_addr, 6);
  temp[7] = local_rfc;

  lmx9830_helper_setuppack( PACK_REQ, SPP_ESTABLISH_LINK, 8, temp );
  write( uartfile, local_sendbuf, 15 );
  usleep(1000);
  
  /*Call the receive routine that handles the confirm packages*/
  uint8_t recv_buffer[LMX9830_PACK_MAXSIZE];
  int8_t ret;
  ret = lmx9830_receive(recv_buffer);
  if(ret == -1)
  {
    printf("Connecting with %X failed\n", bt_addr);
  }
  else if(recv_buffer[1] == SPP_LINK_ESTABLISHED)
  {
    printf("Connected with %X\n", bt_addr);
    return 0;
  }

  return -1;
}


/*
 * Change the BT pin
 *
 * Sets the pin the module uses to pair with the remote device.
 *
 *@param  bt_pin The desired pin as an array of hex values of the
 *               corresponding ascii entry of the pin.
 *               Pin 1234 => const uint8_t bt_pin[] = {0x31, 0x32, 0x33, 0x34}
 */
void
lmx9830_setpin(const uint8_t *bt_pin )
{
  uint8_t payload[5];
  /*The first byte of the payload is the length of the pin, then the pin*/
  payload[0] = 4;
  memcpy( payload + 1, bt_pin, 4 );
  lmx9830_helper_setuppack( PACK_REQ, GAP_SET_FIXED_PIN, 5, payload );
  write( uartfile, local_sendbuf, 12 );
  usleep(10000);
}

