/****************************************************************************
 * src/client/bluetooth_lib.cpp
 *
 *   Copyright (C) 2012 Nils Wenzler. All rights reserved.
 *   Author: Nils Wenzler <wenzlern@ee.ethz.ch>
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
 * 3. Neither the name nor the names of its contributors may be
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

/*****************************************************************************
 * Includes
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <netdb.h>

/*****************************************************************************
 * Variables
 ****************************************************************************/
/* The BT addrinfo struct */
static struct sockaddr_rc sockaddr_client = {0};
/* Socket descriptor */
static int sockfd;
/* Generic return value */
int r;

/**
 * Connects to a waiting bluetooth server
 *@param client_addr of min lenght SOL_RFCOMM (18)
 *@param rfcomm_channel  The rfcomm channel to use(must match with the clients)
 *@return 0 on success, -1 on failure
 */
int bluetooth_connect_to( const char *client_addr, int rfcomm_channel){

  /* Get a socket */
  sockfd = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
  if(sockfd == -1){
    r = errno;
    printf("Error: Getting socket descriptor\n");
    printf("Socket: %s\n", gai_strerror(r));
    return -1;
  }
  /* Set the connection parameters for the destination */
  sockaddr_client.rc_family = AF_BLUETOOTH;
  sockaddr_client.rc_channel = (uint8_t) rfcomm_channel;
  str2ba(client_addr, &sockaddr_client.rc_bdaddr);

  /* Connect to the given Address */
  r = connect(sockfd, (struct sockaddr*)&sockaddr_client, sizeof(sockaddr_client));
  if(r == -1){
    r = errno;
    printf("Error: On connect\n");
    printf("Connect: %s\n", gai_strerror(r));
    return -1;
  }
  return 0;
}

/**
 * Sends data to a connected device
 *@param data The data to be sent
 *@param length Length of the data in bytes
 *@return The number of bytes sent, -1 on failure
 */
int bluetooth_send(const char *data, int length){
  /* Write the data to the socket */
  r = write(sockfd, data, length);
  if(r == -1){
    r = errno;
    printf("Error: On send\n");
    printf("Send: %s\n", gai_strerror(r));
    return -1;
  }
  return r;
}
