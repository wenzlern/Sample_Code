/****************************************************************************
 * src/server/udp_server_lib.h
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
#ifndef UDP_SERVER_LIB_H
#define UDP_SERVER_LIB_H
/**
 * Routine to initialize the udp server application.
 *
 *@param port The port to listen on
 *@param interface The unix interface (eth0, wlan0 etc) to use
 *@param interface_len Length of the interface name (in bytes)
 *@return -1 on error, 0 on success 
 */ 
int init_udp_server(const char *port_string, const char *interface, int interface_len);

/**
 * Reads in the list and prepares the addrinfo structs for each client
 *@param client_list A list of strings of the clients IP address
 *@param nr_clients How many clients we have
 *@return -1 on failure, 0 on success
 */
int udp_server_setup_client(const char *client_addr, const char *port_string, int client_nr);

/**
 * Routine to write to the specified client
 *@param buffer Message to send
 *@param buffer_size Size of the message
 *@param client_nr The client to write to, array index style, lowest = 0
 */
int udp_server_write(const unsigned char *buffer, int buffer_size, int client_nr);


#endif /* UDP_SERVER_LIB_H */

