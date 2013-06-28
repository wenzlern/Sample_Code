/****************************************************************************
 * src/udp_server_lib.cpp
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
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>

/*****************************************************************************
 * Definitions
 ****************************************************************************/
#define DEFAULT_PORT "3334"
#define MAX_NR_CLIENTS  NUMBER_OF_CARS
#define RECV_BUF_SIZE VISION_DATA_BUFFER_SIZE

/*****************************************************************************
 * Variables
 ****************************************************************************/
/** Client **/
/* Socket to listen on and for a new connection */ 
static int sockfd;

/* Structs for finding the own address information, to store these info and 
 * to loop through the list returned by getaddrinfo.
 */
static struct addrinfo addrinfo_hints, *addrinfo_client, *addrinfo_queue;

/** Server **/
static struct addrinfo *addrinfo_server;

/** General **/
/* Char array to store the connectors Address in a readable format */
static char s[INET6_ADDRSTRLEN];
socklen_t socklen;

/* Generic return value */
int ret;


/*****************************************************************************
 * Server
 ****************************************************************************/
/**
 * Generig error function to call in the code.
 *@param error_string String with the error message.
 */
void error( const char *error_string){
  fprintf(stdout, "ERROR: ");
  fprintf(stdout, error_string);
  fprintf(stdout, "\n");
}


/**
 * Routine to initialize the udp server application.
 *
 *@param port The port to listen on
 *@return -1 on error, 0 on success 
 */ 
int init_udp_client(const char *addr_string, const char *port_string){

  /** Check the input data **/
  if(port_string == NULL)
    port_string = DEFAULT_PORT;

  /** Get the information for the client socket **/
  memset(&addrinfo_hints, 0, sizeof addrinfo_hints);
  /* Use either protocol (v4, v6) */
  addrinfo_hints.ai_family = AF_UNSPEC;
  /* Use UDP socket type */
  addrinfo_hints.ai_socktype = SOCK_DGRAM;
  /* Use system IP */
  addrinfo_hints.ai_flags = AI_PASSIVE;

  if( (ret = getaddrinfo(NULL, port_string, &addrinfo_hints, &addrinfo_client)) 
      != 0 ){
    printf("Client:getaddrinfo: %s\n", gai_strerror(ret));  
    return -1;
  }

  /** Loop through the list returned by getaddrinfo and get socket **/
  for( addrinfo_queue = addrinfo_client; addrinfo_queue != NULL; 
      addrinfo_queue = addrinfo_queue->ai_next){
    if((sockfd = socket(addrinfo_queue->ai_family,
            addrinfo_queue->ai_socktype, addrinfo_queue->ai_protocol)) == -1){
      error("Client: get socket failed");
      continue;
    }
    if(bind(sockfd, addrinfo_queue->ai_addr, addrinfo_queue->ai_addrlen)
        == -1){
      close(sockfd);
      error("Client: Bind to socket error");
      continue;
    }
    break;

  }
  /* If we got to addrinfo_queue == NULL, we did not get a valid socket */
  if(addrinfo_queue == NULL){
    error("Client: Could not bind a socket");
    return -1;
  }
  /* We do not need the addrinfo_server anymore */
  freeaddrinfo(addrinfo_client);

  /** We need to set the server addrinfo for recvfrom() **/
  memset(&addrinfo_hints, 0, sizeof addrinfo_hints);
  /* Use either protocol (v4, v6) */
  addrinfo_hints.ai_family = AF_UNSPEC;
  /* Use UDP socket type */
  addrinfo_hints.ai_socktype = SOCK_DGRAM;

  if( (ret = getaddrinfo( addr_string, port_string, &addrinfo_hints,
          &addrinfo_server)) 
      != 0 ){
    printf("Server:getaddrinfo: %s", gai_strerror(ret));  
    return -1;
  }
  else
    printf("Server registered: %s", addr_string);

  return 0;
}


/**
 * Routine to read the messages from the server
 *@param buffer pointer to a receive buffer
 *@param len how many bytes to read
 *@return Number of bytes read, -1 on error
 */
int udp_client_read(const unsigned char *buffer, int len){
  /* Sanity check of the input */
  if(buffer == NULL){
    error("Not a valid buffer address");
    return -1;
  }
  socklen = sizeof addrinfo_server->ai_addr; 
  if((ret = recvfrom(sockfd, (void*)buffer, len, 0,
        (sockaddr*)addrinfo_server->ai_addr, &socklen )) == -1){
    error("Failed to receive message");
    return -1;    
  }
  return ret;
}
