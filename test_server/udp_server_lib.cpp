/****************************************************************************
 * src/server/udp_server_lib.cpp
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
#define MAX_NR_CLIENTS 8 /* At the moment everything supports at max 8 cars */

/* Define wheter to allow use of Broadcast address */
#define BROADCAST 1

/*****************************************************************************
 * Variables
 ****************************************************************************/
/** Server **/
/* Socket to listen on and for a new connection */ 
static int sockfd;

/* Structs for finding the own address information, to store these info and 
 * to loop through the list returned by getaddrinfo.
 */
static struct addrinfo addrinfo_hints, *addrinfo_server, *addrinfo_queue;

/** Client **/
static struct addrinfo *addrinfo_clients[8];

/* Used to go through the linked list */
struct addrinfo *current;


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
 *@param interface The unix interface (eth0, wlan0 etc) to use
 *@param interface_len Length of the interface name (in bytes)
 *@return -1 on error, 0 on success 
 */ 
int init_udp_server(const char *port_string, const char *interface, int interface_len){

  /** Check the input data **/
  if(port_string == NULL)
    port_string = DEFAULT_PORT;

  /** Get the information for the server **/
  memset(&addrinfo_hints, 0, sizeof addrinfo_hints);
  /* Use either protocol (v4, v6) */
  addrinfo_hints.ai_family = AF_UNSPEC;
  /* Use UDP socket type */
  addrinfo_hints.ai_socktype = SOCK_DGRAM;
  /* Use system IP */
  addrinfo_hints.ai_flags = AI_PASSIVE;

  if( (ret = getaddrinfo(NULL, port_string, &addrinfo_hints, &addrinfo_server)) 
      != 0 ){
    printf("Server:getaddrinfo: %s\n", gai_strerror(ret));  
    return -1;
  }

  /** Loop through the list returned by getaddrinfo and get socket **/
  for( addrinfo_queue = addrinfo_server; addrinfo_queue != NULL; 
      addrinfo_queue = addrinfo_queue->ai_next){
    if((sockfd = socket(addrinfo_queue->ai_family,
            addrinfo_queue->ai_socktype, addrinfo_queue->ai_protocol)) == -1){
      error("Server: get socket failed");
      continue;
    }

#if 0
    /** Try to bind to a specific interface **/
    ret = setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, interface, 
        interface_len);
    if(ret == -1){
      ret == errno;
      error("Server: Failed to set socket option: Bind to interface");
      printf("Setsockopt: %s\n", gai_strerror(ret));  
      return -1;
    }
#endif

#if BROADCAST
    int broadcastEnable=1;
    ret = setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, 
        sizeof(broadcastEnable));
    if(ret == -1){
      ret == errno;
      error("Server: Failed to set socket option: Broadcast");
      printf("Setsockopt: %s\n", gai_strerror(ret));  
      return -1;
    }
#endif
    if(bind(sockfd, addrinfo_queue->ai_addr, addrinfo_queue->ai_addrlen)
        == -1){
      close(sockfd);
      error("Server: Bind to socket error");
      continue;
    }
    break;

  }
  /* If we got to addrinfo_queue == NULL, we did not get a valid socket */
  if(addrinfo_queue == NULL){
    error("Server: Could not bind a socket");
    return -1;
  }
  /* We do not need the addrinfo_server anymore */
  freeaddrinfo(addrinfo_server);
  return 0;
}

/**
 * Reads in the list and prepares the addrinfo structs for each client
 *@param client_list A list of strings of the clients IP address
 *@param nr_clients How many clients we have
 *@return -1 on failure, 0 on success
 */
int udp_server_setup_client(const char *client_addr, const char *port_string, int client_nr){


  /** Check the input data **/
  if(port_string == NULL)
    port_string = DEFAULT_PORT;
  if(client_addr == NULL){
    error("No valid client list");
    return -1;
  }
  if(client_nr < 0 || client_nr > 7){
    error("No valid client Nr.");
    return -1;
  }

  memset(&addrinfo_hints, 0, sizeof addrinfo_hints);
  /* Use either protocol (v4, v6) */
  addrinfo_hints.ai_family = AF_UNSPEC;
  /* Use UDP socket type */
  addrinfo_hints.ai_socktype = SOCK_DGRAM;

  /* Get the information for the client */
  if( (ret = getaddrinfo( client_addr, port_string, &addrinfo_hints,
          &current)) != 0 ){
    printf("Client:getaddrinfo: %s\n", gai_strerror(ret));  
    return -1;
  }
  else{
    /* We read out the IP, kind of a nice check to see wheter all went fine */
    char ip4[INET_ADDRSTRLEN];
    struct sockaddr_in *sa = (struct sockaddr_in*) current->ai_addr;
    inet_ntop(AF_INET, &(sa->sin_addr),ip4, INET_ADDRSTRLEN);
    printf("Clients address: %s\n",ip4);
    addrinfo_clients[client_nr] = current;
  }
  return 0;
}

/**
 * Routine to write to the specified client
 *@param buffer Message to send
 *@param buffer_size Size of the message
 *@param client_nr The client to write to, array index style, lowest = 0
 */
int udp_server_write(const unsigned char *buffer, int buffer_size, int client_nr){
  /* Sanity check of the input */
  if(client_nr > (MAX_NR_CLIENTS - 1) || client_nr < 0){
    error("Not a valid client");
    return -1;
  }
  if(buffer == NULL){
    error("Not a valid buffer address");
    return -1;
  }
  /* Just so we type less */
  current = addrinfo_clients[client_nr];

  if((ret = sendto(sockfd, (void*)buffer, buffer_size, 0,
        (sockaddr*)current->ai_addr, current->ai_addrlen)) == -1){
    ret = errno;
    printf("Failed to send message to client %i\n", client_nr);
    printf("Error Code: (%i) %s\n",ret,gai_strerror(ret)); 
    return -1;    
    }
  else if(ret < buffer_size){
    printf("Wrote only %i of %i bytes\n", ret, buffer_size);
    return -1;
  }
  return ret;
}
