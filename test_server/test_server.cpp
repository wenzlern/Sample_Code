/****************************************************************************
 * src/controller_framework.c
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
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "udp_server_lib.h"
#include <parsing_lib.h>


/*****************************************************************************
 * Definitions
 ****************************************************************************/
/* Define the default port */
#define PORT "3334"
#define INTERFACE "wlan0"

/*****************************************************************************
 * Variables
 ****************************************************************************/
/* Send Buffer */
unsigned char send_buf[VISION_DATA_BUFFER_SIZE] = {0};
/* Return value holder */
int retval,i;

/* Struct that holds the vision information */
PI_IN vision_data;



/*****************************************************************************
 * Communication
 ****************************************************************************/
/**
 * Init the communication
 *@param  port  Which port to use
 *@return 0 on success, -1 on failure
 */
inline int init_communication(const char *port, const char *interface){
  return init_udp_server(port, interface, 5); 
}

/**
 * Writes the test data to the clients
 */
int write_vision(){
  vision_data_to_char_array(&vision_data, send_buf);
  udp_server_write(send_buf, VISION_DATA_BUFFER_SIZE, 0);
  printf("Sent message:\n");
  for(i=0; i < VISION_DATA_BUFFER_SIZE; i++)
    printf("%c", send_buf[i]);
  printf("\n");

  return 0;
}


/*****************************************************************************
 * Main
 ****************************************************************************/

int main(void){

  /* Initialize communication */
  if(init_communication(PORT, INTERFACE)){
    printf("Failed to initialize server");
    return -1;
  }


  /* Register clients */
    udp_server_setup_client("192.168.1.255", PORT,0);
  for(i=0; i < NUMBER_OF_CARS; i++){
    vision_data.car[i].t = ((float)i)/10;
    vision_data.car[i].x = ((float)i)/20;
    vision_data.car[i].y = -1.0*((float)i)/30;
  }
  
  for(i=0; i < NUMBER_OF_CARS; i++){
    printf("Vision_Data of car %i: T: %0.2f, X: %0.2f, Y: %0.2f\n", i,
        vision_data.car[i].t, vision_data.car[i].x ,vision_data.car[i].y);
  }

  while(1){
    write_vision();
  }

  /* Call controller and pass read/write as function pointer */

  return 0;
}


