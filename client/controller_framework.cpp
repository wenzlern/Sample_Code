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
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "udp_client_lib.h"
#include <errno.h>
#include <parsing_lib.h>
#include <xap.h>
#include "bluetooth_lib.h"
#include <sys/time.h>


/*****************************************************************************
 * Definitons
 ****************************************************************************/
#define BLUETOOTH_BUFFER_SIZE 4
#define BLUETOOTH_PROTOCOL_START '#'
#define BLUETOOTH_PROTOCOL_STOP  '\r'

/* The clock to use. The system wide realtime clock is probably the best,
 * since this should also account for the time the process was suspended */
#define CLK_ID CLOCK_REALTIME
/* Define the speed of the loop in Hz*/
#define LOOP_SPEED 100 /* [Hz] */
/* The loop speed in nanoseconds (our max. resolution) */
#define LOOP_SPEED_NS 1e9/LOOP_SPEED

/*****************************************************************************
 * Variables
 ****************************************************************************/
/* Receive Buffer */
unsigned char rec_buf[VISION_DATA_BUFFER_SIZE];
/* Send Buffer */
char send_buf[BLUETOOTH_BUFFER_SIZE];
/* Generic return value holder */
int retval,i;
/* Struct that holds the vision information */
PI_IN vision_data;
/* Return values for the steering and throttle */
//TODO: Check/change them to the appropriate type
int steering_val, throttle_val;

/* Clock structs for the realtime implementation */
struct timespec start_time, end_time;
/* Runtime in nanoseconds */
long runtime;
/* Store the loopspeed value */
long loop_speed_ns;

/** Thread variables **/
/* Thread pointer */
pthread_t visionReadThread;
/* Mutex for synchronising reading thread and main algorithm */
pthread_mutex_t readMutex;
/* Copied value, the mutex is only locked when duplicating */
PI_IN vision_data_copy;


/*****************************************************************************
 * Communication
 ****************************************************************************/
/**
 * Initializes the udp receive service
 *@param addr IP address of the server
 *@param port  Port the server will send to
 *@return -1 on error, 0 on success
 */
int init_udp_conn(const char *addr, const char *port){
  return init_udp_client(addr, port);
}

/**
 * Initializes the BT service
 *@param addr BT address of the server
 *@param channel  RF_COMM channel of the server
 *@return -1 on error, 0 on success
 */
int init_bt_conn(const char *addr, const char *channel){
  return bluetooth_connect_to(addr, atoi(channel));
}



/**
 * Reads the vision server data stream
 *@return -1 on failure, 0 on success
 */
int read_vision_local(){
  retval = udp_client_read(rec_buf, VISION_DATA_BUFFER_SIZE);
  if( retval > 0){
    /* Parse to vision_data struct */
    char_array_to_vision_data(&vision_data, rec_buf);
//    for(i=0; i < NUMBER_OF_CARS; i++){
//      printf("Vision_Data of car %i: T: %0.2f, X: %0.2f, Y: %0.2f\n", i,
//          vision_data.car[i].t, vision_data.car[i].x ,vision_data.car[i].y);
//    }
    return 0;
  }
  return -1;
}


/**
 * Reads the vision server data stream, run in separate thread
 *@return -1 on failure, 0 on success
 */
void *read_vision_thread(void *dummy)
{
  while(1) {
    retval = udp_client_read(rec_buf, VISION_DATA_BUFFER_SIZE);
    if( retval > 0 && pthread_mutex_trylock(&readMutex) == 0) {
      /* Parse to vision_data struct */
      char_array_to_vision_data(&vision_data, rec_buf);
      pthread_mutex_unlock(&readMutex);
    }
  }
  return 0;
}


/**
 * Writes the output of the controller to the car.
 */
int write_control(){
  send_buf[1] = throttle_val;
  send_buf[2] = steering_val;
  bluetooth_send(send_buf, BLUETOOTH_BUFFER_SIZE);
  return 0;
}


/**
 * Calculate the total loop runtime and sleep for the remaining time
 */
void finishCycle()
{
  /* Calculate runtime */
  runtime = (end_time.tv_sec - start_time.tv_sec)*1e9 + (end_time.tv_nsec - start_time.tv_nsec);

  /* Check if runtime is slower than loop */
  if(runtime > loop_speed_ns){
    printf("Waring: Runtime (%fms) longer than clock cycle time (%fms)\n",(float) runtime/1e6, (float) loop_speed_ns/1e6);
  }
  else{
    /* We just reuse the end_time struct to store the sleep parameters */
    end_time.tv_sec = 0;
    end_time.tv_nsec = loop_speed_ns - runtime;
    printf("runtime %ld, sleep: %ld\n",runtime, end_time.tv_nsec);
    nanosleep(&end_time, NULL);
  }
}

/*
 * Write the vision data as Matlab struct
 *@param vision_data The struct with the vision data to process
 *@param car_nr The number of the car to print
 *@param name format for printing
 *@param fp Filepointer to destination file.
 */
void writeVisionData(PI_IN vision_data, int car_nr, char *name, FILE *fp)
{
  fprintf(fp, name);
  fprintf(fp, ".car{%i}.x = %f; ", car_nr, vision_data.car[car_nr].x);
  
  fprintf(fp, name);
  fprintf(fp, ".car{%i}.y = %f; ", car_nr, vision_data.car[car_nr].y);
  
  fprintf(fp, name); 
  fprintf(fp, ".car{%i}.psi = %f; ", car_nr, vision_data.car[car_nr].psi);

  fprintf(fp, name);
  fprintf(fp, ".car{%i}.vx = %f; ", car_nr, vision_data.car[car_nr].vx);

  fprintf(fp, name);
  fprintf(fp, ".car{%i}.vy = %f;\n", car_nr, vision_data.car[car_nr].vy);
}




/*****************************************************************************
 * Main
 ****************************************************************************/

int main(int argc, char **argv){

  /* Set the loop speed to default. This will get raised if the loop is to
   * slow 
   */
  loop_speed_ns = LOOP_SPEED_NS;

  /* Parse the input */
  if(argc < 3){
    printf("Usage: controller_framework [IP_ADDR] [PORT] [BT_ADDR] "); 
    printf("[RF_COMM CHANNEL]\n");
    return EINVAL;
  }

  /* Initialize communication */
  if(init_udp_conn(argv[1], argv[2]))
    return -1;

  if(init_bt_conn(argv[3], argv[4]))
    return -1;

  /* Preset the send buffer, since the protocol part of it does not change */
  send_buf[0] = BLUETOOTH_PROTOCOL_START;
  send_buf[3] = BLUETOOTH_PROTOCOL_STOP;

  /* initialization of mutex for threading */
  pthread_mutex_init(&readMutex, NULL);


  /* if the thread is successfully created -> start the algorithm */
  if (!pthread_create(&visionReadThread, NULL, read_vision_thread, (void *)NULL))
  {

    /* Intitialize the controller*/
    //The controller needs vision data to initialize, therefore it sits here.

    while(1){

      /* Get the current time */
      clock_gettime(CLK_ID, &start_time);


      /* Write the control information at the begining of the loop.
       * This way we have a defined point at where we write it.
       */
      write_control();

      /* Copy the vision data from the read thread */
      pthread_mutex_lock(&readMutex);
      memcpy(&vision_data_copy, &vision_data, sizeof(PI_IN));
      pthread_mutex_unlock(&readMutex);


      /* Call the controller */
      //the_algorithm(vision_data_copy, &steering_val, &throttle_val);


      /* Get current time */
      clock_gettime(CLK_ID, &end_time);
      /* Calculate runtime and sleep */
      finishCycle();

    }
  }
  else
  {
    printf("Error when creating the thread!\n");
  }

  pthread_exit(NULL);

  return 0;
}
