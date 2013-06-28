/****************************************************************************
 * src/common/parsing_lib.c
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
#include <string.h>
#include <parsing_lib.h>

/*****************************************************************************
 * Definitions
 ****************************************************************************/


/*****************************************************************************
 * Variables
 ****************************************************************************/

/*****************************************************************************
 * Private Functions
 ****************************************************************************/
/**
 * Converts a single float to a char array
 *@param f  float to convert
 *@param dest char array of min size FLOAT_SIZE
 */
void float_to_char_array(float *f, const unsigned char* dest){
  /* To speed up the conversion I use a fixed size for float size, rather than
   * sizeof(float). This should not give errors, however if you encounter
   * Segfaults or other memory problems, check this value on your system with
   * printf("Size of float: %lu", sizeof(float));
   */
  memcpy((void*)dest, f, FLOAT_SIZE);
}

/**
 * Converts a char array to a float
 *@param f  float to store to
 *@param source char array to read from
 */
void char_array_to_float(float *f, const unsigned char* source){
  /* To speed up the conversion I use a fixed size for float size, rather than
   * sizeof(float). This should not give errors, however if you encounter
   * Segfaults or other memory problems, check this value on your system with
   * printf("Size of float: %lu", sizeof(float));
   */
  memcpy(f, source, FLOAT_SIZE);
}


/*****************************************************************************
 * Public Functions
 ****************************************************************************/
/**
 * Converts a complete car data struct to a char array
 * If the buffer has only the size for 1 car, use 1 as car index.
 *@param car_data struct that holds the car data as defined in xap.h
 *@param buff char buffer of the size of VISION_DATA_BUFFER_SIZE
 */
void car_data_to_char_array(states *data, const unsigned char *buff, int car_index){

  /* This next part may look ugly, but it basically only calls the float to
   * char function with the appropriate float and the shifted index
   */
  float_to_char_array(&(data->t),             
      ((buff + (FLOAT_SIZE*CAR_T) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->x),             
      ((buff + (FLOAT_SIZE*CAR_X) +  (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->y),             
      ((buff + (FLOAT_SIZE*CAR_Y) +  (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->psi),           
      ((buff + (FLOAT_SIZE*CAR_PSI) +(car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->vx),            
      ((buff + (FLOAT_SIZE*CAR_VX) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->vy),            
      ((buff + (FLOAT_SIZE*CAR_VY) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->omega),         
      ((buff + (FLOAT_SIZE*CAR_OMEGA) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_1_raw_x),   
      ((buff + (FLOAT_SIZE*CAM_1_RAW_X) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_1_raw_y),   
      ((buff + (FLOAT_SIZE*CAM_1_RAW_Y) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_1_raw_psi), 
      ((buff + (FLOAT_SIZE*CAM_1_RAW_PSI) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_1_seen),
      ((buff + (FLOAT_SIZE*CAM_1_SEEN) + (car_index*CAR_DATA_BUFFER_SIZE))));
  /* This part is only here for backwards compatibility. The new system has
   * only one camera.
   */
  float_to_char_array(&(data->cam_2_raw_x),   
      ((buff + (FLOAT_SIZE*CAM_2_RAW_X) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_2_raw_y),   
      ((buff + (FLOAT_SIZE*CAM_2_RAW_Y) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_2_raw_psi), 
      ((buff + (FLOAT_SIZE*CAM_2_RAW_PSI) + (car_index*CAR_DATA_BUFFER_SIZE))));
  float_to_char_array(&(data->cam_2_seen),
      ((buff + (FLOAT_SIZE*CAM_2_SEEN) + (car_index*CAR_DATA_BUFFER_SIZE))));
}

/**
 * Converts a char array to a states struct with the car data in it.
 * If the buffer has only the size for 1 car, use 1 as car index.
 *@param car_data struct that holds the car data as defined in xap.h
 *@param buff char buffer of the size of VISION_DATA_BUFFER_SIZE
 */
void char_array_to_car_data(states *data, const unsigned char *buff, int car_index){

  /* This next part may look ugly, but it basically only calls the float to
   * char function with the appropriate float and the shifted index
   */
  char_array_to_float(&(data->t),             
      ((buff + (FLOAT_SIZE*CAR_T) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->x),             
      ((buff + (FLOAT_SIZE*CAR_X) +  (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->y),             
      ((buff + (FLOAT_SIZE*CAR_Y) +  (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->psi),           
      ((buff + (FLOAT_SIZE*CAR_PSI) +(car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->vx),            
      ((buff + (FLOAT_SIZE*CAR_VX) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->vy),            
      ((buff + (FLOAT_SIZE*CAR_VY) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->omega),         
      ((buff + (FLOAT_SIZE*CAR_OMEGA) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_1_raw_x),   
      ((buff + (FLOAT_SIZE*CAM_1_RAW_X) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_1_raw_y),   
      ((buff + (FLOAT_SIZE*CAM_1_RAW_Y) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_1_raw_psi), 
      ((buff + (FLOAT_SIZE*CAM_1_RAW_PSI) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_1_seen),
      ((buff + (FLOAT_SIZE*CAM_1_SEEN) + (car_index*CAR_DATA_BUFFER_SIZE))));
  /* This part is only here for backwards compatibility. The new system has
   * only one camera.
   */
  char_array_to_float(&(data->cam_2_raw_x),   
      ((buff + (FLOAT_SIZE*CAM_2_RAW_X) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_2_raw_y),   
      ((buff + (FLOAT_SIZE*CAM_2_RAW_Y) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_2_raw_psi), 
      ((buff + (FLOAT_SIZE*CAM_2_RAW_PSI) + (car_index*CAR_DATA_BUFFER_SIZE))));
  char_array_to_float(&(data->cam_2_seen),
      ((buff + (FLOAT_SIZE*CAM_2_SEEN) + (car_index*CAR_DATA_BUFFER_SIZE))));
}

/**
 * Function for putting the states array in PI_IN into a char array
 * (the rest of the PI_IN struct is not read, as I could
 * not yet confirm that it is needed by the client. TODO: Check this
 *@param  vision_data  Struct holding the complete vision data
 *@param  buff  buffer holding the data
 */
void vision_data_to_char_array(PI_IN *vision_data, const unsigned char *buff){
  /* Parse through all cars */
  int i;
  for(i=0; i < NUMBER_OF_CARS; i++){
    car_data_to_char_array(&(vision_data->car[i]), buff, i);
  }
}
 

/**
 * Function for parsing a char buffer to the states array in a PI_IN struct
 * as defined in xap.h (the rest of the PI_IN struct is not filled, as I could
 * not yet confirm that it is needed by the client. TODO: Check this)
 *@param  vision_data  Struct holding the complete vision data
 *@param  buff  buffer holding the data
 */
void char_array_to_vision_data(PI_IN *vision_data, const unsigned char *buff){
  /* Parse through all cars */
  int i;
  for(i=0; i < NUMBER_OF_CARS; i++){
    char_array_to_car_data(&(vision_data->car[i]), buff, i);
  }
}
  
