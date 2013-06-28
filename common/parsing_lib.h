/****************************************************************************
 * src/common/parsing_lib.h
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
#ifndef PARSING_LIB_H
#define PARSING_LIB_H

/*****************************************************************************
 * Includes
 ****************************************************************************/
#include "xap.h"



/*****************************************************************************
 * Settings
 ****************************************************************************/

/* The size of a float in bytes (IEEE 754 Standard) */
#define FLOAT_SIZE 4

/* Define Message for convenient use with a enum */
enum car_data{
  CAR_T=0,
  CAR_X,
  CAR_Y,
  CAR_PSI,
  CAR_VX,
  CAR_VY,
  CAR_OMEGA,
  CAM_1_RAW_X,
  CAM_1_RAW_Y,
  CAM_1_RAW_PSI,
  CAM_1_SEEN,
  /* This part is only here for backwards compatibility. The new system has
   * only one camera.
   */
  CAM_2_RAW_X,
  CAM_2_RAW_Y,
  CAM_2_RAW_PSI,
  CAM_2_SEEN,


  /* Leave this at the Bottom */
  CAR_DATA_SIZE
};
/* The number of cars, normally 8, since our systems supports that much */
#define NUMBER_OF_CARS 8

/* These settings should not be changed unless you know what you are doing */
#define CAR_DATA_BUFFER_SIZE FLOAT_SIZE*CAR_DATA_SIZE 
#define VISION_DATA_BUFFER_SIZE NUMBER_OF_CARS*CAR_DATA_BUFFER_SIZE

/*****************************************************************************
 * Functions
 ****************************************************************************/
/**
 * Converts a complete car data struct to a char array
 * If the buffer has only the size for 1 car, use 1 as car index.
 *@param car_data struct that holds the car data as defined in xap.h
 *@param buff char buffer of the size of VISION_DATA_BUFFER_SIZE
 */
void car_data_to_char_array(states *data, const unsigned char *buff, int car_index);

/**
 * Converts a char array to a states struct with the car data in it.
 * If the buffer has only the size for 1 car, use 1 as car index.
 *@param car_data struct that holds the car data as defined in xap.h
 *@param buff char buffer of the size of VISION_DATA_BUFFER_SIZE
 */
void char_array_to_car_data(states *data, const unsigned char *buff, int car_index);

/**
 * Function for putting the states array in PI_IN into a char array
 * (the rest of the PI_IN struct is not read, as I could
 * not yet confirm that it is needed by the client. TODO: Check this
 *@param  vision_data  Struct holding the complete vision data
 *@param  buff  buffer holding the data
 */
void vision_data_to_char_array(PI_IN *vision_data, const unsigned char *buff);

/**
 * Function for parsing a char buffer to the states array in a PI_IN struct
 * as defined in xap.h (the rest of the PI_IN struct is not filled, as I could
 * not yet confirm that it is needed by the client. TODO: Check this)
 *@param  vision_data  Struct holding the complete vision data
 *@param  buff  buffer holding the data
 */
void char_array_to_vision_data(PI_IN *vision_data, const unsigned char *buff);


#endif /* PARSING_LIB_H */
